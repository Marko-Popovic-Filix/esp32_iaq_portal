#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "bme68x.h"
#include "bsec_interface.h"
#include "bsec_datatypes.h"
#include "bsec_port.h"

#include "sensor_iaq.h"

static const char *TAG = "SENSOR_IAQ";

/* ---------- Pins / address (override via sdkconfig if you like) ---------- */
#ifndef CONFIG_BME68X_SDA
#define CONFIG_BME68X_SDA 21
#endif
#ifndef CONFIG_BME68X_SCL
#define CONFIG_BME68X_SCL 22
#endif
#ifndef CONFIG_BME68X_ADDR
#define CONFIG_BME68X_ADDR 0x76
#endif

/* ---------- Accuracy helper ---------- */
static inline const char* acc_text(int8_t acc)
{
    switch (acc) {
        case 3: return "high";
        case 2: return "medium";
        case 1: return "low";
        default: return "unreliable";
    }
}

/* ---------- NVS helpers for BSEC state ---------- */
#define NVS_NAMESPACE   "bsec"
#define NVS_KEY_STATE   "state"

static bool bsec_state_load(uint8_t *buf, uint32_t *len)
{
    if (!buf || !len) return false;
    *len = 0;

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;

    size_t sz = 0;
    esp_err_t err = nvs_get_blob(h, NVS_KEY_STATE, NULL, &sz);
    if (err == ESP_OK && sz > 0 && sz <= BSEC_MAX_STATE_BLOB_SIZE) {
        err = nvs_get_blob(h, NVS_KEY_STATE, buf, &sz);
        if (err == ESP_OK) {
            *len = (uint32_t)sz;
            nvs_close(h);
            return true;
        }
    }
    nvs_close(h);
    return false;
}

static void bsec_state_save(const uint8_t *buf, uint32_t len)
{
    if (!buf || len == 0 || len > BSEC_MAX_STATE_BLOB_SIZE) return;

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    if (nvs_set_blob(h, NVS_KEY_STATE, buf, len) == ESP_OK) {
        nvs_commit(h);
        ESP_LOGI(TAG, "Saved BSEC state (%u bytes).", len);
    }
    nvs_close(h);
}

/* ---------- Last-values snapshot (thread-safe) ---------- */
static sensor_iaq_snapshot_t s_last = {0};
static SemaphoreHandle_t s_last_mtx = NULL;

bool sensor_iaq_get_last(sensor_iaq_snapshot_t *out)
{
    if (!out || !s_last_mtx) return false;
    if (xSemaphoreTake(s_last_mtx, pdMS_TO_TICKS(50)) != pdTRUE) return false;
    *out = s_last;
    xSemaphoreGive(s_last_mtx);
    return true;
}

/* ---------- Task control ---------- */
static TaskHandle_t s_task = NULL;

static void iaq_task(void *arg)
{
    // init I2C + sensor + BSEC
    if (bsec_port_init_i2c(CONFIG_BME68X_SDA, CONFIG_BME68X_SCL, CONFIG_BME68X_ADDR) != 0) {
        ESP_LOGE(TAG, "bsec_port_init_i2c failed");
        vTaskDelete(NULL);
        return;
    }

    // Restore BSEC state if available
    {
        uint8_t state[BSEC_MAX_STATE_BLOB_SIZE] = {0};
        uint32_t state_len = 0;
        if (bsec_state_load(state, &state_len) && state_len > 0) {
            if (bsec_port_set_state(state, state_len) == 0) {
                ESP_LOGI(TAG, "Restored BSEC state (%u bytes).", state_len);
            } else {
                ESP_LOGW(TAG, "Failed to apply saved BSEC state.");
            }
        } else {
            ESP_LOGI(TAG, "No previous BSEC state in NVS (fresh start).");
        }
    }

    // Subscribe to virtual outputs (LP ~3s cadence)
    const bsec_virtual_sensor_t subs[] = {
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
        BSEC_OUTPUT_RAW_PRESSURE, // include pressure
    };
    if (bsec_port_update_subscription(subs, sizeof(subs)/sizeof(subs[0]), BSEC_SAMPLE_RATE_LP) != 0) {
        ESP_LOGE(TAG, "bsec_port_update_subscription failed");
        vTaskDelete(NULL);
        return;
    }

    if (!s_last_mtx) s_last_mtx = xSemaphoreCreateMutex();

    // State save cadence
    int64_t last_state_save_ms = (int64_t)(esp_timer_get_time() / 1000);
    const int64_t save_period_ms = 15 * 60 * 1000; // 15 min
    bool saved_once_at_acc3 = false;

    // Main loop
    bsec_output_t outputs[BSEC_NUMBER_OUTPUTS] = {0};
    uint8_t n_outputs_cap = BSEC_NUMBER_OUTPUTS;
    uint64_t next_call_ms = 0;

    const TickType_t fallback_delay = pdMS_TO_TICKS(3000);

    while (1) {
        uint8_t n_outputs = n_outputs_cap;
        int rc = bsec_port_run_and_fetch(outputs, &n_outputs, &next_call_ms);
        if (rc != 0 || n_outputs == 0) {
            ESP_LOGW(TAG, "No outputs produced this cycle; retrying.");
            vTaskDelay(fallback_delay);
            continue;
        }

        float iaq = NAN, co2 = NAN, bvoc = NAN, t = NAN, rh = NAN, p_hpa = NAN;
        int8_t iaq_acc = 0, co2_acc = 0, bvoc_acc = 0;

        for (uint8_t i = 0; i < n_outputs; ++i) {
            const bsec_output_t *o = &outputs[i];
            switch (o->sensor_id) {
                case BSEC_OUTPUT_IAQ:
                    iaq = o->signal; iaq_acc = o->accuracy; break;
                case BSEC_OUTPUT_CO2_EQUIVALENT:
                    co2 = o->signal; co2_acc = o->accuracy; break;
                case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
                    bvoc = o->signal; bvoc_acc = o->accuracy; break;
                case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                    t = o->signal; break;
                case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                    rh = o->signal; break;
                case BSEC_OUTPUT_RAW_PRESSURE:
                    p_hpa = o->signal / 100.0f; // Pa -> hPa
                    break;
                default: break;
            }
        }

        // log line
        ESP_LOGI(TAG,
                 "IAQ=%s  CO2eq=%s  bVOCeq=%s  T=%s  RH=%s  P=%s",
                 isnan(iaq)  ? "--" : ({ static char buf[32]; snprintf(buf, sizeof(buf), "%.1f (%s)", iaq, acc_text(iaq_acc)); buf; }),
                 isnan(co2)  ? "--" : ({ static char buf[32]; snprintf(buf, sizeof(buf), "%.1f ppm (%s)", co2, acc_text(co2_acc)); buf; }),
                 isnan(bvoc) ? "--" : ({ static char buf[32]; snprintf(buf, sizeof(buf), "%.3f ppm (%s)", bvoc, acc_text(bvoc_acc)); buf; }),
                 isnan(t)    ? "--" : ({ static char buf[32]; snprintf(buf, sizeof(buf), "%.2f°C", t); buf; }),
                 isnan(rh)   ? "--" : ({ static char buf[32]; snprintf(buf, sizeof(buf), "%.2f%%", rh); buf; }),
                 isnan(p_hpa)? "--" : ({ static char buf[32]; snprintf(buf, sizeof(buf), "%.2f hPa", p_hpa); buf; })
        );

        // publish snapshot
        if (s_last_mtx && xSemaphoreTake(s_last_mtx, pdMS_TO_TICKS(10)) == pdTRUE) {
            s_last.iaq = iaq; s_last.iaq_acc = iaq_acc;
            s_last.co2eq = co2; s_last.co2_acc = co2_acc;
            s_last.bvoc = bvoc; s_last.bvoc_acc = bvoc_acc;
            s_last.temperature_c = t;
            s_last.humidity_rh = rh;
            s_last.pressure_hpa = p_hpa;
            xSemaphoreGive(s_last_mtx);
        }

        // Save BSEC state periodically / once at acc=3
        int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000);
        bool time_to_save = (now_ms - last_state_save_ms) >= save_period_ms;
        bool acc3_event   = (iaq_acc >= 3) && !saved_once_at_acc3;

        if (time_to_save || acc3_event) {
            uint8_t blob[BSEC_MAX_STATE_BLOB_SIZE] = {0};
            uint32_t blob_len = sizeof(blob);
            if (bsec_port_get_state(blob, &blob_len) == 0 && blob_len > 0) {
                bsec_state_save(blob, blob_len);
                last_state_save_ms = now_ms;
                if (acc3_event) saved_once_at_acc3 = true;
            }
        }

        // Respect BSEC's schedule
        if (next_call_ms != 0) {
            int64_t ms_to_wait = (int64_t)next_call_ms - (int64_t)(esp_timer_get_time() / 1000);
            if (ms_to_wait < 0) ms_to_wait = 0;
            vTaskDelay(pdMS_TO_TICKS((uint32_t)ms_to_wait));
        } else {
            vTaskDelay(fallback_delay);
        }
    }
}

/* ---------- Public API ---------- */
esp_err_t sensor_iaq_start_on_core(BaseType_t core_id)
{
    if (s_task) return ESP_OK; // already running

    // Ensure NVS is ready for state save/restore (OK if already inited in main)
    // (It's safe to call nvs_flash_init() again.)
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(e);
    }

    const uint32_t stack_words = 8192; // generous for BSEC
    const UBaseType_t prio = 5;

    if (core_id == 0 || core_id == 1) {
        if (xTaskCreatePinnedToCore(iaq_task, "iaq", stack_words, NULL, prio, &s_task, core_id) != pdPASS) {
            return ESP_FAIL;
        }
    } else {
        if (xTaskCreate(iaq_task, "iaq", stack_words, NULL, prio, &s_task) != pdPASS) {
            return ESP_FAIL;
        }
    }
    ESP_LOGI(TAG, "sensor_iaq task started (core=%ld)", (long)core_id);
    return ESP_OK;
}

void sensor_iaq_stop(void)
{
    if (s_task) {
        TaskHandle_t t = s_task;
        s_task = NULL;
        vTaskDelete(t);
        ESP_LOGI(TAG, "sensor_iaq task stopped");
    }
}
