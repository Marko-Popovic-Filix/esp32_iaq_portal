// components/bsec2/bsec_port.c
#include <string.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"       // esp_timer_get_time()
#include "esp_rom_sys.h"     // esp_rom_delay_us()
#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bme68x.h"
#include "bsec_interface.h"
#include "bsec_datatypes.h"   // BSEC_MAX_* sizes
#include "bsec_port.h"

#define TAG "bsec_port"
#define I2C_PORT I2C_NUM_0

// ---- Module-scope state ----
static struct bme68x_dev g_bme;
static uint8_t g_i2c_addr = 0x76;

// ---- Time helpers ----
// esp_timer_get_time() returns microseconds; BSEC v1 expects timestamps in nanoseconds.
static inline int64_t now_ns(void)
{
    return (int64_t)esp_timer_get_time() * 1000; // us -> ns (absolute since boot)
}

// ---- I2C helpers for BME68x ----
static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t addr = *(uint8_t*)intf_ptr;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, reg_data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return (err == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t addr = *(uint8_t*)intf_ptr;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, (uint8_t*)reg_data, len, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return (err == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

static void delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    // Safe for short delays only; we avoid long busy-waits elsewhere.
    esp_rom_delay_us(period);
}

// ---- Public API ----
int bsec_port_init_i2c(int sda, int scl, int addr)
{
    g_i2c_addr = (uint8_t)addr;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = { .clk_speed = 100000 }, // 100 kHz; raise to 400kHz if wiring is solid
        .clk_flags = 0
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));

    g_bme.read     = i2c_read;
    g_bme.write    = i2c_write;
    g_bme.delay_us = delay_us;
    g_bme.intf     = BME68X_I2C_INTF;
    g_bme.intf_ptr = &g_i2c_addr;
    g_bme.amb_temp = 25; // default ambient temp for compensations

    int8_t rslt = bme68x_init(&g_bme);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "bme68x_init failed: %d", rslt);
        return -1;
    }

    bsec_library_return_t brslt = bsec_init();
    if (brslt != BSEC_OK) {
        ESP_LOGE(TAG, "bsec_init failed: %d", brslt);
        return -2;
    }

    return 0;
}

// Keep this so main() can enable virtual outputs at a chosen rate.
// For BSEC v1, pass capacity IN for 'required', get count OUT in n_required.
int bsec_port_update_subscription(const bsec_virtual_sensor_t *list, int n, float sample_rate)
{
    bsec_sensor_configuration_t requested[BSEC_MAX_PHYSICAL_SENSOR];
    bsec_sensor_configuration_t required[BSEC_MAX_PHYSICAL_SENSOR];

    if (n > (int)BSEC_MAX_PHYSICAL_SENSOR) n = BSEC_MAX_PHYSICAL_SENSOR;

    uint8_t n_req = 0;
    for (int i = 0; i < n; ++i) {
        requested[i].sensor_id   = list[i];
        requested[i].sample_rate = sample_rate;
        n_req++;
    }

    uint8_t n_required = BSEC_MAX_PHYSICAL_SENSOR;
    bsec_library_return_t rslt = bsec_update_subscription(
        requested, n_req,
        required, &n_required
    );
    if (rslt != BSEC_OK) {
        ESP_LOGE(TAG, "bsec_update_subscription failed: %d", rslt);
        return -1;
    }

    for (uint8_t i = 0; i < n_required; ++i) {
        ESP_LOGD(TAG, "required[%d]: id=%u rate=%f", i, required[i].sensor_id, required[i].sample_rate);
    }
    return 0;
}

// Drive BME68x according to BSEC's timing plan and return BSEC outputs.
// 'outputs' must have capacity >= BSEC_NUMBER_OUTPUTS. If *n_outputs is set,
// it's treated as provided capacity; actual count is written back.
int bsec_port_run_and_fetch(bsec_output_t *outputs, uint8_t *n_outputs, uint64_t *next_call_ms)
{
    // 1) Ask BSEC what to do next
    bsec_bme_settings_t ctrl = (bsec_bme_settings_t){0};
    int64_t t_ns = now_ns();
    bsec_library_return_t br = bsec_sensor_control(t_ns, &ctrl);

    /* In BSEC v1:
     *   br < 0  -> error
     *   br == 0 -> OK
     *   br > 0  -> WARNING (e.g., timing violation) — not fatal
     */
    if (br < BSEC_OK) {
        ESP_LOGE(TAG, "bsec_sensor_control error: %d", br);
        if (next_call_ms) *next_call_ms = (uint64_t)(ctrl.next_call / 1000000ULL); // ABSOLUTE ms
        return -1;
    } else if (br > BSEC_OK) {
        ESP_LOGW(TAG, "bsec_sensor_control warning: %d", br);
        if (next_call_ms) *next_call_ms = (uint64_t)(ctrl.next_call / 1000000ULL); // ABSOLUTE ms
        return 0; // let main() sleep until next_call
    }

    // If no work yet, just report the next absolute time
    if (ctrl.process_data == 0 && ctrl.trigger_measurement == 0) {
        if (next_call_ms) *next_call_ms = (uint64_t)(ctrl.next_call / 1000000ULL); // ABSOLUTE ms
        return 0; // main() will delay
    }

    // 2) Configure BME68x per BSEC plan
    struct bme68x_conf conf;
    if (bme68x_get_conf(&conf, &g_bme) != BME68X_OK) {
        ESP_LOGE(TAG, "bme68x_get_conf failed");
        if (next_call_ms) *next_call_ms = (uint64_t)(ctrl.next_call / 1000000ULL);
        return -1;
    }
    conf.os_hum  = ctrl.humidity_oversampling;
    conf.os_temp = ctrl.temperature_oversampling;
    conf.os_pres = ctrl.pressure_oversampling;
    if (bme68x_set_conf(&conf, &g_bme) != BME68X_OK) {
        ESP_LOGE(TAG, "bme68x_set_conf failed");
        if (next_call_ms) *next_call_ms = (uint64_t)(ctrl.next_call / 1000000ULL);
        return -1;
    }

    struct bme68x_heatr_conf hconf = {0};
    if (ctrl.heater_temperature > 0 && ctrl.heating_duration > 0) {
        hconf.enable     = BME68X_ENABLE;
        hconf.heatr_temp = ctrl.heater_temperature;  // °C
        hconf.heatr_dur  = ctrl.heating_duration;    // ms
    } else {
        hconf.enable = BME68X_DISABLE;
    }

    // BSEC LP plans are fine in FORCED mode here
    const uint8_t op_mode = BME68X_FORCED_MODE;

    if (bme68x_set_heatr_conf(op_mode, &hconf, &g_bme) != BME68X_OK) {
        ESP_LOGE(TAG, "bme68x_set_heatr_conf failed");
        if (next_call_ms) *next_call_ms = (uint64_t)(ctrl.next_call / 1000000ULL);
        return -1;
    }

    if (ctrl.trigger_measurement) {
        if (bme68x_set_op_mode(op_mode, &g_bme) != BME68X_OK) {
            ESP_LOGE(TAG, "bme68x_set_op_mode failed");
            if (next_call_ms) *next_call_ms = (uint64_t)(ctrl.next_call / 1000000ULL);
            return -1;
        }
    }

    // 3) Wait for the measurement to complete (non-busy sleep)
    uint32_t conv_time_us = bme68x_get_meas_dur(op_mode, &conf, &g_bme);
    uint32_t meas_ms = (conv_time_us / 1000) + (uint32_t)ctrl.heating_duration + 3; // +3ms margin
    if (meas_ms < 5) meas_ms = 5;
    vTaskDelay(pdMS_TO_TICKS(meas_ms));

    // 4) Read raw sample(s)
    struct bme68x_data data[3] = {0}; // safe for parallel mode too
    uint8_t n_fields = 0;
    if (bme68x_get_data(op_mode, data, &n_fields, &g_bme) != BME68X_OK || n_fields == 0) {
        ESP_LOGW(TAG, "No data from BME68x");
        if (next_call_ms) *next_call_ms = (uint64_t)(ctrl.next_call / 1000000ULL); // ABSOLUTE ms
        return -1;
    }

    // 5) Build BSEC inputs from the latest valid field and run BSEC
    struct bme68x_data *d = &data[n_fields - 1];
    bsec_input_t in[4];
    uint8_t nin = 0;
    int64_t ts_ns = now_ns();

    in[nin++] = (bsec_input_t){ .sensor_id = BSEC_INPUT_TEMPERATURE, .signal = d->temperature,     .time_stamp = ts_ns };
    in[nin++] = (bsec_input_t){ .sensor_id = BSEC_INPUT_HUMIDITY,    .signal = d->humidity,        .time_stamp = ts_ns };
    in[nin++] = (bsec_input_t){ .sensor_id = BSEC_INPUT_PRESSURE,    .signal = d->pressure,        .time_stamp = ts_ns };
    in[nin++] = (bsec_input_t){ .sensor_id = BSEC_INPUT_GASRESISTOR, .signal = d->gas_resistance,  .time_stamp = ts_ns };

    uint8_t out_cap = (n_outputs && *n_outputs) ? *n_outputs : BSEC_NUMBER_OUTPUTS;
    if (out_cap < BSEC_NUMBER_OUTPUTS) out_cap = BSEC_NUMBER_OUTPUTS; // safety
    uint8_t nout = out_cap;

    br = bsec_do_steps(in, nin, outputs, &nout);
    if (br != BSEC_OK) {
        ESP_LOGE(TAG, "bsec_do_steps failed: %d", br);
        if (next_call_ms) *next_call_ms = (uint64_t)(ctrl.next_call / 1000000ULL); // ABSOLUTE ms
        return -2;
    }

    if (n_outputs) *n_outputs = nout;

    // 6) Tell caller when to run next (ABSOLUTE ms since boot)
    if (next_call_ms) *next_call_ms = (uint64_t)(ctrl.next_call / 1000000ULL);
    return 0;
}

// ---- BSEC state helpers ----
int bsec_port_get_state(uint8_t *blob, uint32_t *len)
{
    if (!blob || !len || *len == 0) return -1;

    uint8_t  work_buf[BSEC_MAX_WORKBUFFER_SIZE] = {0};
    uint32_t out_len = 0;

    bsec_library_return_t rslt = bsec_get_state(
        0,                  // state_set_id
        blob, *len,         // output buffer + its max size
        work_buf, sizeof(work_buf),
        &out_len            // actual size returned
    );
    if (rslt != BSEC_OK) {
        ESP_LOGE(TAG, "bsec_get_state failed: %d", rslt);
        return -1;
    }
    *len = out_len;
    return 0;
}

int bsec_port_set_state(const uint8_t *blob, uint32_t len)
{
    if (!blob || len == 0) return -1;

    uint8_t work_buf[BSEC_MAX_WORKBUFFER_SIZE] = {0};

    bsec_library_return_t rslt = bsec_set_state(
        blob, len,
        work_buf, sizeof(work_buf)
    );
    if (rslt != BSEC_OK) {
        ESP_LOGE(TAG, "bsec_set_state failed: %d", rslt);
        return -1;
    }
    return 0;
}
