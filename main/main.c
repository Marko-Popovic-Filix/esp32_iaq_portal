// main.c — minimal launcher: start IAQ task (Core 1) + Wi-Fi setup portal (Core 0)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "sensor_iaq.h"
#include "wifi_setup.h"

static const char *TAG = "MAIN";

static void nvs_init_once(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "boot: init NVS");
    nvs_init_once();

    // IAQ task on Core 1 (APP CPU)
    ESP_ERROR_CHECK(sensor_iaq_start_on_core(1));

    // Wi-Fi setup portal on Core 0 (PRO CPU)
    ESP_ERROR_CHECK(wifi_setup_start_on_core(0));

    // Long-press (GPIO0) monitor on Core 0 so you can clear creds
    wifi_setup_start_button_monitor_on_core(0);

    ESP_LOGI(TAG, "launched: IAQ@Core1, Wi-Fi portal@Core0, BTN monitor@Core0");
}
