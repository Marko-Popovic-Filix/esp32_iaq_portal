#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Start/stop the IAQ sensing loop as a FreeRTOS task.
// core_id: 0 or 1 to pin, or -1 for unpinned.
// stack_words: task stack size in *words* (not bytes). Pass 0 to use default (8192 words).
// priority: FreeRTOS task priority. Pass 0 to use default (5).
esp_err_t sensor_iaq_start_on_core(BaseType_t core_id);
void      sensor_iaq_stop(void);

// Optional getters for last values (thread-safe snapshots).
typedef struct {
    float iaq, co2eq, bvoc, temperature_c, humidity_rh, pressure_hpa;
    int8_t iaq_acc, co2_acc, bvoc_acc;
} sensor_iaq_snapshot_t;

bool sensor_iaq_get_last(sensor_iaq_snapshot_t *out);

#ifdef __cplusplus
}
#endif
