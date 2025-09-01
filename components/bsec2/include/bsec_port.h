#pragma once
#include <stdint.h>
#include "bsec_datatypes.h"   // Provided by Bosch BSEC2 package
#ifdef __cplusplus
extern "C" {
#endif

// I2C init + BME68x init. Returns 0 on success.
int bsec_port_init_i2c(int sda, int scl, int addr);

// Load config: You call bsec_set_configuration() yourself in app_main using the array provided in bsec_config.c
extern const uint8_t bsec_config_iaq[];
extern const uint32_t bsec_config_iaq_len;

// Subscribe helper
int bsec_port_update_subscription(const bsec_virtual_sensor_t *list, int n, float sample_rate);

// Run one BSEC cycle, fill outputs (array len must be >= BSEC_NUMBER_OUTPUTS).
// Returns 0 on success; sets *n_outputs and *next_call_ms (absolute ms since boot).
int bsec_port_run_and_fetch(bsec_output_t *outputs, uint8_t *n_outputs, uint64_t *next_call_ms);

// State helpers
int bsec_port_get_state(uint8_t *blob, uint32_t *len);
int bsec_port_set_state(const uint8_t *blob, uint32_t len);

#ifdef __cplusplus
}
#endif