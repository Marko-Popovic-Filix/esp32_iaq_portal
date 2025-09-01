#include "app_cfg.h"
#include "sdkconfig.h"
void app_cfg_init(void) {}
const char* app_cfg_ap_ssid_prefix(void) { return CONFIG_APP_AP_SSID_PREFIX; }
const char* app_cfg_ap_password(void)    { return CONFIG_APP_AP_PASSWORD; }
int app_cfg_button_gpio(void)            { return CONFIG_APP_BUTTON_GPIO; }
int app_cfg_button_long_ms(void)         { return CONFIG_APP_BUTTON_LONG_MS; }
