idf_component_register(
    SRCS "sensor_iaq.c"
    INCLUDE_DIRS "."
    REQUIRES bsec2 bme68x nvs_flash esp_timer driver
)
