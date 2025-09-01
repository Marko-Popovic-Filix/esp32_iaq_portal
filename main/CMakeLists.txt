idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        sensor_iaq
        wifi_setup
        app_cfg
        utils
)
