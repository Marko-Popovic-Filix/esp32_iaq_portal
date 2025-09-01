idf_component_register(
    SRCS "bsec_port.c" "bsec_config.c"
    INCLUDE_DIRS "include"          # exposes bsec_port.h to other components
    REQUIRES bme68x                 # uses the BME68x component's API
    PRIV_REQUIRES driver esp_timer  # uses driver/i2c.h and esp_timer_get_time()
)

# Link Bosch BSEC2 prebuilt lib (adjust path/variant if needed)
set(BSEC_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/xtensa/libalgobsec.a")
if (EXISTS "${BSEC_LIB}")
    message(STATUS "Linking BSEC library: ${BSEC_LIB}")
    target_link_libraries(${COMPONENT_LIB} INTERFACE "${BSEC_LIB}")
else()
    message(WARNING "BSEC library not found at ${BSEC_LIB}. Build will fail at link step until you add it.")
endif()