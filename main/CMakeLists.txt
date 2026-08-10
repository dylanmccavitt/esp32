idf_component_register(
    SRCS "app_main.cpp"
         "board.cpp"
         "dev_console.cpp"
         "motion.cpp"
         "fluid.cpp"
         "snapshot_exchange.cpp"
         "renderer.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES console esp_lcd esp_driver_gpio esp_driver_i2c esp_driver_spi esp_timer esp_psram spi_flash heap
)

# Global app constraint: C++20 without exceptions or RTTI.
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++20 -fno-exceptions -fno-rtti)
