idf_component_register(
    SRCS "app_main.cpp"
         "board.cpp"
         "console_service.cpp"
         "display_service.cpp"
         "fluid_app.cpp"
         "input_service.cpp"
         "motion_service.cpp"
         "motion.cpp"
         "fluid.cpp"
         "snapshot_exchange.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES console esp_lcd esp_driver_gpio esp_driver_i2c esp_driver_spi esp_timer esp_psram spi_flash heap
)

# Global app constraint: C++20 without exceptions or RTTI.
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++20 -fno-exceptions -fno-rtti)
