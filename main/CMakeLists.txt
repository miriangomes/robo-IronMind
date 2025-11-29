idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES
                        esp_driver_gpio
                        esp_driver_ledc
                        freertos
                        esp_hw_support
                        spi_flash
                        esp_system
                        esp_timer
                        esp_rom)