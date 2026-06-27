idf_component_register(
                    SRCS "gmeter.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_i2c utils
                    )
