idf_component_register(SRCS "esp-mma7361.c"
                    INCLUDE_DIRS "include"
                    REQUIRES 
                        driver
                        esp_adc)
