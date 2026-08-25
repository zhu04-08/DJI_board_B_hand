idf_component_register(SRCS "tf.c" "sc16is752.c" "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver fatfs sdmmc)
                    