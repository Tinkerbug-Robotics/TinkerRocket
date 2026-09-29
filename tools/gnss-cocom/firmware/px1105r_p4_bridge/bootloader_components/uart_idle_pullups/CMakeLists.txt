# Bootloader hook, linked through bootloader_hooks_include (the bootloader's main component asks for that symbol).
idf_component_register(SRCS "uart_idle_pullups.c"
                       PRIV_REQUIRES esp_hal_gpio)
