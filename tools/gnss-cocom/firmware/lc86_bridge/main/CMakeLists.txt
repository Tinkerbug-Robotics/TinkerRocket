idf_component_register(SRCS "lc86_bridge.c"
                       PRIV_REQUIRES esp_driver_uart esp_driver_usb_serial_jtag
                                     esp_driver_gpio esp_rom vfs)
