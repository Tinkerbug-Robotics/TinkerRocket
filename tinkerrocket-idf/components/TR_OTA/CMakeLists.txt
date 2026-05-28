idf_component_register(
    SRCS "TR_OTA_Receiver.cpp" "TR_OTA_Backend_esp.cpp"
    INCLUDE_DIRS "include"
    REQUIRES app_update esp_partition mbedtls
)
