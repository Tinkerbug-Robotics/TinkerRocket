# Lc86Parser.cpp is deliberately IDF-free (bytes in, GNSSData out) so the
# protocol parsing is host-testable — tests_cpp/test_lc86_parser.cpp compiles
# it directly. Keep ESP-only code in TR_GNSSReceiverLC86_Serial.cpp.
idf_component_register(SRCS "TR_GNSSReceiverLC86_Serial.cpp" "Lc86Parser.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_uart TR_RocketComputerTypes driver esp_timer)
