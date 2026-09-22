idf_component_register(SRCS "adc_utils.cpp" "dtw.cpp" "state_utils.cpp" "comms_utils.cpp" "imu_utils.cpp" "main.cpp"
                    PRIV_REQUIRES nvs_flash esp_event esp_netif esp_wifi esp_adc
                    INCLUDE_DIRS ".")

add_compile_definitions(DEVICE_CENTRAL)
# add_compile_definitions(DEVICE_PERIPHERAL)
add_compile_definitions(DEVICE_HAND)
# add_compile_definitions(DEVICE_ARM)