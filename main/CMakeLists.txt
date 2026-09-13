idf_component_register(SRCS "comms_utils.cpp" "imu_utils.cpp" "main.cpp"
                    PRIV_REQUIRES nvs_flash esp_event esp_netif esp_wifi
                    INCLUDE_DIRS ".")

add_compile_definitions(DEVICE_CENTRAL)
# add_compile_definitions(DEVICE_PERIPHERAL)