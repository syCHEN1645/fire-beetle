#include "comms_utils.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_http_client.h"

#include "func_config.h"

/// @brief Display the MAC address of the device's Wi-Fi interface.
/// @note This function assumes that the Wi-Fi interface has been initialized.
void display_mac_address() {
    uint8_t mac[6];

    ESP_ERROR_CHECK(
        esp_wifi_get_mac(WIFI_IF_STA, mac)
    );
    ESP_LOGI(
        "MAC",
        "MAC: %02X:%02X:%02X:%02X:%02X:%02X",
        mac[0], mac[1], mac[2],
        mac[3], mac[4], mac[5]
    );
}

void display_ip_info() {
    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip_info{};
    if (sta_netif == nullptr) {
        ESP_LOGE("WIFI", "STA netif not found");
    } else {
        esp_err_t err = esp_netif_get_ip_info(sta_netif, &ip_info);
        if (err == ESP_OK) {
            ESP_LOGI(
                "WIFI",
                "ESP IP: " IPSTR,
                IP2STR(&ip_info.ip)
            );
            ESP_LOGI(
                "WIFI",
                "Gateway: " IPSTR,
                IP2STR(&ip_info.gw)
            );
            ESP_LOGI(
                "WIFI",
                "Netmask: " IPSTR,
                IP2STR(&ip_info.netmask)
            );
        }
    }
}

void wifi_init() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#ifdef DEVICE_CENTRAL
    // creates the TCP/IP network interface associated with the ESP32's Wi-Fi station
    // needed to use wifi comms like http, not needed for esp-now
    esp_netif_create_default_wifi_sta();
#endif

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

#ifdef DEVICE_CENTRAL
    wifi_config_t wifi_config{};

    strcpy(
        reinterpret_cast<char *>(wifi_config.sta.ssid),
        DATA_COLLECT_WIFI_SSID
    );

    strcpy(
        reinterpret_cast<char *>(wifi_config.sta.password),
        DATA_COLLECT_WIFI_PASSWORD
    );

    ESP_ERROR_CHECK(
        esp_wifi_set_config(
            WIFI_IF_STA,
            &wifi_config
        )
    );

    ESP_ERROR_CHECK(
        esp_event_handler_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &wifi_event_handler,
            nullptr
        )
    );
#endif

    ESP_ERROR_CHECK(esp_wifi_start());

#ifdef DEVICE_CENTRAL
    ESP_ERROR_CHECK(esp_wifi_connect());
#endif
}

/// @brief Initialize ESP-NOW communication. Assume Wi-Fi is already initialized and connected.
/// @param channel The Wi-Fi channel to use for ESP-NOW communication.
void espnow_init(uint8_t channel = 0) {
    ESP_ERROR_CHECK(esp_now_init());

    ESP_ERROR_CHECK(
        esp_now_register_recv_cb(espnow_receive_callback)
    );

    ESP_ERROR_CHECK(
        esp_now_register_send_cb(espnow_send_callback)
    );

#ifdef DEVICE_CENTRAL
    // Central sends control messages to all peripherals
    // using the broadcast MAC address.
    esp_now_peer_info_t peer_info{};
    memcpy(
        peer_info.peer_addr,
        BOARDCAST_MAC,
        ESP_NOW_ETH_ALEN
    );
    // channel 0 is the current channel
    peer_info.channel = channel;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;

    ESP_ERROR_CHECK(
        esp_now_add_peer(&peer_info)
    );

    ESP_LOGI(
        "ESP-NOW",
        "Central ESP-NOW ready, channel %u",
        channel
    );

#endif

#ifdef DEVICE_PERIPHERAL
    // Peripheral sends IMU data to the central device.
    ESP_ERROR_CHECK(
        esp_wifi_set_channel(
            channel,
            WIFI_SECOND_CHAN_NONE
        )
    );

    esp_now_peer_info_t peer_info{};
    memcpy(
        peer_info.peer_addr,
        CENTRAL_MAC,
        ESP_NOW_ETH_ALEN
    );
    peer_info.channel = channel;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;

    ESP_ERROR_CHECK(
        esp_now_add_peer(&peer_info)
    );

    ESP_LOGI(
        "ESP-NOW",
        "Peripheral ESP-NOW ready, channel %u",
        channel
    );
#endif
}

#ifdef DEVICE_CENTRAL
void espnow_receive_callback(
    const esp_now_recv_info_t *recv_info,
    const uint8_t *data,
    int len) {
    imu_msg_t imu_msg;
    if (len == sizeof(imu_msg_t)) {
        memcpy(&imu_msg, data, sizeof(imu_msg_t));
    } else {
        return;
    }

    if (imu_msg.index >= IMU_DATA_LEN) {
        ESP_LOGW(
            "ESP-NOW",
            "Invalid IMU index: %d",
            imu_msg.index
        );
        return;
    }

    size_t imu_index = SIZE_MAX;
    // check the mac address of the sender
    if (memcmp(recv_info->src_addr, LEFT_LOWER_MAC, 6) == 0) {
        imu_index = 0;
    } else if (memcmp(recv_info->src_addr, LEFT_UPPER_MAC, 6) == 0) {
        imu_index = 1;
    } else if (memcmp(recv_info->src_addr, RIGHT_LOWER_MAC, 6) == 0) {
        imu_index = 2;
    } else if (memcmp(recv_info->src_addr, RIGHT_UPPER_MAC, 6) == 0) {
        imu_index = 3;
    } else {
        ESP_LOGW("ESP-NOW", "Received IMU message from unknown MAC address");
        return;
    }

    // imu_index in all_imu_data:
    // 0: LEFT_LOWER, 1: LEFT_UPPER, 2: RIGHT_LOWER, 3: RIGHT_UPPER
    // imu_index in trigger_reference:
    // 0: LEFT_LOWER_ACCEL, 1: LEFT_LOWER_GYRO, 2: LEFT_UPPER_ACCEL, 3: LEFT_UPPER_GYRO,
    // 4: RIGHT_LOWER_ACCEL, 5: RIGHT_LOWER_GYRO, 6: RIGHT_UPPER_ACCEL, 7: RIGHT_UPPER_GYRO
    switch (imu_msg.type) {
        case MessageType::DATA:
            // store the received imu message into the all_imu_data buffer
            all_imu_data[imu_index][imu_msg.index][0] = imu_msg.accel_x;
            all_imu_data[imu_index][imu_msg.index][1] = imu_msg.accel_y;
            all_imu_data[imu_index][imu_msg.index][2] = imu_msg.accel_z;
            all_imu_data[imu_index][imu_msg.index][3] = imu_msg.gyro_x;
            all_imu_data[imu_index][imu_msg.index][4] = imu_msg.gyro_y;
            all_imu_data[imu_index][imu_msg.index][5] = imu_msg.gyro_z;
            break;
        case MessageType::CALI_TRIGGER:
            // store into the trigger_reference array
            trigger_reference[imu_index * 2][imu_msg.index][0] = imu_msg.accel_x;
            trigger_reference[imu_index * 2][imu_msg.index][1] = imu_msg.accel_y;
            trigger_reference[imu_index * 2][imu_msg.index][2] = imu_msg.accel_z;
            trigger_reference[imu_index * 2 + 1][imu_msg.index][0] = imu_msg.gyro_x;
            trigger_reference[imu_index * 2 + 1][imu_msg.index][1] = imu_msg.gyro_y;
            trigger_reference[imu_index * 2 + 1][imu_msg.index][2] = imu_msg.gyro_z;
            break;
        default:
            ESP_LOGW("ESP-NOW", "Received unknown IMU message type");
            break;
    }
}

void espnow_send_callback(
    const esp_now_send_info_t *tx_info,
    esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI("ESP-NOW", "ESP-NOW send success");
    } else {
        ESP_LOGE("ESP-NOW", "ESP-NOW send failed");
    }
}

void send_ctrl_msg(const ctrl_msg_t &msg) {
    esp_err_t result = esp_now_send(
        BOARDCAST_MAC,
        reinterpret_cast<const uint8_t *>(&msg),
        sizeof(msg)
    );
    if (result == ESP_OK) {
        ESP_LOGI("ESP-NOW", "Control message sent successfully");
    } else {
        ESP_LOGE("ESP-NOW", "Failed to send control message");
    }
}

void send_start_msg() {
    ctrl_msg_t msg;
    msg.timestamp = xTaskGetTickCount();
    msg.index = 0;
    msg.type = MessageType::DATA;
    send_ctrl_msg(msg);
}

void send_pause_msg() {
    ctrl_msg_t msg;
    msg.timestamp = xTaskGetTickCount();
    msg.index = 0;
    msg.type = MessageType::PAUSE;
    send_ctrl_msg(msg);
}

void send_cali_zero_msg() {
    ctrl_msg_t msg;
    msg.timestamp = xTaskGetTickCount();
    msg.index = 0;
    msg.type = MessageType::CALI_ZERO;
    send_ctrl_msg(msg);
}

void send_trigger_ref_msg() {
    ctrl_msg_t msg;
    msg.timestamp = xTaskGetTickCount();
    msg.index = 0;
    msg.type = MessageType::CALI_TRIGGER;
    send_ctrl_msg(msg);
}

void wifi_event_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data) {
    if (event_base == IP_EVENT &&
        event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event =
            static_cast<ip_event_got_ip_t *>(event_data);

        ESP_LOGI(
            "WIFI",
            "ESP IP: " IPSTR,
            IP2STR(&event->ip_info.ip)
        );

        ESP_LOGI(
            "WIFI",
            "Gateway: " IPSTR,
            IP2STR(&event->ip_info.gw)
        );

        ESP_LOGI(
            "WIFI",
            "Netmask: " IPSTR,
            IP2STR(&event->ip_info.netmask)
        );
    }
}

void send_imu_data_to_laptop() {
    esp_http_client_config_t config = {};
    config.url = DATA_COLLECT_URL;

    esp_http_client_handle_t client =
        esp_http_client_init(&config);

    if (client == nullptr) {
        ESP_LOGE("HTTP", "Failed to initialize HTTP client");
        return;
    }

    esp_http_client_set_method(
        client,
        HTTP_METHOD_POST
    );

    esp_http_client_set_header(
        client,
        "Content-Type",
        "application/octet-stream"
    );

    esp_http_client_set_post_field(
        client,
        reinterpret_cast<const char *>(all_imu_data),
        sizeof(all_imu_data)
    );

    ESP_LOGI(
        "HTTP",
        "Sending %u bytes to laptop",
        sizeof(all_imu_data)
    );

    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK)
    {
        ESP_LOGI(
            "HTTP",
            "HTTP status = %d",
            esp_http_client_get_status_code(client)
        );
    }
    else
    {
        ESP_LOGE(
            "HTTP",
            "HTTP POST failed: %s",
            esp_err_to_name(err)
        );
    }

    esp_http_client_cleanup(client);
}

#endif

#ifdef DEVICE_PERIPHERAL
/// @brief Scan for the Wi-Fi channel of the project Wi-Fi network.
/// @return The Wi-Fi channel of the project network, or 0 if not found.
uint8_t scan_wifi_channel() {
    wifi_scan_config_t scan_config{};

    // Scan all Wi-Fi channels
    scan_config.channel = 0;
    scan_config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    ESP_ERROR_CHECK(
        esp_wifi_scan_start(&scan_config, true)
    );

    uint16_t ap_count = 0;
    ESP_ERROR_CHECK(
        esp_wifi_scan_get_ap_num(&ap_count)
    );

    if (ap_count == 0) {
        ESP_LOGE("WIFI", "No APs found");
        return 0;
    }

    wifi_ap_record_t *ap_records = new wifi_ap_record_t[ap_count];
    ESP_ERROR_CHECK(
        esp_wifi_scan_get_ap_records(
            &ap_count,
            ap_records
        )
    );

    uint8_t found_channel = 0;
    for (int i = 0; i < ap_count; i++) {
        ESP_LOGI(
            "WIFI",
            "Found SSID: %s, channel: %d",
            reinterpret_cast<char *>(ap_records[i].ssid),
            ap_records[i].primary
        );

        if (strcmp(reinterpret_cast<char *>(ap_records[i].ssid), DATA_COLLECT_WIFI_SSID) == 0) {
            found_channel = ap_records[i].primary;
            ESP_LOGI(
                "WIFI",
                "Found project Wi-Fi on channel %d",
                found_channel
            );
            break;
        }
    }
    delete[] ap_records;

    return found_channel;
}

static void espnow_receive_callback(
    const esp_now_recv_info_t *recv_info,
    const uint8_t *data,
    int len) {
    if (len != sizeof(ctrl_msg_t)) {
        return;
    }
    ctrl_msg_t msg;
    memcpy(&msg, data, sizeof(ctrl_msg_t));
    handle_control_message(msg);
}

static void espnow_send_callback(
    const esp_now_send_info_t *tx_info,
    esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI("ESP-NOW", "ESP-NOW send success");
    } else {
        ESP_LOGE("ESP-NOW", "ESP-NOW send failed");
    }
}
#endif