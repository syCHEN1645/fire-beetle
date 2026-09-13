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

#ifdef DEVICE_CENTRAL
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
    msg.type = MessageType::START;
    send_ctrl_msg(msg);
}

void send_cali_msg() {
    ctrl_msg_t msg;
    msg.timestamp = xTaskGetTickCount();
    msg.index = 0;
    msg.type = MessageType::CALI;
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
#endif