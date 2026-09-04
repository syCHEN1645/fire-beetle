// Choose to define device as central or peripheral
#define DEVICE_CENTRAL
// #define DEVICE_PERIPHERAL

#if defined(DEVICE_CENTRAL) && defined(DEVICE_PERIPHERAL)
#error "Defined 2 roles"
#endif

#if !defined(DEVICE_CENTRAL) && !defined(DEVICE_PERIPHERAL)
#error "No role defined"
#endif

#include <stdio.h>
#include <algorithm>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "hw_config.h"
#include "func_config.h"
#include "sensor_utils.h"

// IMU data buffers
uint8_t imu_data_buffer[12];
float imu_data_collection[6][IMU_DATA_LEN];
float all_imu_data[4][6][IMU_DATA_LEN];
uint16_t imu_data_collection_count = 0;
// IMU zero calibration data (double to avoid integer division)
float imu_zero_calibration[6];
float gyro_offset[3];
float accel_rotation[3][3];

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

// Central/Peripheral specific data
#ifdef DEVICE_CENTRAL
// Central-specific data
// MAC addr
static uint8_t boardcast_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const uint8_t LEFT_LOWER_MAC[]  = LEFT_LOWER_MAC_ADDR;
static const uint8_t LEFT_UPPER_MAC[]  = LEFT_UPPER_MAC_ADDR;
static const uint8_t RIGHT_LOWER_MAC[] = RIGHT_LOWER_MAC_ADDR;
static const uint8_t RIGHT_UPPER_MAC[] = RIGHT_UPPER_MAC_ADDR;

#endif

#ifdef DEVICE_PERIPHERAL
// Peripheral-specific data

#endif

// -----------IMU-----------

struct imu_msg_t {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    int index;
};

enum class MessageType : uint8_t {
    SYNC = 1,
    START = 2,
    END = 3
};

struct ctrl_msg_t {
    uint32_t timestamp;
    uint16_t index;
    MessageType type;
};

void data_collection_callback(i2c_master_dev_handle_t dev_handle) {
    float imu_data[6];
    read_from_lsm6dsox_imu(dev_handle, imu_data_buffer, sizeof(imu_data_buffer));
    parse_lsm6dsox_imu_data(imu_data_buffer, imu_data);

    // add parsed data to collection buffer
    for (int i = 0; i < 6; i++) {
        imu_data_collection[i][imu_data_collection_count] = imu_data[i];
    }
    imu_data_collection_count++;
    if (imu_data_collection_count >= IMU_DATA_LEN) {
        imu_data_collection_count = 0;
    }

    if (imu_data_collection_count == 0) {
        // now the collection buffer is full, start processing the collected data here.
        preprocess_lsm6dsox_imu_data();
    }
}

// ----------------ESP-NOW----------------
static void wifi_init() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

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

#endif

    ESP_ERROR_CHECK(esp_wifi_start());

#ifdef DEVICE_CENTRAL

    ESP_ERROR_CHECK(esp_wifi_connect());
    // delay to ensure connection is established
    // vTaskDelay(1000 / portTICK_PERIOD_MS);

#endif
}

#ifdef DEVICE_CENTRAL
// if this is the central device

static void espnow_receive_callback(
    const esp_now_recv_info_t *recv_info,
    const uint8_t *data,
    int len) {
    imu_msg_t imu_msg;
    if (len == sizeof(imu_msg_t)) {
        memcpy(&imu_msg, data, sizeof(imu_msg_t));
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

    // store the received imu message into the all_imu_data buffer
    // struct imu_msg_t {
    //     float accel_x;
    //     float accel_y;
    //     float accel_z;
    //     float gyro_x;
    //     float gyro_y;
    //     float gyro_z;
    //     int index;
    // };
    if (imu_index != SIZE_MAX) {
        all_imu_data[imu_index][0][imu_msg.index] = imu_msg.accel_x;
        all_imu_data[imu_index][1][imu_msg.index] = imu_msg.accel_y;
        all_imu_data[imu_index][2][imu_msg.index] = imu_msg.accel_z;
        all_imu_data[imu_index][3][imu_msg.index] = imu_msg.gyro_x;
        all_imu_data[imu_index][4][imu_msg.index] = imu_msg.gyro_y;
        all_imu_data[imu_index][5][imu_msg.index] = imu_msg.gyro_z;
        imu_data_collection_count = imu_msg.index;
    }
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

static void send_ctrl_msg(const ctrl_msg_t &msg) {
    esp_err_t result = esp_now_send(
        boardcast_mac,
        reinterpret_cast<const uint8_t *>(&msg),
        sizeof(msg)
    );
    if (result == ESP_OK) {
        ESP_LOGI("ESP-NOW", "Control message sent successfully");
    } else {
        ESP_LOGE("ESP-NOW", "Failed to send control message");
    }
}

static void send_start_msg() {
    ctrl_msg_t msg;
    msg.timestamp = xTaskGetTickCount();
    msg.index = 0;
    msg.type = MessageType::START;
    send_ctrl_msg(msg);
}

#endif

#ifdef DEVICE_PERIPHERAL

static void handle_control_message(const ctrl_msg_t& msg) {
    switch (msg.type)
    {
        case MessageType::START:
            collecting = true;
            ESP_LOGI("RECV", "START received");
            break;

        case MessageType::END:
            collecting = false;
            ESP_LOGI("RECV", "END received");
            break;

        case MessageType::SYNC:
            ESP_LOGI("RECV", "SYNC received");
            break;
    }
}

static void espnow_receive_callback(
    const esp_now_recv_info_t *recv_info,
    const uint8_t *data,
    int len) {
    if (len == sizeof(ctrl_msg_t)) {
        ctrl_msg_t msg;
        memcpy(&msg, data, sizeof(ctrl_msg_t));
        handle_control_message(msg);
    }
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

void read_imu_task(void *arg) {
    if (imu_data_collection_count >= IMU_DATA_LEN) {
        imu_data_collection_count = 0;
        // suspend the data collection process
        vTaskSuspend(NULL);
    }
    
    esp_err_t err = read_from_lsm6dsox_imu(dev_handle, imu_data_buffer, sizeof(imu_data_buffer));
    if (err != ESP_OK) {
        ESP_LOGE("IMU", "Failed to read from LSM6DSOX IMU: %s", esp_err_to_name(err));
    }
    float imu_data[6];
    parse_lsm6dsox_imu_data(imu_data_buffer, imu_data);

    // preprocess
    for (int i = 0; i < 6; i++) {
        all_imu_data[0][i][imu_data_collection_count] = imu_data[i];
    }
    imu_data_collection_count++;
    vTaskDelay(pdMS_TO_TICKS(1000 / IMU_DATA_F));
}

#ifdef DEVICE_PERIPHERAL
/// @brief Scan for the Wi-Fi channel of the project Wi-Fi network.
/// @return The Wi-Fi channel of the project network, or 0 if not found.
static uint8_t scan_wifi_channel()
{
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


/// @brief Initialize ESP-NOW communication. Assume Wi-Fi is already initialized and connected.
/// @param channel The Wi-Fi channel to use for ESP-NOW communication.
static void espnow_init(uint8_t channel = 0) {
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
        boardcast_mac,
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
    esp_now_peer_info_t peer_info{};
    memcpy(
        peer_info.peer_addr,
        CENTRAL_MAC_ADDR,
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

void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LSM6DSOX_LOW_ADDR_GYRO,
        .scl_speed_hz = I2C_CLK_FREQ_HZ,
        .scl_wait_us = 0
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));

    ESP_LOGI("I2C", "I2C master initialized successfully");
}

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

extern "C" void app_main() {
    // initialize I2C master

    i2c_master_init(&bus_handle, &dev_handle);

    esp_err_t ret = nvs_flash_init();
    // reinitialize NVS if necessary
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // start wifi and esp-now
    wifi_init();
    vTaskDelay(pdMS_TO_TICKS(3000));
    display_mac_address();
    
#ifdef DEVICE_CENTRAL
    // get actual wifi channel
    // make sure wifi is connected
    // TODO: change to while loop
    uint8_t primary_channel;
    wifi_second_chan_t secondary_channel;
    ESP_ERROR_CHECK(
        esp_wifi_get_channel(
            &primary_channel,
            &secondary_channel
        )
    );
    espnow_init(primary_channel);

    // set send/receive routines

    // TODO: if a button is pressed
    // central device
    send_start_msg();
    vTaskDelay(pdMS_TO_TICKS(20));

#else
    // peripheral device
    uint8_t count, channel = 0;
    while (channel == 0 && count <= 5) {
        channel = scan_wifi_channel();
        if (channel != 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        count++;
    }
    if (count > 5) {
        ESP_LOGE(
            "WIFI",
            "Project Wi-Fi not found"
        );
        return;
    }

    LOGI("WIFI", "Scanned Wi-Fi channel: %u", channel);
    espnow_init(channel);

#endif
    // all firebeetles should have the task to read its own imu
    static TaskHandle_t read_imu_task_handle = NULL;
    xTaskCreatePinnedToCore(
        read_imu_task,
        "Read IMU Task",
        4096,
        NULL,
        5,
        &read_imu_task_handle,
        tskNO_AFFINITY
    );
}