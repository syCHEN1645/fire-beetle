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
float imu_data_collection[6][312];
float all_imu_data[4][6][312];
uint16_t imu_data_collection_count = 0;
// IMU zero calibration data (double to avoid integer division)
float imu_zero_calibration[6];
float gyro_offset[3];
float accel_rotation[3][3];

// Central/Peripheral specific data
#ifdef DEVICE_CENTRAL
// Central-specific data
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
    if (imu_data_collection_count >= 312) {
        imu_data_collection_count = 0;
    }

    if (imu_data_collection_count == 0) {
        // now the collection buffer is full, start processing the collected data here.
        preprocess_lsm6dsox_imu_data();
    }
}

// ----------------ESP-NOW----------------

static void wifi_init()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(
        esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE)
    );
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

static void espnow_init()
{
    ESP_ERROR_CHECK(esp_now_init());

    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_receive_callback));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_callback));

#ifdef DEVICE_CENTRAL
    esp_now_peer_info_t peer_info{};
    memcpy(
        peer_info.peer_addr,
        boardcast_mac,
        ESP_NOW_ETH_ALEN
    );
    peer_info.channel = ESPNOW_CHANNEL;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    ESP_LOGI("ESP-NOW", "ESP-NOW sender ready");
#endif

#ifdef DEVICE_PERIPHERAL
    // set up peer info for sending to the central device
    esp_now_peer_info_t peer_info{};
    memcpy(
        peer_info.peer_addr,
        CENTRAL_MAC_ADDR,
        ESP_NOW_ETH_ALEN
    );
    peer_info.channel = ESPNOW_CHANNEL;
    peer_info.ifidx = WIFI_IF_STA;
    // TODO: to check if encryption needed
    peer_info.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    ESP_LOGI("ESP-NOW", "ESP-NOW receiver ready");
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
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
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
    display_mac_address();
    espnow_init();


    // set send/receive routines
#ifdef DEVICE_CENTRAL
    // if a button is pressed
    // central device
    send_start_msg();
    vTaskDelay(pdMS_TO_TICKS(20));

#else
    // peripheral device
#endif
}