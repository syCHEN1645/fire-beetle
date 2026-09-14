#ifndef COMMS_UTILS_H
#define COMMS_UTILS_H

#include <stdio.h>
#include <stdint.h>
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"

#include "func_config.h"

enum class MessageType : uint8_t {
    SYNC = 1,
    START = 2,
    CALI = 3
};

struct ctrl_msg_t {
    uint32_t timestamp;
    uint16_t index;
    MessageType type;
};

struct imu_msg_t {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    int index;
};

void wifi_init();
void display_mac_address();
void display_ip_info();
void espnow_receive_callback(
    const esp_now_recv_info_t *recv_info,
    const uint8_t *data,
    int len
);
void espnow_send_callback(
    const esp_now_send_info_t *tx_info,
    esp_now_send_status_t status
);
void espnow_init(uint8_t channel);

#ifdef DEVICE_CENTRAL
inline const uint8_t BOARDCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
inline const uint8_t LEFT_LOWER_MAC[6] = LEFT_LOWER_MAC_ADDR;
inline const uint8_t LEFT_UPPER_MAC[6] = LEFT_UPPER_MAC_ADDR;
inline const uint8_t RIGHT_LOWER_MAC[6] = RIGHT_LOWER_MAC_ADDR;
inline const uint8_t RIGHT_UPPER_MAC[6] = RIGHT_UPPER_MAC_ADDR;

inline float all_imu_data[4][6][IMU_DATA_LEN] = {};
inline uint16_t imu_data_collection_count = 0;

void send_ctrl_msg(const ctrl_msg_t &msg);
void send_start_msg();
void send_cali_msg();
void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void send_imu_data_to_laptop();
#endif

#ifdef DEVICE_PERIPHERAL
inline const uint8_t CENTRAL_MAC[6] = CENTRAL_MAC_ADDR;
uint8_t scan_wifi_channel();
#endif

#endif // COMMS_UTILS_H