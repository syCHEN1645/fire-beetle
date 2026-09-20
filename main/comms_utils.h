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
    DATA = 2,
    CALI_ZERO = 3,
    CALI_TRIGGER = 4,
};

// enum trigger_ref_index_t {
//     LEFT_L_ACCEL = 0,
//     LEFT_L_GYRO = 1,
//     LEFT_U_ACCEL = 2,
//     LEFT_U_GYRO = 3,
//     RIGHT_L_ACCEL = 4,
//     RIGHT_L_GYRO = 5,
//     RIGHT_U_ACCEL = 6,
//     RIGHT_U_GYRO = 7
// };

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
    // tell central device what this message is responding to
    MessageType type;
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
inline uint16_t imu_data_collection_count = 0;

inline float all_imu_data[4][IMU_DATA_LEN][6] = {};
inline float trigger_reference[8][IMU_TRIGGER_LEN][3] = {};

void send_ctrl_msg(const ctrl_msg_t &msg);
void send_start_msg();
void send_cali_zero_msg();
void send_trigger_ref_msg();
void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void send_imu_data_to_laptop();
#endif

#ifdef DEVICE_PERIPHERAL
inline const uint8_t CENTRAL_MAC[6] = CENTRAL_MAC_ADDR;
uint8_t scan_wifi_channel();
extern void handle_control_message(const ctrl_msg_t &msg);
#endif

#endif // COMMS_UTILS_H