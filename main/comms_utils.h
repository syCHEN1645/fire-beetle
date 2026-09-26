#pragma once

#include <stdio.h>
#include <stdint.h>
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"

#include "data_struct_utils.h"
#include "func_config.h"

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

inline uint16_t imu_data_collection_count = 0;

#ifdef DEVICE_CENTRAL
extern float all_imu_data[4][IMU_DATA_LEN][6];
extern bool all_flex_data[2][IMU_DATA_LEN][2];

inline const uint8_t BOARDCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
inline const uint8_t LEFT_LOWER_MAC[6] = LEFT_LOWER_MAC_ADDR;
inline const uint8_t LEFT_UPPER_MAC[6] = LEFT_UPPER_MAC_ADDR;
inline const uint8_t RIGHT_LOWER_MAC[6] = RIGHT_LOWER_MAC_ADDR;
inline const uint8_t RIGHT_UPPER_MAC[6] = RIGHT_UPPER_MAC_ADDR;

void send_ctrl_msg(const ctrl_msg_t &msg);
void send_start_msg();
void send_pause_msg();
void send_cali_zero_msg();
void send_trigger_ref_msg();
void send_state_event_msg(const state_event_msg_t &msg);
void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void send_imu_data_to_laptop(float all_imu_data[4][IMU_DATA_LEN][6]);
#endif

#ifdef DEVICE_PERIPHERAL
inline const uint8_t CENTRAL_MAC[6] = CENTRAL_MAC_ADDR;
uint8_t scan_wifi_channel();
extern void handle_control_message(const ctrl_msg_t &msg);
extern void handle_state_event_message(const state_event_msg_t &msg);
#endif