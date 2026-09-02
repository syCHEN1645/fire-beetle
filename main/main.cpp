#include <stdio.h>
#include <math.h>
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

// IMU data buffers
uint8_t imu_data_buffer[12];
float imu_data_collection[6][312];
uint8_t imu_data_collection_count = 0;
// IMU zero calibration data (double to avoid integer division)
float imu_zero_calibration[6];
float gyro_offset[3];
float accel_rotation[3][3];

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

/// @brief Reads from I2C sensor, writes into i2c_buffer.
/// @param dev_handle The I2C device handle for the sensor.
/// @return ESP_OK if the read was successful.
static esp_err_t read_from_lsm6dsox_imu(i2c_master_dev_handle_t dev_handle, uint8_t *recv_buffer) {
    uint8_t reg_addr = LSM6DSOX_LOW_ADDR_GYRO;
    
    // timeout -1 to wait forever
    esp_err_t ret = i2c_master_transmit_receive(
        dev_handle,
        &reg_addr,
        1,
        recv_buffer,
        sizeof(recv_buffer),
        -1
    );

    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Read from LSM6DSOX sensor error: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI("I2C", "Read from LSM6DSOX sensor successful");
    }
    return ret;
}

/// @brief Parses the raw IMU data from the sensor.
/// @param imu_data Array to store the parsed IMU data: accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z.
/// @return None
void parse_lsm6dsox_imu_data(uint8_t bytes_buffer[12], float raw_buffer[6]) {
    raw_buffer[0] = (float)(((uint16_t)bytes_buffer[1] << 8) | bytes_buffer[0]);
    raw_buffer[1] = (float)(((uint16_t)bytes_buffer[3] << 8) | bytes_buffer[2]);
    raw_buffer[2] = (float)(((uint16_t)bytes_buffer[5] << 8) | bytes_buffer[4]);
    raw_buffer[3] = (float)(((uint16_t)bytes_buffer[7] << 8) | bytes_buffer[6]);
    raw_buffer[4] = (float)(((uint16_t)bytes_buffer[9] << 8) | bytes_buffer[8]);
    raw_buffer[5] = (float)(((uint16_t)bytes_buffer[11] << 8) | bytes_buffer[10]);
    // reset bytes_buffer after parsing
    for (int i = 0; i < sizeof(bytes_buffer); i++) {
        bytes_buffer[i] = 0;
    }
}

/// @brief Preprocesses the collected IMU data once the collection buffer is full.
/// @param None
void preprocess_lsm6dsox_imu_data(void) {
}

void data_collection_callback(i2c_master_dev_handle_t dev_handle) {
    float imu_data[6];
    read_from_lsm6dsox_imu(dev_handle, imu_data_buffer);
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

/// @brief Performs zero calibration on 1 piece of IMU data. Gyro minus offset. Accel dot by R matrix. 
/// @param None
void zero_calibrate_lsm6dsox_collection(float imu_data[6]) {
    imu_data[3] -= gyro_offset[0];
    imu_data[4] -= gyro_offset[1];
    imu_data[5] -= gyro_offset[2];

    float accel_temp[3] = {imu_data[0], imu_data[1], imu_data[2]};
    for (int i = 0; i < 3; i++) {
        imu_data[i] = accel_rotation[i][0] * accel_temp[0] + accel_rotation[i][1] * accel_temp[1] + accel_rotation[i][2] * accel_temp[2];
    }
    return;
}

/// @brief Computes the median value of an array of float data.
/// @param data Pointer to the array of float data.
/// @param size Number of elements in the data array.
/// @return The median value of the data array.
float get_median(float *data, size_t size) {
    float* temp = (float*)malloc(size * sizeof(float));
    memcpy(temp, data, size * sizeof(float));
    std::sort(temp, temp + size);
    float median;
    if (size % 2 == 0) {
        median = (temp[size / 2 - 1] + temp[size / 2]) / 2.0;
    } else {
        median = temp[size / 2];
    }
    free(temp);
    return median;
}

void median_smooth_lsm6dsox_collection(void) {
    for (int i = 2; i < 312; i++) {
        for (int j = 0; j < 6; j++) {
            float temp[3] = {imu_data_collection[j][i-2], imu_data_collection[j][i-1], imu_data_collection[j][i]};
            imu_data_collection[j][i] = get_median(temp, sizeof(temp)/sizeof(temp[0]));
        }
    }
}

void get_lsm6dsox_zero_calibration(i2c_master_dev_handle_t dev_handle) {
    // count of collected samples
    float calibration_data[6] = {0};
    size_t count = 0;

    // In an interval of 3 seconds, collect multiple IMU readings and compute the average
    uint64_t start_time = esp_timer_get_time();
    while (esp_timer_get_time() < start_time + 3000000) {
        uint8_t bytes_buffer[12];
        float raw_buffer[6];
        read_from_lsm6dsox_imu(dev_handle, bytes_buffer);
        parse_lsm6dsox_imu_data(bytes_buffer, raw_buffer);
        // update calibration data by adding in the new sample
        for (int i = 0; i < 6; i++) {
            calibration_data[i] += raw_buffer[i];
        }
        count++;
    }

    // set zero calibration values to be the mean of collected data
    if (count > 0) {
        for (int i = 0; i < 6; i++) {
            imu_zero_calibration[i] = calibration_data[i] /= (float)count;
        }
    }
    ESP_LOGI("IMU", "Zero calibration completed: x=%f, y=%f, z=%f, gx=%f, gy=%f, gz=%f",
             imu_zero_calibration[0], imu_zero_calibration[1], imu_zero_calibration[2],
             imu_zero_calibration[3], imu_zero_calibration[4], imu_zero_calibration[5]);
    
    // gyroscope offset
    for (int i = 0; i < 3; i++) {
        gyro_offset[i] = imu_zero_calibration[3 + i];
    }

    // accelerometer offset
    // find R such that R.dot(a) = g, R is accel_rotation[3][3]
    // g = [0, 0, 1].T
    float a[3], v[3];
    
    float accel_magnitude = sqrt(imu_zero_calibration[0] * imu_zero_calibration[0] +
                           imu_zero_calibration[1] * imu_zero_calibration[1] +
                           imu_zero_calibration[2] * imu_zero_calibration[2]);
    for (int i = 0; i < 3; i++) {
        a[i] = imu_zero_calibration[i] / accel_magnitude;
    }

    // v = a.cross(g), then normalise v
    v[0] = a[1];
    v[1] = -a[0];
    v[2] = 0;
    float v_magnitude = sqrt(v[0] * v[0] + v[1] * v[1]);
    if (v_magnitude < 1e-6) {
        // meaning it is already aligned with z-axis, R = I
        accel_rotation[0][0] = 1.0f;
        accel_rotation[0][1] = 0.0f; 
        accel_rotation[0][2] = 0.0f;
        accel_rotation[1][0] = 0.0f; 
        accel_rotation[1][1] = 1.0f;
        accel_rotation[1][2] = 0.0f;
        accel_rotation[2][0] = 0.0f; 
        accel_rotation[2][1] = 0.0f; 
        accel_rotation[2][2] = 1.0f;
        return;
    }

    for (int i = 0; i < 3; i++) {
        v[i] /= v_magnitude;
    }

    // cos = a.dot(g), sin = |a.cross(g)| because unit vectors
    float c = a[2];
    float s = v_magnitude;
    // use formula to get R
    accel_rotation[0][0] = c + v[0] * v[0] * (1 - c);
    accel_rotation[0][1] = v[0] * v[1] * (1 - c) - v[2] * s;
    accel_rotation[0][2] = v[0] * v[2] * (1 - c) + v[1] * s;

    accel_rotation[1][0] = v[1] * v[0] * (1 - c) + v[2] * s;
    accel_rotation[1][1] = c + v[1] * v[1] * (1 - c);
    accel_rotation[1][2] = v[1] * v[2] * (1 - c) - v[0] * s;

    accel_rotation[2][0] = v[2] * v[0] * (1 - c) - v[1] * s;
    accel_rotation[2][1] = v[2] * v[1] * (1 - c) + v[0] * s;
    accel_rotation[2][2] = c + v[2] * v[2] * (1 - c);

    return;
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

static void espnow_receive_callback(
    const esp_now_recv_info_t *recv_info,
    const uint8_t *data,
    int len)
{
    if (data == nullptr || len <= 0) {
        return;
    }

    ESP_LOGI(
        "ESP-NOW",
        "Received %d bytes: %.*s",
        len,
        len,
        reinterpret_cast<const char *>(data)
    );
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

static void espnow_init()
{
    ESP_ERROR_CHECK(esp_now_init());

    ESP_ERROR_CHECK(
        esp_now_register_recv_cb(espnow_receive_callback)
    );

    ESP_ERROR_CHECK(
        esp_now_register_send_cb(espnow_send_callback)
    );

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
}


void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LSM6DSOX_LOW_ADDR_GYRO,
        .scl_speed_hz = I2C_CLK_FREQ_HZ,
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
}