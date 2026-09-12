#include "sensor_utils.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensor_utils.h"

/// @brief Sets the configuration of the LSM6DSOX IMU sensor via I2C.
/// @param dev_handle 
/// @return 
esp_err_t set_lsm6dsox_imu_config(i2c_master_dev_handle_t dev_handle) {
    uint8_t accel_config[2] = {0x10, 0x20};
    uint8_t gyro_config[2] = {0x11, 0x20};

    esp_err_t ret = i2c_master_transmit(
        dev_handle,
        accel_config,
        sizeof(accel_config),
        1000
    );

    if (ret != ESP_OK) {
        ESP_LOGE("IMU", "Failed to configure accelerometer: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_master_transmit(
        dev_handle,
        gyro_config,
        sizeof(gyro_config),
        1000
    );

    if (ret != ESP_OK) {
        ESP_LOGE("IMU", "Failed to configure gyroscope: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI("IMU", "LSM6DSOX initialized");
    return ESP_OK;
}

/// @brief Reads from I2C sensor, writes into i2c_buffer.
/// @param dev_handle The I2C device handle for the sensor.
/// @param recv_buffer Output buffer to receive sensor data.
/// @param recv_len Number of bytes to read.
/// @return ESP_OK if the read was successful.
esp_err_t read_from_lsm6dsox_imu(i2c_master_dev_handle_t dev_handle, uint8_t *recv_buffer, size_t recv_len) {
    uint8_t reg_addr = LSM6DSOX_LOW_ADDR_GYRO;

    // debug
    // uint8_t reg = 0x0F;
    // uint8_t who_am_i = 0;

    // esp_err_t r = i2c_master_transmit_receive(
    //     dev_handle,
    //     &reg,
    //     1,
    //     &who_am_i,
    //     1,
    //     1000
    // );

    // ESP_LOGI("IMU", "WHO_AM_I = 0x%02X", who_am_i);
    
    // timeout -1 to wait forever
    esp_err_t ret = i2c_master_transmit_receive(
        dev_handle,
        &reg_addr,
        1,
        recv_buffer,
        recv_len,
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
    // cast to int16_t to preserve sign
    raw_buffer[0] = (float)(int16_t)(((uint16_t)bytes_buffer[1] << 8) | bytes_buffer[0]);
    raw_buffer[1] = (float)(int16_t)(((uint16_t)bytes_buffer[3] << 8) | bytes_buffer[2]);
    raw_buffer[2] = (float)(int16_t)(((uint16_t)bytes_buffer[5] << 8) | bytes_buffer[4]);
    raw_buffer[3] = (float)(int16_t)(((uint16_t)bytes_buffer[7] << 8) | bytes_buffer[6]);
    raw_buffer[4] = (float)(int16_t)(((uint16_t)bytes_buffer[9] << 8) | bytes_buffer[8]);
    raw_buffer[5] = (float)(int16_t)(((uint16_t)bytes_buffer[11] << 8) | bytes_buffer[10]);
    // reset bytes_buffer after parsing
    for (int i = 0; i < 12; i++) {
        bytes_buffer[i] = 0;
    }
}

/// @brief Performs zero calibration on 1 piece of IMU data. Gyro minus offset. Accel dot by R matrix. 
/// @param imu_data Array containing the IMU data: accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z.
void zero_calibrate_lsm6dsox_imu_data (float imu_data[6]) {
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

/// @brief For each data, smooths the data using a median filter with a window size of 3.
/// @param imu_data Pointer to the 2D array of IMU data to be smoothed.
void median_smooth_lsm6dsox_imu_data(float imu_data[6][IMU_DATA_LEN]) {
    float imu_copy[6][IMU_DATA_LEN];
    memcpy(imu_copy, imu_data, sizeof(imu_copy));
    for (int i = 2; i < IMU_DATA_LEN; i++) {
        for (int j = 0; j < 6; j++) {
            float temp[3] = {imu_copy[j][i-2], imu_copy[j][i-1], imu_copy[j][i]};
            imu_data[j][i] = get_median(temp, sizeof(temp)/sizeof(temp[0]));
        }
    }
}

void get_lsm6dsox_zero_calibration(i2c_master_dev_handle_t dev_handle) {
    // count of collected samples
    float calibration_data[6] = {0};
    size_t count = 0;

    // In an interval of 3 seconds, collect multiple IMU readings and compute the average
    while (count < IMU_DATA_LEN) {
        uint8_t bytes_buffer[12];
        float raw_buffer[6];
        read_from_lsm6dsox_imu(dev_handle, bytes_buffer, sizeof(bytes_buffer));
        parse_lsm6dsox_imu_data(bytes_buffer, raw_buffer);
        // update calibration data by adding in the new sample
        for (int i = 0; i < 6; i++) {
            calibration_data[i] += raw_buffer[i];
        }
        count++;
        vTaskDelay(pdMS_TO_TICKS(1000 / IMU_DATA_F));
    }
    ESP_LOGI("IMU", "Collected %d samples for zero calibration", count);

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
        accel_rotation[0][0] = 1.0f;
        accel_rotation[0][1] = 0.0f; 
        accel_rotation[0][2] = 0.0f;
        accel_rotation[1][0] = 0.0f; 
        accel_rotation[1][2] = 0.0f;
        accel_rotation[2][0] = 0.0f; 
        accel_rotation[2][1] = 0.0f;
        if (a[2] > 0) {
            // meaning it is already aligned with z-axis, R = I
            accel_rotation[1][1] = 1.0f;
            accel_rotation[2][2] = 1.0f;
        } else {
            // meaning it is aligned with -z-axis, R = diag(1, -1, -1)
            accel_rotation[1][1] = -1.0f;
            accel_rotation[2][2] = -1.0f;
        }
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