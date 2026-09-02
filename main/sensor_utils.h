#ifndef SENSOR_UTILS_H
#define SENSOR_UTILS_H

#include <stddef.h>
#include <stdint.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t imu_data_buffer[12];
extern float imu_data_collection[6][312];
extern uint16_t imu_data_collection_count;
extern float imu_zero_calibration[6];
extern float gyro_offset[3];
extern float accel_rotation[3][3];

esp_err_t read_from_lsm6dsox_imu(i2c_master_dev_handle_t dev_handle, uint8_t *recv_buffer, size_t recv_len);
void parse_lsm6dsox_imu_data(uint8_t bytes_buffer[12], float raw_buffer[6]);
void preprocess_lsm6dsox_imu_data(void);
void zero_calibrate_lsm6dsox_collection(float imu_data[6]);
float get_median(float *data, size_t size);
void median_smooth_lsm6dsox_collection(void);
void get_lsm6dsox_zero_calibration(i2c_master_dev_handle_t dev_handle);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_UTILS_H
