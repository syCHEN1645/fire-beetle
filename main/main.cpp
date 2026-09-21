// Definition of device is in CMakeLists.txt
#if defined(DEVICE_CENTRAL) && defined(DEVICE_PERIPHERAL)
#error "Defined 2 roles"
#endif

#if !defined(DEVICE_CENTRAL) && !defined(DEVICE_PERIPHERAL)
#error "No role defined"
#endif

#include <stdio.h>
#include <algorithm>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
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
#include "esp_http_client.h"

#include "hw_config.h"
#include "func_config.h"
#include "imu_utils.h"
#include "comms_utils.h"
#ifdef DEVICE_CENTRAL
#include "dtw.h"
#include "state_utils.h"
#endif

// IMU data buffers
uint8_t imu_data_buffer[12];
// IMU zero calibration data (double to avoid integer division)
float imu_zero_calibration[6];
float gyro_offset[3];
float accel_rotation[3][3];
float all_imu_data[4][IMU_DATA_LEN][6] = {};
size_t oldest_index = 0;
float trigger_reference[8][IMU_TRIGGER_LEN][3] = {};

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

static TaskHandle_t fsm_task_handle = NULL;
static TaskHandle_t read_imu_task_handle = NULL;
static TaskHandle_t calibration_task_handle = NULL;
static TaskHandle_t trigger_reference_task_handle = NULL;

#ifdef DEVICE_CENTRAL
// session progress tracking
static int exercise_count = 0;
// TODO: Adjust trigger thresholds based on empirical data
static float trigger_thresholds[8] = {1, 2, 3, 4, 5, 6, 7, 8};

static void reset_session_progress() {
    exercise_count = 0;
    // reset imu data buffer
    oldest_index = 0;
    for (int i = 0; i < 4 * IMU_DATA_LEN * 6; i++) {
        *((float*)all_imu_data[0] + i) = 0;
    }
}

static void IRAM_ATTR button_a_isr(void* arg) {
    push_event(SE_DOWN);
}

static void IRAM_ATTR button_b_isr(void* arg) {
    push_event(SE_CLICK);
}

static void button_init() {
    gpio_config_t io_conf{};

    io_conf.pin_bit_mask = (1ULL << START_BUTTON_PIN) | (1ULL << CALI_BUTTON_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(
        gpio_install_isr_service(0)
    );

    ESP_ERROR_CHECK(
        gpio_isr_handler_add(
            START_BUTTON_PIN,
            button_a_isr,
            nullptr
        )
    );

    ESP_ERROR_CHECK(
        gpio_isr_handler_add(
            CALI_BUTTON_PIN,
            button_b_isr,
            nullptr
        )
    );
}

#endif

// ----------------WiFi----------------
#ifdef DEVICE_PERIPHERAL

static void handle_control_message(const ctrl_msg_t& msg) {
    switch (msg.type)
    {
        case MessageType::DATA:
            ESP_LOGI("RECV", "DATA received");
            xTaskNotifyGive(read_imu_task_handle);
            break;

        case MessageType::CALI_ZERO:
            ESP_LOGI("RECV", "CALI_ZERO received");
            xTaskNotifyGive(calibration_task_handle);
            break;

        case MessageType::CALI_TRIGGER:
            ESP_LOGI("RECV", "CALI_TRIGGER received");
            xTaskNotifyGive(trigger_reference_task_handle);
            break;

        case MessageType::SYNC:
            ESP_LOGI("RECV", "SYNC received");
            break;
    }
}

#endif

void send_to_ar() {
    // dummy
}

void recv_from_ar() {
    // dummy
    // expect instruction messages that trigger state change
}

#ifdef DEVICE_CENTRAL
// ----------------DTW----------------
// Dynamic Time Warping instances for gyro and accel for each IMU sensor
static DTW dtws[8];

void init_dtw() {
    for (size_t i = 0; i < 8; ++i) {
        dtws[i].set_reference(
            &trigger_reference[i][0][0],
            IMU_TRIGGER_LEN,
            3
        );
    }
}
#endif

/// @brief Task to collect IMU data for trigger reference before session starts. Reuse IMU data buffer. 
void trigger_reference_task(void *arg) {
    while (true) {
        // put it to suspension upon creation
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // TODO: use semaphore to lock, this is accessed by multiple tasks
        imu_data_collection_count = 0;
        // same as read_imu_task but store the samples locally on central device
        while (imu_data_collection_count < IMU_TRIGGER_LEN) {
            esp_err_t err = read_from_lsm6dsox_imu(dev_handle, imu_data_buffer, sizeof(imu_data_buffer));
            if (err != ESP_OK) {
                ESP_LOGE("IMU", "Failed to read from LSM6DSOX IMU: %s", esp_err_to_name(err));
            }
            float imu_data[6];
            parse_lsm6dsox_imu_data(imu_data_buffer, imu_data);
            zero_calibrate_lsm6dsox_imu_data(imu_data);
            
            ESP_LOGI("IMU", "Collected IMU data:");
#ifdef DEVICE_CENTRAL
            for (int i = 0; i < 4; i++) {
                trigger_reference[0][imu_data_collection_count][i] = imu_data[i];
                trigger_reference[1][imu_data_collection_count][i] = imu_data[i + 3];
                ESP_LOGI("IMU", "%f", imu_data[i]);
            }
#endif
#ifdef DEVICE_PERIPHERAL
            for (int i = 0; i < 6; i++) {
                // imu_data_collection[imu_data_collection_count][i] = imu_data[i];
                ESP_LOGI("IMU", "%f", imu_data[i]);
            }
            // send the collected IMU data to the central device via ESP-NOW
            imu_msg_t imu_msg;
            imu_msg.accel_x = imu_data[0];
            imu_msg.accel_y = imu_data[1];
            imu_msg.accel_z = imu_data[2];
            imu_msg.gyro_x = imu_data[3];
            imu_msg.gyro_y = imu_data[4];
            imu_msg.gyro_z = imu_data[5];
            imu_msg.index = imu_data_collection_count;
            imu_msg.type = MessageType::CALI_TRIGGER;
            esp_err_t result = esp_now_send(
                CENTRAL_MAC,
                reinterpret_cast<const uint8_t *>(&imu_msg),
                sizeof(imu_msg)
            );
            if (result != ESP_OK) {
                ESP_LOGE("ESP-NOW", "Failed to send IMU data with error %d", result);
            }
#endif
            imu_data_collection_count++;
            vTaskDelay(pdMS_TO_TICKS(1000 / IMU_DATA_F));
        }
#ifdef DEVICE_CENTRAL
        // wait for a short period for all data to arrive
        vTaskDelay(pdMS_TO_TICKS(100));
        median_smooth_lsm6dsox_imu_data(
            (float*)trigger_reference, 
            8, 
            IMU_TRIGGER_LEN,
            6
        );
        // assign reference samples to dtw objects
        for (size_t i = 0; i < 8; i++) {
            dtws[i].set_reference((float*)trigger_reference[i], IMU_TRIGGER_LEN, 3);
        }
#endif
    }
}

bool if_dtw_trigger() {
    // 0: IMU0 accel
    // 1: IMU0 gyro
    // 2: IMU1 accel
    // 3: IMU1 gyro ...
    float trigger_window[8][IMU_TRIGGER_LEN][3];

    // the newest sample is at: oldest_index - 1
    for (size_t imu = 0; imu < 4; ++imu) {
        // add IMU_DATA_LEN to ensure positive modulo result
        size_t start_index = (oldest_index + IMU_DATA_LEN - IMU_TRIGGER_LEN) % IMU_DATA_LEN;

        for (size_t sample = 0; sample < IMU_TRIGGER_LEN; ++sample) {
            size_t buffer_index = (start_index + sample) % IMU_DATA_LEN;

            // Acceleration
            trigger_window[imu * 2][sample][0] = all_imu_data[imu][buffer_index][0];
            trigger_window[imu * 2][sample][1] = all_imu_data[imu][buffer_index][1];
            trigger_window[imu * 2][sample][2] = all_imu_data[imu][buffer_index][2];

            // Gyroscope
            trigger_window[imu * 2 + 1][sample][0] = all_imu_data[imu][buffer_index][3];
            trigger_window[imu * 2 + 1][sample][1] = all_imu_data[imu][buffer_index][4];
            trigger_window[imu * 2 + 1][sample][2] = all_imu_data[imu][buffer_index][5];
        }
    }

    // run all 8 dtw comparisons
    // ensure at least 6 matches
    int non_matches = 0;
    for (size_t i = 0; i < 8; ++i) {
        float dist = dtws[i].compute(&trigger_window[i][0][0], IMU_TRIGGER_LEN, 3);

        ESP_LOGI(
            "DTW",
            "DTW[%d] distance = %f, threshold = %f",
            (int)i,
            dist,
            trigger_thresholds[i]
        );

        if (dist >= trigger_thresholds[i]) {
            non_matches++;
        }
    }

    return non_matches <= 2;
}


void sense_trigger_task(void *arg) {
    while (true) {
        // Wait until the session begins.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int64_t next_compare_time = esp_timer_get_time() + TRIGGER_INTERVAL_MS * 1000;
        int64_t trigger_lock_until = 0;

        while (current_state == SS_SESSION || current_state == SS_SESSION_PAUSE) {
            int64_t now = esp_timer_get_time();
            // Perform a DTW comparison every 300 ms,
            if (now >= next_compare_time && now >= trigger_lock_until) {
                if (if_dtw_trigger()) {
                    ESP_LOGI(
                        "TRIGGER",
                        "Trigger movement detected!"
                    );
                    // debounce for 1 second.
                    trigger_lock_until = now + TRIGGER_LOCKOUT_MS * 1000;
                    // send trigger event
                    push_event(system_event_t::SE_TRIGGER);
                }
                // next comparison after 300 ms
                next_compare_time = now + TRIGGER_INTERVAL_MS * 1000;
            }

            // Don't busy-loop.
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void inference_data_task(void *arg) {
    while (true) {
        // Wait until the session begins.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (current_state == SS_SESSION || current_state == SS_SESSION_PAUSE) {
            // Take a snapshot of the latest 3 seconds of IMU data.
            float inference_window[4][IMU_DATA_LEN][6];
            size_t start_index = (oldest_index + IMU_DATA_LEN - IMU_TRIGGER_LEN) % IMU_DATA_LEN;

            for (size_t imu = 0; imu < 4; ++imu) {
                for (size_t sample = 0; sample < IMU_TRIGGER_LEN; ++sample) {
                    size_t buffer_index = (start_index + sample) % IMU_DATA_LEN;
                    for (size_t i = 0; i < 6; ++i) {
                        inference_window[imu][sample][i] = all_imu_data[imu][buffer_index][i];
                    }
                }
            }

            // TODO: send to ultra96
            send_imu_data_to_laptop(inference_window);
            // Wait 2 seconds before requesting another window.
            for (int i = 0; i < 40; ++i) {
                // Check every 50 ms whether the session has ended.
                if (!(current_state == SS_SESSION ||
                      current_state == SS_SESSION_PAUSE)) {
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
    }
}

/// @brief Task function for reading IMU data continuously during a session.
/// @param arg 
void read_imu_task(void *arg) {
    while (true) {
        // put it to suspension upon creation
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // only run during a session (including pause)
        while (current_state == SS_SESSION || current_state == SS_SESSION_PAUSE) {
            esp_err_t err = read_from_lsm6dsox_imu(dev_handle, imu_data_buffer, sizeof(imu_data_buffer));
            if (err != ESP_OK) {
                ESP_LOGE("IMU", "Failed to read from LSM6DSOX IMU: %s", esp_err_to_name(err));
            }
            float imu_data[6];
            parse_lsm6dsox_imu_data(imu_data_buffer, imu_data);
            zero_calibrate_lsm6dsox_imu_data(imu_data);
            
            ESP_LOGI("IMU", "Collected IMU data:");

#ifdef DEVICE_CENTRAL
            for (int i = 0; i < 6; i++) {
                all_imu_data[0][oldest_index][i] = imu_data[i];
                ESP_LOGI("IMU", "%f", imu_data[i]);
            }
#endif

#ifdef DEVICE_PERIPHERAL
            for (int i = 0; i < 6; i++) {
                // imu_data_collection[oldest_index][i] = imu_data[i];
                ESP_LOGI("IMU", "%f", imu_data[i]);
            }
            // send the collected IMU data to the central device via ESP-NOW
            imu_msg_t imu_msg;
            imu_msg.accel_x = imu_data[0];
            imu_msg.accel_y = imu_data[1];
            imu_msg.accel_z = imu_data[2];
            imu_msg.gyro_x = imu_data[3];
            imu_msg.gyro_y = imu_data[4];
            imu_msg.gyro_z = imu_data[5];
            imu_msg.index = oldest_index;
            imu_msg.type = MessageType::DATA;
            esp_err_t result = esp_now_send(
                CENTRAL_MAC,
                reinterpret_cast<const uint8_t *>(&imu_msg),
                sizeof(imu_msg)
            );
            if (result != ESP_OK) {
                ESP_LOGE("ESP-NOW", "Failed to send IMU data with error %d", result);
            }
#endif
            oldest_index = (oldest_index + 1) % IMU_DATA_LEN;
            // clear in advance the new oldest index
            for (int j = 0; j < 4; j++) {
                for (int i = 0; i < 6; i++) {
                    all_imu_data[j][oldest_index][i] = 0.0f;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1000 / IMU_DATA_F));
        }
#ifdef DEVICE_CENTRAL
        // send_imu_data_to_laptop();
#endif
    }
}

void calibration_task(void *arg) {
    while (true) {
        // put it to suspension upon creation, suspend after every calibration cycle
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI("CALIBRATION", "Waiting 2 seconds before start");
        vTaskDelay(pdMS_TO_TICKS(2000));
        get_lsm6dsox_zero_calibration(dev_handle);
        ESP_LOGI("CALIBRATION", "Zero calibration completed");
        ESP_LOGI("CALIBRATION", "Gyro offsets: %f, %f, %f", gyro_offset[0], gyro_offset[1], gyro_offset[2]);
        ESP_LOGI("CALIBRATION", "Accel rotation: %f, %f, %f", accel_rotation[0][0], accel_rotation[0][1], accel_rotation[0][2]);
        ESP_LOGI("CALIBRATION", "Accel rotation: %f, %f, %f", accel_rotation[1][0], accel_rotation[1][1], accel_rotation[1][2]);
        ESP_LOGI("CALIBRATION", "Accel rotation: %f, %f, %f", accel_rotation[2][0], accel_rotation[2][1], accel_rotation[2][2]);
    }
}

void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
            .allow_pd = false,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LSM6DSOX_I2C_ADDR,
        .scl_speed_hz = I2C_CLK_FREQ_HZ,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = false,
        },
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));

    // debug
    vTaskDelay(100 / portTICK_PERIOD_MS);
    esp_err_t ret = i2c_master_probe(
        *bus_handle,
        LSM6DSOX_I2C_ADDR,
        1000
    );

    ESP_LOGI("I2C", "LSM6DSOX probe: %s", esp_err_to_name(ret));

    ESP_LOGI("I2C", "I2C master initialized successfully");
}

// ------------------Events control ------------------

void start_calibration() {
    send_cali_zero_msg();
    xTaskNotifyGive(calibration_task_handle);
}

void start_trigger_reference() {
    send_trigger_ref_msg();
    xTaskNotifyGive(trigger_reference_task_handle);
}

void start_session() {
    send_start_msg();
    xTaskNotifyGive(read_imu_task_handle);
}

void pause_session() {
    // pause inference task, but keep trigger reference task running
}

void end_session() {
    send_pause_msg();
    pause_session();
    xTaskNotifyGive(read_imu_task_handle);
    reset_session_progress();
}

/// @brief Finite State Machine task that handles system states transitions based on events.
/// @param arg 
void fsm_task(void *arg) {
    // event would be a system_event_t, which is a signal for the FSM to transition states.
    system_event_t event;
    while (true) {
        // Wait for the next event from the event queue
        xQueueReceive(event_queue, &event, portMAX_DELAY);
        // Implement the finite state machine logic here
        // under each case, give/take tasks based on the event received
        switch (current_state) {
            case SS_STARTUP:
                // Handle startup state
                handle_startup(event);
                break;
            // case SS_IDLE:
            //     // Handle idle state
            //     handle_idle(event);
            //     break;
            case SS_SESSION:
                // Handle session state
                handle_session(event);
                break;
            case SS_SESSION_PAUSE:
                // Handle session pause state
                handle_session_pause(event);
                break;
            case SS_ERROR:
                // Handle error state
                handle_error(event);
                break;
            case SS_MENU:
                // Handle menu state
                handle_menu(event);
                // TODO: Make sure AR side is synchronized with the menu selection, handshake to check
                break;
            default:
                break;
        }
    }
}

extern "C" void app_main() {
    // initialize I2C master
    i2c_master_init(&bus_handle, &dev_handle);
    // set IMU configuration
    set_lsm6dsox_imu_config(dev_handle);

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
    // debug
    // display_mac_address();
    // display_ip_info();
    
    // configure wifi channel
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

#endif

#ifdef DEVICE_PERIPHERAL
    uint8_t count = 0, channel = 0;
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

    ESP_LOGI("WIFI", "Scanned Wi-Fi channel: %u", channel);
    espnow_init(channel);

#endif

#ifdef DEVICE_CENTRAL
    // attach to interrupt for buttons
    button_init();
#endif

    // create imu read task
    xTaskCreatePinnedToCore(
        read_imu_task,
        "Read IMU Task",
        4096,
        NULL,
        5,
        &read_imu_task_handle,
        tskNO_AFFINITY
    );
    // create imu calibration task
    xTaskCreatePinnedToCore(
        calibration_task,
        "Calibration Task",
        4096,
        NULL,
        5,
        &calibration_task_handle,
        tskNO_AFFINITY
    );
    // create trigger reference task
    xTaskCreatePinnedToCore(
        trigger_reference_task,
        "Trigger Reference Task",
        4096,
        NULL,
        5,
        &trigger_reference_task_handle,
        tskNO_AFFINITY
    );

    ESP_LOGI("SYSTEM", "All initialization done");
    // All init done, notify system
    ESP_LOGI("SYSTEM", "Pushing SE_INIT_READY event");
    push_event(SE_INIT_READY);

    // keep main task alive
    // FSM
    xTaskCreatePinnedToCore(
        fsm_task,
        "FSM Task",
        4096,
        NULL,
        5,
        &fsm_task_handle,
        tskNO_AFFINITY
    );

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}