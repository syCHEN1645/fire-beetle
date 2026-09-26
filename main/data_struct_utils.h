#pragma once

// State and event related data structures
typedef enum system_event_t {
    // user input events
    SE_DOWN,
    SE_UP,
    SE_LEFT,
    SE_RIGHT,
    SE_CLICK,
    // non user input events
    SE_INIT_READY,
    SE_SESSION_START,
    SE_SESSION_END,
    SE_TRIGGER,
    SE_COUNT,
    SE_ERROR,
} system_event_t;

#define UI_EVENT_LOW 0
#define UI_EVENT_HIGH 4

typedef enum system_state_t {
    SS_STARTUP,
    SS_IDLE,
    SS_SESSION,
    SS_SESSION_PAUSE,
    SS_ERROR,
    SS_MENU,
} system_state_t;

typedef enum {
    // main menu options set
    MENU_CALIBRATION = 0,
    MENU_SESSION = 1,
    MENU_SETTINGS = 2,
    MENU_EXIT = 3,
    // session menu options set (link to MENU_SESSION)
    MENU_SESSION_START = 4,
    MENU_SESSION_INTERVAL = 5,
    MENU_SESSION_ACTION_1 = 6,
    MENU_SESSION_ACTION_2 = 7,
    MENU_SESSION_ACTION_3 = 8,
    MENU_SESSION_ACTION_4 = 9,
    MENU_SESSION_ACTION_5 = 10,
    MENU_SESSION_ACTION_6 = 11,
    // settings menu options set (link to MENU_SETTINGS)
    MENU_SETTINGS_VOLUME = 12,
    MENU_SETTINGS_BRIGHTNESS = 13,
    // add more settings menu options here if needed
} menu_selection_t;

typedef enum {
    AE_CLICK,
    AE_OK,
    AE_INVALID,
    AE_ERROR,
    AE_ACTION,
    AE_COUNT,
} actuator_event_t;

inline QueueHandle_t actuator_queue = xQueueCreate(5, sizeof(actuator_event_t));
inline QueueHandle_t event_queue = xQueueCreate(5, sizeof(system_event_t));

// Comms related data structures
enum class MessageType : uint8_t {
    SYNC = 1,
    DATA_START = 2,
    CALI_ZERO = 3,
    CALI_TRIGGER = 4,
    DATA_PAUSE = 5,
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

struct sensor_msg_t {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    // default to false (straight)
    bool flex_mid = false;
    bool flex_ind = false;
    int index;
    // tell central device what this message is responding to
    MessageType type;
};

struct state_event_msg_t {
    system_state_t state;
    actuator_event_t event;
    // true if state, false if event
    bool is_state;
};