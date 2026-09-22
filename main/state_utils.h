#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    SE_DOWN,
    SE_UP,
    SE_LEFT,
    SE_RIGHT,
    SE_CLICK,
    SE_INIT_READY,
    SE_SESSION_START,
    SE_TRIGGER,
    SE_COUNT,
    SE_ERROR,
} system_event_t;

typedef enum {
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

inline volatile system_state_t current_state = SS_STARTUP;
inline volatile menu_selection_t current_menu = MENU_CALIBRATION;
inline QueueHandle_t event_queue = xQueueCreate(10, sizeof(system_event_t));

extern void start_calibration();
extern void start_session();
extern void pause_session();
extern void end_session();

void push_event(system_event_t event);
void handle_menu_click();
void handle_startup(system_event_t event);
void handle_idle(system_event_t event);
void handle_session(system_event_t event);
void handle_session_pause(system_event_t event);
void handle_error(system_event_t event);
void handle_menu(system_event_t event);