#include "state_utils.h"

#include <math.h>
#include "esp_log.h"

/// @brief Push an event to the event queue from an ISR.
/// @param e The event to be pushed.
/// @note This function will be called from an ISR context, do not call ESP_LOG inside.
void push_event(system_event_t event) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(event_queue, &event, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/// @brief Handles the action when a menu item is clicked (event SE_CLICK).
void handle_menu_click() {
    switch (current_menu) {
        case MENU_CALIBRATION:
            ESP_LOGI("FSM", "Selected MENU_CALIBRATION");
            // start calibration handled by extern function in main
            start_calibration();
            break;
        case MENU_SESSION:
            ESP_LOGI("FSM", "Selected MENU_SESSION");
            // notify all devices to read imu data
            current_menu = MENU_SESSION_START;
            break;
        case MENU_SETTINGS:
            ESP_LOGI("FSM", "Selected MENU_SETTINGS");
            // go to settings layer
            current_menu = MENU_SETTINGS_VOLUME;
            break;
        case MENU_EXIT:
            ESP_LOGI("FSM", "Selected MENU_EXIT");
            // sleep mode
            break;
        case MENU_SESSION_START:
            ESP_LOGI("FSM", "Selected MENU_SESSION_START");
            push_event(SE_SESSION_START);
            break;
        case MENU_SESSION_INTERVAL:
            ESP_LOGI("FSM", "Selected MENU_SESSION_INTERVAL");
            // handle session interval
            break;
        case MENU_SESSION_ACTION_1:
            ESP_LOGI("FSM", "Selected MENU_SESSION_ACTION_1");
            // handle session action 1
            break;
        case MENU_SESSION_ACTION_2:
            ESP_LOGI("FSM", "Selected MENU_SESSION_ACTION_2");
            // handle session action 2
            break;
        default:
            break;
    }
}

void handle_startup(system_event_t event) {
    switch (event) {
        case SE_INIT_READY:
            ESP_LOGI("FSM", "SS_STARTUP state, received SE_INIT_READY event");
            current_state = SS_MENU;
            break;
        case SE_ERROR:
            ESP_LOGI("FSM", "SS_STARTUP state, received SE_ERROR event");
            current_state = SS_ERROR;
            break;
        default:
            break;
    }
}

void handle_idle(system_event_t event) {
}

void handle_session(system_event_t event) {
    switch (event) {
        case SE_TRIGGER:
            ESP_LOGI("FSM", "SS_SESSION state, received SE_TRIGGER event");
            current_state = SS_SESSION_PAUSE;
            pause_session();
            break;
        case SE_CLICK:
            ESP_LOGI("FSM", "SS_SESSION state, received SE_CLICK event");
            current_state = SS_SESSION_PAUSE;
            pause_session();
            break;
        case SE_LEFT:
            ESP_LOGI("FSM", "SS_SESSION state, received SE_LEFT event");
            current_state = SS_SESSION_PAUSE;
            pause_session();
            break;
        case SE_ERROR:
            ESP_LOGI("FSM", "SS_SESSION state, received SE_ERROR event");
            current_state = SS_ERROR;
            break;
        default:
            break;
    }
}

void handle_session_pause(system_event_t event) {
    switch (event) {
        case SE_TRIGGER:
            ESP_LOGI("FSM", "SS_SESSION_PAUSE state, received SE_TRIGGER event");
            current_state = SS_SESSION;
            // continue session
            start_session();
            break;
        case SE_LEFT:
            ESP_LOGI("FSM", "SS_SESSION_PAUSE state, received SE_LEFT event");
            current_state = SS_SESSION;
            // continue session
            start_session();
            break;
        case SE_CLICK:
            ESP_LOGI("FSM", "SS_SESSION_PAUSE state, received SE_CLICK event");
            current_state = SS_MENU;
            // end the session early, back to menu
            end_session();
            break;
        case SE_ERROR:
            ESP_LOGI("FSM", "SS_SESSION_PAUSE state, received SE_ERROR event");
            current_state = SS_ERROR;
            break;
        default:
            break;
    }
}

void handle_error(system_event_t event) {
}

/// @brief Handles moving to a different menu option or an action based on the event. Handles state transitions.
/// @param event The system event triggering the menu action
void handle_menu(system_event_t event) {
    switch (event) {
        case SE_DOWN:
            ESP_LOGI("FSM", "SS_MENU state, received SE_DOWN event");
            // menu selection moves down by 1, loop back over limit
            if (current_menu >= 0 && current_menu < 4) {
                current_menu = static_cast<menu_selection_t>((current_menu + 1) % 4);
            } else if (current_menu >= 4 && current_menu < 12) {
                current_menu = static_cast<menu_selection_t>(std::max((current_menu + 1) % 12, 4));
            } else if (current_menu >= 12 && current_menu < 14) {
                current_menu = static_cast<menu_selection_t>(std::max((current_menu + 1) % 14, 12));
            }
            break;
        case SE_UP:
            ESP_LOGI("FSM", "SS_MENU state, received SE_UP event");
            // menu selection moves up by 1, loop back over limit
            if (current_menu >= 0 && current_menu < 4) {
                current_menu = static_cast<menu_selection_t>((current_menu + 3) % 4);
            } else if (current_menu >= 4 && current_menu < 12) {
                current_menu = static_cast<menu_selection_t>(std::max((current_menu + 11) % 12, 4));
            } else if (current_menu >= 12 && current_menu < 14) {
                current_menu = static_cast<menu_selection_t>(std::max((current_menu + 13) % 14, 12));
            }
            break;
        case SE_CLICK:
            ESP_LOGI("FSM", "SS_MENU state, received SE_CLICK event");
            // "click" on the current menu option
            handle_menu_click();
            break;
        case SE_LEFT:
            // go back to previous layer
            ESP_LOGI("FSM", "SS_MENU state, received SE_LEFT event");
            if (current_menu >= 4 && current_menu < 12) {
                current_menu = MENU_SESSION;
            } else if (current_menu >= 12 && current_menu < 14) {
                current_menu = MENU_SETTINGS;
            }
            break;
        case SE_SESSION_START:
            ESP_LOGI("RECV", "SE_SESSION_START received");
            current_state = SS_SESSION;
            start_session();
            break;
        case SE_ERROR:
            ESP_LOGI("FSM", "SS_MENU state, received SE_ERROR event");
            current_state = SS_ERROR;
            break;
        default:
            break;
    }
    ESP_LOGI("FSM", "current menu: %d", current_menu);
}