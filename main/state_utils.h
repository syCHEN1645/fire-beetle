#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "data_struct_utils.h"
#include "comms_utils.h"

inline volatile system_state_t current_state = SS_STARTUP;

#ifdef DEVICE_CENTRAL
inline volatile menu_selection_t current_menu = MENU_CALIBRATION;
inline int session_count = 0;
inline int session_action_index = 0;
inline int session_target = 0;

extern void start_calibration();
extern void start_session();
extern void pause_session();
extern void end_session();

void push_sys_event(system_event_t event);
void push_actuator_event(actuator_event_t event);
void change_sys_state(system_state_t new_state);
void handle_menu_click();
void handle_startup(system_event_t event);
void handle_idle(system_event_t event);
void handle_session(system_event_t event);
void handle_session_pause(system_event_t event);
void handle_error(system_event_t event);
void handle_menu(system_event_t event);
void reset_session_progress();
#endif