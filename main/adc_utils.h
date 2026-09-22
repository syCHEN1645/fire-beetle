#include <cstddef>
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"

#include "func_config.h"
#include "hw_config.h"

#ifdef DEVICE_HAND
// flex sensor calibration data
inline float v_0 = 0.0f;
inline float v_90 = 0.0f;

bool is_finger_bent(float val);
#endif