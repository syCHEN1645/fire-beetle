#include "adc_utils.h"

#include "esp_log.h"

#ifdef DEVICE_CENTRAL
#endif

#ifdef DEVICE_HAND
bool is_finger_bent(float val) {
    float bend_ratio = (v_0 - val) / (v_0 - v_90);

    // clamp to [0, 1]
    if (bend_ratio < 0.0f) {
        bend_ratio = 0.0f;
    } else if (bend_ratio > 1.0f) {
        bend_ratio = 1.0f;
    }

    return bend_ratio >= FLEX_BENT_THRESHOLD;
}
#endif