#pragma once

#include <AP_HAL_ESP32/HAL_ESP32_Namespace.h>

class ESP32::ESPNowBridge {
public:
    static bool start_receiver();
    static bool is_started();
};
