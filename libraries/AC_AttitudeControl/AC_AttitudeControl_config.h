#pragma once

#include <AP_Logger/AP_Logger_config.h>

// Opt-in yaw diagnostics; no extra controller work in normal builds.
#ifndef AC_ATTITUDE_CONTROL_YAW_DIAGNOSTICS_ENABLED
#define AC_ATTITUDE_CONTROL_YAW_DIAGNOSTICS_ENABLED 0
#endif
