# VL53L3CX bare driver

The ST VL53LX bare-driver sources in this directory were imported from the
M5Stack StampFly firmware repository at commit
`561b1ee5e7ecbffc8462cae8f88d23fb970865cf`:

https://github.com/m5stack/M5StampFly/tree/561b1ee5e7ecbffc8462cae8f88d23fb970865cf/lib/vl53l3c

The upstream files are dual licensed under GPL-2.0-or-later or BSD-3-Clause as
declared in their SPDX headers. `vl53lx_platform_ap.cpp` is the ArduPilot AP_HAL
I2C adaptation. The upstream ESP-IDF `vl53lx_platform.c` and GPIO-oriented
`vl53lx_platform_init.c` are retained for provenance but are excluded from the
ArduPilot build.
