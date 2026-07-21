/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

#include "AP_RangeFinder_VL53L3CX.h"

#if AP_RANGEFINDER_VL53L3CX_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

#ifndef AP_RANGEFINDER_VL53L3CX_DIAG_GCS_TEXT_LIMIT
#define AP_RANGEFINDER_VL53L3CX_DIAG_GCS_TEXT_LIMIT 0
#endif

#if AP_RANGEFINDER_VL53L3CX_DIAG_GCS_TEXT_LIMIT > 0
static uint8_t vl53l3cx_gcs_text_count;
#define vl53l3cx_diag(fmt, args ...) do { \
        hal.console->printf(fmt "\n", ## args); \
        if (vl53l3cx_gcs_text_count < AP_RANGEFINDER_VL53L3CX_DIAG_GCS_TEXT_LIMIT) { \
            vl53l3cx_gcs_text_count++; \
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, fmt, ## args); \
        } \
    } while (0)
#else
#define vl53l3cx_diag(fmt, args ...) do {} while (0)
#endif

#ifndef AP_RANGEFINDER_VL53L3CX_XSHUT_PIN
#define AP_RANGEFINDER_VL53L3CX_XSHUT_PIN -1
#endif

#ifndef AP_RANGEFINDER_VL53L3CX_OTHER_XSHUT_PIN
#define AP_RANGEFINDER_VL53L3CX_OTHER_XSHUT_PIN -1
#endif

static constexpr uint16_t VL53L3CX_DEVICE_ID = 0xEAAA;
static constexpr uint16_t VL53L3CX_DEVICE_ID_REG = 0x010F;
static constexpr uint32_t VL53L3CX_CALLBACK_PERIOD_US = 50000;
static constexpr uint32_t VL53L3CX_TIMING_BUDGET_US = 33000;
static constexpr uint16_t VL53L3CX_MAX_READING_MM = 4000;

AP_RangeFinder_VL53L3CX::AP_RangeFinder_VL53L3CX(
    RangeFinder::RangeFinder_State &_state,
    AP_RangeFinder_Params &_params,
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev) :
    AP_RangeFinder_Backend(_state, _params),
    dev(std::move(_dev))
{
    vl53_device.i2c_slave_address = dev->get_bus_address();
    vl53_device.comms_speed_khz = 400;
    vl53_device.I2cHandle = dev.get();
}

AP_RangeFinder_Backend *AP_RangeFinder_VL53L3CX::detect(
    RangeFinder::RangeFinder_State &_state,
    AP_RangeFinder_Params &_params,
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    // Avoid driving the legacy ESP-IDF I2C recovery path with ten immediate
    // retries if the sensor temporarily NACKs while switching ranges.
    dev->set_retries(1);

    auto *sensor = NEW_NOTHROW AP_RangeFinder_VL53L3CX(_state, _params, std::move(dev));
    if (sensor == nullptr) {
        return nullptr;
    }

    WITH_SEMAPHORE(sensor->dev->get_semaphore());
    sensor->reset_sensor();

    if (!sensor->check_id() || !sensor->init()) {
        vl53l3cx_diag("VL53L3CX: init failed");
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_RangeFinder_VL53L3CX::reset_sensor()
{
#if AP_RANGEFINDER_VL53L3CX_OTHER_XSHUT_PIN >= 0
    hal.gpio->pinMode(AP_RANGEFINDER_VL53L3CX_OTHER_XSHUT_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->write(AP_RANGEFINDER_VL53L3CX_OTHER_XSHUT_PIN, 0);
#endif

#if AP_RANGEFINDER_VL53L3CX_XSHUT_PIN >= 0
    hal.gpio->pinMode(AP_RANGEFINDER_VL53L3CX_XSHUT_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->write(AP_RANGEFINDER_VL53L3CX_XSHUT_PIN, 0);
    hal.scheduler->delay(10);
    hal.gpio->write(AP_RANGEFINDER_VL53L3CX_XSHUT_PIN, 1);
    hal.scheduler->delay(100);
#endif
}

bool AP_RangeFinder_VL53L3CX::check_id()
{
    uint16_t device_id = 0;
    const int status = VL53LX_RdWord(&vl53_device, VL53L3CX_DEVICE_ID_REG, &device_id);
    if (status != VL53LX_ERROR_NONE || device_id != VL53L3CX_DEVICE_ID) {
        return false;
    }

    vl53l3cx_diag("VL53L3CX: device id=0x%04x", (unsigned)device_id);
    return true;
}

bool AP_RangeFinder_VL53L3CX::init()
{
    int status = VL53LX_WaitDeviceBooted(&vl53_device);
    if (status != VL53LX_ERROR_NONE) {
        vl53l3cx_diag("VL53L3CX: WaitDeviceBooted failed status=%d", status);
        return false;
    }

    status = VL53LX_DataInit(&vl53_device);
    if (status != VL53LX_ERROR_NONE) {
        vl53l3cx_diag("VL53L3CX: DataInit failed status=%d", status);
        return false;
    }

    status = VL53LX_SetDistanceMode(&vl53_device, VL53LX_DISTANCEMODE_MEDIUM);
    if (status != VL53LX_ERROR_NONE) {
        vl53l3cx_diag("VL53L3CX: SetDistanceMode failed status=%d", status);
        return false;
    }

    status = VL53LX_SetMeasurementTimingBudgetMicroSeconds(&vl53_device,
                                                            VL53L3CX_TIMING_BUDGET_US);
    if (status != VL53LX_ERROR_NONE) {
        vl53l3cx_diag("VL53L3CX: timing budget failed status=%d", status);
        return false;
    }

    if (!start_measurement()) {
        return false;
    }

    periodic_handle = dev->register_periodic_callback(
        VL53L3CX_CALLBACK_PERIOD_US,
        FUNCTOR_BIND_MEMBER(&AP_RangeFinder_VL53L3CX::timer, void));
    if (periodic_handle == nullptr) {
        vl53l3cx_diag("VL53L3CX: timer registration failed");
        return false;
    }

    vl53l3cx_diag("VL53L3CX: init success");
    return true;
}

bool AP_RangeFinder_VL53L3CX::start_measurement()
{
    const int status = VL53LX_StartMeasurement(&vl53_device);
    if (status != VL53LX_ERROR_NONE) {
        vl53l3cx_diag("VL53L3CX: start ranging failed status=%d", status);
        return false;
    }

    vl53l3cx_diag("VL53L3CX: start ranging success budget_us=%lu period_us=%lu",
                  (unsigned long)VL53L3CX_TIMING_BUDGET_US,
                  (unsigned long)VL53L3CX_CALLBACK_PERIOD_US);
    return true;
}

bool AP_RangeFinder_VL53L3CX::get_reading(uint16_t &reading_mm, uint8_t &range_status)
{
    uint8_t ready = 0;
    int status = VL53LX_GetMeasurementDataReady(&vl53_device, &ready);
    if (status != VL53LX_ERROR_NONE || ready == 0) {
        return false;
    }

    VL53LX_MultiRangingData_t ranging_data {};
    status = VL53LX_GetMultiRangingData(&vl53_device, &ranging_data);

    uint16_t selected_mm = 0;
    uint8_t selected_status = 0xFF;
    if (status == VL53LX_ERROR_NONE) {
        for (uint8_t i = 0; i < ranging_data.NumberOfObjectsFound; i++) {
            const auto &target = ranging_data.RangeData[i];
            if (target.RangeStatus == VL53LX_RANGESTATUS_RANGE_VALID &&
                target.RangeMilliMeter > 0 &&
                target.RangeMilliMeter <= VL53L3CX_MAX_READING_MM &&
                target.RangeMilliMeter > selected_mm) {
                selected_mm = target.RangeMilliMeter;
                selected_status = target.RangeStatus;
            }
        }
    }

    const int clear_status = VL53LX_ClearInterruptAndStartMeasurement(&vl53_device);
    if (status != VL53LX_ERROR_NONE || clear_status != VL53LX_ERROR_NONE || selected_mm == 0) {
        return false;
    }

    reading_mm = selected_mm;
    range_status = selected_status;
    return true;
}

void AP_RangeFinder_VL53L3CX::timer()
{
    uint16_t range_mm = 0;
    uint8_t range_status = 0xFF;
    if (!get_reading(range_mm, range_status)) {
        return;
    }

    WITH_SEMAPHORE(sem);
    sum_mm += range_mm;
    sample_count++;
}

void AP_RangeFinder_VL53L3CX::update()
{
    WITH_SEMAPHORE(sem);

    if (sample_count > 0) {
        state.distance_m = (sum_mm * 0.001f) / sample_count;
        state.last_reading_ms = AP_HAL::millis();
        sum_mm = 0;
        sample_count = 0;
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 300) {
        set_status(RangeFinder::Status::NoData);
    }

    if (!status_reported || last_status != status()) {
        status_reported = true;
        last_status = status();
        if (status() == RangeFinder::Status::Good) {
            if (!healthy_reported) {
                healthy_reported = true;
                vl53l3cx_diag("VL53L3CX: healthy");
            }
        } else if (!error_status_reported || last_error_status != status()) {
            error_status_reported = true;
            last_error_status = status();
            vl53l3cx_diag("VL53L3CX: unhealthy status=%u", (unsigned)status());
        }
    }
}

#endif  // AP_RANGEFINDER_VL53L3CX_ENABLED
