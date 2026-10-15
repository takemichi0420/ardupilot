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

#ifndef AP_RANGEFINDER_VL53L3CX_REQUIRED_VALID_SAMPLES
#define AP_RANGEFINDER_VL53L3CX_REQUIRED_VALID_SAMPLES 1
#endif

#ifndef AP_RANGEFINDER_VL53L3CX_MAX_JUMP_MM
// Zero disables jump rejection. Board-specific filtering requires an override.
#define AP_RANGEFINDER_VL53L3CX_MAX_JUMP_MM 0
#endif

#ifndef AP_RANGEFINDER_VL53L3CX_MAX_JUMP_REJECTS
#define AP_RANGEFINDER_VL53L3CX_MAX_JUMP_REJECTS 4
#endif

#ifndef AP_RANGEFINDER_VL53L3CX_NODATA_TIMEOUT_MS
#define AP_RANGEFINDER_VL53L3CX_NODATA_TIMEOUT_MS 300
#endif

static constexpr uint16_t VL53L3CX_DEVICE_ID = 0xEAAA;
static constexpr uint16_t VL53L3CX_DEVICE_ID_REG = 0x010F;
static constexpr uint32_t VL53L3CX_CALLBACK_PERIOD_US = 50000;
static constexpr uint32_t VL53L3CX_TIMING_BUDGET_US = 33000;
static constexpr uint16_t VL53L3CX_MAX_READING_MM = 4000;

static bool vl53l3cx_timestamp_expired(uint32_t now_ms, uint32_t timestamp_ms)
{
    return uint32_t(now_ms - timestamp_ms) >= AP_RANGEFINDER_VL53L3CX_NODATA_TIMEOUT_MS;
}

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
    bool id_ok = false;
#if AP_RANGEFINDER_VL53L3CX_XSHUT_PIN >= 0
    id_ok = sensor->select_sensor_with_xshut(AP_RANGEFINDER_VL53L3CX_XSHUT_PIN,
                                            AP_RANGEFINDER_VL53L3CX_OTHER_XSHUT_PIN,
                                            true);
#if AP_RANGEFINDER_VL53L3CX_OTHER_XSHUT_PIN >= 0
    if (!id_ok) {
        id_ok = sensor->select_sensor_with_xshut(AP_RANGEFINDER_VL53L3CX_OTHER_XSHUT_PIN,
                                                AP_RANGEFINDER_VL53L3CX_XSHUT_PIN,
                                                true);
    }
    if (!id_ok) {
        id_ok = sensor->select_sensor_with_xshut(AP_RANGEFINDER_VL53L3CX_XSHUT_PIN,
                                                AP_RANGEFINDER_VL53L3CX_OTHER_XSHUT_PIN,
                                                false);
    }
#endif
#else
    sensor->reset_sensor();
    id_ok = sensor->check_id();
#endif

    if (!id_ok || !sensor->init()) {
        vl53l3cx_diag("VL53L3CX: init failed");
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_RangeFinder_VL53L3CX::set_xshut_pin(int8_t pin, bool enabled)
{
    if (pin < 0) {
        return;
    }
    hal.gpio->pinMode(pin, HAL_GPIO_OUTPUT);
    hal.gpio->write(pin, enabled ? 1 : 0);
}

bool AP_RangeFinder_VL53L3CX::select_sensor_with_xshut(int8_t active_pin,
                                                       int8_t inactive_pin,
                                                       bool hold_inactive_low)
{
    if (inactive_pin >= 0) {
        set_xshut_pin(inactive_pin, false);
    }
    if (active_pin >= 0) {
        set_xshut_pin(active_pin, false);
    }
    hal.scheduler->delay(10);
    if (active_pin >= 0) {
        set_xshut_pin(active_pin, true);
    }
    if (!hold_inactive_low && inactive_pin >= 0) {
        set_xshut_pin(inactive_pin, true);
    }
    hal.scheduler->delay(200);
    return check_id();
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
    if (!configure_sensor()) {
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

bool AP_RangeFinder_VL53L3CX::configure_sensor()
{
    invalidate_readings();
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

bool AP_RangeFinder_VL53L3CX::range_is_valid_for_control(int16_t range_mm) const
{
    if (range_mm <= 0 || range_mm > int16_t(VL53L3CX_MAX_READING_MM)) {
        return false;
    }

    const float distance_m = range_mm * 0.001f;
    return distance_m >= min_distance() && distance_m <= max_distance();
}

RangeFinder::Status AP_RangeFinder_VL53L3CX::classify_target(uint8_t range_status,
                                                              int16_t range_mm) const
{
    const int32_t min_mm = int32_t(min_distance() * 1000.0f);
    const int32_t max_mm = int32_t(max_distance() * 1000.0f);

    // Device-internal failures do not describe a target being too near or too
    // far. Treating these as out-of-range would incorrectly pass pre-arm.
    switch (range_status) {
    case VL53LX_RANGESTATUS_HARDWARE_FAIL:
    case VL53LX_RANGESTATUS_PROCESSING_FAIL:
    case VL53LX_RANGESTATUS_SYNCRONISATION_INT:
    case VL53LX_RANGESTATUS_NONE:
        return RangeFinder::Status::NoData;
    case VL53LX_RANGESTATUS_RANGE_VALID:
    case VL53LX_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED:
    case VL53LX_RANGESTATUS_MIN_RANGE_FAIL:
    case VL53LX_RANGESTATUS_RANGE_INVALID:
    case VL53LX_RANGESTATUS_SIGMA_FAIL:
    case VL53LX_RANGESTATUS_SIGNAL_FAIL:
    case VL53LX_RANGESTATUS_OUTOFBOUNDS_FAIL:
    case VL53LX_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL:
    case VL53LX_RANGESTATUS_WRAP_TARGET_FAIL:
    case VL53LX_RANGESTATUS_XTALK_SIGNAL_FAIL:
    case VL53LX_RANGESTATUS_RANGE_VALID_MERGED_PULSE:
    case VL53LX_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL:
        break;
    default:
        return RangeFinder::Status::NoData;
    }

    // A numeric result outside the configured range is authoritative even
    // when the sensor also reports a quality-related RangeStatus.
    if (range_mm <= 0 || (min_mm > 0 && range_mm < min_mm)) {
        return RangeFinder::Status::OutOfRangeLow;
    }
    if (range_mm > max_mm || range_mm > int16_t(VL53L3CX_MAX_READING_MM)) {
        return RangeFinder::Status::OutOfRangeHigh;
    }

    if (range_status == VL53LX_RANGESTATUS_RANGE_VALID) {
        return RangeFinder::Status::Good;
    }

    switch (range_status) {
    case VL53LX_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED:
    case VL53LX_RANGESTATUS_MIN_RANGE_FAIL:
    case VL53LX_RANGESTATUS_RANGE_INVALID:
        return RangeFinder::Status::OutOfRangeLow;

    // These are completed measurement results, not transport failures.  They
    // indicate that no trustworthy in-range target was returned, so expose
    // them as high/out-of-range instead of losing sensor health as NoData.
    case VL53LX_RANGESTATUS_SIGMA_FAIL:
    case VL53LX_RANGESTATUS_SIGNAL_FAIL:
    case VL53LX_RANGESTATUS_OUTOFBOUNDS_FAIL:
    case VL53LX_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL:
    case VL53LX_RANGESTATUS_WRAP_TARGET_FAIL:
    case VL53LX_RANGESTATUS_XTALK_SIGNAL_FAIL:
    case VL53LX_RANGESTATUS_RANGE_VALID_MERGED_PULSE:
    case VL53LX_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL:
    default:
        return RangeFinder::Status::OutOfRangeHigh;
    }
}

AP_RangeFinder_VL53L3CX::ReadResult AP_RangeFinder_VL53L3CX::get_reading(RawReading &reading)
{
    reading = {};
    reading.validated_status = VL53LX_RANGESTATUS_NONE;

    uint8_t ready = 0;
    int status = VL53LX_GetMeasurementDataReady(&vl53_device, &ready);
    if (status != VL53LX_ERROR_NONE) {
        return ReadResult::Error;
    }
    if (ready == 0) {
        return ReadResult::NotReady;
    }

    VL53LX_MultiRangingData_t ranging_data {};
    status = VL53LX_GetMultiRangingData(&vl53_device, &ranging_data);

    if (status == VL53LX_ERROR_NONE) {
        const uint8_t target_count = MIN(uint8_t(VL53LX_MAX_RANGE_RESULTS),
                                         ranging_data.NumberOfObjectsFound);
        if (target_count == 0) {
            // A completed measurement with no target is not a bus failure.
            reading.classified_status = RangeFinder::Status::OutOfRangeHigh;
        }
        uint8_t selected_priority = 0;
        for (uint8_t i = 0; i < target_count; i++) {
            const auto &target = ranging_data.RangeData[i];

            const RangeFinder::Status target_status = classify_target(target.RangeStatus,
                                                                       target.RangeMilliMeter);
            const uint8_t priority = target_status == RangeFinder::Status::Good ? 3 :
                                     target_status == RangeFinder::Status::OutOfRangeLow ? 2 :
                                     target_status == RangeFinder::Status::OutOfRangeHigh ? 1 : 0;
            if (priority > selected_priority ||
                (priority == selected_priority &&
                 target.RangeMilliMeter > 0 &&
                 (reading.raw_mm <= 0 || target.RangeMilliMeter > reading.raw_mm))) {
                selected_priority = priority;
                reading.raw_mm = target.RangeMilliMeter;
                reading.classified_mm = target.RangeMilliMeter > 0 ?
                    uint16_t(target.RangeMilliMeter) : 0;
                reading.classified_status = target_status;

                if (target_status == RangeFinder::Status::Good) {
                    reading.validated_mm = uint16_t(target.RangeMilliMeter);
                    reading.validated_status = target.RangeStatus;
                } else {
                    reading.validated_mm = 0;
                    reading.validated_status = VL53LX_RANGESTATUS_NONE;
                }
            }
        }
    }

    const int clear_status = VL53LX_ClearInterruptAndStartMeasurement(&vl53_device);
    if (status != VL53LX_ERROR_NONE || clear_status != VL53LX_ERROR_NONE) {
        return ReadResult::Error;
    }

    return ReadResult::Complete;
}

void AP_RangeFinder_VL53L3CX::clear_pending_readings()
{
    sum_mm = 0;
    sample_count = 0;
    last_sample_ms = 0;
    last_valid_mm = 0;
    last_candidate_ms = 0;
    consecutive_valid_count = 0;
    jump_reject_count = 0;
    pending_out_of_range = false;
    pending_out_of_range_mm = 0;
    pending_out_of_range_status = RangeFinder::Status::NoData;
}

void AP_RangeFinder_VL53L3CX::invalidate_readings()
{
    clear_pending_readings();
    valid_publish_available = false;
    set_status(RangeFinder::Status::NoData);
}

void AP_RangeFinder_VL53L3CX::timer()
{
    RawReading reading {};
    const ReadResult read_result = get_reading(reading);
    if (read_result == ReadResult::NotReady) {
        const uint32_t now_ms = AP_HAL::millis();
        WITH_SEMAPHORE(sem);
        if (!not_ready_active) {
            not_ready_active = true;
            not_ready_since_ms = now_ms;
        }
        if (vl53l3cx_timestamp_expired(now_ms, not_ready_since_ms)) {
            invalidate_readings();
        }
        return;
    }
    not_ready_active = false;
    if (read_result == ReadResult::Error) {
        WITH_SEMAPHORE(sem);
        invalidate_readings();
        return;
    }

    if (reading.classified_status == RangeFinder::Status::NoData) {
        WITH_SEMAPHORE(sem);
        invalidate_readings();
        return;
    }

    WITH_SEMAPHORE(sem);
    last_measurement_ms = AP_HAL::millis();

    if (reading.classified_status == RangeFinder::Status::OutOfRangeLow ||
        reading.classified_status == RangeFinder::Status::OutOfRangeHigh) {
        clear_pending_readings();
        valid_publish_available = false;
        pending_out_of_range = true;
        pending_out_of_range_mm = reading.classified_mm;
        pending_out_of_range_status = reading.classified_status;
        return;
    }

    pending_out_of_range = false;

    if (reading.validated_mm == 0 ||
        reading.validated_status != VL53LX_RANGESTATUS_RANGE_VALID ||
        reading.classified_status != RangeFinder::Status::Good) {
        invalidate_readings();
        return;
    }

    if (!range_is_valid_for_control(reading.validated_mm)) {
        clear_pending_readings();
        valid_publish_available = false;
        pending_out_of_range = true;
        pending_out_of_range_mm = reading.validated_mm;
        pending_out_of_range_status = reading.validated_mm * 0.001f < min_distance() ?
            RangeFinder::Status::OutOfRangeLow : RangeFinder::Status::OutOfRangeHigh;
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (last_valid_mm != 0 && vl53l3cx_timestamp_expired(now_ms, last_candidate_ms)) {
        // Discard the expired average as well as the filter sequence before
        // adding a fresh sample. This must match update(), whichever runs first.
        clear_pending_readings();
    }
#if AP_RANGEFINDER_VL53L3CX_MAX_JUMP_MM > 0
    if (last_valid_mm != 0) {
        const uint16_t jump_mm = reading.validated_mm > last_valid_mm ?
            reading.validated_mm - last_valid_mm :
            last_valid_mm - reading.validated_mm;
        if (jump_mm > AP_RANGEFINDER_VL53L3CX_MAX_JUMP_MM) {
            sum_mm = 0;
            sample_count = 0;
            last_sample_ms = 0;
            consecutive_valid_count = 0;
            jump_reject_count++;
            if (jump_reject_count < AP_RANGEFINDER_VL53L3CX_MAX_JUMP_REJECTS) {
                return;
            }

            // A persistent jump is treated as a reacquisition, but it still
            // must pass the consecutive-valid gate before reaching control.
            last_valid_mm = reading.validated_mm;
            last_candidate_ms = now_ms;
            jump_reject_count = 0;
            consecutive_valid_count = 1;
            return;
        }
    }
#endif

    last_valid_mm = reading.validated_mm;
    last_candidate_ms = now_ms;
    jump_reject_count = 0;
    if (consecutive_valid_count < AP_RANGEFINDER_VL53L3CX_REQUIRED_VALID_SAMPLES) {
        consecutive_valid_count++;
        if (consecutive_valid_count < AP_RANGEFINDER_VL53L3CX_REQUIRED_VALID_SAMPLES) {
            return;
        }
    }

    sum_mm += reading.validated_mm;
    sample_count++;
    last_sample_ms = now_ms;
    pending_out_of_range = false;
}

void AP_RangeFinder_VL53L3CX::update()
{
    WITH_SEMAPHORE(sem);

    const uint32_t now_ms = AP_HAL::millis();
    if (sample_count > 0 &&
        vl53l3cx_timestamp_expired(now_ms, last_sample_ms)) {
        invalidate_readings();
    }
    if (sample_count > 0) {
        state.distance_m = (sum_mm * 0.001f) / sample_count;
        last_valid_publish_ms = now_ms;
        valid_publish_available = true;
        state.last_reading_ms = now_ms;
        sum_mm = 0;
        sample_count = 0;
        pending_out_of_range = false;
        update_status();
    } else if (pending_out_of_range) {
        if (pending_out_of_range_status == RangeFinder::Status::OutOfRangeLow) {
            const float low_sentinel_m = MAX(0.0f, min_distance() - 0.001f);
            state.distance_m = pending_out_of_range_mm > 0 ?
                MIN(pending_out_of_range_mm * 0.001f, low_sentinel_m) : low_sentinel_m;
        } else {
            const float high_sentinel_m = max_distance() + 0.001f;
            state.distance_m = pending_out_of_range_mm > 0 ?
                MAX(pending_out_of_range_mm * 0.001f, high_sentinel_m) : high_sentinel_m;
        }
        state.last_reading_ms = last_measurement_ms;
        set_status(pending_out_of_range_status);
        pending_out_of_range = false;
    } else {
        const uint32_t now_ms = AP_HAL::millis();
        const bool fresh_valid = valid_publish_available &&
            !vl53l3cx_timestamp_expired(now_ms, last_valid_publish_ms);
        const bool fresh_out_of_range =
            (status() == RangeFinder::Status::OutOfRangeLow ||
             status() == RangeFinder::Status::OutOfRangeHigh) &&
            !vl53l3cx_timestamp_expired(now_ms, last_measurement_ms);
        if (fresh_valid || fresh_out_of_range) {
            return;
        }

        valid_publish_available = false;
        // Keep a fresh priming sequence across update() calls. Only a gap in
        // valid candidates invalidates that sequence; raw activity is irrelevant.
        if (vl53l3cx_timestamp_expired(now_ms, last_candidate_ms)) {
            clear_pending_readings();
        }
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
