#pragma once

#include "AP_RangeFinder_config.h"

#ifndef AP_RANGEFINDER_VL53L3CX_DIAGNOSTICS
#define AP_RANGEFINDER_VL53L3CX_DIAGNOSTICS 0
#endif

#if AP_RANGEFINDER_VL53L3CX_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#include <AP_HAL/I2CDevice.h>

extern "C" {
#include <vl53lx_api.h>
}

class AP_RangeFinder_VL53L3CX : public AP_RangeFinder_Backend
{
public:
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &state,
                                          AP_RangeFinder_Params &params,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    void update() override;

protected:
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override
    {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    AP_RangeFinder_VL53L3CX(RangeFinder::RangeFinder_State &state,
                            AP_RangeFinder_Params &params,
                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    bool init();
    bool configure_sensor();
    bool check_id();
    bool start_measurement();
    enum class ReadResult : uint8_t {
        NotReady,
        Complete,
        Error,
    };

    struct RawReading {
        int16_t raw_mm {};
        uint16_t validated_mm {};
        uint8_t validated_status {VL53LX_RANGESTATUS_NONE};
        uint16_t classified_mm {};
        RangeFinder::Status classified_status {RangeFinder::Status::NoData};
    };

#if AP_RANGEFINDER_VL53L3CX_DIAGNOSTICS
    enum class DiagnosticEvent : uint8_t { API_ERROR, RANGE_STATUS_CHANGE };
    enum class DiagnosticOperation : uint8_t { NONE, READY, RANGING_DATA, CLEAR_INTERRUPT };
    struct DiagnosticRecord {
        DiagnosticEvent event {};
        DiagnosticOperation operation {};
        VL53LX_Error api_status {};
        uint8_t previous_status {};
        uint8_t new_status {};
        uint8_t object_count {};
        uint32_t occurrences {};
    };
    // All diagnostic state is protected by the existing backend semaphore.
    // One regular update context consumes records; callbacks only publish.
    DiagnosticRecord diagnostic_pending {};
    uint32_t diagnostic_suppressed {};
    uint32_t diagnostic_emitted {};
    uint32_t diagnostic_last_report_ms {};
    uint8_t diagnostic_previous_status {};
    bool diagnostic_have_status {};
    bool diagnostic_pending_valid {};
    bool diagnostic_have_reported {};

    void queue_diagnostic_locked(const DiagnosticRecord &record);
    void record_api_error(DiagnosticOperation operation, VL53LX_Error api_status);
    void record_range_status(uint8_t range_status, uint8_t object_count);
    void service_diagnostics();
#endif

    ReadResult get_reading(RawReading &reading);
    RangeFinder::Status classify_target(uint8_t range_status, int16_t range_mm) const;
    bool range_is_valid_for_control(int16_t range_mm) const;
    void clear_pending_readings();
    void invalidate_readings();
    void timer();
    void reset_sensor();
    void set_xshut_pin(int8_t pin, bool enabled);
    bool select_sensor_with_xshut(int8_t active_pin,
                                  int8_t inactive_pin,
                                  bool hold_inactive_low);

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;
    VL53LX_Dev_t vl53_device {};
    HAL_Semaphore sem;
    AP_HAL::Device::PeriodicHandle periodic_handle {};
    uint32_t sum_mm {};
    uint32_t sample_count {};
    uint16_t last_valid_mm {};
    uint32_t last_candidate_ms {};    // Valid filter reference, including priming/reacquisition.
    uint32_t last_measurement_ms {};  // Completed Good/OutOfRange result, before sample filtering.
    uint32_t last_sample_ms {};       // Last sample accepted into the unpublished average.
    uint32_t last_valid_publish_ms {};
    uint32_t not_ready_since_ms {};
    bool valid_publish_available {};
    bool not_ready_active {};
    uint8_t consecutive_valid_count {};
    uint8_t jump_reject_count {};
    uint16_t pending_out_of_range_mm {};
    RangeFinder::Status pending_out_of_range_status {RangeFinder::Status::NoData};
    bool pending_out_of_range {};
    bool healthy_reported {};
    bool error_status_reported {};
    bool status_reported {};
    RangeFinder::Status last_status {RangeFinder::Status::NotConnected};
    RangeFinder::Status last_error_status {RangeFinder::Status::NotConnected};
};

#endif  // AP_RANGEFINDER_VL53L3CX_ENABLED
