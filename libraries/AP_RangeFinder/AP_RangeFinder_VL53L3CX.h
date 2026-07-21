#pragma once

#include "AP_RangeFinder_config.h"

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
    bool check_id();
    bool start_measurement();
    bool get_reading(uint16_t &reading_mm, uint8_t &range_status);
    void timer();
    void reset_sensor();

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;
    VL53LX_Dev_t vl53_device {};
    HAL_Semaphore sem;
    AP_HAL::Device::PeriodicHandle periodic_handle {};
    uint32_t sum_mm {};
    uint32_t sample_count {};
    bool healthy_reported {};
    bool error_status_reported {};
    bool status_reported {};
    RangeFinder::Status last_status {RangeFinder::Status::NotConnected};
    RangeFinder::Status last_error_status {RangeFinder::Status::NotConnected};
};

#endif  // AP_RANGEFINDER_VL53L3CX_ENABLED
