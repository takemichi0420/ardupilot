/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  driver for ST VL53L1X lidar

  Many thanks to Pololu, https://github.com/pololu/vl53l1x-arduino and
  the ST example code
 */
#include "AP_RangeFinder_VL53L1X.h"

#if AP_RANGEFINDER_VL53L1X_ENABLED

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#ifndef AP_RANGEFINDER_VL53L1X_DIAG_GCS_TEXT_LIMIT
#define AP_RANGEFINDER_VL53L1X_DIAG_GCS_TEXT_LIMIT 0
#endif

#if AP_RANGEFINDER_VL53L1X_DIAG_GCS_TEXT_LIMIT > 0
static uint8_t vl53_gcs_text_count;
#define vl53_diag(fmt, args ...) do { \
        printf(fmt, ## args); \
        printf("\n"); \
        if (vl53_gcs_text_count < AP_RANGEFINDER_VL53L1X_DIAG_GCS_TEXT_LIMIT) { \
            vl53_gcs_text_count++; \
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, fmt, ## args); \
        } \
    } while (0)
#else
#define vl53_diag(fmt, args ...) do { printf(fmt, ## args); printf("\n"); GCS_SEND_TEXT(MAV_SEVERITY_INFO, fmt, ## args); } while (0)
#endif

#ifndef AP_RANGEFINDER_VL53L1X_ACCEPT_L3CX_ID
#define AP_RANGEFINDER_VL53L1X_ACCEPT_L3CX_ID 0
#endif

#ifndef AP_RANGEFINDER_VL53L1X_DIAG_RAW_READS
#define AP_RANGEFINDER_VL53L1X_DIAG_RAW_READS 0
#endif

#ifndef AP_RANGEFINDER_VL53L1X_DIAG_READY_READS
#define AP_RANGEFINDER_VL53L1X_DIAG_READY_READS 0
#endif

#ifndef AP_RANGEFINDER_VL53L1X_DIAG_READ_ON_READY_TIMEOUT
#define AP_RANGEFINDER_VL53L1X_DIAG_READ_ON_READY_TIMEOUT 0
#endif

#ifndef AP_RANGEFINDER_VL53L1X_DIAG_DUMP_EVERY_READ
#define AP_RANGEFINDER_VL53L1X_DIAG_DUMP_EVERY_READ 0
#endif

#ifndef AP_RANGEFINDER_VL53L1X_USE_INTERRUPT_STATUS_READY
#define AP_RANGEFINDER_VL53L1X_USE_INTERRUPT_STATUS_READY 0
#endif

#ifndef AP_RANGEFINDER_VL53L1X_DIAG_SINGLE_SHOT_ON_TIMEOUT
#define AP_RANGEFINDER_VL53L1X_DIAG_SINGLE_SHOT_ON_TIMEOUT 0
#endif

#ifndef AP_RANGEFINDER_VL53L1X_DIAG_SINGLE_SHOT_TIMEOUT_MS
#define AP_RANGEFINDER_VL53L1X_DIAG_SINGLE_SHOT_TIMEOUT_MS 100
#endif

#ifndef AP_RANGEFINDER_VL53L1X_RESTART_AFTER_CLEAR
#define AP_RANGEFINDER_VL53L1X_RESTART_AFTER_CLEAR 0
#endif

#ifndef AP_RANGEFINDER_VL53L1X_DIAG_ACCEPT_ANY_RANGE_STATUS
#define AP_RANGEFINDER_VL53L1X_DIAG_ACCEPT_ANY_RANGE_STATUS 0
#endif

#ifndef AP_RANGEFINDER_VL53L1X_DIAG_ALLOW_UNCALIBRATED
#define AP_RANGEFINDER_VL53L1X_DIAG_ALLOW_UNCALIBRATED 0
#endif

#ifndef AP_RANGEFINDER_VL53L1X_DIAG_PUBLISH_READY_RANGE
#define AP_RANGEFINDER_VL53L1X_DIAG_PUBLISH_READY_RANGE 0
#endif

static bool vl53_diag_raw_allowed(uint8_t count)
{
#if AP_RANGEFINDER_VL53L1X_DIAG_RAW_READS > 0
    return count < AP_RANGEFINDER_VL53L1X_DIAG_RAW_READS;
#else
    (void)count;
    return false;
#endif
}

static bool vl53_diag_ready_allowed(uint8_t count)
{
#if AP_RANGEFINDER_VL53L1X_DIAG_READY_READS > 0
    return count < AP_RANGEFINDER_VL53L1X_DIAG_READY_READS;
#else
    (void)count;
    return false;
#endif
}

static const uint8_t MEASUREMENT_TIME_MS = 50; // Start continuous readings at a rate of one measurement every 50 ms

AP_RangeFinder_VL53L1X::AP_RangeFinder_VL53L1X(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev)
    : AP_RangeFinder_Backend(_state, _params)
    , dev(std::move(_dev))
    , calibrated(false)
    , diag_status_reported(false)
    , diag_publish_reported(false)
    , diag_last_status(RangeFinder::Status::NotConnected)
    , diag_raw_count(0)
    , diag_ready_count(0) {}

/*
   detect if a VL53L1X rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_VL53L1X::detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, DistanceMode mode)
{
    vl53_diag("VL53: probe start VL53L1X");
    if (!dev) {
        vl53_diag("VL53: I2C open failed VL53L1X");
        return nullptr;
    }
    vl53_diag("VL53: I2C addr 0x%02x bus %u VL53L1X", dev->get_bus_address(), (unsigned)dev->bus_num());

    AP_RangeFinder_VL53L1X *sensor
        = NEW_NOTHROW AP_RangeFinder_VL53L1X(_state, _params, std::move(dev));

    if (!sensor) {
        vl53_diag("VL53: init failed VL53L1X alloc");
        delete sensor;
        return nullptr;
    }

    sensor->dev->get_semaphore()->take_blocking();

    const bool id_ok = sensor->check_id();
    const bool init_ok = id_ok && sensor->init(mode);
    if (!id_ok || !init_ok) {
        vl53_diag("VL53: init failed VL53L1X");
        sensor->dev->get_semaphore()->give();
        delete sensor;
        return nullptr;
    }
    vl53_diag("VL53: init success VL53L1X");

    sensor->dev->get_semaphore()->give();

    return sensor;
}

// check sensor ID registers
bool AP_RangeFinder_VL53L1X::check_id(void)
{
    uint8_t v1, v2;
    if (!(read_register(0x010F, v1) &&
          read_register(0x0110, v2))) {
        vl53_diag("VL53: device id read failed VL53L1X");
        return false;
    }

    vl53_diag("VL53: device id 0x%02x%02x VL53L1X", v1, v2);
    if ((v1 == 0xEA) && (v2 == 0xCC)) {
        printf("Detected VL53L1X on bus 0x%x\n", unsigned(dev->get_bus_id()));
        return true;
    }

#if AP_RANGEFINDER_VL53L1X_ACCEPT_L3CX_ID
    if ((v1 == 0xEA) && (v2 == 0xAA)) {
        vl53_diag("VL53L1X/L3CX: accepting device id 0xeaaa for diagnostic");
        return true;
    }
#endif

    {
        vl53_diag("VL53: device id mismatch VL53L1X");
        return false;
    }
}

bool AP_RangeFinder_VL53L1X::reset(void) {
    if (dev->get_bus_id()!=0x29) {
        // if sensor is on a different port than the default do not  reset sensor otherwise we will lose the addess.
        // we assume it is already confirgured.
        return true;
    }
    if (!write_register(SOFT_RESET, 0x00)) {
        return false;
    }
    hal.scheduler->delay_microseconds(100);
    if (!write_register(SOFT_RESET, 0x01)) {
        return false;
    }
    hal.scheduler->delay(1000);
    return true;
}

/*
  initialise sensor
 */
bool AP_RangeFinder_VL53L1X::init(DistanceMode mode)
{
    vl53_diag("VL53: init start VL53L1X/L3CX mode=%u", (unsigned)mode);
#define VL53_INIT_STEP(_name, _expr) do { \
        if (!(_expr)) { \
            vl53_diag("VL53: init step %s failed", _name); \
            return false; \
        } \
        vl53_diag("VL53: init step %s success", _name); \
    } while (0)

    // we need to do resets and delays in order to configure the sensor, don't do this if we are trying to fast boot
    if (hal.util->was_watchdog_armed()) {
        vl53_diag("VL53: init step watchdog fastboot failed");
        return false;
    }

    uint8_t pad_i2c_hv_extsup_config = 0;
    uint16_t mm_config_outer_offset_mm = 0;

    VL53_INIT_STEP("reset", reset());
    VL53_INIT_STEP("read PAD_I2C_HV__EXTSUP_CONFIG", read_register(PAD_I2C_HV__EXTSUP_CONFIG, pad_i2c_hv_extsup_config));
    VL53_INIT_STEP("write PAD_I2C_HV__EXTSUP_CONFIG", write_register(PAD_I2C_HV__EXTSUP_CONFIG, pad_i2c_hv_extsup_config | 0x01));
    VL53_INIT_STEP("read fast oscillator", read_register16(OSC_MEASURED__FAST_OSC__FREQUENCY, fast_osc_frequency));
    VL53_INIT_STEP("read osc calibration", read_register16(RESULT__OSC_CALIBRATE_VAL, osc_calibrate_val));
    VL53_INIT_STEP("static config",
                   write_register16(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TargetRate) &&
                   write_register(GPIO__TIO_HV_STATUS, 0x02) &&
                   write_register(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8) &&
                   write_register(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16) &&
                   write_register(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01) &&
                   write_register(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF) &&
                   write_register(ALGO__RANGE_MIN_CLIP, 0) &&
                   write_register(ALGO__CONSISTENCY_CHECK__TOLERANCE, 2));
    VL53_INIT_STEP("general config",
                   write_register16(SYSTEM__THRESH_RATE_HIGH, 0x0000) &&
                   write_register16(SYSTEM__THRESH_RATE_LOW, 0x0000) &&
                   write_register(DSS_CONFIG__APERTURE_ATTENUATION, 0x38));
    VL53_INIT_STEP("timing config",
                   write_register16(RANGE_CONFIG__SIGMA_THRESH, 360) &&
                   write_register16(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192));
    VL53_INIT_STEP("dynamic config",
                   write_register(SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01) &&
                   write_register(SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01) &&
                   write_register(SD_CONFIG__QUANTIFIER, 2));
    VL53_INIT_STEP("low power auto config",
                   write_register(SYSTEM__GROUPED_PARAMETER_HOLD, 0x00) &&
                   write_register(SYSTEM__SEED_CONFIG, 1) &&
                   write_register(SYSTEM__SEQUENCE_CONFIG, 0x8B) &&
                   write_register16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8) &&
                   write_register(DSS_CONFIG__ROI_MODE_CONTROL, 2));
    VL53_INIT_STEP("read MM_CONFIG__OUTER_OFFSET_MM", read_register16(MM_CONFIG__OUTER_OFFSET_MM, mm_config_outer_offset_mm));
    VL53_INIT_STEP("setDistanceMode", setDistanceMode(mode));
    VL53_INIT_STEP("setMeasurementTimingBudget", setMeasurementTimingBudget(40000));
    VL53_INIT_STEP("write part-to-part offset", write_register16(ALGO__PART_TO_PART_RANGE_OFFSET_MM, mm_config_outer_offset_mm * 4));
    VL53_INIT_STEP("startContinuous", startContinuous(MEASUREMENT_TIME_MS));

    // call timer() every MEASUREMENT_TIME_MS. We expect new data to be available every MEASUREMENT_TIME_MS
    dev->register_periodic_callback(MEASUREMENT_TIME_MS * 1000,
                                    FUNCTOR_BIND_MEMBER(&AP_RangeFinder_VL53L1X::timer, void));

    vl53_diag("VL53: init success VL53L1X/L3CX");
#undef VL53_INIT_STEP
    return true;
}

// set distance mode to Short, Medium, or Long
// based on VL53L1_SetDistanceMode()
bool AP_RangeFinder_VL53L1X::setDistanceMode(DistanceMode distance_mode)
{
    // save existing timing budget
    uint32_t budget_us = 0;
    if (!getMeasurementTimingBudget(budget_us)) {
        return false;
    }

    switch (distance_mode) {
      case DistanceMode::Short:
            // from VL53L1_preset_mode_standard_ranging_short_range()

            if (!(// timing config
                  write_register(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07) &&
                  write_register(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05) &&
                  write_register(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38) &&

                  // dynamic config
                  write_register(SD_CONFIG__WOI_SD0, 0x07) &&
                  write_register(SD_CONFIG__WOI_SD1, 0x05) &&
                  write_register(SD_CONFIG__INITIAL_PHASE_SD0, 6) && // tuning parm default
                  write_register(SD_CONFIG__INITIAL_PHASE_SD1, 6))) { // tuning parm default
                return false;
            }

            break;

        case DistanceMode::Medium:
            // from VL53L1_preset_mode_standard_ranging()

            if (!(// timing config
                  write_register(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B) &&
                  write_register(RANGE_CONFIG__VCSEL_PERIOD_B, 0x09) &&
                  write_register(RANGE_CONFIG__VALID_PHASE_HIGH, 0x78) &&

                  // dynamic config
                  write_register(SD_CONFIG__WOI_SD0, 0x0B) &&
                  write_register(SD_CONFIG__WOI_SD1, 0x09) &&
                  write_register(SD_CONFIG__INITIAL_PHASE_SD0, 10) && // tuning parm default
                  write_register(SD_CONFIG__INITIAL_PHASE_SD1, 10))) { // tuning parm default
                return false;
            }

            break;

        case DistanceMode::Long:
            // from VL53L1_preset_mode_standard_ranging_long_range()

            if (!(// timing config
                  write_register(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F) &&
                  write_register(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D) &&
                  write_register(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8) &&

                  // dynamic config
                  write_register(SD_CONFIG__WOI_SD0, 0x0F) &&
                  write_register(SD_CONFIG__WOI_SD1, 0x0D) &&
                  write_register(SD_CONFIG__INITIAL_PHASE_SD0, 14) && // tuning parm default
                  write_register(SD_CONFIG__INITIAL_PHASE_SD1, 14))) { // tuning parm default
                return false;
            }

            break;

        default:
            // unrecognized mode - do nothing
            return false;
    }

    // reapply timing budget
    return setMeasurementTimingBudget(budget_us);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement. A longer timing budget allows for more accurate measurements.
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
bool AP_RangeFinder_VL53L1X::setMeasurementTimingBudget(uint32_t budget_us)
{
    // assumes PresetMode is LOWPOWER_AUTONOMOUS
    if (budget_us <= TimingGuard) {
        return false;
    }

    uint32_t range_config_timeout_us = budget_us - TimingGuard;
    if (range_config_timeout_us > 1100000) {
        return false; // FDA_MAX_TIMING_BUDGET_US * 2
    }

    range_config_timeout_us /= 2;

    // VL53L1_calc_timeout_register_values() begin

    uint8_t range_config_vcsel_period = 0;
    if (!read_register(RANGE_CONFIG__VCSEL_PERIOD_A, range_config_vcsel_period)) {
        return false;
    }

    // "Update Macro Period for Range A VCSEL Period"
    uint32_t macro_period_us = calcMacroPeriod(range_config_vcsel_period);

    // "Update Phase timeout - uses Timing A"
    // Timeout of 1000 is tuning parm default (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT)
    // via VL53L1_get_preset_mode_timing_cfg().
    uint32_t phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);
    if (phasecal_timeout_mclks > 0xFF) {
        phasecal_timeout_mclks = 0xFF;
    }

    if (!( write_register(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks) &&

          // "Update MM Timing A timeout"
          // Timeout of 1 is tuning parm default (LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT)
          // via VL53L1_get_preset_mode_timing_cfg(). With the API, the register
          // actually ends up with a slightly different value because it gets assigned,
          // retrieved, recalculated with a different macro period, and reassigned,
          // but it probably doesn't matter because it seems like the MM ("mode
          // mitigation"?) sequence steps are disabled in low power auto mode anyway.
          write_register16(MM_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
              timeoutMicrosecondsToMclks(1, macro_period_us))) &&

          // "Update Range Timing A timeout"
          write_register16(RANGE_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
              timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us))) &&

          // "Update Macro Period for Range B VCSEL Period"
          read_register(RANGE_CONFIG__VCSEL_PERIOD_B, range_config_vcsel_period)
         )) {
        return false;
    }

    // "Update Macro Period for Range B VCSEL Period"
    macro_period_us = calcMacroPeriod(range_config_vcsel_period);

    // "Update MM Timing B timeout"
    // (See earlier comment about MM Timing A timeout.)
    return write_register16(MM_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
               timeoutMicrosecondsToMclks(1, macro_period_us))) &&

           // "Update Range Timing B timeout"
           write_register16(RANGE_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
               timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));
}

// Get the measurement timing budget in microseconds
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
bool AP_RangeFinder_VL53L1X::getMeasurementTimingBudget(uint32_t &budget)
{
    // assumes PresetMode is LOWPOWER_AUTONOMOUS and these sequence steps are
    // enabled: VHV, PHASECAL, DSS1, RANGE

    // "Update Macro Period for Range A VCSEL Period"
    uint8_t range_config_vcsel_period_a = 0;
    if (!read_register(RANGE_CONFIG__VCSEL_PERIOD_A, range_config_vcsel_period_a)) {
        return false;
    }

    uint32_t macro_period_us = calcMacroPeriod(range_config_vcsel_period_a);

    uint16_t timeout_macrop_a = 0;
    if (!read_register16(RANGE_CONFIG__TIMEOUT_MACROP_A, timeout_macrop_a)) {
        return false;
    }

    // "Get Range Timing A timeout"
    uint32_t range_config_timeout_us = timeoutMclksToMicroseconds(decodeTimeout(timeout_macrop_a), macro_period_us);

    budget = 2 * range_config_timeout_us + TimingGuard;
    return true;
}

// Start continuous ranging measurements, with the given inter-measurement
// period in milliseconds determining how often the sensor takes a measurement.
bool AP_RangeFinder_VL53L1X::startContinuous(uint32_t period_ms)
{
    // fix for actual measurement period shorter than set
    uint32_t adjusted_period_ms = period_ms + (period_ms * 64 / 1000);
    const uint32_t period_reg = adjusted_period_ms * osc_calibrate_val;

    // from VL53L1_set_inter_measurement_period_ms()
    vl53_diag("VL53: startContinuous period_ms=%lu adjusted_ms=%lu osc=0x%04x period_reg=0x%08lx",
              (unsigned long)period_ms,
              (unsigned long)adjusted_period_ms,
              (unsigned)osc_calibrate_val,
              (unsigned long)period_reg);

    if (!write_register32(SYSTEM__INTERMEASUREMENT_PERIOD, period_reg)) {
        vl53_diag("VL53: startContinuous write period failed");
        return false;
    }

    uint32_t period_readback = 0;
    if (read_register32(SYSTEM__INTERMEASUREMENT_PERIOD, period_readback)) {
        vl53_diag("VL53: startContinuous period_readback=0x%08lx", (unsigned long)period_readback);
    } else {
        vl53_diag("VL53: startContinuous period_readback failed");
    }

    if (!write_register(SYSTEM__INTERRUPT_CLEAR, 0x01)) { // sys_interrupt_clear_range
        vl53_diag("VL53: startContinuous interrupt clear failed");
        return false;
    }

    if (!write_register(SYSTEM__MODE_START, 0x40)) { // mode_range__timed
        vl53_diag("VL53: startContinuous mode start failed");
        return false;
    }

    uint8_t mode_start = 0;
    if (read_register(SYSTEM__MODE_START, mode_start)) {
        vl53_diag("VL53: startContinuous mode_start=0x%02x", mode_start);
    }

    log_diagnostic_registers("after startContinuous");
    return true;
}

bool AP_RangeFinder_VL53L1X::clearInterruptAndStartNextRange(uint8_t mode_start)
{
    if (!write_register(SYSTEM__INTERRUPT_CLEAR, 0x01)) {
        vl53_diag("VL53: clearInterrupt failed");
        return false;
    }

#if AP_RANGEFINDER_VL53L1X_RESTART_AFTER_CLEAR
    if (!write_register(SYSTEM__MODE_START, mode_start)) {
        vl53_diag("VL53: restart after clear failed mode=0x%02x", mode_start);
        return false;
    }

#if AP_RANGEFINDER_VL53L1X_DIAG_DUMP_EVERY_READ
    uint8_t mode_readback = 0;
    if (read_register(SYSTEM__MODE_START, mode_readback)) {
        vl53_diag("VL53: restart after clear mode_start=0x%02x", mode_readback);
    }
#endif
#endif

    return true;
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L1_decode_timeout()
uint32_t AP_RangeFinder_VL53L1X::decodeTimeout(uint16_t reg_val)
{
    return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L1_encode_timeout()
uint16_t AP_RangeFinder_VL53L1X::encodeTimeout(uint32_t timeout_mclks)
{
    // encoded format: "(LSByte * 2^MSByte) + 1"
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0) {
        ls_byte = timeout_mclks - 1;
        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte >>= 1;
            ms_byte++;
        }
        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    else {
        return 0;
    }
}

// Convert sequence step timeout from macro periods to microseconds with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_us()
uint32_t AP_RangeFinder_VL53L1X::timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
    return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

// Convert sequence step timeout from microseconds to macro periods with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_mclks()
uint32_t AP_RangeFinder_VL53L1X::timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
    return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

// Calculate macro period in microseconds (12.12 format) with given VCSEL period
// assumes fast_osc_frequency has been read and stored
// based on VL53L1_calc_macro_period_us()
uint32_t AP_RangeFinder_VL53L1X::calcMacroPeriod(uint8_t vcsel_period) const
{
    // from VL53L1_calc_pll_period_us()
    // fast osc frequency in 4.12 format; PLL period in 0.24 format
    uint32_t pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency;

    // from VL53L1_decode_vcsel_period()
    uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

    // VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
    uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
    macro_period_us >>= 6;
    macro_period_us *= vcsel_period_pclks;
    macro_period_us >>= 6;

    return macro_period_us;
}

// "Setup ranges after the first one in low power auto mode by turning off
// FW calibration steps and programming static values"
// based on VL53L1_low_power_auto_setup_manual_calibration()
bool AP_RangeFinder_VL53L1X::setupManualCalibration(void)
{
    uint8_t saved_vhv_init = 0;
    uint8_t saved_vhv_timeout = 0;
    uint8_t phasecal_result_vcsel_start = 0;

    return // "save original vhv configs"
           read_register(VHV_CONFIG__INIT, saved_vhv_init) &&
           read_register(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, saved_vhv_timeout) &&

           // "disable VHV init"
           write_register(VHV_CONFIG__INIT, saved_vhv_init & 0x7F) &&

          // "set loop bound to tuning param"
          write_register(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
                         (saved_vhv_timeout & 0x03) + (3 << 2)) && // tuning parm default (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)

          // "override phasecal"
          write_register(PHASECAL_CONFIG__OVERRIDE, 0x01) &&
          read_register(PHASECAL_RESULT__VCSEL_START, phasecal_result_vcsel_start) &&
          write_register(CAL_CONFIG__VCSEL_START, phasecal_result_vcsel_start);
}

// check if sensor has new reading available
// assumes interrupt is active low (GPIO_HV_MUX__CTRL bit 4 is 1)
bool AP_RangeFinder_VL53L1X::dataReady(void)
{
    uint8_t gpio_tio_hv_status = 0;

    return read_register(GPIO__TIO_HV_STATUS, gpio_tio_hv_status) &&
           ((gpio_tio_hv_status & 0x01) == 0);
}

bool AP_RangeFinder_VL53L1X::dataReadyInterruptStatus(uint8_t &interrupt_status)
{
    if (!read_register(RESULT__INTERRUPT_STATUS, interrupt_status)) {
        return false;
    }

    return (interrupt_status & 0x07) != 0;
}

void AP_RangeFinder_VL53L1X::log_result_registers(const char *context)
{
    uint8_t interrupt_status = 0;
    uint8_t range_status = 0;
    uint16_t range_mm = 0;
    const bool ok =
        read_register(RESULT__INTERRUPT_STATUS, interrupt_status) &&
        read_register(RESULT__RANGE_STATUS, range_status) &&
        read_register16(RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, range_mm);

    vl53_diag("VL53: result %s ok=%u int=0x%02x range_status=0x%02x range=%u",
              context,
              (unsigned)ok,
              (unsigned)interrupt_status,
              (unsigned)range_status,
              (unsigned)range_mm);
}

void AP_RangeFinder_VL53L1X::log_diagnostic_registers(const char *context)
{
    uint8_t gpio = 0;
    uint8_t interrupt_status = 0;
    uint8_t range_status = 0;
    uint8_t report_status = 0;
    uint8_t stream_count = 0;
    uint16_t range_mm = 0;
    const bool live_ok =
        read_register(GPIO__TIO_HV_STATUS, gpio) &&
        read_register(RESULT__INTERRUPT_STATUS, interrupt_status) &&
        read_register(RESULT__RANGE_STATUS, range_status) &&
        read_register(RESULT__REPORT_STATUS, report_status) &&
        read_register(RESULT__STREAM_COUNT, stream_count) &&
        read_register16(RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, range_mm);

    vl53_diag("VL53: diag %s live ok=%u gpio=0x%02x int=0x%02x range_status=0x%02x report=0x%02x stream=%u range=%u",
              context,
              (unsigned)live_ok,
              (unsigned)gpio,
              (unsigned)interrupt_status,
              (unsigned)range_status,
              (unsigned)report_status,
              (unsigned)stream_count,
              (unsigned)range_mm);

    uint8_t shadow_interrupt_status = 0;
    uint8_t shadow_range_status = 0;
    uint8_t shadow_report_status = 0;
    uint8_t shadow_stream_count = 0;
    uint16_t shadow_range_mm = 0;
    const bool shadow_ok =
        read_register(SHADOW_RESULT__INTERRUPT_STATUS, shadow_interrupt_status) &&
        read_register(SHADOW_RESULT__RANGE_STATUS, shadow_range_status) &&
        read_register(SHADOW_RESULT__REPORT_STATUS, shadow_report_status) &&
        read_register(SHADOW_RESULT__STREAM_COUNT, shadow_stream_count) &&
        read_register16(SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, shadow_range_mm);

    vl53_diag("VL53: diag %s shadow ok=%u int=0x%02x range_status=0x%02x report=0x%02x stream=%u range=%u",
              context,
              (unsigned)shadow_ok,
              (unsigned)shadow_interrupt_status,
              (unsigned)shadow_range_status,
              (unsigned)shadow_report_status,
              (unsigned)shadow_stream_count,
              (unsigned)shadow_range_mm);

    uint8_t prev_interrupt_status = 0;
    uint8_t prev_range_status = 0;
    uint8_t prev_report_status = 0;
    uint8_t prev_stream_count = 0;
    uint16_t prev_range_mm = 0;
    const bool prev_ok =
        read_register(PREV_SHADOW_RESULT__INTERRUPT_STATUS, prev_interrupt_status) &&
        read_register(PREV_SHADOW_RESULT__RANGE_STATUS, prev_range_status) &&
        read_register(PREV_SHADOW_RESULT__REPORT_STATUS, prev_report_status) &&
        read_register(PREV_SHADOW_RESULT__STREAM_COUNT, prev_stream_count) &&
        read_register16(PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, prev_range_mm);

    vl53_diag("VL53: diag %s prev ok=%u int=0x%02x range_status=0x%02x report=0x%02x stream=%u range=%u",
              context,
              (unsigned)prev_ok,
              (unsigned)prev_interrupt_status,
              (unsigned)prev_range_status,
              (unsigned)prev_report_status,
              (unsigned)prev_stream_count,
              (unsigned)prev_range_mm);
}

bool AP_RangeFinder_VL53L1X::readResult(uint16_t &reading_mm, const char *context)
{
    uint8_t interrupt_status = 0;
    uint8_t range_status = 0;
    uint16_t raw_mm = 0;

    log_result_registers(context);

    if (!(read_register(RESULT__INTERRUPT_STATUS, interrupt_status) &&
          read_register(RESULT__RANGE_STATUS, range_status) &&
          read_register16(RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, raw_mm))) {
        if (vl53_diag_raw_allowed(diag_raw_count)) {
            diag_raw_count++;
            vl53_diag("VL53: distance read failed result registers");
        }
        return false;
    }

    reading_mm = raw_mm;

    // "apply correction gain"
    // gain factor of 2011 is tuning parm default (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT)
    // Basically, this appears to scale the result by 2011/2048, or about 98%
    // (with the 1024 added for proper rounding).
    reading_mm = ((uint32_t)reading_mm * 2011 + 0x0400) / 0x0800;

    if (!clearInterruptAndStartNextRange(0x40)) {
        if (vl53_diag_raw_allowed(diag_raw_count)) {
            diag_raw_count++;
            vl53_diag("VL53: distance read failed interrupt clear/restart");
        }
        return false;
    }

#if AP_RANGEFINDER_VL53L1X_DIAG_PUBLISH_READY_RANGE
    if (((interrupt_status & 0x07) != 0) && (raw_mm > 0)) {
        if (!diag_publish_reported) {
            diag_publish_reported = true;
            vl53_diag("VL53: publishing diagnostic range int=0x%02x status=0x%02x raw=%u corrected=%u",
                      (unsigned)interrupt_status,
                      (unsigned)range_status,
                      (unsigned)raw_mm,
                      (unsigned)reading_mm);
        }
        return true;
    }
#endif

    if ((DeviceError)range_status != RANGECOMPLETE) {
        if (vl53_diag_raw_allowed(diag_raw_count)) {
            diag_raw_count++;
            vl53_diag("VL53: distance raw = %u mm status=%u not complete", (unsigned)raw_mm, (unsigned)range_status);
        }
#if AP_RANGEFINDER_VL53L1X_DIAG_ACCEPT_ANY_RANGE_STATUS
        if (raw_mm == 0 || raw_mm > 4000) {
            return false;
        }
        vl53_diag("VL53: accepting range status=%u for diagnostic raw=%u corrected=%u",
                  (unsigned)range_status,
                  (unsigned)raw_mm,
                  (unsigned)reading_mm);
#else
#ifdef VL53L1X_DEBUG
        hal.console->printf("VL53L1X: %d ms status %d\n", AP_HAL::millis(), (int)range_status);
#endif // VL53L1X_DEBUG
        return false;
#endif
    }

    if (!calibrated) {
        calibrated = setupManualCalibration();
        if (!calibrated && vl53_diag_raw_allowed(diag_raw_count)) {
            diag_raw_count++;
            vl53_diag("VL53: setupManualCalibration failed");
        }
#if AP_RANGEFINDER_VL53L1X_DIAG_ALLOW_UNCALIBRATED
        if (!calibrated) {
            vl53_diag("VL53: accepting uncalibrated range for diagnostic");
        }
#else
        if (!calibrated) {
            return false;
        }
#endif
    }

    if ((calibrated || AP_RANGEFINDER_VL53L1X_DIAG_ALLOW_UNCALIBRATED) &&
        vl53_diag_raw_allowed(diag_raw_count)) {
        diag_raw_count++;
        vl53_diag("VL53: distance raw = %u mm corrected=%u status=%u",
                  (unsigned)raw_mm,
                  (unsigned)reading_mm,
                  (unsigned)range_status);
    }

    return calibrated || AP_RANGEFINDER_VL53L1X_DIAG_ALLOW_UNCALIBRATED;
}

bool AP_RangeFinder_VL53L1X::trySingleShotReading(uint16_t &reading_mm)
{
#if !AP_RANGEFINDER_VL53L1X_DIAG_SINGLE_SHOT_ON_TIMEOUT
    return false;
#else
    vl53_diag("VL53: single-shot start");

    if (!write_register(SYSTEM__INTERRUPT_CLEAR, 0x01)) {
        vl53_diag("VL53: single-shot interrupt clear failed");
        return false;
    }

    if (!write_register(SYSTEM__MODE_START, 0x10)) {
        vl53_diag("VL53: single-shot mode start failed");
        return false;
    }

    for (uint16_t i = 0; i < AP_RANGEFINDER_VL53L1X_DIAG_SINGLE_SHOT_TIMEOUT_MS; i++) {
        uint8_t interrupt_status = 0;
        const bool interrupt_ready = dataReadyInterruptStatus(interrupt_status);
        const bool gpio_ready = dataReady();
        if (interrupt_ready || gpio_ready) {
            vl53_diag("VL53: single-shot ready gpio=%u int=0x%02x",
                      (unsigned)gpio_ready,
                      (unsigned)interrupt_status);
            return readResult(reading_mm, "single-shot ready");
        }

        if ((i % 20) == 0) {
            log_result_registers("single-shot wait");
        }
        hal.scheduler->delay(1);
    }

    vl53_diag("VL53: single-shot timeout");
    log_result_registers("single-shot timeout");
    return readResult(reading_mm, "single-shot timeout");
#endif
}

// read - return last value measured by sensor
bool AP_RangeFinder_VL53L1X::get_reading(uint16_t &reading_mm)
{
    uint8_t tries = 10;
    bool ready = false;
    while (true) {
        uint8_t interrupt_status = 0;
        const bool gpio_ready = dataReady();
        const bool interrupt_ready = dataReadyInterruptStatus(interrupt_status);
        const bool selected_ready =
#if AP_RANGEFINDER_VL53L1X_USE_INTERRUPT_STATUS_READY
            interrupt_ready;
#else
            gpio_ready;
#endif

#if AP_RANGEFINDER_VL53L1X_DIAG_DUMP_EVERY_READ
        if (tries == 10) {
            log_result_registers("poll");
        }
#endif

        if (selected_ready) {
            ready = true;
            if (!gpio_ready && interrupt_ready && vl53_diag_ready_allowed(diag_ready_count)) {
                diag_ready_count++;
                vl53_diag("VL53: ready by RESULT__INTERRUPT_STATUS int=0x%02x", interrupt_status);
            }
            break;
        }

        if (vl53_diag_ready_allowed(diag_ready_count) &&
            ((tries == 10) || (tries == 5) || (tries == 1))) {
            diag_ready_count++;
            log_diagnostic_registers("dataReady wait");
        }
        tries--;
        hal.scheduler->delay(1);
        if (tries == 0) {
            if (vl53_diag_ready_allowed(diag_ready_count)) {
                diag_ready_count++;
                log_diagnostic_registers("dataReady timeout");
            }
            if (vl53_diag_raw_allowed(diag_raw_count)) {
                diag_raw_count++;
                vl53_diag("VL53: distance read failed dataReady timeout");
            }
#if AP_RANGEFINDER_VL53L1X_DIAG_READ_ON_READY_TIMEOUT
            vl53_diag("VL53: dataReady timeout, reading result registers anyway");
            break;
#else
            return false;
#endif
        }
    }

    if (ready && vl53_diag_ready_allowed(diag_ready_count)) {
        diag_ready_count++;
        log_diagnostic_registers("dataReady true");
    }

    if (ready || AP_RANGEFINDER_VL53L1X_DIAG_READ_ON_READY_TIMEOUT) {
        if (readResult(reading_mm, ready ? "ready" : "timeout")) {
            return true;
        }
    }

    return trySingleShotReading(reading_mm);
}

bool AP_RangeFinder_VL53L1X::read_register(uint16_t reg, uint8_t &value)
{
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    return dev->transfer(b, 2, &value, 1);
}

bool AP_RangeFinder_VL53L1X::read_register16(uint16_t reg, uint16_t & value)
{
    uint16_t v = 0;
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    if (!dev->transfer(b, 2, (uint8_t *)&v, 2)) {
        return false;
    }
    value = be16toh(v);
    return true;
}

bool AP_RangeFinder_VL53L1X::read_register32(uint16_t reg, uint32_t &value)
{
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    uint8_t v[4] {};
    if (!dev->transfer(b, 2, v, 4)) {
        return false;
    }
    value = be32toh_ptr(v);
    return true;
}

bool AP_RangeFinder_VL53L1X::write_register(uint16_t reg, uint8_t value)
{
    uint8_t b[3] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF), value };
    return dev->transfer(b, 3, nullptr, 0);
}

bool AP_RangeFinder_VL53L1X::write_register16(uint16_t reg, uint16_t value)
{
    uint8_t b[4] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF), uint8_t(value >> 8), uint8_t(value & 0xFF) };
    return dev->transfer(b, 4, nullptr, 0);
}

bool AP_RangeFinder_VL53L1X::write_register32(uint16_t reg, uint32_t value)
{
    uint8_t b[6] = { uint8_t(reg >> 8),
                     uint8_t(reg & 0xFF),
                     uint8_t((value >> 24) & 0xFF),
                     uint8_t((value >> 16) & 0xFF),
                     uint8_t((value >>  8) & 0xFF),
                     uint8_t((value)       & 0xFF) };
    return dev->transfer(b, 6, nullptr, 0);
}

/*
  timer called at 20Hz
*/
void AP_RangeFinder_VL53L1X::timer(void)
{
    uint16_t range_mm;
    if ((get_reading(range_mm)) && (range_mm <= 4000)) {
        WITH_SEMAPHORE(_sem);
        sum_mm += range_mm;
        counter++;
    }
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_VL53L1X::update(void)
{
    WITH_SEMAPHORE(_sem);
    if (counter > 0) {
        state.distance_m = (sum_mm * 0.001f) / counter;
        state.last_reading_ms = AP_HAL::millis();
        update_status();
        sum_mm = 0;
        counter = 0;
    } else if (AP_HAL::millis() - state.last_reading_ms > 200) {
        // if no updates for 0.2s set no-data
        set_status(RangeFinder::Status::NoData);
    }

    if (!diag_status_reported || diag_last_status != status()) {
        diag_status_reported = true;
        diag_last_status = status();
        if (status() == RangeFinder::Status::Good) {
            vl53_diag("VL53: healthy VL53L1X");
        } else {
            vl53_diag("VL53: unhealthy VL53L1X status=%u", (unsigned)status());
        }
    }
}

#endif  // AP_RANGEFINDER_VL53L1X_ENABLED
