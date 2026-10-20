#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <stdint.h>

#ifndef HAL_ESP32_EKF3_BLOCK_DIAGNOSTICS
#define HAL_ESP32_EKF3_BLOCK_DIAGNOSTICS 0
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32 && HAL_ESP32_EKF3_BLOCK_DIAGNOSTICS

namespace ESP32 {
namespace EKF3BlockDiagnostics {

static constexpr uint8_t BLOCK_COUNT = 6;

// Developer diagnostics only: measured elapsed sections, not a complete
// UpdateFilter partition. Sections 1..6 cover input/control, strapdown/covariance,
// yaw/mag/vel-pos, SelectFlowFusion, other prediction/fusion work, and output.
// Section 5 combines non-contiguous regions; later health/reset work is outside
// these measurements. No sample age, semaphore wait or exact FuseOptFlow
// duration is recorded. uint32 micros subtraction assumes a section < 71.6 min.

void scheduler_loop_begin();
void scheduler_loop_end(uint32_t loop_dt_us, float filtered_loop_rate_hz,
                        bool armed, uint8_t flight_mode);

void set_metadata(uint8_t num_cores, uint8_t primary_core,
                  uint8_t imu_mask, uint8_t frames_per_prediction);

// One call per EKF3 core update. Only the current primary core is accumulated.
void record_update(uint8_t core_index, bool allow_state_prediction,
                   bool states_initialised, bool run_updates,
                   const uint32_t block_us[BLOCK_COUNT]);

// Function entry counts, including early returns. really_fuse counts its entry
// flag, not innovation-gate acceptance or successful state updates.
void flow_selection(uint8_t core_index);
void flow_fusion(uint8_t core_index, bool really_fuse);

// Single IO-task caller, at most one console line per call. Pauses while armed;
// one console call already in flight may finish after ARM. Snapshot completion
// only releases its slot, never resets live recording. Two slots permit one
// immutable snapshot plus one pending live dump; a new ARM drops that pending
// dump if no slot is free, incrementing a saturating lifetime dropped counter.
void service_output();

} // namespace EKF3BlockDiagnostics
} // namespace ESP32

#endif
