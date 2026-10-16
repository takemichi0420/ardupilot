#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <stdint.h>

#ifndef HAL_ESP32_FLIGHT_LOAD_DIAGNOSTICS
#define HAL_ESP32_FLIGHT_LOAD_DIAGNOSTICS 0
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32 && HAL_ESP32_FLIGHT_LOAD_DIAGNOSTICS

namespace ESP32 {
namespace FlightLoadDiagnostics {

// Task-context APIs. Shared state is protected internally; no API acquires a
// bus/backend semaphore or holds the diagnostic lock across an I/O operation.
// Callback begin/end calls must be balanced. Overlapping callbacks of the same
// sensor type are discarded rather than publishing a mixed measurement.
void scheduler_loop_begin();
void scheduler_task(uint8_t task_index, const char *task_name, uint32_t duration_us);
void scheduler_loop_end(uint32_t loop_dt_us, float filtered_loop_rate_hz,
                        bool armed, uint8_t flight_mode);
void set_spool_state(uint8_t spool_state);

void pmw_semaphore_wait(uint32_t wait_us);
void pmw_callback_begin();
void pmw_spi_transaction(uint32_t transaction_us);
void pmw_callback_end(uint32_t callback_us);

void bmi_sample(uint32_t timestamp_us, uint32_t expected_interval_us);

void vl53_callback_begin();
void vl53_i2c_transfer(uint8_t address, uint32_t elapsed_us,
                       uint8_t retry_count, bool is_read);
void vl53_data_ready(bool ready);
void vl53_callback_end(uint32_t callback_us);

void wifi_io(uint32_t elapsed_us);

// Called only from the low-priority ESP32 IO task. Emits at most one snapshot
// line per call, starting/resuming only while disarmed. A console call already
// in progress can finish after arming; live recording remains independent.
void service_output();

} // namespace FlightLoadDiagnostics
} // namespace ESP32

#endif
