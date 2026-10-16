#include "FlightLoadDiagnostics.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32 && HAL_ESP32_FLIGHT_LOAD_DIAGNOSTICS

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include <float.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern const AP_HAL::HAL &hal;

namespace ESP32 {
namespace FlightLoadDiagnostics {

static constexpr uint16_t RING_SIZE = 512;
static constexpr uint8_t EVENT_SIZE = 32;
// Capture/report policy for the original 150Hz bench experiment only. These
// thresholds never change scheduling, sensor handling or flight control.
static constexpr uint16_t POST_TRIGGER_LOOPS = 225;
static constexpr uint32_t LONG_LOOP_US = 8000;
static constexpr float CRITICAL_RATE_HZ = 135.0f;
static constexpr uint64_t BASELINE_DURATION_US = 30000000ULL;
static constexpr uint8_t MODE_ALT_HOLD = 2;
static constexpr uint8_t MODE_LOITER = 5;

enum class StateId : uint8_t {
    DISARM = 0,
    ARM_ALT_HOLD,
    ARM_LOITER,
    ARM_OTHER,
    COUNT,
};

// All fields below, including compound sensor publications, use state_lock.
// 32bit microsecond differences describe single events shorter than ~71.6min;
// session timestamps and accumulated loop durations are 64bit.
struct SensorState {
    uint32_t pmw_callback_us;
    uint32_t pmw_wait_us;
    uint32_t pmw_spi_us;
    uint32_t pmw_timestamp_us;
    uint32_t pmw_pending_wait_us;
    uint32_t pmw_callback_wait_us;
    uint32_t pmw_spi_accum_us;
    uint32_t pmw_depth;
    bool pmw_overlap;

    uint32_t bmi_dt_us;
    uint32_t bmi_jitter_us;
    uint32_t bmi_timestamp_us;
    uint32_t bmi_previous_us;
    bool bmi_have_previous;

    uint32_t vl53_callback_us;
    uint32_t vl53_i2c_us;
    uint32_t vl53_timestamp_us;
    uint32_t vl53_i2c_accum_us;
    uint16_t vl53_retry_accum;
    uint8_t vl53_retry_count;
    uint8_t vl53_data_ready;
    uint8_t vl53_pending_ready;
    uint32_t vl53_depth;
    bool vl53_overlap;

    uint32_t wifi_io_us;
    uint32_t wifi_timestamp_us;
};

struct Record {
    uint64_t timestamp_us;
    uint32_t loop_dt_us;
    float filtered_hz;
    const char *longest_task_name;
    uint32_t longest_task_us;
    uint32_t pmw_callback_us;
    uint32_t pmw_wait_us;
    uint32_t pmw_spi_us;
    uint32_t bmi_dt_us;
    uint32_t bmi_jitter_us;
    uint32_t vl53_callback_us;
    uint32_t vl53_i2c_us;
    uint32_t wifi_io_us;
    uint8_t vl53_retry_count;
    uint8_t vl53_data_ready;
    uint8_t armed;
    uint8_t flight_mode;
    uint8_t spool_state;
};

struct Summary {
    uint64_t loop_dt_sum_us;
    uint64_t loop_count;
    uint32_t max_loop_us;
    uint64_t long_loop_count;
    float min_filtered_hz;
    float max_filtered_hz;
    const char *max_task_name;
    uint32_t max_task_us;
    uint32_t pmw_callback_max_us;
    uint32_t pmw_wait_max_us;
    uint32_t pmw_spi_max_us;
    uint32_t bmi_dt_max_us;
    uint32_t bmi_jitter_max_us;
    uint32_t vl53_callback_max_us;
    uint32_t vl53_i2c_max_us;
    uint32_t vl53_retry_max;
    uint32_t wifi_io_max_us;
};

struct Session {
    Record ring[RING_SIZE];
    Record events[EVENT_SIZE];
    Summary summaries[uint8_t(StateId::COUNT)];
    uint16_t ring_write;
    uint16_t ring_count;
    uint8_t event_write;
    uint8_t event_count;
    uint16_t post_trigger_remaining;
    bool critical_triggered;
    bool ring_frozen;
    bool dump_pending;
};

static portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;
class StateLock {
public:
    StateLock() { portENTER_CRITICAL(&state_lock); }
    ~StateLock() { portEXIT_CRITICAL(&state_lock); }
    StateLock(const StateLock &) = delete;
    StateLock &operator=(const StateLock &) = delete;
};

// Writers/readers/resetters of shared metadata and sensor state always hold
// StateLock. Main writes the live Session. IO owns the immutable snapshot until
// it releases its slot. No live lock is held for console output or external I/O.
static Session sessions[2];
static constexpr uint8_t NO_SNAPSHOT = 2;
static uint8_t live_index;
static uint8_t snapshot_index = NO_SNAPSHOT;
static bool initialized;
static uint32_t dropped_dumps;
static SensorState sensor;
static bool baseline_done;
static bool baseline_started;
static uint64_t baseline_start_us;
static bool previous_armed;
static bool have_previous_armed;
static uint8_t current_spool_state;
static const char *loop_longest_task_name;
static uint32_t loop_longest_task_us;

static const char *state_name(StateId state)
{
    switch (state) {
    case StateId::DISARM:
        return "DISARM";
    case StateId::ARM_ALT_HOLD:
        return "ARM_ALT_HOLD";
    case StateId::ARM_LOITER:
        return "ARM_LOITER";
    case StateId::ARM_OTHER:
        return "ARM_OTHER";
    case StateId::COUNT:
        break;
    }
    return "UNKNOWN";
}

static StateId state_id(bool armed, uint8_t mode)
{
    if (!armed) {
        return StateId::DISARM;
    }
    if (mode == MODE_ALT_HOLD) {
        return StateId::ARM_ALT_HOLD;
    }
    if (mode == MODE_LOITER) {
        return StateId::ARM_LOITER;
    }
    return StateId::ARM_OTHER;
}

// O(1) metadata reset: never clear/copy the 512-entry ring while locked.
// Entries outside ring_count/event_count are not read.
static void reset_session_locked(Session &session)
{
    session.ring_write = 0;
    session.ring_count = 0;
    session.event_write = 0;
    session.event_count = 0;
    session.post_trigger_remaining = 0;
    session.critical_triggered = false;
    session.ring_frozen = false;
    session.dump_pending = false;
    for (auto &summary : session.summaries) {
        summary = {};
        summary.min_filtered_hz = FLT_MAX;
    }
}

static void initialize_locked()
{
    if (!initialized) {
        reset_session_locked(sessions[live_index]);
        initialized = true;
    }
}

static uint32_t add_saturated(uint32_t a, uint32_t b)
{
    return UINT32_MAX - a < b ? UINT32_MAX : a + b;
}

// Constant-time ownership handoff, not a large memcpy. Call only with no
// outstanding snapshot. The other slot is free and becomes the new live state.
static void detach_snapshot_locked()
{
    snapshot_index = live_index;
    live_index ^= 1U;
    reset_session_locked(sessions[live_index]);
}

void scheduler_loop_begin()
{
    StateLock lock;
    loop_longest_task_name = nullptr;
    loop_longest_task_us = 0;
}

void scheduler_task(uint8_t, const char *task_name, uint32_t duration_us)
{
    StateLock lock;
    if (duration_us > loop_longest_task_us) {
        loop_longest_task_us = duration_us;
        loop_longest_task_name = task_name;
    }
}

void set_spool_state(uint8_t spool_state)
{
    StateLock lock;
    current_spool_state = spool_state;
}

static Record make_record_locked(uint64_t now_us, uint32_t loop_dt_us,
                                 float filtered_hz, bool armed, uint8_t flight_mode)
{
    Record record {};
    record.timestamp_us = now_us;
    record.loop_dt_us = loop_dt_us;
    record.filtered_hz = filtered_hz;
    record.longest_task_name = loop_longest_task_name;
    record.longest_task_us = loop_longest_task_us;
    record.pmw_callback_us = sensor.pmw_callback_us;
    record.pmw_wait_us = sensor.pmw_wait_us;
    record.pmw_spi_us = sensor.pmw_spi_us;
    record.bmi_dt_us = sensor.bmi_dt_us;
    record.bmi_jitter_us = sensor.bmi_jitter_us;
    record.vl53_callback_us = sensor.vl53_callback_us;
    record.vl53_i2c_us = sensor.vl53_i2c_us;
    record.wifi_io_us = sensor.wifi_io_us;
    sensor.wifi_io_us = 0;
    record.vl53_retry_count = sensor.vl53_retry_count;
    record.vl53_data_ready = sensor.vl53_data_ready;
    record.armed = armed;
    record.flight_mode = flight_mode;
    record.spool_state = current_spool_state;
    return record;
}

static void update_summary_locked(Session &session, const Record &record)
{
    Summary &summary = session.summaries[uint8_t(state_id(record.armed != 0, record.flight_mode))];
    summary.loop_count++;
    summary.loop_dt_sum_us += record.loop_dt_us;
    summary.max_loop_us = MAX(summary.max_loop_us, record.loop_dt_us);
    if (record.loop_dt_us > LONG_LOOP_US) {
        summary.long_loop_count++;
    }
    summary.min_filtered_hz = MIN(summary.min_filtered_hz, record.filtered_hz);
    summary.max_filtered_hz = MAX(summary.max_filtered_hz, record.filtered_hz);
    if (record.longest_task_us > summary.max_task_us) {
        summary.max_task_us = record.longest_task_us;
        summary.max_task_name = record.longest_task_name;
    }
    summary.pmw_callback_max_us = MAX(summary.pmw_callback_max_us, record.pmw_callback_us);
    summary.pmw_wait_max_us = MAX(summary.pmw_wait_max_us, record.pmw_wait_us);
    summary.pmw_spi_max_us = MAX(summary.pmw_spi_max_us, record.pmw_spi_us);
    summary.bmi_dt_max_us = MAX(summary.bmi_dt_max_us, record.bmi_dt_us);
    summary.bmi_jitter_max_us = MAX(summary.bmi_jitter_max_us, record.bmi_jitter_us);
    summary.vl53_callback_max_us = MAX(summary.vl53_callback_max_us, record.vl53_callback_us);
    summary.vl53_i2c_max_us = MAX(summary.vl53_i2c_max_us, record.vl53_i2c_us);
    summary.vl53_retry_max = MAX(summary.vl53_retry_max, uint32_t(record.vl53_retry_count));
    summary.wifi_io_max_us = MAX(summary.wifi_io_max_us, record.wifi_io_us);
}

static void save_event_locked(Session &session, const Record &record)
{
    session.events[session.event_write] = record;
    session.event_write = (session.event_write + 1U) % EVENT_SIZE;
    if (session.event_count < EVENT_SIZE) {
        session.event_count++;
    }
}

static void save_ring_locked(Session &session, const Record &record)
{
    if (session.ring_frozen) {
        return;
    }
    session.ring[session.ring_write] = record;
    session.ring_write = (session.ring_write + 1U) % RING_SIZE;
    if (session.ring_count < RING_SIZE) {
        session.ring_count++;
    }
    const bool critical_now = record.filtered_hz < CRITICAL_RATE_HZ || record.loop_dt_us > LONG_LOOP_US;
    if (!session.critical_triggered && critical_now) {
        session.critical_triggered = true;
        session.post_trigger_remaining = POST_TRIGGER_LOOPS;
    } else if (session.critical_triggered && session.post_trigger_remaining > 0) {
        session.post_trigger_remaining--;
        if (session.post_trigger_remaining == 0) {
            session.ring_frozen = true;
        }
    }
}

void scheduler_loop_end(uint32_t loop_dt_us, float filtered_loop_rate_hz,
                        bool armed, uint8_t flight_mode)
{
    const uint64_t now_us = AP_HAL::micros64();
    StateLock lock;
    initialize_locked();

    if (armed && (!have_previous_armed || !previous_armed)) {
        if (sessions[live_index].dump_pending && snapshot_index == NO_SNAPSHOT) {
            detach_snapshot_locked();
        } else {
            if (sessions[live_index].dump_pending) {
                // Two-slot policy: one immutable dump plus one pending live
                // session. If another ARM arrives before that pending dump can
                // be detached, drop that completed dump explicitly, not the
                // active snapshot or the new recording. No unbounded queue.
                dropped_dumps = add_saturated(dropped_dumps, 1);
            }
            reset_session_locked(sessions[live_index]);
        }
    }
    Session &session = sessions[live_index];
    if (!armed && session.dump_pending) {
        previous_armed = false;
        have_previous_armed = true;
        return;
    }

    const Record record = make_record_locked(now_us, loop_dt_us, filtered_loop_rate_hz, armed, flight_mode);
    update_summary_locked(session, record);
    if (loop_dt_us > LONG_LOOP_US || filtered_loop_rate_hz < CRITICAL_RATE_HZ) {
        save_event_locked(session, record);
    }
    save_ring_locked(session, record);
    if (!armed && have_previous_armed && previous_armed) {
        session.ring_frozen = true;
        session.dump_pending = true;
    }
    if (!armed && !baseline_done && !session.dump_pending) {
        if (!baseline_started) {
            baseline_started = true;
            baseline_start_us = now_us;
        } else if (now_us - baseline_start_us >= BASELINE_DURATION_US) {
            baseline_done = true;
            session.ring_frozen = true;
            session.dump_pending = true;
        }
    }
    previous_armed = armed;
    have_previous_armed = true;
}

void pmw_semaphore_wait(uint32_t wait_us)
{
    StateLock lock;
    sensor.pmw_pending_wait_us = wait_us;
}

void pmw_callback_begin()
{
    StateLock lock;
    if (sensor.pmw_depth == 0) {
        sensor.pmw_spi_accum_us = 0;
        sensor.pmw_callback_wait_us = sensor.pmw_pending_wait_us;
        sensor.pmw_overlap = false;
    } else {
        sensor.pmw_overlap = true;
    }
    sensor.pmw_depth = add_saturated(sensor.pmw_depth, 1);
}

void pmw_spi_transaction(uint32_t transaction_us)
{
    StateLock lock;
    if (sensor.pmw_depth == 1 && !sensor.pmw_overlap) {
        sensor.pmw_spi_accum_us = add_saturated(sensor.pmw_spi_accum_us, transaction_us);
    }
}

void pmw_callback_end(uint32_t callback_us)
{
    const uint32_t now_us = AP_HAL::micros();
    StateLock lock;
    if (sensor.pmw_depth == 0 || --sensor.pmw_depth != 0 || sensor.pmw_overlap) {
        return;
    }
    sensor.pmw_callback_us = callback_us;
    sensor.pmw_wait_us = sensor.pmw_callback_wait_us;
    sensor.pmw_spi_us = sensor.pmw_spi_accum_us;
    sensor.pmw_timestamp_us = now_us;
}

void bmi_sample(uint32_t timestamp_us, uint32_t expected_interval_us)
{
    StateLock lock;
    if (sensor.bmi_have_previous) {
        const uint32_t dt_us = timestamp_us - sensor.bmi_previous_us;
        sensor.bmi_dt_us = dt_us;
        sensor.bmi_jitter_us = dt_us > expected_interval_us ?
            dt_us - expected_interval_us : expected_interval_us - dt_us;
    }
    sensor.bmi_previous_us = timestamp_us;
    sensor.bmi_have_previous = true;
    sensor.bmi_timestamp_us = timestamp_us;
}

void vl53_callback_begin()
{
    StateLock lock;
    if (sensor.vl53_depth == 0) {
        sensor.vl53_i2c_accum_us = 0;
        sensor.vl53_retry_accum = 0;
        sensor.vl53_pending_ready = 0;
        sensor.vl53_overlap = false;
    } else {
        sensor.vl53_overlap = true;
    }
    sensor.vl53_depth = add_saturated(sensor.vl53_depth, 1);
}

void vl53_i2c_transfer(uint8_t address, uint32_t elapsed_us,
                       uint8_t retry_count, bool is_read)
{
    StateLock lock;
    if (sensor.vl53_depth != 1 || sensor.vl53_overlap || address != 0x29) {
        return;
    }
    if (is_read) {
        sensor.vl53_i2c_accum_us = add_saturated(sensor.vl53_i2c_accum_us, elapsed_us);
    }
    sensor.vl53_retry_accum = MIN(uint16_t(sensor.vl53_retry_accum + retry_count), uint16_t(UINT8_MAX));
}

void vl53_data_ready(bool ready)
{
    StateLock lock;
    if (sensor.vl53_depth == 1 && !sensor.vl53_overlap) {
        sensor.vl53_pending_ready = ready;
    }
}

void vl53_callback_end(uint32_t callback_us)
{
    const uint32_t now_us = AP_HAL::micros();
    StateLock lock;
    if (sensor.vl53_depth == 0 || --sensor.vl53_depth != 0 || sensor.vl53_overlap) {
        return;
    }
    sensor.vl53_callback_us = callback_us;
    sensor.vl53_i2c_us = sensor.vl53_i2c_accum_us;
    sensor.vl53_retry_count = sensor.vl53_retry_accum;
    sensor.vl53_data_ready = sensor.vl53_pending_ready;
    sensor.vl53_timestamp_us = now_us;
}

void wifi_io(uint32_t elapsed_us)
{
    const uint32_t now_us = AP_HAL::micros();
    StateLock lock;
    sensor.wifi_io_us = MAX(sensor.wifi_io_us, elapsed_us);
    sensor.wifi_timestamp_us = now_us;
}

static void print_record(const char *prefix, const Record &record)
{
    hal.console->printf(
        "%s t=%llu dt=%lu hz=%.1f arm=%u mode=%u spool=%u task=%s task_us=%lu "
        "pmw_cb=%lu pmw_wait=%lu pmw_spi=%lu bmi_dt=%lu bmi_jit=%lu "
        "vl_cb=%lu vl_i2c=%lu vl_retry=%u ready=%u wifi=%lu\n",
        prefix,
        (unsigned long long)record.timestamp_us,
        (unsigned long)record.loop_dt_us,
        (double)record.filtered_hz,
        (unsigned)record.armed,
        (unsigned)record.flight_mode,
        (unsigned)record.spool_state,
        record.longest_task_name != nullptr ? record.longest_task_name : "none",
        (unsigned long)record.longest_task_us,
        (unsigned long)record.pmw_callback_us,
        (unsigned long)record.pmw_wait_us,
        (unsigned long)record.pmw_spi_us,
        (unsigned long)record.bmi_dt_us,
        (unsigned long)record.bmi_jitter_us,
        (unsigned long)record.vl53_callback_us,
        (unsigned long)record.vl53_i2c_us,
        (unsigned)record.vl53_retry_count,
        (unsigned)record.vl53_data_ready,
        (unsigned long)record.wifi_io_us);
}

static void print_summary(StateId state, const Summary &summary)
{
    if (summary.loop_count == 0) {
        return;
    }
    const double average_hz = summary.loop_dt_sum_us > 0 ?
        double(summary.loop_count) * 1000000.0 / summary.loop_dt_sum_us : 0.0f;
    hal.console->printf(
        "[FLD-SUM] state=%s loops=%llu minHz=%.1f avgHz=%.1f maxHz=%.1f max_loop_us=%lu long_loops=%llu "
        "max_task=%s max_task_us=%lu pmw_cb_max=%lu pmw_wait_max=%lu pmw_spi_max=%lu "
        "bmi_dt_max=%lu bmi_jit_max=%lu vl_cb_max=%lu vl_i2c_max=%lu "
        "vl_retry_max=%lu wifi_max=%lu\n",
        state_name(state),
        (unsigned long long)summary.loop_count,
        (double)summary.min_filtered_hz,
        (double)average_hz,
        (double)summary.max_filtered_hz,
        (unsigned long)summary.max_loop_us,
        (unsigned long long)summary.long_loop_count,
        summary.max_task_name != nullptr ? summary.max_task_name : "none",
        (unsigned long)summary.max_task_us,
        (unsigned long)summary.pmw_callback_max_us,
        (unsigned long)summary.pmw_wait_max_us,
        (unsigned long)summary.pmw_spi_max_us,
        (unsigned long)summary.bmi_dt_max_us,
        (unsigned long)summary.bmi_jitter_max_us,
        (unsigned long)summary.vl53_callback_max_us,
        (unsigned long)summary.vl53_i2c_max_us,
        (unsigned long)summary.vl53_retry_max,
        (unsigned long)summary.wifi_io_max_us);
}

// IO-task-only cursor. Unlike shared live metadata it needs no cross-task
// synchronization: service_output has exactly one caller/context.
enum class OutputPhase : uint8_t { BEGIN, SUMMARY, EVENT, RING, END };
struct OutputCursor {
    bool active;
    OutputPhase phase;
    uint16_t index;
    uint32_t dropped;
};
static OutputCursor output;

void service_output()
{
    // Never take external locks (including console locks) under StateLock.
    // Pause a dump while armed, resuming the same immutable snapshot later.
    if (hal.util->get_soft_armed()) {
        return;
    }
    const Session *snapshot;
    {
        StateLock lock;
        initialize_locked();
        if (snapshot_index == NO_SNAPSHOT && sessions[live_index].dump_pending) {
            detach_snapshot_locked();
        }
        if (snapshot_index == NO_SNAPSHOT) {
            return;
        }
        snapshot = &sessions[snapshot_index];
        if (!output.active) {
            output.active = true;
            output.phase = OutputPhase::BEGIN;
            output.index = 0;
            output.dropped = dropped_dumps;
        }
    }

    // At most one line per IO pass; no large dump blocks the whole IO task.
    // Main/sensor writers can update the other slot throughout console output.
    switch (output.phase) {
    case OutputPhase::BEGIN:
        hal.console->printf("[FLD-BEGIN] critical=%u ring=%u events=%u dropped=%lu\n",
                            (unsigned)snapshot->critical_triggered,
                            (unsigned)snapshot->ring_count,
                            (unsigned)snapshot->event_count,
                            (unsigned long)output.dropped);
        output.phase = OutputPhase::SUMMARY;
        return;
    case OutputPhase::SUMMARY:
        while (output.index < uint8_t(StateId::COUNT)) {
            const uint8_t i = output.index++;
            if (snapshot->summaries[i].loop_count > 0) {
                print_summary(StateId(i), snapshot->summaries[i]);
                return;
            }
        }
        output.phase = OutputPhase::EVENT;
        output.index = 0;
        return;
    case OutputPhase::EVENT:
        if (output.index < snapshot->event_count) {
            const uint8_t first = snapshot->event_count < EVENT_SIZE ? 0 : snapshot->event_write;
            print_record("[FLD-EVENT]", snapshot->events[(first + output.index++) % EVENT_SIZE]);
            return;
        }
        output.phase = OutputPhase::RING;
        output.index = 0;
        return;
    case OutputPhase::RING:
        if (snapshot->critical_triggered && output.index < snapshot->ring_count) {
            const uint16_t first = snapshot->ring_count < RING_SIZE ? 0 : snapshot->ring_write;
            print_record("[FLD-RING]", snapshot->ring[(first + output.index++) % RING_SIZE]);
            return;
        }
        output.phase = OutputPhase::END;
        return;
    case OutputPhase::END:
        hal.console->printf("[FLD-END]\n");
        {
            StateLock lock;
            // Only release the old snapshot. Never reset a new live session.
            snapshot_index = NO_SNAPSHOT;
        }
        output = {};
        return;
    }
}

} // namespace FlightLoadDiagnostics
} // namespace ESP32

#endif
