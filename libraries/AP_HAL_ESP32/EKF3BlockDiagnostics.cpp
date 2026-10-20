#include "EKF3BlockDiagnostics.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32 && HAL_ESP32_EKF3_BLOCK_DIAGNOSTICS

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include <float.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern const AP_HAL::HAL &hal;

namespace ESP32 {
namespace EKF3BlockDiagnostics {

// Developer capture policy, never EKF control thresholds. The baseline sums
// DISARM loop intervals across sessions; it is not continuous wall time.
static constexpr uint64_t BASELINE_DURATION_US = 30000000ULL;
// Copter mode IDs used only to categorize diagnostic summaries.
static constexpr uint8_t MODE_ALT_HOLD = 2;
static constexpr uint8_t MODE_LOITER = 5;

enum class StateId : uint8_t {
    DISARM = 0,
    ARM_ALT_HOLD,
    ARM_LOITER,
    ARM_OTHER,
    COUNT,
};

struct BlockTiming {
    uint64_t total_us;
    uint32_t max_us;
    uint32_t executed;
    uint32_t over_500us;
    uint32_t over_1000us;
    uint32_t over_2000us;
};

struct RunSummary {
    uint32_t calls;
    BlockTiming blocks[BLOCK_COUNT];
};

struct StateSummary {
    uint64_t loop_dt_sum_us;
    uint32_t loop_count;
    uint32_t max_loop_us;
    float min_filtered_hz;
    float max_filtered_hz;
    uint32_t update_calls;
    uint32_t initialised_calls;
    uint32_t allow_prediction_false;
    uint32_t flow_selection_calls;
    uint32_t fuse_optflow_calls;
    uint32_t fuse_optflow_real_calls;
    RunSummary run[2];
};

struct Metadata {
    bool valid;
    uint8_t num_cores;
    uint8_t primary_core;
    uint8_t imu_mask;
    uint8_t frames_per_prediction;
};

struct PendingLoop {
    Metadata metadata;
    uint32_t update_calls;
    uint32_t initialised_calls;
    uint32_t allow_prediction_false;
    uint32_t flow_selection_calls;
    uint32_t fuse_optflow_calls;
    uint32_t fuse_optflow_real_calls;
    RunSummary run[2];
};

struct Session {
    StateSummary summaries[uint8_t(StateId::COUNT)];
    Metadata metadata;
    uint32_t primary_switches;
    bool have_primary;
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

// All mutable shared state below is accessed under StateLock. Main/EKF owns
// the live aggregate and per-loop events. Detaching a slot transfers immutable
// ownership to the single IO reader; only IO may release that snapshot slot.
static Session sessions[2];
static constexpr uint8_t NO_SNAPSHOT = 2;
static uint8_t live_index;
static uint8_t snapshot_index = NO_SNAPSHOT;
static PendingLoop pending;
static Metadata current_metadata;
static bool previous_armed;
static bool have_previous_armed;
static bool initialized;
static bool baseline_done;
static uint64_t baseline_elapsed_us;
static uint32_t dropped_dumps;

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

static void reset_session_locked(Session &session)
{
    session = {};
    for (auto &summary : session.summaries) {
        summary.min_filtered_hz = FLT_MAX;
    }
}

static void initialize_locked()
{
    if (!initialized) {
        live_index = 0;
        snapshot_index = NO_SNAPSHOT;
        reset_session_locked(sessions[0]);
        reset_session_locked(sessions[1]);
        pending = {};
        current_metadata = {};
        previous_armed = false;
        have_previous_armed = false;
        baseline_done = false;
        baseline_elapsed_us = 0;
        dropped_dumps = 0;
        initialized = true;
    }
}

static uint32_t add_saturated(uint32_t a, uint32_t b)
{
    return UINT32_MAX - a < b ? UINT32_MAX : a + b;
}

// Caller holds StateLock and there is no outstanding snapshot. Switch slot
// ownership without copying a report; reset only the other, now-live slot.
static void detach_snapshot_locked()
{
    snapshot_index = live_index;
    live_index ^= 1U;
    reset_session_locked(sessions[live_index]);
}

void scheduler_loop_begin()
{
    StateLock lock;
    initialize_locked();
    pending = {};
    pending.metadata = current_metadata;
}

void set_metadata(uint8_t num_cores, uint8_t primary_core,
                  uint8_t imu_mask, uint8_t frames_per_prediction)
{
    StateLock lock;
    initialize_locked();
    current_metadata.valid = true;
    current_metadata.num_cores = num_cores;
    current_metadata.primary_core = primary_core;
    current_metadata.imu_mask = imu_mask;
    current_metadata.frames_per_prediction = frames_per_prediction;
    pending.metadata = current_metadata;
}

static void add_timing(BlockTiming &timing, uint32_t elapsed_us)
{
    timing.total_us += elapsed_us;
    timing.max_us = MAX(timing.max_us, elapsed_us);
    timing.executed = add_saturated(timing.executed, 1);
    // Strictly-greater diagnostic buckets, not warning/control decisions.
    timing.over_500us = add_saturated(timing.over_500us, elapsed_us > 500U);
    timing.over_1000us = add_saturated(timing.over_1000us, elapsed_us > 1000U);
    timing.over_2000us = add_saturated(timing.over_2000us, elapsed_us > 2000U);
}

void record_update(uint8_t core_index, bool allow_state_prediction,
                   bool states_initialised, bool run_updates,
                   const uint32_t block_us[BLOCK_COUNT])
{
    StateLock lock;
    initialize_locked();
    if (!current_metadata.valid || core_index != current_metadata.primary_core) {
        return;
    }

    pending.update_calls = add_saturated(pending.update_calls, 1);
    pending.allow_prediction_false = add_saturated(pending.allow_prediction_false, !allow_state_prediction);
    if (!states_initialised) {
        return;
    }

    pending.initialised_calls = add_saturated(pending.initialised_calls, 1);
    RunSummary &run = pending.run[run_updates ? 1U : 0U];
    run.calls = add_saturated(run.calls, 1);
    add_timing(run.blocks[0], block_us[0]);
    if (run_updates) {
        for (uint8_t i = 1; i <= 4; i++) {
            add_timing(run.blocks[i], block_us[i]);
        }
    }
    add_timing(run.blocks[5], block_us[5]);
}

void flow_selection(uint8_t core_index)
{
    StateLock lock;
    initialize_locked();
    if (current_metadata.valid && core_index == current_metadata.primary_core) {
        pending.flow_selection_calls = add_saturated(pending.flow_selection_calls, 1);
    }
}

void flow_fusion(uint8_t core_index, bool really_fuse)
{
    StateLock lock;
    initialize_locked();
    if (current_metadata.valid && core_index == current_metadata.primary_core) {
        pending.fuse_optflow_calls = add_saturated(pending.fuse_optflow_calls, 1);
        pending.fuse_optflow_real_calls = add_saturated(pending.fuse_optflow_real_calls, really_fuse);
    }
}

static void merge_timing(BlockTiming &destination, const BlockTiming &source)
{
    destination.total_us += source.total_us;
    destination.max_us = MAX(destination.max_us, source.max_us);
    destination.executed = add_saturated(destination.executed, source.executed);
    destination.over_500us = add_saturated(destination.over_500us, source.over_500us);
    destination.over_1000us = add_saturated(destination.over_1000us, source.over_1000us);
    destination.over_2000us = add_saturated(destination.over_2000us, source.over_2000us);
}

static void merge_pending(StateSummary &summary)
{
    summary.update_calls = add_saturated(summary.update_calls, pending.update_calls);
    summary.initialised_calls = add_saturated(summary.initialised_calls, pending.initialised_calls);
    summary.allow_prediction_false = add_saturated(summary.allow_prediction_false, pending.allow_prediction_false);
    summary.flow_selection_calls = add_saturated(summary.flow_selection_calls, pending.flow_selection_calls);
    summary.fuse_optflow_calls = add_saturated(summary.fuse_optflow_calls, pending.fuse_optflow_calls);
    summary.fuse_optflow_real_calls = add_saturated(summary.fuse_optflow_real_calls, pending.fuse_optflow_real_calls);
    for (uint8_t run_index = 0; run_index < 2; run_index++) {
        summary.run[run_index].calls = add_saturated(summary.run[run_index].calls, pending.run[run_index].calls);
        for (uint8_t block = 0; block < BLOCK_COUNT; block++) {
            merge_timing(summary.run[run_index].blocks[block],
                         pending.run[run_index].blocks[block]);
        }
    }
}

void scheduler_loop_end(uint32_t loop_dt_us, float filtered_loop_rate_hz,
                        bool armed, uint8_t flight_mode)
{
    StateLock lock;
    initialize_locked();

    if (armed && (!have_previous_armed || !previous_armed)) {
        if (sessions[live_index].dump_pending && snapshot_index == NO_SNAPSHOT) {
            detach_snapshot_locked();
        } else {
            if (sessions[live_index].dump_pending) {
                // At most one immutable snapshot plus one completed live
                // session awaiting a slot. A new flight supersedes the latter
                // when both slots are occupied. Never overwrite the snapshot.
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

    if (pending.metadata.valid) {
        if (session.have_primary &&
            pending.metadata.primary_core != session.metadata.primary_core) {
            session.primary_switches = add_saturated(session.primary_switches, 1);
        }
        session.metadata = pending.metadata;
        session.have_primary = true;
    }

    StateSummary &summary = session.summaries[uint8_t(state_id(armed, flight_mode))];
    summary.loop_count = add_saturated(summary.loop_count, 1);
    summary.loop_dt_sum_us += loop_dt_us;
    summary.max_loop_us = MAX(summary.max_loop_us, loop_dt_us);
    summary.min_filtered_hz = MIN(summary.min_filtered_hz, filtered_loop_rate_hz);
    summary.max_filtered_hz = MAX(summary.max_filtered_hz, filtered_loop_rate_hz);
    merge_pending(summary);

    if (!armed && have_previous_armed && previous_armed) {
        session.dump_pending = true;
    } else if (!armed && !baseline_done && !session.dump_pending) {
        baseline_elapsed_us += loop_dt_us;
        if (baseline_elapsed_us >= BASELINE_DURATION_US) {
            baseline_done = true;
            session.dump_pending = true;
        }
    }

    previous_armed = armed;
    have_previous_armed = true;
    if (session.dump_pending && snapshot_index == NO_SNAPSHOT) {
        detach_snapshot_locked();
    }
}

static void print_state(StateId state, const StateSummary &summary)
{
    if (summary.loop_count == 0) {
        return;
    }

    const float average_hz = summary.loop_dt_sum_us > 0 ?
        summary.loop_count * 1000000.0f / summary.loop_dt_sum_us : 0.0f;
    const uint64_t run_total = uint64_t(summary.run[0].calls) + summary.run[1].calls;
    const float run_true_percent = run_total > 0 ?
        summary.run[1].calls * 100.0f / run_total : 0.0f;

    hal.console->printf(
        "[EKF3-STATE] state=%s loops=%lu minHz=%.1f avgHz=%.1f maxHz=%.1f max_loop_us=%lu "
        "updates=%lu initialised=%lu run_false=%lu run_true=%lu run_true_pct=%.1f "
        "allow_prediction_false=%lu flow_select=%lu fuse_optflow=%lu really_fuse_flag=%lu\n",
        state_name(state),
        (unsigned long)summary.loop_count,
        (double)summary.min_filtered_hz,
        (double)average_hz,
        (double)summary.max_filtered_hz,
        (unsigned long)summary.max_loop_us,
        (unsigned long)summary.update_calls,
        (unsigned long)summary.initialised_calls,
        (unsigned long)summary.run[0].calls,
        (unsigned long)summary.run[1].calls,
        (double)run_true_percent,
        (unsigned long)summary.allow_prediction_false,
        (unsigned long)summary.flow_selection_calls,
        (unsigned long)summary.fuse_optflow_calls,
        (unsigned long)summary.fuse_optflow_real_calls);
}

static void print_block(StateId state, const RunSummary &run, uint8_t run_index, uint8_t block)
{
    const BlockTiming &timing = run.blocks[block];
    const float average_us = timing.executed > 0 ?
        float(timing.total_us) / timing.executed : 0.0f;
    hal.console->printf(
        "[EKF3-BLOCK] state=%s run=%u block=%u calls=%lu exec=%lu total_us=%llu "
        "avg_us=%.1f max_us=%lu gt500=%lu gt1000=%lu gt2000=%lu\n",
        state_name(state),
        (unsigned)run_index,
        (unsigned)(block + 1U),
        (unsigned long)run.calls,
        (unsigned long)timing.executed,
        (unsigned long long)timing.total_us,
        (double)average_us,
        (unsigned long)timing.max_us,
        (unsigned long)timing.over_500us,
        (unsigned long)timing.over_1000us,
        (unsigned long)timing.over_2000us);
}

// Only the IO task reads/writes this cursor. Shared slot ownership is instead
// protected by StateLock; main never changes the cursor or an owned snapshot.
enum class OutputPhase : uint8_t { BEGIN, META, STATE, BLOCK, END };
struct OutputCursor {
    bool active;
    OutputPhase phase;
    uint8_t state;
    uint8_t run;
    uint8_t block;
    uint32_t dropped;
};
static OutputCursor output;

void service_output()
{
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
            output = {};
            output.active = true;
            output.phase = OutputPhase::BEGIN;
            output.dropped = dropped_dumps;
        }
    }

    // Recheck after the handoff; no console/external lock is taken under the
    // diagnostic lock. Re-ARM pauses this cursor, including a pending END.
    // One console call already in flight may finish after ARM.
    if (hal.util->get_soft_armed()) {
        return;
    }
    // At most one line per call. Even empty phase transitions return, so no
    // unbounded scanning/printing loop can monopolize the low-priority IO task.
    switch (output.phase) {
    case OutputPhase::BEGIN:
        hal.console->printf("[EKF3-BEGIN] sections=6 complete_update=0 dropped=%lu counts_saturate=1\n",
                            (unsigned long)output.dropped);
        output.phase = OutputPhase::META;
        return;
    case OutputPhase::META:
        if (snapshot->metadata.valid) {
            hal.console->printf(
                "[EKF3-META] num_cores=%u primary=%u imu_mask=%u frames_per_prediction=%u primary_switches=%lu\n",
                (unsigned)snapshot->metadata.num_cores,
                (unsigned)snapshot->metadata.primary_core,
                (unsigned)snapshot->metadata.imu_mask,
                (unsigned)snapshot->metadata.frames_per_prediction,
                (unsigned long)snapshot->primary_switches);
        }
        output.phase = OutputPhase::STATE;
        return;
    case OutputPhase::STATE:
        if (output.state == uint8_t(StateId::COUNT)) {
            output.phase = OutputPhase::END;
        } else if (snapshot->summaries[output.state].loop_count == 0) {
            output.state++;
        } else {
            print_state(StateId(output.state), snapshot->summaries[output.state]);
            output.run = 0;
            output.block = 0;
            output.phase = OutputPhase::BLOCK;
        }
        return;
    case OutputPhase::BLOCK:
        print_block(StateId(output.state), snapshot->summaries[output.state].run[output.run],
                    output.run, output.block);
        if (++output.block == BLOCK_COUNT) {
            output.block = 0;
            if (++output.run == 2) {
                output.state++;
                output.phase = OutputPhase::STATE;
            }
        }
        return;
    case OutputPhase::END:
        hal.console->printf("[EKF3-END]\n");
        {
            StateLock lock;
            // Release only the immutable snapshot. Never clear a live or
            // pending session recorded during this output (including re-ARM).
            snapshot_index = NO_SNAPSHOT;
        }
        output = {};
        return;
    }
}

} // namespace EKF3BlockDiagnostics
} // namespace ESP32

#endif
