#pragma once

#include <stdint.h>

// Convert momentary L/R buttons into standard RC5 AuxSwitch pulses and a
// persistent RC6 flight-mode position. Arming and mode policy remain in the
// normal ArduPilot RC_Channel paths.
class ESPNowButtons {
public:
    struct Outputs {
        uint16_t rc5;
        uint16_t rc6;
    };

    enum class State : uint8_t {
        WAIT_RELEASE,
        IDLE,
        L_PRESSED,
        L_LONG_FIRED,
        R_PRESSED,
        R_ARM_BLOCKED,
        PULSE_ARM,
        PULSE_DISARM,
    };

    static constexpr uint16_t LEFT = 1U << 0;
    static constexpr uint16_t RIGHT = 1U << 1;
    static constexpr uint32_t L_LONG_PRESS_MS = 1000;
    static constexpr uint32_t R_SHORT_PRESS_MS = 1000;
    static constexpr uint32_t R_ARM_HOLD_MS = 1000;
    static constexpr uint32_t PULSE_MS = 500;
    static constexpr uint16_t ARM_THROTTLE_MAX_PWM = 1050;
    static constexpr uint16_t RC5_LOW = 1000;
    static constexpr uint16_t RC5_CENTER = 1500;
    static constexpr uint16_t RC5_HIGH = 2000;
    static constexpr uint16_t RC6_STABILIZE = 1165;
    // Alternate between two switch positions assigned to the same mode. This
    // creates a real mode-switch edge on every L event, even after a GCS has
    // changed mode while RC6 remained at its previous position.
    static constexpr uint16_t RC6_ALT_HOLD_A = 1425;
    static constexpr uint16_t RC6_ALT_HOLD_B = 1555;
    static constexpr uint16_t RC6_LOITER_A = 1685;
    static constexpr uint16_t RC6_LOITER_B = 1815;

    void reset(uint32_t now, bool reset_flight_mode = false)
    {
        state = State::WAIT_RELEASE;
        since_ms = now;
        if (reset_flight_mode) {
            rc6 = RC6_STABILIZE;
        }
    }

    Outputs update(uint16_t buttons, uint32_t now, bool arm_allowed = true)
    {
        const bool left = (buttons & LEFT) != 0;
        const bool right = (buttons & RIGHT) != 0;

        // A simultaneous press cancels every pending action. Keep extending
        // the release gate until both buttons have actually been released.
        if (left && right) {
            state = State::WAIT_RELEASE;
            since_ms = now;
            return output(RC5_CENTER);
        }

        switch (state) {
        case State::PULSE_ARM:
        case State::PULSE_DISARM:
            // A normal release cannot truncate an AuxSwitch pulse before its
            // 200ms debounce has completed.
            if (uint32_t(now - since_ms) < PULSE_MS) {
                return output(state == State::PULSE_ARM ? RC5_HIGH : RC5_LOW);
            }
            reset(now);
            return output(RC5_CENTER);

        case State::WAIT_RELEASE:
            if (left || right) {
                since_ms = now;
            } else if (uint32_t(now - since_ms) >= PULSE_MS) {
                state = State::IDLE;
            }
            return output(RC5_CENTER);

        case State::IDLE:
            if (left) {
                state = State::L_PRESSED;
                since_ms = now;
            } else if (right) {
                state = State::R_PRESSED;
                since_ms = now;
            }
            return output(RC5_CENTER);

        case State::L_PRESSED:
            if (uint32_t(now - since_ms) >= L_LONG_PRESS_MS) {
                rc6 = rc6 == RC6_LOITER_A ? RC6_LOITER_B : RC6_LOITER_A;
                state = left ? State::L_LONG_FIRED : State::IDLE;
            } else if (!left) {
                rc6 = rc6 == RC6_ALT_HOLD_A ? RC6_ALT_HOLD_B : RC6_ALT_HOLD_A;
                state = State::IDLE;
            }
            return output(RC5_CENTER);

        case State::L_LONG_FIRED:
            if (!left) {
                state = State::IDLE;
            }
            return output(RC5_CENTER);

        case State::R_PRESSED:
            if (uint32_t(now - since_ms) >= R_ARM_HOLD_MS) {
                if (!arm_allowed) {
                    state = State::R_ARM_BLOCKED;
                    return output(RC5_CENTER);
                }
                state = State::PULSE_ARM;
                since_ms = now;
                return output(RC5_HIGH);
            }
            if (!right) {
                if (uint32_t(now - since_ms) < R_SHORT_PRESS_MS) {
                    state = State::PULSE_DISARM;
                    since_ms = now;
                    return output(RC5_LOW);
                }
                state = State::IDLE;
            }
            return output(RC5_CENTER);

        case State::R_ARM_BLOCKED:
            if (!right) {
                state = State::IDLE;
            }
            return output(RC5_CENTER);
        }
        return output(RC5_CENTER);
    }

    State get_state() const { return state; }
    uint16_t get_rc6() const { return rc6; }
    static bool is_alt_hold(uint16_t pwm)
    {
        return pwm == RC6_ALT_HOLD_A || pwm == RC6_ALT_HOLD_B;
    }
    static bool is_loiter(uint16_t pwm)
    {
        return pwm == RC6_LOITER_A || pwm == RC6_LOITER_B;
    }

private:
    Outputs output(uint16_t rc5) const { return {rc5, rc6}; }

    State state = State::WAIT_RELEASE;
    uint32_t since_ms = 0;
    uint16_t rc6 = RC6_STABILIZE;
};
