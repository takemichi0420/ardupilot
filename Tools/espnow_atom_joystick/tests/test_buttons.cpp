// AI-assisted host tests: no hardware access or ARM commands.
#include <cassert>
#include <cstdio>
#include <initializer_list>
#include <AP_HAL_ESP32/ESPNowButtons.h>

using Buttons = ESPNowButtons;

static_assert(Buttons::L_LONG_PRESS_MS == 1000, "L boundary must be 1000ms");
static_assert(Buttons::R_SHORT_PRESS_MS == 1000, "R short boundary must be 1000ms");
static_assert(Buttons::R_ARM_HOLD_MS == 1000, "R long boundary must be 1000ms");
static_assert(Buttons::PULSE_MS == 500, "AUX pulse must last 500ms");

static Buttons::Outputs update(Buttons &buttons, uint16_t input, uint32_t now, bool arm_allowed = true)
{
    return buttons.update(input, now, arm_allowed);
}

static void accept_release(Buttons &buttons, uint32_t now)
{
    assert(update(buttons, 0, now).rc5 == Buttons::RC5_CENTER);
    assert(update(buttons, 0, now + Buttons::PULSE_MS).rc5 == Buttons::RC5_CENTER);
}

static void test_l_button(uint32_t start)
{
    Buttons buttons;
    buttons.reset(start, true);
    accept_release(buttons, start);

    uint32_t now = start + 600;
    assert(update(buttons, Buttons::LEFT, now).rc6 == Buttons::RC6_STABILIZE);
    assert(update(buttons, 0, now + 500).rc6 == Buttons::RC6_ALT_HOLD_A);
    assert(update(buttons, 0, now + 5000).rc6 == Buttons::RC6_ALT_HOLD_A);

    // A second short press must produce another RC6 switch edge so ALT_HOLD
    // can be selected again after a Mission Planner mode change.
    now += 6000;
    assert(update(buttons, Buttons::LEFT, now).rc6 == Buttons::RC6_ALT_HOLD_A);
    assert(update(buttons, 0, now + 999).rc6 == Buttons::RC6_ALT_HOLD_B);

    now += 3000;
    assert(update(buttons, Buttons::LEFT, now).rc6 == Buttons::RC6_ALT_HOLD_B);
    assert(update(buttons, Buttons::LEFT, now + 999).rc6 == Buttons::RC6_ALT_HOLD_B);
    assert(update(buttons, Buttons::LEFT, now + 1000).rc6 == Buttons::RC6_LOITER_A);
    assert(update(buttons, Buttons::LEFT, now + 2500).rc6 == Buttons::RC6_LOITER_A);
    assert(update(buttons, 0, now + 2501).rc6 == Buttons::RC6_LOITER_A);
    assert(update(buttons, 0, now + 2600).rc6 == Buttons::RC6_LOITER_A);

    // LOITER also gets a fresh edge on every new 1s hold.
    now += 3000;
    assert(update(buttons, Buttons::LEFT, now).rc6 == Buttons::RC6_LOITER_A);
    assert(update(buttons, Buttons::LEFT, now + 1000).rc6 == Buttons::RC6_LOITER_B);
    assert(update(buttons, 0, now + 1001).rc6 == Buttons::RC6_LOITER_B);

    now += 2000;
    assert(update(buttons, Buttons::LEFT, now).rc6 == Buttons::RC6_LOITER_B);
    assert(update(buttons, 0, now + 500).rc6 == Buttons::RC6_ALT_HOLD_A);

    // A release first observed exactly at the boundary is a long press.
    now += 2000;
    assert(update(buttons, Buttons::LEFT, now).rc6 == Buttons::RC6_ALT_HOLD_A);
    assert(update(buttons, 0, now + 1000).rc6 == Buttons::RC6_LOITER_A);
    assert(update(buttons, 0, now + 1500).rc6 == Buttons::RC6_LOITER_A);
}

static void test_r_button(uint32_t start)
{
    Buttons buttons;
    buttons.reset(start, true);
    accept_release(buttons, start);

    uint32_t now = start + 600;
    for (uint32_t duration : {500U, 999U}) {
        assert(update(buttons, Buttons::RIGHT, now).rc5 == Buttons::RC5_CENTER);
        assert(update(buttons, 0, now + duration).rc5 == Buttons::RC5_LOW);
        assert(update(buttons, 0, now + duration + 499).rc5 == Buttons::RC5_LOW);
        assert(update(buttons, 0, now + duration + 500).rc5 == Buttons::RC5_CENTER);
        accept_release(buttons, now + duration + 500);
        assert(buttons.get_state() == Buttons::State::IDLE);
        now += 3000;
    }

    assert(update(buttons, Buttons::RIGHT, now).rc5 == Buttons::RC5_CENTER);
    assert(update(buttons, Buttons::RIGHT, now + 999).rc5 == Buttons::RC5_CENTER);
    assert(update(buttons, Buttons::RIGHT, now + 1000).rc5 == Buttons::RC5_HIGH);
    // Release cannot shorten the ARM pulse or produce DISARM.
    assert(update(buttons, 0, now + 1001).rc5 == Buttons::RC5_HIGH);
    assert(update(buttons, 0, now + 1499).rc5 == Buttons::RC5_HIGH);
    assert(update(buttons, 0, now + 1500).rc5 == Buttons::RC5_CENTER);
    accept_release(buttons, now + 1500);

    // Holding R past the pulse cannot retrigger ARM, even after many seconds.
    now += 3000;
    assert(update(buttons, Buttons::RIGHT, now).rc5 == Buttons::RC5_CENTER);
    assert(update(buttons, Buttons::RIGHT, now + 1000).rc5 == Buttons::RC5_HIGH);
    assert(update(buttons, Buttons::RIGHT, now + 1499).rc5 == Buttons::RC5_HIGH);
    assert(update(buttons, Buttons::RIGHT, now + 1500).rc5 == Buttons::RC5_CENTER);
    assert(update(buttons, Buttons::RIGHT, now + 3000).rc5 == Buttons::RC5_CENTER);
    assert(update(buttons, 0, now + 3001).rc5 == Buttons::RC5_CENTER);
    accept_release(buttons, now + 3001);

    // The former 1000..1999ms no-action interval now produces ARM.
    now += 5000;
    for (uint32_t duration : {1000U, 1500U, 1999U}) {
        assert(update(buttons, Buttons::RIGHT, now).rc5 == Buttons::RC5_CENTER);
        assert(update(buttons, 0, now + duration).rc5 == Buttons::RC5_HIGH);
        assert(update(buttons, 0, now + duration + 499).rc5 == Buttons::RC5_HIGH);
        assert(update(buttons, 0, now + duration + 500).rc5 == Buttons::RC5_CENTER);
        accept_release(buttons, now + duration + 500);
        now += 4000;
    }

    assert(update(buttons, Buttons::RIGHT, now, false).rc5 == Buttons::RC5_CENTER);
    assert(update(buttons, Buttons::RIGHT, now + 999, false).rc5 == Buttons::RC5_CENTER);
    assert(update(buttons, Buttons::RIGHT, now + 1000, false).rc5 == Buttons::RC5_CENTER);
    assert(buttons.get_state() == Buttons::State::R_ARM_BLOCKED);
    // Lowering throttle during the same hold cannot retry a rejected ARM.
    assert(update(buttons, Buttons::RIGHT, now + 1500, true).rc5 == Buttons::RC5_CENTER);
    assert(buttons.get_state() == Buttons::State::R_ARM_BLOCKED);
    assert(update(buttons, 0, now + 1600, true).rc5 == Buttons::RC5_CENTER);
    assert(update(buttons, 0, now + 1700, true).rc5 == Buttons::RC5_CENTER);

    now += 3000;
    assert(update(buttons, Buttons::RIGHT, now, true).rc5 == Buttons::RC5_CENTER);
    assert(update(buttons, Buttons::RIGHT, now + 1000, true).rc5 == Buttons::RC5_HIGH);

    // Rejection on a release observed at 1000ms also suppresses DISARM.
    Buttons rejected;
    rejected.reset(start, true);
    accept_release(rejected, start);
    now = start + 600;
    assert(update(rejected, Buttons::RIGHT, now, false).rc5 == Buttons::RC5_CENTER);
    assert(update(rejected, 0, now + 1000, false).rc5 == Buttons::RC5_CENTER);
    assert(update(rejected, 0, now + 1001, true).rc5 == Buttons::RC5_CENTER);
}

static void test_simultaneous_and_recovery()
{
    Buttons buttons;
    buttons.reset(0, true);
    accept_release(buttons, 0);

    assert(update(buttons, Buttons::LEFT, 600).rc5 == Buttons::RC5_CENTER);
    assert(update(buttons, Buttons::LEFT | Buttons::RIGHT, 900).rc5 == Buttons::RC5_CENTER);
    assert(update(buttons, Buttons::RIGHT, 3000).rc5 == Buttons::RC5_CENTER);
    assert(update(buttons, 0, 3100).rc5 == Buttons::RC5_CENTER);
    assert(update(buttons, 0, 3599).rc5 == Buttons::RC5_CENTER);
    assert(update(buttons, Buttons::RIGHT, 3600).rc5 == Buttons::RC5_CENTER);
    assert(update(buttons, 0, 3650).rc5 == Buttons::RC5_LOW);

    // A reconnect cancels the pulse and requires release, while preserving the
    // last selected flight-mode position.
    buttons.reset(3700);
    assert(update(buttons, Buttons::RIGHT, 3700).rc5 == Buttons::RC5_CENTER);
    assert(update(buttons, Buttons::RIGHT, 8000).rc5 == Buttons::RC5_CENTER);
    assert(update(buttons, 0, 8050).rc5 == Buttons::RC5_CENTER);
    assert(update(buttons, 0, 8550).rc5 == Buttons::RC5_CENTER);
    assert(update(buttons, Buttons::LEFT, 8600).rc6 == Buttons::RC6_STABILIZE);
    assert(update(buttons, 0, 8650).rc6 == Buttons::RC6_ALT_HOLD_A);
    buttons.reset(9000);
    assert(update(buttons, 0, 9500).rc6 == Buttons::RC6_ALT_HOLD_A);
    buttons.reset(10000, true);
    assert(update(buttons, 0, 10500).rc6 == Buttons::RC6_STABILIZE);

    // Both pulse types can be cancelled by simultaneous press or reset.
    for (bool arm : {false, true}) {
        Buttons pulsing;
        pulsing.reset(0, true);
        accept_release(pulsing, 0);
        assert(update(pulsing, Buttons::RIGHT, 600).rc5 == Buttons::RC5_CENTER);
        const uint32_t event_ms = arm ? 1600 : 1100;
        assert(update(pulsing, arm ? Buttons::RIGHT : 0, event_ms).rc5 ==
               (arm ? Buttons::RC5_HIGH : Buttons::RC5_LOW));
        assert(update(pulsing, Buttons::LEFT | Buttons::RIGHT, event_ms + 1).rc5 == Buttons::RC5_CENTER);
        assert(update(pulsing, Buttons::RIGHT, event_ms + 1500).rc5 == Buttons::RC5_CENTER);
        pulsing.reset(event_ms + 2000);
        accept_release(pulsing, event_ms + 2000);
        assert(update(pulsing, Buttons::RIGHT, event_ms + 2600).rc5 == Buttons::RC5_CENTER);
        assert(update(pulsing, arm ? Buttons::RIGHT : 0, event_ms + 2600 + (arm ? 1000 : 500)).rc5 ==
               (arm ? Buttons::RC5_HIGH : Buttons::RC5_LOW));
        const uint32_t reset_ms = event_ms + 2600 + (arm ? 1000 : 500) + 1;
        pulsing.reset(reset_ms);
        assert(update(pulsing, 0, reset_ms).rc5 == Buttons::RC5_CENTER);
    }
}

int main()
{
    for (uint32_t start : {0U, 0xfffffa00U}) {
        test_l_button(start);
        test_r_button(start);
    }
    for (uint16_t held : {Buttons::LEFT, Buttons::RIGHT,
                          uint16_t(Buttons::LEFT | Buttons::RIGHT)}) {
        Buttons buttons;
        buttons.reset(100, true);
        for (uint32_t now = 100; now < 5000; now += 50) {
            const Buttons::Outputs outputs = update(buttons, held, now);
            assert(outputs.rc5 == Buttons::RC5_CENTER);
            assert(outputs.rc6 == Buttons::RC6_STABILIZE);
        }
    }
    test_simultaneous_and_recovery();
    puts("PASS buttons: <1000ms ALT_HOLD/DISARM, >=1000ms LOITER/ARM, exact boundary, throttle rejection and release suppression, single events, 500ms pulses, simultaneous cancel, startup/recovery gate, rollover");
}
