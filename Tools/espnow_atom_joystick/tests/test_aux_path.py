#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Host regression using actual RC_Channel and ESP-NOW Bridge methods.

AI-assisted test. No hardware, MAVLink or arming commands are used.
"""
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[3]


def method(source, signature):
    start = source.index(signature)
    brace = source.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def main():
    source = (ROOT / 'libraries/RC_Channel/RC_Channel.cpp').read_text()
    methods = '\n'.join(method(source, s) for s in (
        'bool RC_Channel::debounce_completed(',
        'bool RC_Channel::read_6pos_switch(',
        'bool RC_Channel::read_3pos_switch(',
        'void RC_Channel::do_aux_function_armdisarm('))
    bridge = (ROOT / 'libraries/AP_HAL_ESP32/ESPNowBridge.cpp').read_text()
    bridge_methods = '\n'.join(method(bridge, s) for s in (
        'uint16_t throttle_to_pwm(',
        'void apply_control_rcin('))
    timeout_define = next(line for line in bridge.splitlines()
                          if line.startswith('#define HAL_ESP32_ESPNOW_RCIN_TIMEOUT_MS '))
    program = r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <AP_HAL_ESP32/ESPNowButtons.h>
static uint32_t now_ms;
namespace AP_HAL { uint32_t millis() { return now_ms; } }
struct AP_Arming {
    enum class Method { AUXSWITCH };
    bool allowed=false, armed=false, checks=false;
    int arms=0, disarms=0;
    bool arm(Method, bool run_checks) {
        arms++; checks=run_checks;
        if (allowed && run_checks) { armed=true; }
        return armed;
    }
    bool disarm(Method) { disarms++; armed=false; return true; }
};
namespace AP { AP_Arming &arming() { static AP_Arming a; return a; } }
struct RC_Channels {
    enum class Option { ALLOW_SWITCH_REV };
    bool option_is_enabled(Option) { return false; }
};
RC_Channels &rc() { static RC_Channels c; return c; }
#define SWITCH_DEBOUNCE_TIME_MS 200
struct RC_Channel {
    enum class AuxSwitchPos { LOW, MIDDLE, HIGH };
    struct { int8_t debounce_position=-1, current_position=-1; uint32_t last_edge_time_ms=0; } switch_state;
    uint16_t pwm=1500;
    bool reversed=false;
    static constexpr uint16_t RC_MIN_LIMIT_PWM=800, RC_MAX_LIMIT_PWM=2200;
    static constexpr uint16_t AUX_SWITCH_PWM_TRIGGER_LOW=1200, AUX_SWITCH_PWM_TRIGGER_HIGH=1800;
    uint16_t get_radio_in() const { return pwm; }
    bool debounce_completed(int8_t);
    bool read_6pos_switch(int8_t&);
    bool read_3pos_switch(AuxSwitchPos&) const;
    void do_aux_function_armdisarm(AuxSwitchPos);
};
''' + methods + r'''
// Only the hardware publication/logging is stubbed. The sequence filter,
// throttle conversion and button update below come from the actual Bridge.
struct Console { void printf(const char *, ...) {} };
static Console console;
static struct { Console *console; } hal {&console};
struct AP_ESPNowControlPacket {
    int16_t x=0, y=0, r=0;
    uint16_t z=0, buttons=0, sequence=0;
};
static ESPNowButtons button_events;
static uint16_t last_rc5=1500, last_rc6=ESPNowButtons::RC6_STABILIZE;
static uint16_t last_control_sequence;
static uint8_t last_control_mac[6];
static uint32_t last_control_ms;
static uint32_t last_control_log_ms;
static bool control_active;
static uint16_t published_rc5=1500, published_rc6=ESPNowButtons::RC6_STABILIZE;
static int publications, arm_events, disarm_events, alt_events, loiter_events;
void format_mac(const uint8_t *, char *buffer, size_t) { buffer[0]='\0'; }
bool should_log(uint32_t &) { return false; }
uint16_t axis_to_pwm(int16_t) { return 1500; }
void set_rc_input(const uint16_t *channels, uint8_t count) {
    assert(count==8);
    publications++;
    if (channels[4]!=published_rc5) {
        arm_events+=channels[4]==ESPNowButtons::RC5_HIGH;
        disarm_events+=channels[4]==ESPNowButtons::RC5_LOW;
    }
    if (channels[5]!=published_rc6) {
        alt_events+=ESPNowButtons::is_alt_hold(channels[5]);
        loiter_events+=ESPNowButtons::is_loiter(channels[5]);
    }
    published_rc5=channels[4]; published_rc6=channels[5];
}
''' + timeout_define + '\n' + bridge_methods + r'''
void test_bridge_packets() {
    using B=ESPNowButtons;
    // Exercise both sides of the real RC3 <= 1050 gate.
    for (uint16_t throttle : {50U, 51U}) {
        button_events=B{};
        last_rc5=published_rc5=B::RC5_CENTER;
        last_rc6=published_rc6=B::RC6_STABILIZE;
        control_active=false;
        publications=arm_events=disarm_events=alt_events=loiter_events=0;
        AP_ESPNowControlPacket packet;
        packet.z=throttle;
        const uint8_t mac[6]={};
        uint16_t sequence=0;
        const auto send=[&](uint16_t buttons, uint32_t time) {
            packet.buttons=buttons;
            packet.sequence=sequence++;
            now_ms=time;
            apply_control_rcin(packet, mac);
            const int before_publications=publications;
            const int before_arm=arm_events, before_disarm=disarm_events;
            const int before_alt=alt_events, before_loiter=loiter_events;
            const auto before_state=button_events.get_state();
            const uint16_t before_rc5=published_rc5, before_rc6=published_rc6;
            const uint32_t before_control_ms=last_control_ms;
            // Repeated same-sequence frames must not publish, trigger events,
            // advance the FSM, end a pulse, or refresh the link timer.
            for (uint32_t delay : {0U, 1U, 49U}) {
                now_ms=time+delay;
                apply_control_rcin(packet, mac);
                assert(publications==before_publications);
                assert(arm_events==before_arm && disarm_events==before_disarm);
                assert(alt_events==before_alt && loiter_events==before_loiter);
                assert(button_events.get_state()==before_state);
                assert(published_rc5==before_rc5 && published_rc6==before_rc6);
                assert(last_control_ms==before_control_ms);
            }
        };
        for (uint32_t time=0; time<=7500; time+=50) {
            const uint16_t buttons=time>=600 && time<1100 ? B::RIGHT :
                time>=2200 && time<3400 ? B::RIGHT :
                time>=4400 && time<4900 ? B::LEFT :
                time>=5400 && time<6600 ? B::LEFT : 0;
            send(buttons,time);
            if (time==1100) { assert(published_rc5==B::RC5_LOW); }
            if (time==1550) { assert(published_rc5==B::RC5_LOW); }
            if (time==1600) { assert(published_rc5==B::RC5_CENTER); }
            if (time==3200) {
                assert(published_rc5==(throttle==50 ? B::RC5_HIGH : B::RC5_CENTER));
                assert(arm_events==(throttle==50 ? 1 : 0));
            }
            if (time==3400) { assert(disarm_events==1); }
            if (time==3650 && throttle==50) { assert(published_rc5==B::RC5_HIGH); }
            if (time==3700) { assert(published_rc5==B::RC5_CENTER); }
            if (time==4900) { assert(published_rc6==B::RC6_ALT_HOLD_A); }
            if (time==6400) { assert(published_rc6==B::RC6_LOITER_A); }
            if (time==6600) { assert(alt_events==1 && loiter_events==1); }
        }
        assert(arm_events==(throttle==50 ? 1 : 0));
        assert(disarm_events==1 && alt_events==1 && loiter_events==1);
    }
    puts("PASS actual Bridge: RC3=1050 ARM/1051 reject, 1s threshold, release suppression, duplicate packets cannot repeat ARM/DISARM/ALT_HOLD/LOITER or refresh link");
}
int simulate(bool mode_on_rc5) {
    RC_Channel c;
    c.switch_state.current_position=1;
    c.switch_state.debounce_position=1;
    int highs=0;
    for (now_ms=0; now_ms<3000; now_ms+=10) {
        c.pwm=now_ms<500 ? 1500 : now_ms<2000 ? 2000 : 1500;
        if (mode_on_rc5) { int8_t pos; c.read_6pos_switch(pos); }
        if (now_ms%100==0) {
            RC_Channel::AuxSwitchPos pos;
            assert(c.read_3pos_switch(pos));
            if (c.debounce_completed(int8_t(pos)) && pos==RC_Channel::AuxSwitchPos::HIGH) { highs++; }
        }
    }
    return highs;
}
int mode_position(uint16_t pwm) {
    RC_Channel c;
    c.pwm=pwm;
    int8_t pos=-1;
    now_ms=0;
    assert(!c.read_6pos_switch(pos));
    now_ms=201;
    assert(c.read_6pos_switch(pos));
    return pos;
}
int main() {
    assert(simulate(true)==0);
    assert(simulate(false)==1);
    puts("PASS actual RC_Channel: FLTMODE_CH=5 suppresses AUX HIGH; dedicated RC5 confirms exactly once");
    assert(mode_position(ESPNowButtons::RC6_STABILIZE)==0);
    assert(mode_position(ESPNowButtons::RC6_ALT_HOLD_A)==2);
    assert(mode_position(ESPNowButtons::RC6_ALT_HOLD_B)==3);
    assert(mode_position(ESPNowButtons::RC6_LOITER_A)==4);
    assert(mode_position(ESPNowButtons::RC6_LOITER_B)==5);
    puts("PASS actual mode decoder: paired RC6 positions provide repeat ALT_HOLD/LOITER edges");
    // Explicit HIGH/LOW/MIDDLE mapping, including a neutral no-action check.
    {
        RC_Channel c;
        auto &arming=AP::arming();
        arming=AP_Arming{};
        arming.allowed=true;
        for (uint16_t pwm : {1500U, 2000U, 1500U, 1000U, 1500U}) {
            c.pwm=pwm;
            RC_Channel::AuxSwitchPos pos;
            assert(c.read_3pos_switch(pos));
            const int before_arms=arming.arms, before_disarms=arming.disarms;
            c.do_aux_function_armdisarm(pos);
            if (pwm==1500) {
                assert(arming.arms==before_arms && arming.disarms==before_disarms);
            }
        }
        assert(arming.arms==1 && arming.disarms==1 && !arming.armed && arming.checks);
        puts("PASS actual AUX wrapper: RC5=2000 ARM, 1000 DISARM, 1500 no action");
    }
    // Feed the real button FSM into the real switch/debounce and standard
    // ARM/DISARM wrapper. Fake arming policy verifies checks=true is kept.
    for (int phase=0; phase<10; phase++) {
        RC_Channel c;
        ESPNowButtons buttons;
        auto &arming=AP::arming();
        arming=AP_Arming{};
        arming.armed=true;
        c.switch_state.current_position=1;
        c.switch_state.debounce_position=1;
        for (now_ms=0; now_ms<13000; now_ms+=10) {
            uint16_t pressed=now_ms>=800 && now_ms<1300 ? ESPNowButtons::RIGHT :
                now_ms>=3500 && now_ms<4700 ? ESPNowButtons::RIGHT :
                now_ms>=6500 && now_ms<7700 ? ESPNowButtons::RIGHT : 0;
            arming.allowed=now_ms>=6000;
            if (now_ms%50==0) { c.pwm=buttons.update(pressed, now_ms, arming.allowed).rc5; }
            if (now_ms==1300) { assert(c.pwm==ESPNowButtons::RC5_LOW); }
            if (now_ms==4500) { assert(c.pwm==ESPNowButtons::RC5_CENTER); }
            if (now_ms==7500) { assert(c.pwm==ESPNowButtons::RC5_HIGH); }
            if (now_ms%100==uint32_t(phase*10)) {
                RC_Channel::AuxSwitchPos pos;
                assert(c.read_3pos_switch(pos));
                if (c.debounce_completed(int8_t(pos))) { c.do_aux_function_armdisarm(pos); }
            }
            if (now_ms==2500) { assert(!arming.armed && arming.disarms==1); }
            if (now_ms==6000) { assert(!arming.armed && arming.arms==0); }
            if (now_ms==10000) { assert(arming.armed && arming.arms==1 && arming.checks); }
        }
        assert(arming.arms==1 && arming.disarms==1 && arming.armed);
    }
    puts("PASS real AUX wrapper + R FSM: short DISARM, unsafe 1s ARM blocked, safe 1s ARM once, checks=true");
    test_bridge_packets();
}
'''
    with tempfile.TemporaryDirectory(prefix='rc5-path-') as tmp:
        exe = str(pathlib.Path(tmp) / 'test')
        subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra', '-Werror', '-I' + str(ROOT / 'libraries'),
                        '-x', 'c++', '-', '-o', exe], input=program, text=True, check=True)
        subprocess.run([exe], check=True)


if __name__ == '__main__':
    main()
