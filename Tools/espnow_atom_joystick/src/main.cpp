#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <string.h>

#include "ESPNowProtocol.h"
#include "esp_now.h"
#include "esp_wifi.h"

namespace {

constexpr uint8_t atom_joy_i2c_address = 0x59;
constexpr uint8_t atom_joy_sda = 38;
constexpr uint8_t atom_joy_scl = 39;
constexpr uint32_t atom_joy_i2c_frequency = 400000;

constexpr uint8_t reg_left_x = 0x00;
constexpr uint8_t reg_left_y = 0x02;
constexpr uint8_t reg_right_x = 0x20;
constexpr uint8_t reg_right_y = 0x22;
constexpr uint8_t reg_button_base = 0x70;
constexpr uint8_t reg_battery_1 = 0x60;
constexpr uint8_t reg_battery_2 = 0x62;

constexpr uint8_t espnow_channel = AP_ESPNOW_DEFAULT_CHANNEL;
constexpr uint32_t send_interval_ms = 50;
constexpr int32_t axis_half_range = 2048;
constexpr int32_t axis_deadzone = 40;
constexpr uint16_t button_left = 1U << 0;
constexpr uint16_t button_right = 1U << 1;
constexpr uint16_t button_left_stick = 1U << 2;
constexpr uint16_t button_right_stick = 1U << 3;

TwoWire atom_joy_wire(1);

uint8_t peer_mac[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint16_t sequence;
uint32_t last_send_ms;
uint32_t last_status_ms;
bool ready;

struct AtomJoyState {
    uint16_t left_x;
    uint16_t left_y;
    uint16_t right_x;
    uint16_t right_y;
    uint16_t battery_1_mv;
    uint16_t battery_2_mv;
    uint8_t buttons;
};

struct AxisBias {
    uint16_t left_x = 2048;
    uint16_t left_y = 2048;
    uint16_t right_x = 2048;
    uint16_t right_y = 2048;
};

AxisBias axis_bias;
AtomJoyState latest_state {};

void print_mac(const uint8_t *mac)
{
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void on_send(const uint8_t *mac, esp_now_send_status_t status)
{
    Serial.print("[ATOMJOY-ESPNOW] send_cb peer=");
    print_mac(mac);
    Serial.printf(" status=%s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

bool read_register(uint8_t reg, uint8_t *buffer, size_t length)
{
    atom_joy_wire.beginTransmission(atom_joy_i2c_address);
    atom_joy_wire.write(reg);
    if (atom_joy_wire.endTransmission(false) != 0) {
        return false;
    }

    const uint8_t received = atom_joy_wire.requestFrom(int(atom_joy_i2c_address), int(length));
    if (received != length) {
        return false;
    }

    for (size_t i = 0; i < length; i++) {
        buffer[i] = atom_joy_wire.read();
    }
    return true;
}

bool read_u8(uint8_t reg, uint8_t &value)
{
    return read_register(reg, &value, sizeof(value));
}

bool read_u16_le(uint8_t reg, uint16_t &value)
{
    uint8_t buffer[2] {};
    if (!read_register(reg, buffer, sizeof(buffer))) {
        return false;
    }
    value = uint16_t(buffer[0]) | (uint16_t(buffer[1]) << 8);
    return true;
}

bool read_atom_joy(AtomJoyState &state)
{
    bool ok = true;
    ok = read_u16_le(reg_left_x, state.left_x) && ok;
    ok = read_u16_le(reg_left_y, state.left_y) && ok;
    ok = read_u16_le(reg_right_x, state.right_x) && ok;
    ok = read_u16_le(reg_right_y, state.right_y) && ok;
    ok = read_u16_le(reg_battery_1, state.battery_1_mv) && ok;
    ok = read_u16_le(reg_battery_2, state.battery_2_mv) && ok;

    uint8_t buttons = 0;
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t raw = 0xff;
        if (!read_u8(reg_button_base + i, raw)) {
            ok = false;
            continue;
        }
        if (((~raw) & 0x01) != 0) {
            buttons |= uint8_t(1U << i);
        }
    }
    state.buttons = buttons;
    return ok;
}

int16_t scale_axis(uint16_t raw, uint16_t bias, bool invert)
{
    int32_t delta = int32_t(raw) - int32_t(bias);
    if (invert) {
        delta = -delta;
    }
    if (delta > -axis_deadzone && delta < axis_deadzone) {
        return 0;
    }

    int32_t scaled = (delta * 1000) / axis_half_range;
    if (scaled > 1000) {
        scaled = 1000;
    } else if (scaled < -1000) {
        scaled = -1000;
    }
    return int16_t(scaled);
}

uint16_t scale_throttle(uint16_t raw, uint16_t bias)
{
    const int16_t signed_throttle = scale_axis(raw, bias, true);
    return uint16_t((int32_t(signed_throttle) + 1000) / 2);
}

bool calibrate_center()
{
    uint32_t sum_left_x = 0;
    uint32_t sum_left_y = 0;
    uint32_t sum_right_x = 0;
    uint32_t sum_right_y = 0;
    uint16_t samples = 0;

    for (uint8_t i = 0; i < 50; i++) {
        AtomJoyState state {};
        if (read_atom_joy(state)) {
            sum_left_x += state.left_x;
            sum_left_y += state.left_y;
            sum_right_x += state.right_x;
            sum_right_y += state.right_y;
            samples++;
        }
        delay(5);
    }

    if (samples == 0) {
        return false;
    }

    axis_bias.left_x = sum_left_x / samples;
    axis_bias.left_y = sum_left_y / samples;
    axis_bias.right_x = sum_right_x / samples;
    axis_bias.right_y = sum_right_y / samples;

    Serial.printf("[ATOMJOY-ESPNOW] bias lx=%u ly=%u rx=%u ry=%u samples=%u\n",
                  axis_bias.left_x,
                  axis_bias.left_y,
                  axis_bias.right_x,
                  axis_bias.right_y,
                  samples);
    return true;
}

bool add_peer()
{
    esp_now_peer_info_t peer {};
    memcpy(peer.peer_addr, peer_mac, sizeof(peer.peer_addr));
    peer.channel = espnow_channel;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    const esp_err_t result = esp_now_add_peer(&peer);
    return result == ESP_OK || result == ESP_ERR_ESPNOW_EXIST;
}

AP_ESPNowControlPacket make_packet(const AtomJoyState &state)
{
    AP_ESPNowControlPacket packet {};
    packet.magic = AP_ESPNOW_CONTROL_MAGIC;
    packet.version = AP_ESPNOW_CONTROL_VERSION;
    packet.type = AP_ESPNOW_PACKET_TYPE_CONTROL;
    packet.sequence = sequence++;

    packet.x = scale_axis(state.right_x, axis_bias.right_x, false);
    packet.y = scale_axis(state.right_y, axis_bias.right_y, false);
    packet.z = scale_throttle(state.left_y, axis_bias.left_y);
    packet.r = scale_axis(state.left_x, axis_bias.left_x, false);
    packet.buttons = state.buttons;
    packet.crc16 = ap_espnow_control_crc(packet);
    return packet;
}

void send_packet()
{
    AtomJoyState state {};
    if (!read_atom_joy(state)) {
        Serial.println("[ATOMJOY-ESPNOW] joystick read failed");
        return;
    }
    latest_state = state;

    AP_ESPNowControlPacket packet = make_packet(state);
    const esp_err_t result = esp_now_send(peer_mac, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    Serial.printf(
        "[ATOMJOY-ESPNOW] send seq=%u x=%d y=%d z=%u r=%d buttons=0x%04x L=%u R=%u LS=%u RS=%u result=%s\n",
        packet.sequence,
        packet.x,
        packet.y,
        packet.z,
        packet.r,
        packet.buttons,
        (packet.buttons & button_left) != 0,
        (packet.buttons & button_right) != 0,
        (packet.buttons & button_left_stick) != 0,
        (packet.buttons & button_right_stick) != 0,
        result == ESP_OK ? "OK" : "FAIL");
}

void print_status()
{
    Serial.printf(
        "[ATOMJOY-ESPNOW] raw lx=%u ly=%u rx=%u ry=%u buttons=0x%02x bat1=%.3fV bat2=%.3fV\n",
        latest_state.left_x,
        latest_state.left_y,
        latest_state.right_x,
        latest_state.right_y,
        latest_state.buttons,
        latest_state.battery_1_mv / 1000.0f,
        latest_state.battery_2_mv / 1000.0f);
}

} // namespace

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println("[ATOMJOY-ESPNOW] boot");

    atom_joy_wire.begin(atom_joy_sda, atom_joy_scl, atom_joy_i2c_frequency);
    if (!calibrate_center()) {
        Serial.println("[ATOMJOY-ESPNOW] joystick not found at I2C address 0x59");
    }

    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, true);
    esp_wifi_set_channel(espnow_channel, WIFI_SECOND_CHAN_NONE);

    Serial.print("[ATOMJOY-ESPNOW] local_mac=");
    Serial.println(WiFi.macAddress());
    Serial.printf("[ATOMJOY-ESPNOW] channel=%u\n", espnow_channel);
    Serial.println("[ATOMJOY-ESPNOW] mapping roll=right_x pitch=right_y throttle=left_y(center) yaw=left_x");

    const esp_err_t init_result = esp_now_init();
    if (init_result != ESP_OK) {
        Serial.printf("[ATOMJOY-ESPNOW] esp_now_init failed: %d\n", init_result);
        return;
    }

    esp_now_register_send_cb(on_send);

    if (!add_peer()) {
        Serial.println("[ATOMJOY-ESPNOW] add broadcast peer failed");
        return;
    }

    Serial.print("[ATOMJOY-ESPNOW] peer=");
    print_mac(peer_mac);
    Serial.println();
    ready = true;
}

void loop()
{
    if (!ready) {
        delay(1000);
        return;
    }

    const uint32_t now_ms = millis();
    if (now_ms - last_send_ms >= send_interval_ms) {
        last_send_ms = now_ms;
        send_packet();
    }

    if (now_ms - last_status_ms >= 1000) {
        last_status_ms = now_ms;
        print_status();
    }
}
