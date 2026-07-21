#include <Arduino.h>
#include <WiFi.h>
#include <string.h>

#include "ESPNowProtocol.h"
#include "esp_now.h"
#include "esp_wifi.h"

namespace {

constexpr uint8_t espnow_channel = AP_ESPNOW_DEFAULT_CHANNEL;
constexpr uint32_t send_interval_ms = 50;

uint8_t peer_mac[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint16_t sequence;
uint32_t last_send_ms;
bool ready;

void print_mac(const uint8_t *mac)
{
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void on_send(const uint8_t *mac, esp_now_send_status_t status)
{
    Serial.print("[ATOM-ESPNOW] send_cb peer=");
    print_mac(mac);
    Serial.printf(" status=%s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
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

AP_ESPNowControlPacket make_neutral_packet()
{
    AP_ESPNowControlPacket packet {};
    packet.magic = AP_ESPNOW_CONTROL_MAGIC;
    packet.version = AP_ESPNOW_CONTROL_VERSION;
    packet.type = AP_ESPNOW_PACKET_TYPE_CONTROL;
    packet.sequence = sequence++;
    packet.x = 0;
    packet.y = 0;
    packet.z = 0;
    packet.r = 0;
    packet.buttons = 0;
    packet.crc16 = ap_espnow_control_crc(packet);
    return packet;
}

void send_packet()
{
    AP_ESPNowControlPacket packet = make_neutral_packet();
    const esp_err_t result = esp_now_send(peer_mac, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    Serial.printf(
        "[ATOM-ESPNOW] send seq=%u x=%d y=%d z=%u r=%d buttons=0x%04x result=%s\n",
        packet.sequence,
        packet.x,
        packet.y,
        packet.z,
        packet.r,
        packet.buttons,
        result == ESP_OK ? "OK" : "FAIL");
}

} // namespace

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println("[ATOM-ESPNOW] boot");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, true);
    esp_wifi_set_channel(espnow_channel, WIFI_SECOND_CHAN_NONE);

    Serial.print("[ATOM-ESPNOW] local_mac=");
    Serial.println(WiFi.macAddress());
    Serial.printf("[ATOM-ESPNOW] channel=%u\n", espnow_channel);

    const esp_err_t init_result = esp_now_init();
    if (init_result != ESP_OK) {
        Serial.printf("[ATOM-ESPNOW] esp_now_init failed: %d\n", init_result);
        return;
    }

    esp_now_register_send_cb(on_send);

    if (!add_peer()) {
        Serial.println("[ATOM-ESPNOW] add broadcast peer failed");
        return;
    }

    Serial.print("[ATOM-ESPNOW] peer=");
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
}
