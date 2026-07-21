#pragma once

#include <stddef.h>
#include <stdint.h>

#define AP_ESPNOW_CONTROL_MAGIC 0x4150454EU
#define AP_ESPNOW_CONTROL_VERSION 1U
#define AP_ESPNOW_PACKET_TYPE_CONTROL 1U
#define AP_ESPNOW_DEFAULT_CHANNEL 1U

struct __attribute__((packed)) AP_ESPNowControlPacket {
    uint32_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t sequence;
    int16_t x;
    int16_t y;
    uint16_t z;
    int16_t r;
    uint16_t buttons;
    uint16_t crc16;
};

static inline uint16_t ap_espnow_crc16_ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xffffU;

    for (size_t i = 0; i < length; i++) {
        crc ^= uint16_t(data[i]) << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = uint16_t((crc << 1) ^ 0x1021U);
            } else {
                crc = uint16_t(crc << 1);
            }
        }
    }

    return crc;
}

static inline uint16_t ap_espnow_control_crc(const AP_ESPNowControlPacket &packet)
{
    AP_ESPNowControlPacket packet_without_crc = packet;
    packet_without_crc.crc16 = 0;
    return ap_espnow_crc16_ccitt(
        reinterpret_cast<const uint8_t *>(&packet_without_crc),
        sizeof(packet_without_crc));
}

static inline bool ap_espnow_control_packet_valid(const AP_ESPNowControlPacket &packet)
{
    return packet.magic == AP_ESPNOW_CONTROL_MAGIC &&
           packet.version == AP_ESPNOW_CONTROL_VERSION &&
           packet.type == AP_ESPNOW_PACKET_TYPE_CONTROL &&
           packet.crc16 == ap_espnow_control_crc(packet);
}
