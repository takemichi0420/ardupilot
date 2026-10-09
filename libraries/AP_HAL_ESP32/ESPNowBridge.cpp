#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_ESP32/ESPNowBridge.h>
#include "ESPNowButtons.h"
#include <AP_HAL_ESP32/ESPNowProtocol.h>
#include <AP_HAL_ESP32/RCInput.h>
#include <AP_HAL_ESP32/Scheduler.h>

#include <string.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_idf_version.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#ifndef HAL_ESP32_ESPNOW_ENABLED
#define HAL_ESP32_ESPNOW_ENABLED 0
#endif

#ifndef HAL_ESP32_ESPNOW_QUEUE_SIZE
#define HAL_ESP32_ESPNOW_QUEUE_SIZE 8
#endif

#ifndef HAL_ESP32_ESPNOW_TASK_STACK
#define HAL_ESP32_ESPNOW_TASK_STACK 3072
#endif

#ifndef WIFI_CHANNEL
#define WIFI_CHANNEL AP_ESPNOW_DEFAULT_CHANNEL
#endif

#ifndef HAL_ESP32_ESPNOW_RCIN_ENABLED
#define HAL_ESP32_ESPNOW_RCIN_ENABLED HAL_ESP32_ESPNOW_ENABLED
#endif

#ifndef HAL_ESP32_ESPNOW_RCIN_TIMEOUT_MS
#define HAL_ESP32_ESPNOW_RCIN_TIMEOUT_MS 500
#endif

#ifndef HAL_ESP32_ESPNOW_LOG_INTERVAL_MS
#define HAL_ESP32_ESPNOW_LOG_INTERVAL_MS 1000
#endif

using namespace ESP32;

extern const AP_HAL::HAL& hal;

#if HAL_ESP32_ESPNOW_ENABLED

namespace {

struct ESPNOWFrame {
    uint8_t source_mac[6];
    int data_length;
    uint8_t data[sizeof(AP_ESPNowControlPacket)];
};

QueueHandle_t receive_queue;
TaskHandle_t worker_task;
bool started;
uint32_t last_control_ms;
uint32_t last_rx_log_ms;
uint32_t last_control_log_ms;
bool control_active;
ESPNowButtons button_events;
uint16_t last_rc5 = 1500;
uint16_t last_rc6 = ESPNowButtons::RC6_STABILIZE;
uint16_t last_control_sequence;
uint8_t last_control_mac[6];

bool init_nvs()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ret = nvs_flash_erase();
        if (ret != ESP_OK) {
            hal.console->printf("[ESPNOW] nvs erase failed: %d\n", int(ret));
            return false;
        }
        ret = nvs_flash_init();
    }

    if (ret != ESP_OK) {
        hal.console->printf("[ESPNOW] nvs init failed: %d\n", int(ret));
        return false;
    }
    return true;
}

bool init_netif_and_event_loop()
{
    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        hal.console->printf("[ESPNOW] esp_netif_init failed: %d\n", int(ret));
        return false;
    }

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        hal.console->printf("[ESPNOW] event loop init failed: %d\n", int(ret));
        return false;
    }
    return true;
}

bool ensure_wifi_started()
{
    wifi_mode_t mode = WIFI_MODE_NULL;
    esp_err_t ret = esp_wifi_get_mode(&mode);

    if (ret == ESP_ERR_WIFI_NOT_INIT) {
        if (!init_nvs() || !init_netif_and_event_loop()) {
            return false;
        }

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ret = esp_wifi_init(&cfg);
        if (ret != ESP_OK && ret != ESP_ERR_WIFI_INIT_STATE) {
            hal.console->printf("[ESPNOW] esp_wifi_init failed: %d\n", int(ret));
            return false;
        }

        ret = esp_wifi_set_storage(WIFI_STORAGE_RAM);
        if (ret != ESP_OK) {
            hal.console->printf("[ESPNOW] esp_wifi_set_storage failed: %d\n", int(ret));
            return false;
        }

        mode = WIFI_MODE_STA;
        ret = esp_wifi_set_mode(mode);
        if (ret != ESP_OK) {
            hal.console->printf("[ESPNOW] esp_wifi_set_mode STA failed: %d\n", int(ret));
            return false;
        }
    } else if (ret != ESP_OK) {
        hal.console->printf("[ESPNOW] esp_wifi_get_mode failed: %d\n", int(ret));
        return false;
    } else if (mode == WIFI_MODE_NULL) {
        mode = WIFI_MODE_STA;
        ret = esp_wifi_set_mode(mode);
        if (ret != ESP_OK) {
            hal.console->printf("[ESPNOW] esp_wifi_set_mode STA failed: %d\n", int(ret));
            return false;
        }
    }

    uint8_t primary_channel = 0;
    wifi_second_chan_t second_channel = WIFI_SECOND_CHAN_NONE;
    ret = esp_wifi_get_channel(&primary_channel, &second_channel);
    if (ret == ESP_ERR_WIFI_NOT_STARTED) {
        ret = esp_wifi_start();
        if (ret != ESP_OK) {
            hal.console->printf("[ESPNOW] esp_wifi_start failed: %d\n", int(ret));
            return false;
        }

        ret = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
        if (ret != ESP_OK) {
            hal.console->printf("[ESPNOW] esp_wifi_set_channel failed: %d\n", int(ret));
            return false;
        }
    } else if (ret == ESP_OK) {
#if WIFI_STATION
        // In station mode the radio channel is owned by the connected AP.
        // ESP-NOW must share that channel; forcing WIFI_CHANNEL can drop
        // the iPhone tethering link that carries MAVLink.
#else
        if (primary_channel != WIFI_CHANNEL) {
            esp_err_t channel_ret = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
            if (channel_ret != ESP_OK) {
                hal.console->printf("[ESPNOW] keep existing WiFi channel=%u, set channel %u failed: %d\n",
                                    unsigned(primary_channel),
                                    unsigned(WIFI_CHANNEL),
                                    int(channel_ret));
            }
        }
#endif
    } else {
        hal.console->printf("[ESPNOW] esp_wifi_get_channel failed: %d\n", int(ret));
        return false;
    }

    return true;
}

void format_mac(const uint8_t *mac, char *buffer, size_t buffer_size)
{
    if (mac == nullptr || buffer_size < 18) {
        if (buffer_size > 0) {
            buffer[0] = '\0';
        }
        return;
    }

    snprintf(buffer, buffer_size, "%02x:%02x:%02x:%02x:%02x:%02x",
             unsigned(mac[0]), unsigned(mac[1]), unsigned(mac[2]),
             unsigned(mac[3]), unsigned(mac[4]), unsigned(mac[5]));
}

bool should_log(uint32_t &last_log_ms)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (last_log_ms != 0 && now_ms - last_log_ms < HAL_ESP32_ESPNOW_LOG_INTERVAL_MS) {
        return false;
    }

    last_log_ms = now_ms;
    return true;
}

uint16_t constrain_pwm(int32_t value)
{
    if (value < 1000) {
        return 1000;
    }
    if (value > 2000) {
        return 2000;
    }
    return uint16_t(value);
}

uint16_t axis_to_pwm(int16_t axis)
{
    return constrain_pwm(1500 + (int32_t(axis) / 2));
}

uint16_t throttle_to_pwm(uint16_t throttle)
{
    if (throttle > 1000) {
        throttle = 1000;
    }
    return uint16_t(1000 + throttle);
}

void set_rc_input(const uint16_t *channels, uint8_t channel_count)
{
    ESP32::RCInput *rcin = static_cast<ESP32::RCInput *>(hal.rcin);
    rcin->set_espnow_input(channels, channel_count, 255, 255);
}

void set_rc_safe(uint16_t rc6, bool reset_flight_mode)
{
    const uint16_t channels[8] {
        1500,
        1500,
        1000,
        1500,
        1500,
        rc6,
        1500,
        1500,
    };
    set_rc_input(channels, 8);
    button_events.reset(AP_HAL::millis(), reset_flight_mode);
    last_rc5 = 1500;
    last_rc6 = rc6;
}

void apply_control_rcin(const AP_ESPNowControlPacket &packet, const uint8_t *mac)
{
    char mac_text[18];
    format_mac(mac, mac_text, sizeof(mac_text));

    const uint32_t now_ms = AP_HAL::millis();
    const bool same_source = memcmp(last_control_mac, mac, sizeof(last_control_mac)) == 0;
    const uint16_t sequence_delta = uint16_t(packet.sequence - last_control_sequence);
    const bool link_expired = !control_active || uint32_t(now_ms - last_control_ms) >= HAL_ESP32_ESPNOW_RCIN_TIMEOUT_MS;
    if (!link_expired && same_source && sequence_delta == 0) {
        return; // A duplicate cannot extend a hold or keep the link alive.
    }
    // A backwards sequence (excluding normal uint16 wrap) may be a reboot.
    // Re-establish the release gate even if reboot took less than the timeout.
    if (link_expired || !same_source || sequence_delta > 0x7fffU) {
        button_events.reset(now_ms);
        last_rc5 = 1500;
        hal.console->printf("[ESPNOW-BUTTON] link start/recovery RC5=1500 RC6=%u wait release\n",
                            unsigned(button_events.get_rc6()));
    }
    const ESPNowButtons::State previous_state = button_events.get_state();
    const uint16_t throttle_pwm = throttle_to_pwm(packet.z);
    const bool arm_allowed = throttle_pwm <= ESPNowButtons::ARM_THROTTLE_MAX_PWM;
    const ESPNowButtons::Outputs outputs = button_events.update(packet.buttons, now_ms, arm_allowed);
    if ((packet.buttons & (ESPNowButtons::LEFT | ESPNowButtons::RIGHT)) ==
        (ESPNowButtons::LEFT | ESPNowButtons::RIGHT) &&
        previous_state != ESPNowButtons::State::WAIT_RELEASE) {
        hal.console->printf("[ESPNOW-BUTTON] L+R cancel, wait release\n");
    }
    if (button_events.get_state() == ESPNowButtons::State::R_ARM_BLOCKED &&
        previous_state != ESPNowButtons::State::R_ARM_BLOCKED) {
        hal.console->printf("[ESPNOW-BUTTON] R long: ARM blocked RC3=%u; release R and lower throttle below %u\n",
                            unsigned(throttle_pwm),
                            unsigned(ESPNowButtons::ARM_THROTTLE_MAX_PWM));
    }
    if (outputs.rc5 != last_rc5) {
        hal.console->printf("[ESPNOW-BUTTON] %s RC5=%u state=%u now=%lu\n",
            outputs.rc5 == ESPNowButtons::RC5_HIGH ? "R 1s: ARM" :
            outputs.rc5 == ESPNowButtons::RC5_LOW ? "R short: DISARM" : "AUX MIDDLE",
            unsigned(outputs.rc5), unsigned(button_events.get_state()), (unsigned long)now_ms);
        last_rc5 = outputs.rc5;
    }
    if (outputs.rc6 != last_rc6) {
        hal.console->printf("[ESPNOW-BUTTON] %s RC6=%u state=%u now=%lu\n",
            ESPNowButtons::is_alt_hold(outputs.rc6) ? "L short: ALT_HOLD" :
            ESPNowButtons::is_loiter(outputs.rc6) ? "L 1s: LOITER" : "STABILIZE",
            unsigned(outputs.rc6), unsigned(button_events.get_state()), (unsigned long)now_ms);
        last_rc6 = outputs.rc6;
    }

    const uint16_t channels[8] {
        axis_to_pwm(packet.x),
        axis_to_pwm(packet.y),
        throttle_pwm,
        axis_to_pwm(packet.r),
        outputs.rc5,
        outputs.rc6,
        1500,
        1500,
    };
    set_rc_input(channels, 8);
    last_control_ms = now_ms;
    last_control_sequence = packet.sequence;
    memcpy(last_control_mac, mac, sizeof(last_control_mac));
    control_active = true;

    if (should_log(last_control_log_ms)) {
        hal.console->printf(
            "[ESPNOW] control rcin mac=%s seq=%u rc1=%u rc2=%u rc3=%u rc4=%u rc5=%u rc6=%u buttons=0x%04x\n",
            mac_text,
            unsigned(packet.sequence),
            unsigned(channels[0]),
            unsigned(channels[1]),
            unsigned(channels[2]),
            unsigned(channels[3]),
            unsigned(channels[4]),
            unsigned(channels[5]),
            unsigned(packet.buttons));
    }
}

void stop_rc_input_if_stale()
{
    if (!control_active) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_control_ms < HAL_ESP32_ESPNOW_RCIN_TIMEOUT_MS) {
        return;
    }

    // Preserve the existing link-loss RC output, including RC6=1500. The
    // configured FLTMODE4 determines the resulting flight mode until recovery.
    set_rc_safe(1500, false);
    control_active = false;
    hal.console->printf("[ESPNOW] rcin failsafe: no packets for %u ms; RC3=1000 RC5=1500 RC6=1500, no AUX DISARM\n",
                        unsigned(HAL_ESP32_ESPNOW_RCIN_TIMEOUT_MS));
}

#if !HAL_ESP32_ESPNOW_RCIN_ENABLED
void apply_control_stub(const AP_ESPNowControlPacket &packet, const uint8_t *mac)
{
    char mac_text[18];
    format_mac(mac, mac_text, sizeof(mac_text));

    hal.console->printf(
        "[ESPNOW] control stub mac=%s seq=%u x=%d y=%d z=%u r=%d buttons=0x%04x\n",
        mac_text,
        unsigned(packet.sequence),
        int(packet.x),
        int(packet.y),
        unsigned(packet.z),
        int(packet.r),
        unsigned(packet.buttons));
}
#endif

void worker(void *arg)
{
    (void)arg;

    ESPNOWFrame frame {};

#if HAL_ESP32_ESPNOW_RCIN_ENABLED
    // The ESP32 RCInput thread is initialised just after this worker starts.
    // Keep publishing the safe frame until RCInput confirms that it accepted
    // RC5 and RC6; no received button can be handled before that point.
    do {
        set_rc_safe(ESPNowButtons::RC6_STABILIZE, true);
        if (hal.rcin->read(4) == ESPNowButtons::RC5_CENTER &&
            hal.rcin->read(5) == ESPNowButtons::RC6_STABILIZE) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    } while (true);
    hal.console->printf("[ESPNOW-BUTTON] startup RC5=1500 RC6=1165/STABILIZE wait release\n");
#endif

    while (true) {
#if HAL_ESP32_ESPNOW_RCIN_ENABLED
        // Invalid packets must not keep the link alive by keeping the queue busy.
        stop_rc_input_if_stale();
#endif
        if (xQueueReceive(receive_queue, &frame, pdMS_TO_TICKS(100)) != pdTRUE) {
#if HAL_ESP32_ESPNOW_RCIN_ENABLED
            stop_rc_input_if_stale();
#endif
            continue;
        }

        char mac_text[18];
        format_mac(frame.source_mac, mac_text, sizeof(mac_text));

        if (frame.data_length != int(sizeof(AP_ESPNowControlPacket))) {
            hal.console->printf("[ESPNOW] drop mac=%s len=%d expected=%u\n",
                                mac_text,
                                frame.data_length,
                                unsigned(sizeof(AP_ESPNowControlPacket)));
            continue;
        }

        AP_ESPNowControlPacket packet {};
        memcpy(&packet, frame.data, sizeof(packet));

        if (!ap_espnow_control_packet_valid(packet)) {
            hal.console->printf("[ESPNOW] drop mac=%s invalid packet\n", mac_text);
            continue;
        }

        if (should_log(last_rx_log_ms)) {
            hal.console->printf("[ESPNOW] rx mac=%s seq=%u len=%u\n",
                                mac_text,
                                unsigned(packet.sequence),
                                unsigned(sizeof(packet)));
        }
#if HAL_ESP32_ESPNOW_RCIN_ENABLED
        apply_control_rcin(packet, frame.source_mac);
#else
        apply_control_stub(packet, frame.source_mac);
#endif
    }
}

void enqueue_frame(const uint8_t *source_mac, const uint8_t *data, int data_length)
{
    if (receive_queue == nullptr || source_mac == nullptr || data == nullptr) {
        return;
    }

    if (data_length < 0 || data_length > int(sizeof(AP_ESPNowControlPacket))) {
        return;
    }

    ESPNOWFrame frame {};
    memcpy(frame.source_mac, source_mac, sizeof(frame.source_mac));
    frame.data_length = data_length;
    memcpy(frame.data, data, data_length);

    xQueueSend(receive_queue, &frame, 0);
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void receive_callback(const esp_now_recv_info_t *info, const uint8_t *data, int data_length)
{
    const uint8_t *source_mac = info != nullptr ? info->src_addr : nullptr;
    enqueue_frame(source_mac, data, data_length);
}
#else
void receive_callback(const uint8_t *source_mac, const uint8_t *data, int data_length)
{
    enqueue_frame(source_mac, data, data_length);
}
#endif

} // namespace

bool ESPNowBridge::start_receiver()
{
    if (started) {
        return true;
    }

    if (!ensure_wifi_started()) {
        return false;
    }

    receive_queue = xQueueCreate(HAL_ESP32_ESPNOW_QUEUE_SIZE, sizeof(ESPNOWFrame));
    if (receive_queue == nullptr) {
        hal.console->printf("[ESPNOW] failed to create receive queue\n");
        return false;
    }

    const esp_err_t init_result = esp_now_init();
    if (init_result != ESP_OK) {
        hal.console->printf("[ESPNOW] esp_now_init failed: %d\n", int(init_result));
        vQueueDelete(receive_queue);
        receive_queue = nullptr;
        return false;
    }

    const esp_err_t cb_result = esp_now_register_recv_cb(receive_callback);
    if (cb_result != ESP_OK) {
        hal.console->printf("[ESPNOW] register receive callback failed: %d\n", int(cb_result));
        esp_now_deinit();
        vQueueDelete(receive_queue);
        receive_queue = nullptr;
        return false;
    }

    if (xTaskCreatePinnedToCore(worker,
                                "APM_ESPNOW",
                                HAL_ESP32_ESPNOW_TASK_STACK,
                                nullptr,
                                Scheduler::WIFI_PRIO2,
                                &worker_task,
                                1) != pdPASS) {
        hal.console->printf("[ESPNOW] failed to create worker task\n");
        esp_now_unregister_recv_cb();
        esp_now_deinit();
        vQueueDelete(receive_queue);
        receive_queue = nullptr;
        return false;
    }

    uint8_t primary_channel = 0;
    wifi_second_chan_t second_channel = WIFI_SECOND_CHAN_NONE;
    esp_wifi_get_channel(&primary_channel, &second_channel);

    started = true;
    hal.console->printf("[ESPNOW] receiver started channel=%u\n", unsigned(primary_channel));
    return true;
}

bool ESPNowBridge::is_started()
{
    return started;
}

#else

bool ESPNowBridge::start_receiver()
{
    return false;
}

bool ESPNowBridge::is_started()
{
    return false;
}

#endif
