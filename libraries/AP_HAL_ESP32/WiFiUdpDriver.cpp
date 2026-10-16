/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL_ESP32/WiFiUdpDriver.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_ESP32/Scheduler.h>
#include <AP_HAL_ESP32/FlightLoadDiagnostics.h>

#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "freertos/idf_additions.h"

using namespace ESP32;

extern const AP_HAL::HAL& hal;

#define UDP_PORT 14550

#if WIFI_STATION
#ifndef GCS_IP
// Do not assume which DHCP address the hotspot assigns to the GCS. Once a
// client sends MAVLink traffic, replies switch to its learned unicast address.
#define GCS_IP "255.255.255.255"
#endif
#ifndef GCS_PORT
#define GCS_PORT UDP_PORT
#endif
#ifndef GCS_PORT_TEXT
#define GCS_PORT_TEXT "14550"
#endif
#ifndef WIFI_INITIAL_CONNECT_TIMEOUT_MS
#define WIFI_INITIAL_CONNECT_TIMEOUT_MS 15000
#endif
#ifndef WIFI_STA_POWER_SAVE
#define WIFI_STA_POWER_SAVE WIFI_PS_MIN_MODEM
#endif
#ifndef WIFI_UDP_CLIENT_TIMEOUT_MS
#define WIFI_UDP_CLIENT_TIMEOUT_MS 5000
#endif
#endif

#ifndef WIFI_UDP_POLL_MS
#define WIFI_UDP_POLL_MS 20
#endif

WiFiUdpDriver::WiFiUdpDriver()
{
    _state = NOT_INITIALIZED;
    accept_socket = -1;
    bzero(&_last_client_addr, sizeof(_last_client_addr));
    _have_client_addr = false;
    _last_client_ms = 0;
}

void WiFiUdpDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    hal.console->printf("WiFiUdpDriver entered\n");
    if (_state == NOT_INITIALIZED) {
        initialize_wifi();
        if (!start_listen()) {
            return;
        }

    // keep main tasks that need speed on CPU 0
    // pin potentially slow stuff to CPU 1, as we have disabled the WDT on that core.
    #define FASTCPU 0
    #define SLOWCPU 1

	if (!_readbuf.set_size(RX_BUF_SIZE) || !_writebuf.set_size(TX_BUF_SIZE)) {
            AP_HAL::panic("MAVLink UDP buffer allocation failed");
        }
        _state = INITIALIZED;

	if (xTaskCreatePinnedToCore(_wifi_thread2, "APM_WIFI2", Scheduler::WIFI_SS2, this, Scheduler::WIFI_PRIO2, &_wifi_task_handle, SLOWCPU) != pdPASS) {
            hal.console->printf("FAILED to create task _wifi_thread2 on SLOWCPU\n");
            _state = NOT_INITIALIZED;
        } else {
	    hal.console->printf("OK created task _wifi_thread2 for UDP on port 14550 on SLOWCPU\n"); //UDP_PORT
    	}
    }
}

void WiFiUdpDriver::_end()
{
    //TODO
}

void WiFiUdpDriver::_flush()
{
}

bool WiFiUdpDriver::is_initialized()
{
    return _state != NOT_INITIALIZED;
}

bool WiFiUdpDriver::tx_pending()
{
    return (_writebuf.available() > 0);
}

uint32_t WiFiUdpDriver::_available()
{
    return _readbuf.available();
}

uint32_t WiFiUdpDriver::txspace()
{
    int result =  _writebuf.space();
    result -= TX_BUF_SIZE / 4;
    return MAX(result, 0);
}

ssize_t WiFiUdpDriver::_read(uint8_t *buf, uint16_t count)
{
    if (!_read_mutex.take_nonblocking()) {
        return false;
    }

    auto ret = _readbuf.read(buf, count);

    _read_mutex.give();
    return ret;
}

bool WiFiUdpDriver::start_listen()
{
    if (accept_socket >= 0) {
        close(accept_socket);
        accept_socket = -1;
    }

    accept_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (accept_socket < 0) {
        accept_socket = -1;
        hal.console->printf("MAVLink UDP socket creation failed errno=%d\n", errno);
        return false;
    }
    int opt = 1;
    setsockopt(accept_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(accept_socket, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));
    struct sockaddr_in destAddr;
    destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(UDP_PORT);
    int err = bind(accept_socket, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (err != 0) {
        hal.console->printf("MAVLink UDP bind failed errno=%d\n", errno);
        close(accept_socket);
        accept_socket = -1;
        return false;
    }
    //memset(&client_addr, 0, sizeof(client_addr));
    fcntl(accept_socket, F_SETFL, O_NONBLOCK);

    return true;

}

bool WiFiUdpDriver::read_all()
{
    _read_mutex.take_blocking();
    bool received = false;

    // Drain a bounded number of datagrams so Mission Planner parameter and
    // command bursts do not wait for another select interval.
    for (uint8_t i = 0; i < 8; i++) {
        struct sockaddr_in client_addr {};
        socklen_t socklen = sizeof(client_addr);
        const int count = recvfrom(accept_socket, _buffer, sizeof(_buffer), 0,
                                   (struct sockaddr *)&client_addr, &socklen);
        if (count <= 0) {
            break;
        }

        const bool client_changed = !_have_client_addr ||
                                    client_addr.sin_addr.s_addr != _last_client_addr.sin_addr.s_addr ||
                                    client_addr.sin_port != _last_client_addr.sin_port;
        _last_client_addr = client_addr;
        _have_client_addr = true;
        _last_client_ms = AP_HAL::millis();
        _readbuf.write(_buffer, count);
        received = true;

        if (client_changed) {
            hal.console->printf("MAVLink UDP client %s:%u\n",
                                inet_ntoa(client_addr.sin_addr),
                                unsigned(ntohs(client_addr.sin_port)));
        }
    }

    _read_mutex.give();
    return received;
}

bool WiFiUdpDriver::write_data()
{

    _write_mutex.take_blocking();
    int count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
    if (count > 0) {
        struct sockaddr_in dest_addr;
#if WIFI_STATION
        const bool client_is_current = _have_client_addr &&
                                       AP_HAL::millis() - _last_client_ms <= WIFI_UDP_CLIENT_TIMEOUT_MS;
        if (client_is_current) {
            dest_addr = _last_client_addr;
        } else {
            _have_client_addr = false;
            dest_addr.sin_addr.s_addr = inet_addr(GCS_IP);
            dest_addr.sin_family = AF_INET;
            dest_addr.sin_port = htons(GCS_PORT);
        }

        static bool using_fallback;
        if (!client_is_current && !using_fallback) {
            hal.console->printf("MAVLink UDP fallback " GCS_IP ":" GCS_PORT_TEXT "\n");
        }
        using_fallback = !client_is_current;
#else
        if (_have_client_addr) {
            dest_addr = _last_client_addr;
        } else {
            dest_addr.sin_addr.s_addr = inet_addr("192.168.4.255");
            dest_addr.sin_family = AF_INET;
            dest_addr.sin_port = htons(UDP_PORT);
        }
#endif
        count = sendto(accept_socket, _buffer, count, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (count > 0) {
            _writebuf.advance(count);
        } else {
            _write_mutex.give();
            return false;
        }
    }
    _write_mutex.give();
    return true;
}

#if WIFI_STATION

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WIFI_SOCKET_RESET_BIT BIT2

#ifndef ESP_STATION_MAXIMUM_RETRY
#define ESP_STATION_MAXIMUM_RETRY 10
#endif

static const char *TAG = "wifi station";
static uint32_t s_retry_num;
static EventGroupHandle_t s_wifi_event_group;

static void _sta_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_wifi_event_group != nullptr) {
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            xEventGroupSetBits(s_wifi_event_group, WIFI_SOCKET_RESET_BIT);
        }

        if (s_retry_num < UINT32_MAX) {
            s_retry_num++;
        }
        if (s_retry_num == ESP_STATION_MAXIMUM_RETRY && s_wifi_event_group != nullptr) {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }

        const auto *disconnected = static_cast<wifi_event_sta_disconnected_t *>(event_data);
        if (s_retry_num <= 3 || s_retry_num == ESP_STATION_MAXIMUM_RETRY || (s_retry_num % 10) == 0) {
            ESP_LOGI(TAG, "disconnected reason=%u retry=%lu",
                     disconnected != nullptr ? unsigned(disconnected->reason) : 0U,
                     (unsigned long)s_retry_num);
        }

        const esp_err_t ret = esp_wifi_connect();
        if (ret != ESP_OK && ret != ESP_ERR_WIFI_STATE) {
            ESP_LOGW(TAG, "esp_wifi_connect failed: %d", int(ret));
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0;
        if (s_wifi_event_group != nullptr) {
            xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_SOCKET_RESET_BIT);
        }

        const auto *got_ip = static_cast<ip_event_got_ip_t *>(event_data);
        hal.console->printf("WiFi connected\n");
        if (got_ip != nullptr) {
            hal.console->printf("IP: " IPSTR "\n", IP2STR(&got_ip->ip_info.ip));
        }
    }
}
#endif

void WiFiUdpDriver::initialize_wifi()
{
#ifndef WIFI_PWD
    #define WIFI_PWD "ardupilot1"
#endif
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_config_t wifi_config;
    bzero(&wifi_config, sizeof(wifi_config));

/*
	Acting as an Access Point (softAP)
*/
#if !WIFI_STATION
#ifndef WIFI_SSID
    #define WIFI_SSID "ardupilot"
#endif
#ifndef WIFI_CHANNEL
	#define WIFI_CHANNEL 1
#endif

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));


    strcpy((char *)wifi_config.ap.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.ap.password, WIFI_PWD);
    wifi_config.ap.ssid_len = strlen(WIFI_SSID),
    wifi_config.ap.max_connection = WIFI_MAX_CONNECTION,
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.ap.channel = WIFI_CHANNEL;

    if (strlen(WIFI_PWD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    hal.console->printf("WiFi softAP init finished. SSID: %s channel: %d\n",
                        wifi_config.ap.ssid, wifi_config.ap.channel);

/*
	Acting as a Station (WiFi Client)
*/
#else
#ifndef WIFI_SSID_STATION
    #define WIFI_SSID_STATION "ardupilot"
#endif
#ifndef WIFI_HOSTNAME
    #define WIFI_HOSTNAME "ArduPilotESP32"
#endif
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == nullptr) {
        AP_HAL::panic("WiFi event group allocation failed");
    }
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    if (netif == nullptr) {
        AP_HAL::panic("WiFi station netif creation failed");
    }
    esp_netif_set_hostname(netif, WIFI_HOSTNAME);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &_sta_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &_sta_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    strcpy((char *)wifi_config.sta.ssid, WIFI_SSID_STATION);
    strcpy((char *)wifi_config.sta.password, WIFI_PWD);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
    wifi_config.sta.failure_retry_cnt = 3;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_STA_POWER_SAVE));

    hal.console->printf("WiFi connecting...\n");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE, pdFALSE, pdMS_TO_TICKS(WIFI_INITIAL_CONNECT_TIMEOUT_MS));

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to SSID: %s", wifi_config.sta.ssid);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGW(TAG, "initial connection failed for SSID: %s; retrying in background",
                 wifi_config.sta.ssid);
    } else {
        ESP_LOGW(TAG, "initial connection timed out for SSID: %s; retrying in background",
                 wifi_config.sta.ssid);
    }
#endif
}

size_t WiFiUdpDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_write_mutex.take_nonblocking()) {
        return 0;
    }
    size_t ret = _writebuf.write(buffer, size);
    _write_mutex.give();
    return ret;
}

void WiFiUdpDriver::_wifi_thread2(void *arg)
{
    WiFiUdpDriver *self = (WiFiUdpDriver *) arg;
    while (true) {
#if WIFI_STATION
        EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
        if ((bits & WIFI_SOCKET_RESET_BIT) != 0) {
            xEventGroupClearBits(s_wifi_event_group, WIFI_SOCKET_RESET_BIT);
            if (self->accept_socket >= 0) {
                close(self->accept_socket);
                self->accept_socket = -1;
            }
            self->_have_client_addr = false;
            self->_last_client_ms = 0;
            self->_read_mutex.take_blocking();
            self->_readbuf.clear();
            self->_read_mutex.give();
            self->_write_mutex.take_blocking();
            self->_writebuf.clear();
            self->_write_mutex.give();
        }

        bits = xEventGroupGetBits(s_wifi_event_group);
        if ((bits & WIFI_CONNECTED_BIT) == 0) {
            vTaskDelay(pdMS_TO_TICKS(WIFI_UDP_POLL_MS));
            continue;
        }

        if (self->accept_socket < 0 && !self->start_listen()) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
#endif

        struct timeval tv = {
            .tv_sec = 0,
            .tv_usec = WIFI_UDP_POLL_MS * 1000,
        };
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(self->accept_socket, &rfds);
        int s = select(self->accept_socket + 1, &rfds, NULL, NULL, &tv);
#if HAL_ESP32_FLIGHT_LOAD_DIAGNOSTICS
        const uint32_t wifi_io_start_us = AP_HAL::micros();
#endif
        if (s > 0 && FD_ISSET(self->accept_socket, &rfds)) {
            self->read_all();
        }
        self->write_data();
#if HAL_ESP32_FLIGHT_LOAD_DIAGNOSTICS
        FlightLoadDiagnostics::wifi_io(AP_HAL::micros() - wifi_io_start_us);
#endif
    }
}

bool WiFiUdpDriver::_discard_input()
{
    return false;
}
