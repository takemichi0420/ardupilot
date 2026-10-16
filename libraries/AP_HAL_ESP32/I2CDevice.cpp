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
#include "I2CDevice.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL_ESP32/Semaphores.h>
#include "Scheduler.h"
#include "FlightLoadDiagnostics.h"
#include "driver/gpio.h"
#include <stdio.h>

using namespace ESP32;

#define MHZ (1000U*1000U)
#define KHZ (1000U)

#ifndef HAL_ESP32_I2C_DIAGNOSTICS
#define HAL_ESP32_I2C_DIAGNOSTICS 0
#endif

#define esp32_i2c_diag(fmt, args ...)  do { printf(fmt, ## args); printf("\n"); GCS_SEND_TEXT(MAV_SEVERITY_INFO, fmt, ## args); } while(0)

I2CBusDesc i2c_bus_desc[] = { HAL_ESP32_I2C_BUSES };

I2CBus I2CDeviceManager::businfo[ARRAY_SIZE(i2c_bus_desc)];

I2CDeviceManager::I2CDeviceManager(void)
{
    for (uint8_t i=0; i<ARRAY_SIZE(i2c_bus_desc); i++) {
        if (i2c_bus_desc[i].soft) {
            businfo[i].sw_handle.sda = i2c_bus_desc[i].sda;
            businfo[i].sw_handle.scl = i2c_bus_desc[i].scl;
            //TODO make modular
            businfo[i].sw_handle.speed = I2C_SPEED_FAST;
            businfo[i].soft = true;
            i2c_init(&(businfo[i].sw_handle));
        } else {
            i2c_config_t i2c_bus_config;
            i2c_bus_config.mode = I2C_MODE_MASTER;
            i2c_bus_config.sda_io_num = i2c_bus_desc[i].sda;
            i2c_bus_config.scl_io_num = i2c_bus_desc[i].scl;
            i2c_bus_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
            i2c_bus_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
            i2c_bus_config.master.clk_speed = i2c_bus_desc[i].speed;
            i2c_bus_config.clk_flags = 0;
            i2c_port_t p = i2c_bus_desc[i].port;
            businfo[i].port = p;
            businfo[i].bus_clock = i2c_bus_desc[i].speed;
            businfo[i].soft = false;
            esp_err_t err = i2c_param_config(p, &i2c_bus_config);
            if (err != ESP_OK) {
                esp32_i2c_diag("ESP32 I2C: bus config failed bus=%u err=%d", (unsigned)i, (int)err);
            }
            err = i2c_driver_install(p, I2C_MODE_MASTER, 0, 0, ESP_INTR_FLAG_IRAM);
            if (err != ESP_OK) {
                esp32_i2c_diag("ESP32 I2C: driver install failed bus=%u err=%d", (unsigned)i, (int)err);
            }
            err = i2c_filter_enable(p, 7);
            if (err != ESP_OK) {
                esp32_i2c_diag("ESP32 I2C: filter enable failed bus=%u err=%d", (unsigned)i, (int)err);
            }
        }
    }
}

I2CDevice::I2CDevice(uint8_t busnum, uint8_t address, uint32_t bus_clock, bool use_smbus, uint32_t timeout_ms) :
    bus(I2CDeviceManager::businfo[busnum]),
    _retries(10),
    _address(address),
    _timeout_ms(timeout_ms),
    _transfer_failure_reported(false)
{
    set_device_bus(busnum);
    set_device_address(address);
    asprintf(&pname, "I2C:%u:%02x",
             (unsigned)busnum, (unsigned)address);
}

I2CDevice::~I2CDevice()
{
    free(pname);
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    if (!bus.semaphore.check_owner()) {
        printf("I2C: not owner of 0x%x\n", (unsigned)get_bus_id());
        return false;
    }

    bool result = false;
#if HAL_ESP32_FLIGHT_LOAD_DIAGNOSTICS
    const uint32_t diagnostic_start_us = AP_HAL::micros();
    uint8_t diagnostic_attempts = 0;
#endif
    if (bus.soft) {
        uint8_t flag_wr = (recv_len == 0 || recv == nullptr) ? I2C_NOSTOP : 0;
        if (send_len != 0 && send != nullptr) {
            //tx with optional rx (after tx)
            i2c_write_bytes(&bus.sw_handle,
                            _address,
                            send,
                            send_len,
                            flag_wr);
        }
        if (recv_len != 0 && recv != nullptr) {
            //rx only or rx after tx
            //rx separated from tx by (re)start
            i2c_read_bytes(&bus.sw_handle,
                           _address,
                           (uint8_t *)recv, recv_len, 0);
        }
        result = true; //TODO check all
    } else {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if (send_len != 0 && send != nullptr) {
            //tx with optional rx (after tx)
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write(cmd, (uint8_t*)send, send_len, true);
        }
        if (recv_len != 0 && recv != nullptr) {
            //rx only or rx after tx
            //rx separated from tx by (re)start
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_READ, true);
            i2c_master_read(cmd, (uint8_t *)recv, recv_len, I2C_MASTER_LAST_NACK);
        }
        i2c_master_stop(cmd);

        uint32_t timeout_ms = 1 + 16L * (send_len + recv_len) * 1000 / bus.bus_clock;
        timeout_ms = MAX(timeout_ms, _timeout_ms);
        for (int i = 0; !result && i < _retries; i++) {
#if HAL_ESP32_FLIGHT_LOAD_DIAGNOSTICS
            diagnostic_attempts++;
#endif
            result = (i2c_master_cmd_begin(bus.port, cmd, pdMS_TO_TICKS(timeout_ms)) == ESP_OK);
            if (!result) {
                i2c_reset_tx_fifo(bus.port);
                i2c_reset_rx_fifo(bus.port);
            }
        }

        i2c_cmd_link_delete(cmd);
    }

#if HAL_ESP32_FLIGHT_LOAD_DIAGNOSTICS
    FlightLoadDiagnostics::vl53_i2c_transfer(
        _address,
        AP_HAL::micros() - diagnostic_start_us,
        diagnostic_attempts > 0 ? diagnostic_attempts - 1 : 0,
        recv != nullptr && recv_len > 0);
#endif

    if (!result && !_transfer_failure_reported) {
        _transfer_failure_reported = true;
        esp32_i2c_diag("ESP32 I2C: transfer failed bus=%u addr=0x%02x",
                       (unsigned)bus_num(), (unsigned)_address);
    }

    return result;
}

/*
  register a periodic callback
*/
AP_HAL::Device::PeriodicHandle I2CDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return bus.register_periodic_callback(period_usec, cb, this);
}


/*
  adjust a periodic callback
*/
bool I2CDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    return bus.adjust_timer(h, period_usec);
}

AP_HAL::I2CDevice *
I2CDeviceManager::get_device_ptr(uint8_t bus, uint8_t address,
                                 uint32_t bus_clock,
                                 bool use_smbus,
                                 uint32_t timeout_ms)
{
    if (bus >= ARRAY_SIZE(i2c_bus_desc)) {
        esp32_i2c_diag("ESP32 I2C: open failed bus=%u addr=0x%02x", (unsigned)bus, (unsigned)address);
        return nullptr;
    }
    AP_HAL::I2CDevice *dev = NEW_NOTHROW I2CDevice(bus, address, bus_clock, use_smbus, timeout_ms);
    if (dev == nullptr) {
        esp32_i2c_diag("ESP32 I2C: open failed bus=%u addr=0x%02x alloc",
                       (unsigned)bus, (unsigned)address);
    }
    return dev;
}

void I2CDeviceManager::scan_bus(uint8_t busnum, const char *context)
{
#if HAL_ESP32_I2C_DIAGNOSTICS
    if (busnum >= ARRAY_SIZE(i2c_bus_desc)) {
        printf("[I2C-SCAN] invalid bus=%u %s\n",
               (unsigned)busnum,
               context == nullptr ? "" : context);
        return;
    }

    if (businfo[busnum].soft) {
        printf("[I2C-SCAN] soft bus unsupported bus=%u %s\n",
               (unsigned)busnum,
               context == nullptr ? "" : context);
        return;
    }

    auto &semaphore = businfo[busnum].semaphore;
    // Allow one short transaction (the default device timeout is 4 ms) plus
    // a scheduler tick. A busy bus is a reason to skip optional diagnostics.
    static constexpr uint32_t lock_timeout_ms = 5;
    if (!semaphore.take(lock_timeout_ms)) {
        printf("[I2C-SCAN] bus=%u bus lock timeout; scan skipped\n", (unsigned)busnum);
        return;
    }

    enum class ProbeResult : uint8_t { NOT_PROBED, FOUND, NOT_FOUND, TIMEOUT, ERROR };
    ProbeResult results[0x78] {};
    uint8_t found = 0;
    uint8_t not_found = 0;
    uint8_t timeouts = 0;
    uint8_t errors = 0;
    bool aborted = false;
    printf("[I2C-SCAN] start bus=%u SDA=GPIO%d SCL=GPIO%d speed=%luHz %s\n",
           (unsigned)busnum,
           (int)i2c_bus_desc[busnum].sda,
           (int)i2c_bus_desc[busnum].scl,
           (unsigned long)businfo[busnum].bus_clock,
           context == nullptr ? "" : context);

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        // Address ACK probe only: no register, payload or read. Device state
        // changes, including those caused by SDK bus recovery, are not ruled out.
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if (cmd == nullptr) {
            errors++;
            results[addr] = ProbeResult::ERROR;
            aborted = true;
            printf("[I2C-SCAN] command allocation failed at 0x%02X; scan aborted\n",
                   (unsigned)addr);
            break;
        }

        const char *stage = "START";
        esp_err_t err = i2c_master_start(cmd);
        if (err == ESP_OK) {
            stage = "address";
            err = i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        }
        if (err == ESP_OK) {
            stage = "STOP";
            err = i2c_master_stop(cmd);
        }
        if (err != ESP_OK) {
            i2c_cmd_link_delete(cmd);
            errors++;
            results[addr] = ProbeResult::ERROR;
            aborted = true;
            printf("[I2C-SCAN] command construction failed at 0x%02X stage=%s err=%d; scan aborted\n",
                   (unsigned)addr, stage, (int)err);
            break;
        }

        err = i2c_master_cmd_begin(businfo[busnum].port, cmd, pdMS_TO_TICKS(5));
        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK) {
            results[addr] = ProbeResult::FOUND;
            found++;
            printf("[I2C-SCAN] found 0x%02X\n", (unsigned)addr);
        } else if (err == ESP_FAIL) {
            // In ESP-IDF 5.3.1, cmd_begin maps I2C_STATUS_ACK_ERROR to
            // ESP_FAIL. Here only the address is sent, so this is an address
            // NACK, not proof that no physical device is connected.
            results[addr] = ProbeResult::NOT_FOUND;
            not_found++;
        } else {
            if (err == ESP_ERR_TIMEOUT) {
                results[addr] = ProbeResult::TIMEOUT;
                timeouts++;
                printf("[I2C-SCAN] probe timeout at 0x%02X; scan aborted\n", (unsigned)addr);
            } else {
                results[addr] = ProbeResult::ERROR;
                errors++;
                printf("[I2C-SCAN] driver error at 0x%02X err=%d; scan aborted\n",
                       (unsigned)addr, (int)err);
            }
            // Leave SDK recovery to cmd_begin; do not keep probing a failed bus.
            aborted = true;
            break;
        }
    }

    static constexpr uint8_t expected_addresses[] { 0x29, 0x40, 0x76 };
    for (const uint8_t addr : expected_addresses) {
        if (results[addr] != ProbeResult::FOUND) {
            const char *status = "NOT PROBED";
            switch (results[addr]) {
            case ProbeResult::NOT_FOUND: status = "NOT FOUND (address NACK)"; break;
            case ProbeResult::TIMEOUT: status = "TIMEOUT"; break;
            case ProbeResult::ERROR: status = "ERROR"; break;
            default: break;
            }
            printf("[I2C-SCAN] 0x%02X %s\n", (unsigned)addr, status);
        }
    }

    printf("[I2C-SCAN] end bus=%u found=%u not_found=%u timeout=%u error=%u aborted=%u SDA=%d SCL=%d\n",
           (unsigned)busnum,
           (unsigned)found,
           (unsigned)not_found,
           (unsigned)timeouts,
           (unsigned)errors,
           (unsigned)aborted,
           (int)gpio_get_level(i2c_bus_desc[busnum].sda),
           (int)gpio_get_level(i2c_bus_desc[busnum].scl));
    semaphore.give();
#endif
}

/*
  get mask of bus numbers for all configured I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask(void) const
{
    return ((1U << ARRAY_SIZE(i2c_bus_desc)) - 1);
}

/*
  get mask of bus numbers for all configured internal I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_internal(void) const
{
    uint32_t result = 0;
    for (size_t i = 0; i < ARRAY_SIZE(i2c_bus_desc); i++) {
        if (i2c_bus_desc[i].internal) {
            result |= (1u << i);
        }
    }
    return result;
}

/*
  get mask of bus numbers for all configured external I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_external(void) const
{
    return get_bus_mask() & ~get_bus_mask_internal();
}
