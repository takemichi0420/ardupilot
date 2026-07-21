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

#include "SPIDevice.h"

#include <AP_HAL/AP_HAL.h>

#if AP_HAL_SPI_ENABLED

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include "Scheduler.h"
#include "Semaphores.h"
#include <stdio.h>

using namespace ESP32;

#define MHZ (1000U*1000U)
#define KHZ (1000U)

#ifndef HAL_ESP32_SPI_POLLING_TRANSACTIONS
#define HAL_ESP32_SPI_POLLING_TRANSACTIONS 0
#endif

//#define SPIDEBUG 1

#define esp32_spi_diag(fmt, args ...)  do { printf(fmt, ## args); printf("\n"); GCS_SEND_TEXT(MAV_SEVERITY_INFO, fmt, ## args); } while(0)

#ifdef HAL_ESP32_SPI_BUSES
SPIBusDesc bus_desc[] = {HAL_ESP32_SPI_BUSES};
#endif  // HAL_ESP32_SPI_BUSES

#ifdef HAL_ESP32_SPI_DEVICES
SPIDeviceDesc device_desc[] = {HAL_ESP32_SPI_DEVICES};
#endif  // HAL_ESP32_SPI_DEVICES

#ifdef HAL_ESP32_SPI_BUSES
SPIBus::SPIBus(uint8_t _bus):
    DeviceBus(Scheduler::SPI_PRIORITY), bus(_bus)
{

#ifdef SPIDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    spi_bus_config_t config = {
        .mosi_io_num = bus_desc[_bus].mosi,
        .miso_io_num = bus_desc[_bus].miso,
        .sclk_io_num = bus_desc[_bus].sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    const esp_err_t err = spi_bus_initialize(bus_desc[_bus].host, &config, bus_desc[_bus].dma_ch);
    if (err != ESP_OK) {
        esp32_spi_diag("ESP32 SPI: bus init failed bus=%u err=%d", (unsigned)_bus, (int)err);
    }
}
#endif  // HAL_ESP32_SPI_BUSES


#ifdef HAL_ESP32_SPI_DEVICES
SPIDevice::SPIDevice(SPIBus &_bus, SPIDeviceDesc &_device_desc)
    : bus(_bus)
    , device_desc(_device_desc)
    , speed(AP_HAL::Device::SPEED_LOW)
    , pname(nullptr)
    , low_speed_dev_handle(nullptr)
    , high_speed_dev_handle(nullptr)
    , forced_cs_handle(nullptr)
    , cs_forced(false)
{

#ifdef SPIDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    set_device_bus(bus.bus);
    set_device_address(_device_desc.device);
    set_speed(AP_HAL::Device::SPEED_LOW);

    spi_device_interface_config_t cfg_low;
    memset(&cfg_low, 0, sizeof(cfg_low));
    cfg_low.mode = _device_desc.mode;
    cfg_low.clock_speed_hz = _device_desc.lspeed;
    cfg_low.spics_io_num = -1;
    cfg_low.queue_size = 5;
    esp_err_t err = spi_bus_add_device(bus_desc[_bus.bus].host, &cfg_low, &low_speed_dev_handle);
    if (err != ESP_OK) {
        esp32_spi_diag("ESP32 SPI: open failed name=%s bus=%u low err=%d",
                       _device_desc.name, (unsigned)_device_desc.bus, (int)err);
    }
    if (_device_desc.hspeed != _device_desc.lspeed) {
        spi_device_interface_config_t cfg_high;
        memset(&cfg_high, 0, sizeof(cfg_high));
        cfg_high.mode = _device_desc.mode;
        cfg_high.clock_speed_hz = _device_desc.hspeed;
        cfg_high.spics_io_num = -1;
        cfg_high.queue_size = 5;
        err = spi_bus_add_device(bus_desc[_bus.bus].host, &cfg_high, &high_speed_dev_handle);
        if (err != ESP_OK) {
            esp32_spi_diag("ESP32 SPI: open failed name=%s bus=%u high err=%d",
                           _device_desc.name, (unsigned)_device_desc.bus, (int)err);
        }
    }


    asprintf(&pname, "SPI:%s:%u:%u",
             device_desc.name, 0, (unsigned)device_desc.device);
    printf("spi device constructed %s\n", pname);
}
#endif  // HAL_ESP32_SPI_DEVICES

SPIDevice::~SPIDevice()
{
    free(pname);
}

spi_device_handle_t SPIDevice::current_handle()
{
    if (speed == AP_HAL::Device::SPEED_HIGH && high_speed_dev_handle != nullptr) {
        return high_speed_dev_handle;
    }
    return low_speed_dev_handle;
}

bool SPIDevice::set_speed(AP_HAL::Device::Speed _speed)
{
#ifdef SPIDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    speed = _speed;
    return true;
}

bool SPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
#ifdef SPIDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (!send || !recv) {
        // simplest cases
        return transfer_fullduplex(send, recv, recv_len?recv_len:send_len);
    }
    uint8_t buf[send_len+recv_len];
    if (send_len > 0) {
        memcpy(buf, send, send_len);
    }
    if (recv_len > 0) {
        memset(&buf[send_len], 0, recv_len);
    }
    if (!transfer_fullduplex(buf, buf, send_len+recv_len)) {
        return false;
    }
    if (recv_len > 0) {
        memcpy(recv, &buf[send_len], recv_len);
    }
    return true;
}

bool SPIDevice::transfer_fullduplex(uint8_t *send_recv, uint32_t len)
{
    return transfer_fullduplex(send_recv, send_recv, len);
}

bool SPIDevice::transfer_fullduplex(const uint8_t *send, uint8_t *recv, uint32_t len)
{
#ifdef SPIDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    spi_device_handle_t handle = current_handle();
    if (handle == nullptr) {
        esp32_spi_diag("ESP32 SPI: transfer failed name=%s no handle", device_desc.name);
        return false;
    }
    const bool acquired_here = !cs_forced;
    if (acquired_here) {
        if (!acquire_bus(true)) {
            return false;
        }
    } else if (forced_cs_handle != handle) {
        esp32_spi_diag("ESP32 SPI: transfer failed name=%s speed changed while CS forced", device_desc.name);
        return false;
    }
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = len*8;
    t.tx_buffer = send;
    t.rxlength = len*8;
    t.rx_buffer = recv;
#if HAL_ESP32_SPI_POLLING_TRANSACTIONS
    const esp_err_t err = spi_device_polling_transmit(handle, &t);
#else
    const esp_err_t err = spi_device_transmit(handle, &t);
#endif
    bool release_ok = true;
    if (acquired_here) {
        release_ok = acquire_bus(false);
    }
    if (err != ESP_OK) {
        esp32_spi_diag("ESP32 SPI: transfer failed name=%s err=%d", device_desc.name, (int)err);
        return false;
    }
    return release_ok;
}

bool SPIDevice::set_cs_level(bool asserted)
{
    const esp_err_t err = gpio_set_level(device_desc.cs, asserted ? 0 : 1);
    if (err != ESP_OK) {
        esp32_spi_diag("ESP32 SPI: CS set failed name=%s gpio=%u level=%u err=%d",
                       device_desc.name,
                       (unsigned)device_desc.cs,
                       (unsigned)!asserted,
                       (int)err);
        return false;
    }
    return true;
}

bool SPIDevice::acquire_bus(bool acquire)
{
#ifdef SPIDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    spi_device_handle_t handle = current_handle();
    if (handle == nullptr) {
        return false;
    }

    if (acquire) {
        const esp_err_t err = spi_device_acquire_bus(handle, portMAX_DELAY);
        if (err != ESP_OK) {
            esp32_spi_diag("ESP32 SPI: bus acquire failed name=%s err=%d", device_desc.name, (int)err);
            return false;
        }
        if (!set_cs_level(true)) {
            spi_device_release_bus(handle);
            return false;
        }
    } else {
        const bool result = set_cs_level(false);
        spi_device_release_bus(handle);
        return result;
    }
    return true;
}

bool SPIDevice::set_chip_select(bool set)
{
    if (!bus.semaphore.check_owner()) {
        esp32_spi_diag("ESP32 SPI: set CS failed name=%s bus semaphore not owned", device_desc.name);
        return false;
    }

    if (set) {
        if (cs_forced) {
            return true;
        }
        spi_device_handle_t handle = current_handle();
        if (handle == nullptr) {
            return false;
        }
        const esp_err_t err = spi_device_acquire_bus(handle, portMAX_DELAY);
        if (err != ESP_OK) {
            esp32_spi_diag("ESP32 SPI: forced CS acquire failed name=%s err=%d", device_desc.name, (int)err);
            return false;
        }
        if (!set_cs_level(true)) {
            spi_device_release_bus(handle);
            return false;
        }
        forced_cs_handle = handle;
        cs_forced = true;
        return true;
    }

    if (!cs_forced) {
        return false;
    }
    const bool result = set_cs_level(false);
    spi_device_release_bus(forced_cs_handle);
    forced_cs_handle = nullptr;
    cs_forced = false;
    return result;
}

AP_HAL::Semaphore *SPIDevice::get_semaphore()
{
#ifdef SPIDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    return &bus.semaphore;
}

AP_HAL::Device::PeriodicHandle SPIDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
#ifdef SPIDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    return bus.register_periodic_callback(period_usec, cb, this);
}

bool SPIDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    return bus.adjust_timer(h, period_usec);
}

#ifdef HAL_ESP32_SPI_DEVICES
AP_HAL::SPIDevice *
SPIDeviceManager::get_device_ptr(const char *name)
{
#ifdef SPIDEBUG
    printf("%s:%d %s\n", __PRETTY_FUNCTION__, __LINE__, name);
#endif
    uint8_t i;
    for (i = 0; i<ARRAY_SIZE(device_desc); i++) {
        if (strcmp(device_desc[i].name, name) == 0) {
            break;
        }
    }
    if (i == ARRAY_SIZE(device_desc)) {
        esp32_spi_diag("ESP32 SPI: open failed name=%s not found", name);
        return nullptr;
    }
    SPIDeviceDesc &desc = device_desc[i];

#ifdef SPIDEBUG
    printf("%s:%d 222\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    // find the bus
    SPIBus *busp;
    for (busp = buses; busp; busp = (SPIBus *)busp->next) {
        if (busp->bus == desc.bus) {
            break;
        }
    }

#ifdef SPIDEBUG
    printf("%s:%d 333\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (busp == nullptr) {
        // create a new one
        busp = NEW_NOTHROW SPIBus(desc.bus);
        if (busp == nullptr) {
            esp32_spi_diag("ESP32 SPI: open failed name=%s bus alloc", name);
            return nullptr;
        }
        busp->next = buses;
        busp->bus = desc.bus;
        // deassert all CSes on the bus
        for (i = 0; i<ARRAY_SIZE(device_desc); i++) {
            SPIDeviceDesc &curr_desc = device_desc[i];
            if (desc.bus == curr_desc.bus) {
                esp_rom_gpio_pad_select_gpio(curr_desc.cs);
                // Set the output latch before enabling the driver to avoid a
                // low pulse on an active-low chip select during startup.
                const esp_err_t level_err = gpio_set_level(curr_desc.cs, 1);
                const gpio_mode_t mode = GPIO_MODE_OUTPUT;
                const esp_err_t direction_err = gpio_set_direction(curr_desc.cs, mode);
                if (direction_err != ESP_OK || level_err != ESP_OK) {
                    esp32_spi_diag("ESP32 SPI: CS init failed name=%s gpio=%u direction_err=%d level_err=%d",
                                   curr_desc.name,
                                   (unsigned)curr_desc.cs,
                                   (int)direction_err,
                                   (int)level_err);
                }
            }
        }
        buses = busp;
    }

#ifdef SPIDEBUG
    printf("%s:%d 444\n", __PRETTY_FUNCTION__, __LINE__);
#endif

    AP_HAL::SPIDevice *dev = NEW_NOTHROW SPIDevice(*busp, desc);
    if (dev == nullptr) {
        esp32_spi_diag("ESP32 SPI: open failed name=%s device alloc", name);
    }
    return dev;
}
#endif  // HAL_ESP32_SPI_DEVICES

#endif  // AP_HAL_SPI_ENABLED
