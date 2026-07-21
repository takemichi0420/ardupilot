/*
 * ArduPilot AP_HAL platform adapter for the ST VL53LX bare driver.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

#include <cstring>

extern "C" {
#include "vl53lx_platform.h"
}

extern const AP_HAL::HAL &hal;

static AP_HAL::I2CDevice *get_i2c_device(VL53LX_DEV dev)
{
    if (dev == nullptr || dev->I2cHandle == nullptr) {
        return nullptr;
    }
    return reinterpret_cast<AP_HAL::I2CDevice *>(dev->I2cHandle);
}

extern "C" VL53LX_Error VL53LX_CommsInitialise(VL53LX_Dev_t *, uint8_t, uint16_t)
{
    return VL53LX_ERROR_NONE;
}

extern "C" VL53LX_Error VL53LX_CommsClose(VL53LX_Dev_t *)
{
    return VL53LX_ERROR_NONE;
}

extern "C" VL53LX_Error VL53LX_WriteMulti(VL53LX_DEV dev,
                                            uint16_t index,
                                            uint8_t *data,
                                            uint32_t count)
{
    AP_HAL::I2CDevice *i2c = get_i2c_device(dev);
    if (i2c == nullptr || data == nullptr || count > 256) {
        return VL53LX_ERROR_INVALID_PARAMS;
    }

    uint8_t buffer[258];
    buffer[0] = uint8_t(index >> 8);
    buffer[1] = uint8_t(index);
    if (count > 0) {
        memcpy(&buffer[2], data, count);
    }

    return i2c->transfer(buffer, count + 2, nullptr, 0) ?
        VL53LX_ERROR_NONE : VL53LX_ERROR_CONTROL_INTERFACE;
}

extern "C" VL53LX_Error VL53LX_ReadMulti(VL53LX_DEV dev,
                                           uint16_t index,
                                           uint8_t *data,
                                           uint32_t count)
{
    AP_HAL::I2CDevice *i2c = get_i2c_device(dev);
    if (i2c == nullptr || data == nullptr || count == 0 || count > 256) {
        return VL53LX_ERROR_INVALID_PARAMS;
    }

    const uint8_t address[] {uint8_t(index >> 8), uint8_t(index)};
    return i2c->transfer(address, sizeof(address), data, count) ?
        VL53LX_ERROR_NONE : VL53LX_ERROR_CONTROL_INTERFACE;
}

extern "C" VL53LX_Error VL53LX_WrByte(VL53LX_DEV dev, uint16_t index, uint8_t data)
{
    return VL53LX_WriteMulti(dev, index, &data, sizeof(data));
}

extern "C" VL53LX_Error VL53LX_WrWord(VL53LX_DEV dev, uint16_t index, uint16_t data)
{
    uint8_t buffer[] {uint8_t(data >> 8), uint8_t(data)};
    return VL53LX_WriteMulti(dev, index, buffer, sizeof(buffer));
}

extern "C" VL53LX_Error VL53LX_WrDWord(VL53LX_DEV dev, uint16_t index, uint32_t data)
{
    uint8_t buffer[] {
        uint8_t(data >> 24),
        uint8_t(data >> 16),
        uint8_t(data >> 8),
        uint8_t(data),
    };
    return VL53LX_WriteMulti(dev, index, buffer, sizeof(buffer));
}

extern "C" VL53LX_Error VL53LX_UpdateByte(VL53LX_DEV dev,
                                            uint16_t index,
                                            uint8_t and_data,
                                            uint8_t or_data)
{
    uint8_t value = 0;
    VL53LX_Error status = VL53LX_RdByte(dev, index, &value);
    if (status != VL53LX_ERROR_NONE) {
        return status;
    }
    value = (value & and_data) | or_data;
    return VL53LX_WrByte(dev, index, value);
}

extern "C" VL53LX_Error VL53LX_RdByte(VL53LX_DEV dev, uint16_t index, uint8_t *data)
{
    return VL53LX_ReadMulti(dev, index, data, sizeof(*data));
}

extern "C" VL53LX_Error VL53LX_RdWord(VL53LX_DEV dev, uint16_t index, uint16_t *data)
{
    if (data == nullptr) {
        return VL53LX_ERROR_INVALID_PARAMS;
    }

    uint8_t buffer[2];
    const VL53LX_Error status = VL53LX_ReadMulti(dev, index, buffer, sizeof(buffer));
    if (status == VL53LX_ERROR_NONE) {
        *data = (uint16_t(buffer[0]) << 8) | buffer[1];
    }
    return status;
}

extern "C" VL53LX_Error VL53LX_RdDWord(VL53LX_DEV dev, uint16_t index, uint32_t *data)
{
    if (data == nullptr) {
        return VL53LX_ERROR_INVALID_PARAMS;
    }

    uint8_t buffer[4];
    const VL53LX_Error status = VL53LX_ReadMulti(dev, index, buffer, sizeof(buffer));
    if (status == VL53LX_ERROR_NONE) {
        *data = (uint32_t(buffer[0]) << 24) |
                (uint32_t(buffer[1]) << 16) |
                (uint32_t(buffer[2]) << 8) |
                buffer[3];
    }
    return status;
}

extern "C" VL53LX_Error VL53LX_GetTickCount(VL53LX_DEV, uint32_t *time_ms)
{
    if (time_ms == nullptr) {
        return VL53LX_ERROR_INVALID_PARAMS;
    }
    *time_ms = AP_HAL::millis();
    return VL53LX_ERROR_NONE;
}

extern "C" VL53LX_Error VL53LX_GetTimerFrequency(int32_t *timer_freq_hz)
{
    if (timer_freq_hz == nullptr) {
        return VL53LX_ERROR_INVALID_PARAMS;
    }
    *timer_freq_hz = 1000;
    return VL53LX_ERROR_NONE;
}

extern "C" VL53LX_Error VL53LX_GetTimerValue(int32_t *timer_count)
{
    if (timer_count == nullptr) {
        return VL53LX_ERROR_INVALID_PARAMS;
    }
    *timer_count = int32_t(AP_HAL::millis());
    return VL53LX_ERROR_NONE;
}

extern "C" VL53LX_Error VL53LX_WaitMs(VL53LX_Dev_t *, int32_t wait_ms)
{
    if (wait_ms < 0) {
        return VL53LX_ERROR_INVALID_PARAMS;
    }
    hal.scheduler->delay(wait_ms);
    return VL53LX_ERROR_NONE;
}

extern "C" VL53LX_Error VL53LX_WaitUs(VL53LX_Dev_t *, int32_t wait_us)
{
    if (wait_us < 0) {
        return VL53LX_ERROR_INVALID_PARAMS;
    }
    hal.scheduler->delay_microseconds(wait_us);
    return VL53LX_ERROR_NONE;
}

extern "C" VL53LX_Error VL53LX_WaitValueMaskEx(VL53LX_Dev_t *dev,
                                                 uint32_t timeout_ms,
                                                 uint16_t index,
                                                 uint8_t value,
                                                 uint8_t mask,
                                                 uint32_t poll_delay_ms)
{
    const uint32_t start_ms = AP_HAL::millis();
    do {
        uint8_t current = 0;
        const VL53LX_Error status = VL53LX_RdByte(dev, index, &current);
        if (status != VL53LX_ERROR_NONE) {
            return status;
        }
        if ((current & mask) == value) {
            return VL53LX_ERROR_NONE;
        }
        if (poll_delay_ms > 0) {
            hal.scheduler->delay(poll_delay_ms);
        }
    } while (AP_HAL::millis() - start_ms < timeout_ms);

    return VL53LX_ERROR_TIME_OUT;
}

extern "C" VL53LX_Error VL53LX_GpioSetMode(uint8_t, uint8_t)
{
    return VL53LX_ERROR_NOT_IMPLEMENTED;
}

extern "C" VL53LX_Error VL53LX_GpioSetValue(uint8_t, uint8_t)
{
    return VL53LX_ERROR_NOT_IMPLEMENTED;
}

extern "C" VL53LX_Error VL53LX_GpioGetValue(uint8_t, uint8_t *)
{
    return VL53LX_ERROR_NOT_IMPLEMENTED;
}

extern "C" VL53LX_Error VL53LX_GpioXshutdown(uint8_t)
{
    return VL53LX_ERROR_NOT_IMPLEMENTED;
}

extern "C" VL53LX_Error VL53LX_GpioCommsSelect(uint8_t)
{
    return VL53LX_ERROR_NOT_IMPLEMENTED;
}

extern "C" VL53LX_Error VL53LX_GpioPowerEnable(uint8_t)
{
    return VL53LX_ERROR_NOT_IMPLEMENTED;
}

extern "C" VL53LX_Error VL53LX_GpioInterruptEnable(void (*)(void), uint8_t)
{
    return VL53LX_ERROR_NOT_IMPLEMENTED;
}

extern "C" VL53LX_Error VL53LX_GpioInterruptDisable()
{
    return VL53LX_ERROR_NOT_IMPLEMENTED;
}
