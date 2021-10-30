#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_SMBus_Custom.h"
#include <utility>

#define BATTMONITOR_SMBUS_CUSTOM_CELL_VOLTAGE         0x28    // Cell voltage register
#define BATTMONITOR_SMBUS_CUSTOM_CURRENT              0x2a    // Current register
#define BATTMONITOR_SMBUS_CUSTOM_MAX_CELLS            4      // Custom max cell count

extern const AP_HAL::HAL& hal;

/*
 * Other potentially useful registers, listed here for future use
 * #define BATTMONITOR_SMBUS_CUSTOM_VOLTAGE         0x09    // Total Voltage register
 * #define BATTMONITOR_SMBUS_CUSTOM_MIN_VOLT_W      0x26    // Minimum cell voltage write register
 * #define BATTMONITOR_SMBUS_CUSTOM_MIN_VOLT_R      0x27    // Minimum cell voltage read register
 */

// Constructor
AP_BattMonitor_SMBus_Custom::AP_BattMonitor_SMBus_Custom(AP_BattMonitor &mon,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_BattMonitor_Params &params,
                                                   AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                                   uint8_t _cell_count)
    : AP_BattMonitor_SMBus(mon, mon_state, params, std::move(dev)), cell_count(_cell_count)
{
    _pec_supported = false;
}

void AP_BattMonitor_SMBus_Custom::timer()
{
    uint8_t buff[BATTMONITOR_SMBUS_CUSTOM_MAX_CELLS << 1];
    uint32_t tnow = AP_HAL::micros();

    // read cell voltages
    if (read_block(BATTMONITOR_SMBUS_CUSTOM_CELL_VOLTAGE, buff, (MIN(BATTMONITOR_SMBUS_CUSTOM_MAX_CELLS, cell_count) << 1)) == (cell_count << 1)) {
        float pack_voltage_mv = 0.0f;
        for (uint8_t i = 0; i < MIN(BATTMONITOR_SMBUS_CUSTOM_MAX_CELLS, cell_count); i++) {
            uint16_t cell = buff[(i * 2) + 1] << 8 | buff[i * 2];
            _state.cell_voltages.cells[i] = cell;
            pack_voltage_mv += (float)cell;
        }
        _has_cell_voltages = true;

        // accumulate the pack voltage out of the total of the cells
        // because the Solo's I2C bus is so noisy, it's worth not spending the
        // time and bus bandwidth to request the pack voltage as a seperate
        // transaction
        _state.voltage = pack_voltage_mv * 1e-3f;
        _state.last_time_micros = tnow;
        _state.healthy = true;
    }

    // timeout after 5 seconds
    if ((tnow - _state.last_time_micros) > AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS) {
        _state.healthy = false;
        // do not attempt to ready any more data from battery
        return;
    }

    // read current
    if (read_block(BATTMONITOR_SMBUS_CUSTOM_CURRENT, buff, 4) == 4) {
        _state.current_amps = -(float)((int32_t)((uint32_t)buff[3]<<24 | (uint32_t)buff[2]<<16 | (uint32_t)buff[1]<<8 | (uint32_t)buff[0])) / 1000.0f;
        _state.last_time_micros = tnow;
    }
}

// read_block - returns number of characters read if successful, zero if unsuccessful
uint8_t AP_BattMonitor_SMBus_Custom::read_block(uint8_t reg, uint8_t* data, uint8_t max_len) const
{
    uint8_t buff[max_len]; // Buffer to hold received data

    // read bytes
    if (!_dev->read_registers(reg, buff, max_len)) {
        return 0;
    }

    // Copy data
    memcpy(data, buff, max_len);

    // Return success
    return max_len;
}
