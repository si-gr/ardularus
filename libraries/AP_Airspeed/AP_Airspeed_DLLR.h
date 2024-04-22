/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

// backend driver for AllSensors DLLR differential airspeed sensor
// currently assumes a 5" of water, noise reduced, sensor

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_AIRSPEED_DLLR_ENABLED
#define AP_AIRSPEED_DLLR_ENABLED AP_AIRSPEED_BACKEND_DEFAULT_ENABLED
#endif

#if AP_AIRSPEED_DLLR_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

#include "AP_Airspeed_Backend.h"

class AP_Airspeed_DLLR : public AP_Airspeed_Backend
{
public:

    AP_Airspeed_DLLR(AP_Airspeed &frontend, uint8_t _instance, const float _range_inH2O);
    static AP_Airspeed_Backend *probe(AP_Airspeed &frontend, uint8_t _instance, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev, const float _range_inH2O);

    ~AP_Airspeed_DLLR(void) {}
    
    // probe and initialise the sensor
    bool init() override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // return the current temperature in degrees C, if available
    bool get_temperature(float &temperature) override;

private:
    void timer();
    void _measure();

    // read calibration coefficients from EEPROM
    void _read_eeprom();
    const uint8_t eeprom_addr_coeff[10] = {47, 48, 49, 50, 51, 52, 53, 54, 55, 56}; // addresses of calibration coefficients in EEPROM

    // calibration coefficients
    float DLLR_ABCD[4] = {0.0, 0.0, 0.0, 0.0};
    int32_t i32_ABCD[4] = {0, 0, 0, 0};
    float DLLR_A = 0.0;
    float DLLR_B = 0.0;
    float DLLR_C = 0.0;
    float DLLR_D = 0.0;
    float DLLR_E = 0.0;
    float TC50H = 0.0;
    float TC50L = 0.0;

    // calibration coefficients parts
    int32_t i32A = 0;
    int32_t i32B = 0;
    int32_t i32C = 0;
    int32_t i32D = 0;
    int8_t i8E = 0;
    int8_t i8TC50H = 0;
    int8_t i8TC50L = 0;

    uint8_t eeprom_finished = 0;
    uint8_t calibration_vals = 0;
    float zero_pressure = 0;
    bool eeprom_requested = false;

    const uint32_t Tref_Counts = 8724019;
    const float TCKScale = 100.0 * 100.0 * 134218.0;
    int32_t pcomp = 0;

    const float f2p24 = 16777216.0;

    AP_HAL::UARTDriver *uart;
    uint32_t _measurement_started_ms;
    float pressure;
    float pressure_sum;
    uint8_t press_count = 0;
    float temperature;
    float temperature_sum;
    uint8_t temp_count = 0;
    
    uint32_t last_sample_time_ms;

    // initialise the sensor
    void setup();
    
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;
};

#endif  // AP_AIRSPEED_DLLR_ENABLED
