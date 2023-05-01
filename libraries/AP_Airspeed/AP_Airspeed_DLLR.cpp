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
/*
  driver for DLLR differential airspeed sensor
  https://www.allsensors.com/products/DLLR-L01D
 */

#include "AP_Airspeed_DLLR.h"

#if AP_AIRSPEED_DLLR_ENABLED

#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;
//#define DLLR_DEBUG_UART

#define DLLR_I2C_ADDR 0x29
# define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)


AP_Airspeed_DLLR::AP_Airspeed_DLLR(AP_Airspeed &_frontend, uint8_t _instance, const float _range_inH2O) :
    AP_Airspeed_Backend(_frontend, _instance)
{}

/*
  probe for a sensor on a given i2c address
 */
AP_Airspeed_Backend *AP_Airspeed_DLLR::probe(AP_Airspeed &_frontend,
                                             uint8_t _instance,
                                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev,
                                             const float _range_inH2O)
{
    if (!_dev) {
        return nullptr;
    }
    AP_Airspeed_DLLR *sensor = new AP_Airspeed_DLLR(_frontend, _instance, _range_inH2O);
    if (!sensor) {
        return nullptr;
    }
    sensor->dev = std::move(_dev);
    sensor->setup();
    return sensor;
}


void AP_Airspeed_DLLR::_read_eeprom(){
    if(!eeprom_requested){
        // request new eeprom
        dev->transfer(&eeprom_addr_coeff[eeprom_finished], 1, nullptr, 0);
        eeprom_requested = true;
    } else {
        // eeprom was requested
        bool success = false;

        // read eeprom
        uint8_t raw_bytes[3];
        success = dev->read((uint8_t *)&raw_bytes, sizeof(raw_bytes));

        if (success){
            if(eeprom_finished < 8) {
                i32_ABCD[eeprom_finished / 2] = raw_bytes[1] << (((eeprom_finished + 1) % 2) * 16 + 8) | raw_bytes[2] << (((eeprom_finished + 1) % 2) * 16);
            } else if (eeprom_finished < 9) {
                TC50H = (float)raw_bytes[1] / (float)(0x7F);
                TC50L = (float)raw_bytes[2] / (float)(0x7F);
            } else if (eeprom_finished < 10) {
                DLLR_E = (float)(raw_bytes[2]) / (float)(0x7F);
            }
            
            eeprom_finished++;
        }
    
        // request new eeprom
        dev->transfer(&eeprom_addr_coeff[eeprom_finished], 1, nullptr, 0);
        eeprom_requested = true;
    }
}

// initialise the sensor
void AP_Airspeed_DLLR::setup()
{
    WITH_SEMAPHORE(dev->get_semaphore());
    dev->set_speed(AP_HAL::Device::SPEED_LOW);
    dev->set_retries(2);
    dev->set_device_type(uint8_t(DevType::DLLR));
    set_bus_id(dev->get_bus_id());
    
    dev->register_periodic_callback(1000000UL/30U,
                                    FUNCTOR_BIND_MEMBER(&AP_Airspeed_DLLR::timer, void));


}

// probe and initialise the sensor
bool AP_Airspeed_DLLR::init()
{   
    #ifdef DLLR_DEBUG_UART
    uart = hal.serial(5);
    uart->printf("airspeed init\r\n");
    #endif
    dev = hal.i2c_mgr->get_device(get_bus(), DLLR_I2C_ADDR);
    if (!dev) {
    #ifdef DLLR_DEBUG_UART
        uart->printf("init fail\r\n");
        uart->printf("get bus %d\r\n", (int)get_bus());
        uart->printf("get bus %d\r\n", (int)DLLR_I2C_ADDR);
    #endif
        return false;
    }
    setup();
    
    return true;
}

// start a measurement
void AP_Airspeed_DLLR::_measure()
{
    #ifdef DLLR_DEBUG_UART
    uart->printf("start measure\r\n");
    #endif
    uint8_t cmd = 0xAD; // get average4 values
    if (dev->transfer(&cmd, 1, nullptr, 0)) {
        _measurement_started_ms = AP_HAL::millis();
    }
}


#define STATUS_SHIFT 30
#define TEMPERATURE_SHIFT 5
#define TEMPERATURE_MASK ((1 << 11) - 1)
#define PRESSURE_SHIFT 16
#define PRESSURE_MASK ((1 << 14) - 1)

#define DLLR_OFFSET 8192.0f
#define DLLR_SCALE 16384.0f
#define MAX_24B_VAL 0x7FFFFF
#define MAX_32B_VAL 0x7FFFFFFF


// 30Hz timer
void AP_Airspeed_DLLR::timer()
{

    if(eeprom_finished < 10){
        _read_eeprom();
        return;
    }
    if(eeprom_finished == 10){
        for(uint8_t i = 0; i < 4; i++){
            #ifdef DLLR_DEBUG_UART
            uart->printf("converting %d - %d\r\n", i, (int)i32_ABCD[i]);
            #endif
            DLLR_ABCD[i] = ((float)(i32_ABCD[i])) / ((float)(0x7FFFFFFF));
        }
        eeprom_finished++;
    }

    uint8_t raw_bytes[7];
    if (!dev->read((uint8_t *)&raw_bytes, sizeof(raw_bytes))) {
        #ifdef DLLR_DEBUG_UART
        uart->printf("read fail\r\n");
        #endif
        if(AP_HAL::millis() - _measurement_started_ms > 30){
            _measure();
        }
        return;
    }

    uint32_t pressure_data = (raw_bytes[1] << 16) |
                             (raw_bytes[2] << 8) |
                             raw_bytes[3];
    //pressure_data -= 0x800000;
    float pnorm = (float)pressure_data - (float)0x800000;
    pnorm /= (float)MAX_24B_VAL;    // Normalize to +/- 1.0


    int32_t temperature_data = (raw_bytes[4] << 16) |
                             (raw_bytes[5] << 8) |
                             raw_bytes[6];

    float pcorr = pnorm + (DLLR_ABCD[0] * powf(pnorm, 3.0f) + DLLR_ABCD[1] * powf(pnorm, 2.0f) + DLLR_ABCD[2] * pnorm + DLLR_ABCD[3]);
    //int32_t iPcorr = (int32_t)(pcorr * (float)MAX_24B_VAL) + 0x800000;  // Convert +/- 1.0f to 24-bit signed integer -0.7990 * (2^23 - 1) + 8 388 608 = 15 091 104.993

    // Compute difference from reference temperature, in sensor counts:
    temperature_data = temperature_data - Tref_Counts;
    float temp_correction = 0.0f;
    if (temperature_data > 0) {
        temp_correction = TC50H;
    } else {
        temp_correction = TC50L;
    }
    pcorr = (pcorr + 1.0f) / 2.0f;  // value between 0 and 1
    if(pcorr > 0.5f) {
        pcorr = pcorr - 0.5f;
    } else {
        pcorr = 0.5f - pcorr;
    }
    float tcorr = (1.0f - (DLLR_E * 2.5f * pcorr)) * temperature_data * temp_correction / TCKScale; // value between 0 and 1
    pcorr = pcorr - tcorr;
    pcomp = abs((int32_t) ((pcorr - 1.0f) * 2 * (float)MAX_24B_VAL) + 0x800000);   // 0.3995 * (2^24 - 1) = 6 702 497.3925
    pressure =                  1.25f * (((float)(pcomp) - 0.1f * f2p24)/ f2p24) * 2488.4f;
    //float pressure_no_comp =    1.25f * (((float)(iPcorr) - 0.1f * f2p24)/ f2p24) * 2488.4f;
    temperature = ((float)(temperature_data + Tref_Counts) * 125.f)/f2p24 - 40.f;
    #ifdef DLLR_DEBUG_UART
    uart->printf("t %.2f p %.4f\r\n", temperature, pressure);
    uart->printf("a %.4f b %.4f c %.4f d %.4f tcor %.4f\r\n", DLLR_ABCD[0], DLLR_ABCD[1], DLLR_ABCD[2], DLLR_ABCD[3], tcorr);
    uart->printf("pcorr %.8f \r\n", pcorr);
    #endif
    

    WITH_SEMAPHORE(sem);

    last_sample_time_ms = AP_HAL::millis();
    _measure();
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_DLLR::get_differential_pressure(float &_pressure)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - last_sample_time_ms) > 100) {
        return false;
    }

    _pressure = pressure;
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_DLLR::get_temperature(float &_temperature)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - last_sample_time_ms) > 100) {
        return false;
    }

    _temperature = temperature;
    return true;
}

#endif
