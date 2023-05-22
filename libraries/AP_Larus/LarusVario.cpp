/* Variometer class by Samuel Tabor

Manages the estimation of aircraft total energy, drag and vertical air velocity.
*/
#include "LarusVario.h"

#include <AP_Logger/AP_Logger.h>



void LarusVario::start_uart(void){
    //uart->configure_parity(0);
    //uart->set_stop_bits(1);
    //uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    //uart->set_blocking_writes(false);   // updates run in the main thread
    //uart->set_options(AP_HAL::UARTDriver::OPTION_NODMA_TX | AP_HAL::UARTDriver::OPTION_NODMA_RX);
        uart->begin(921600, 512, 512);
    //uart->discard_input();
}

void LarusVario::send_uart(uint8_t *buffer, uint8_t buffer_len){
    uart->write(buffer, buffer_len);
    uart->flush();
}

float LarusVario::get_altitude(void){
    if (AP::ahrs().get_location(_loc)){
        if(_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, _alt)){
            return (double)_alt / 100.0f;
        }
    }
    return 0;
}

float LarusVario::get_energy_height(float airspeed){
    return powf(airspeed, 2) / (2.f * GRAVITY_MSS);
}

float LarusVario::get_te_altitude(float airspeed){
    return get_energy_height(airspeed) + get_altitude();
}

float LarusVario::get_wind_compensation(Vector3f velned, Vector3f wind){
    return powf(velned.x - wind.x, 2) + powf(velned.y - wind.y, 2);
}

void LarusVario::update(float thermalability, float varioReading, float thermallingRadius)
{   
    if(!_uart_started){
        start_uart();
        _uart_started = true;
    }
    float aspd = 0;
    float pres_temp = 0;
    if (aspeed && aspeed->enabled()) {
        aspd = aspeed->get_airspeed();
        aspeed->get_temperature(pres_temp);
    }

    // Save alpha 0 if we are in a level flight, not accelerating
    /*if (aspd > _alpha_0_min_aspd){
        if (abs(_ahrs.get_accel().z - 1.0) < 0.1 ) {
            _alpha_0[MIN(((uint8_t)aspd) - _alpha_0_min_aspd, 0)] = _alpha_0[MIN(((uint8_t)aspd) - _alpha_0_min_aspd, 0)] * 0.9 + _ahrs.get_pitch() * 0.1;
        }
    }

    _c_l = _ahrs.get_accel().z / powf(aspd, 2);    
    _alpha = _c_l + _alpha_0[MIN(((uint8_t)_aspd_filt) - _alpha_0_min_aspd, 0)];
*/
    float dt = (float)(AP_HAL::micros64() - _prev_update_time)/1e6;


    // Logic borrowed from AP_TECS.cpp
    // Update and average speed rate of change
    // Get DCM
    //const Matrix3f &rotMat = _ahrs.get_rotation_body_to_ned();
    // Calculate speed rate of change
    //float temp = rotMat.c.x * GRAVITY_MSS + AP::ins().get_accel().x;

    Vector3f velned;
    Vector3f wind;
    Vector2f groundspeed_vector = _ahrs.groundspeed_vector();

    if (_ahrs.get_velocity_NED(velned)) {

        wind = _ahrs.wind_estimate();
        _ahrs.airspeed_vector_true(_aspd_vec);
        float current_raw_tot_e = get_wind_compensation(_aspd_vec, wind) + velned.z + (double)_alt;
        _raw_climb_rate = (current_raw_tot_e - _prev_raw_total_energy) / dt;
        _prev_raw_total_energy = current_raw_tot_e;
    }

    //float te_altitude = get_te_altitude(aspd);
    //_simple_climb_rate = (te_altitude - _prev_simple_tot_e) / dt;
    //_prev_simple_tot_e = te_altitude;
    
    
    _prev_update_time = AP_HAL::micros64();

    
    //ap gps get singleton
    if(_ahrs.get_location(_loc) && AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
            _loc = AP::gps().location();
    }
    //uart->printf("test print larus %.4f %.4f %.4f\r\n", (double)roll, (double)_height_baro, (double)aspd);
    _larus_variables.airspeed = aspd;   // 4B
    //_larus_variables.airspeed_filtered = _aspd_filt;
    _larus_variables.airspeed_vector_x = _aspd_vec.x;   // 4B
    _larus_variables.airspeed_vector_y = _aspd_vec.y;   // 4B
    _larus_variables.airspeed_vector_z = _aspd_vec.z;   // 4B    
    _larus_variables.roll = (int16_t)((_ahrs.roll / M_PI) * (float)(0x8000)); // 2B
    
    _larus_variables.wind_vector_x = wind.x;   // 4B
    _larus_variables.wind_vector_y = wind.y;   // 4B
    _larus_variables.wind_vector_z = wind.z;   // 4B
    _larus_variables.height_gps = _loc.alt;   // 4B
    _larus_variables.pitch = (int16_t)((_ahrs.pitch / M_PI) * (float)(0x8000)); // 2B

    _larus_variables.latitude = _loc.lat;   // 4B
    _larus_variables.longitude = _loc.lng;   // 4B
    _larus_variables.ground_speed = AP::gps().ground_speed();   // 4B
    _larus_variables.ground_course = AP::gps().ground_course();   // 4B
    _larus_variables.yaw = (int16_t)((_ahrs.yaw / M_PI) * (float)(0x8000));    // 2B 

    _larus_variables.turn_radius = thermallingRadius;//(aspd*aspd) / (GRAVITY_MSS * tanf(_ahrs.roll));   // 4B
    _larus_variables.ekf_ground_speed_x = (int16_t)(groundspeed_vector.x * 500);   // 2B
    _larus_variables.ekf_ground_speed_y = (int16_t)(groundspeed_vector.y * 500);   // 2B
    _larus_variables.raw_climb_rate = _raw_climb_rate;   // 4B
    _larus_variables.reading = (int16_t)(varioReading * 1000.0);   // 2B
    _larus_variables.thermability = thermalability;   // 4B

    _larus_variables.gps_velocity_x = (int16_t)(AP::gps().velocity().x * 500);   // 2B
    _larus_variables.gps_velocity_y = (int16_t)(AP::gps().velocity().y * 500);   // 2B
    _larus_variables.gps_velocity_z = (int16_t)(AP::gps().velocity().z * 500);   // 2B
    _larus_variables.velned_velocity_x = (int16_t)(velned.x * 500);   // 2B
    _larus_variables.velned_velocity_y = (int16_t)(velned.y * 500);   // 2B
    _larus_variables.velned_velocity_z = (int16_t)(velned.z * 500);   // 2B
    _larus_variables.tasstate = (int16_t)(_tecs->getTASState() * 100.0);   // 2B
    _larus_variables.height_baro = get_altitude();

    _larus_variables.acc_x = (int16_t)(_ahrs.get_accel().x * 50);   // 2B
    _larus_variables.acc_y = (int16_t)(_ahrs.get_accel().y * 50);   // 2B
    _larus_variables.acc_z = (int16_t)(_ahrs.get_accel().z * 50);   // 2B
    _larus_variables.battery_voltage = roundf(AP::battery().voltage() * 100.0f);   // 2B
    _larus_variables.gps_time = AP::gps().time_week_ms();
    _larus_variables.spedot = (int16_t)(_tecs->getSPEdot() * 50);
    _larus_variables.skedot = (int16_t)(_tecs->getSKEdot() * 50);
    _larus_variables.gps_status = AP::gps().status();
    
    //_larus_variables.smoothed_climb_rate = (int16_t)(smoothed_climb_rate * 100.0);   // 2B
    //_larus_variables.height_baro = _height_baro;   // 4B
    //_larus_variables.dsp = dsp;   // 4B
    /*
    if (_ble_msg_count == 0) {
        memcpy(_uart_buffer,        &_larus_variables.airspeed,         sizeof(_larus_variables.airspeed));
        memcpy(_uart_buffer + 4,    &_larus_variables.airspeed_vector_x, sizeof(_larus_variables.airspeed_vector_x));
        memcpy(_uart_buffer + 8,    &_larus_variables.airspeed_vector_y, sizeof(_larus_variables.airspeed_vector_y));
        memcpy(_uart_buffer + 12,   &_larus_variables.airspeed_vector_z, sizeof(_larus_variables.airspeed_vector_z));
        memcpy(_uart_buffer + 16,   &_larus_variables.roll,             sizeof(_larus_variables.roll));
    } else if (_ble_msg_count == 1)
    {
        memcpy(_uart_buffer,        &_larus_variables.wind_vector_x, sizeof(_larus_variables.wind_vector_x));
        memcpy(_uart_buffer + 4,    &_larus_variables.wind_vector_y, sizeof(_larus_variables.wind_vector_y));
        memcpy(_uart_buffer + 8,    &_larus_variables.wind_vector_z, sizeof(_larus_variables.wind_vector_z));
        memcpy(_uart_buffer + 12,   &_larus_variables.height_gps,   sizeof(_larus_variables.height_gps));
        memcpy(_uart_buffer + 16,   &_larus_variables.pitch,        sizeof(_larus_variables.pitch));
    } else if (_ble_msg_count == 2)
    {
        memcpy(_uart_buffer,        &_larus_variables.ground_course, sizeof(_larus_variables.ground_course));
        memcpy(_uart_buffer + 4,    &_larus_variables.latitude,     sizeof(_larus_variables.latitude));
        memcpy(_uart_buffer + 8,    &_larus_variables.longitude,    sizeof(_larus_variables.longitude));
        memcpy(_uart_buffer + 12,   &_larus_variables.ground_speed, sizeof(_larus_variables.ground_speed));
        memcpy(_uart_buffer + 16,   &_larus_variables.yaw,          sizeof(_larus_variables.yaw));
    } else if (_ble_msg_count == 3)
    {
        memcpy(_uart_buffer,        &_larus_variables.prev_raw_total_energy,    sizeof(_larus_variables.prev_raw_total_energy));
        memcpy(_uart_buffer + 4,    &_larus_variables.prev_simple_total_energy, sizeof(_larus_variables.prev_simple_total_energy));
        memcpy(_uart_buffer + 8,    &_larus_variables.raw_climb_rate,           sizeof(_larus_variables.raw_climb_rate));
        memcpy(_uart_buffer + 12,   &_larus_variables.simple_climb_rate,        sizeof(_larus_variables.simple_climb_rate));
        memcpy(_uart_buffer + 16,   &_larus_variables.reading,                  sizeof(_larus_variables.reading));
    } else if (_ble_msg_count == 4)
    {
        memcpy(_uart_buffer,        &_larus_variables.gps_velocity_x,   sizeof(_larus_variables.gps_velocity_x));
        memcpy(_uart_buffer + 2,    &_larus_variables.gps_velocity_y,   sizeof(_larus_variables.gps_velocity_y));
        memcpy(_uart_buffer + 4,    &_larus_variables.gps_velocity_z,   sizeof(_larus_variables.gps_velocity_z));
        memcpy(_uart_buffer + 6,    &_larus_variables.velned_velocity_x, sizeof(_larus_variables.velned_velocity_x));
        memcpy(_uart_buffer + 8,    &_larus_variables.velned_velocity_y, sizeof(_larus_variables.velned_velocity_y));
        memcpy(_uart_buffer + 10,   &_larus_variables.velned_velocity_z, sizeof(_larus_variables.velned_velocity_z));
        memcpy(_uart_buffer + 12,   &_larus_variables.smoothed_climb_rate, sizeof(_larus_variables.smoothed_climb_rate));
        memcpy(_uart_buffer + 14,   &_larus_variables.height_baro,      sizeof(_larus_variables.height_baro));
    }
    */
    
    uart->write((uint8_t*)&_larus_variables + (_ble_msg_count * _ble_msg_length), _ble_msg_length);//sizeof(_larus_variables));
    uart->write((uint8_t)_ble_msg_count);
    uart->flush();
    _ble_msg_count++;
    _ble_msg_count %= _num_messages;    // last message is special debug
    
    // Log at 1/10Hz
    //if((float)(AP_HAL::micros64() - _prev_log_time)/1e9 > 10 && _ble_msg_count == 1){
    //    _prev_log_time = AP_HAL::micros64();
    //    _ble_msg_count = _num_messages - 1; // send debug message next
    //}

    //uart8->printf("test print larus p8 %f\r\n", (double)roll);
}

/*
float LarusVario::calculate_aircraft_sinkrate(float phi) const
{
    // Remove aircraft sink rate
    float CL0;  // CL0 = 2*W/(rho*S*V^2)
    float C1;   // C1 = CD0/CL0
    float C2;   // C2 = CDi0/CL0 = B*CL0
    CL0 = _polarParams.K / (_aspd_filt * _aspd_filt);

    C1 = _polarParams.CD0 / CL0;  // constant describing expected angle to overcome zero-lift drag
    C2 = _polarParams.B * CL0;    // constant describing expected angle to overcome lift induced drag at zero bank

    float cosphi = (1 - phi * phi / 2); // first two terms of mclaurin series for cos(phi)
    
    return _aspd_filt * (C1 + C2 / (cosphi * cosphi));
}*/