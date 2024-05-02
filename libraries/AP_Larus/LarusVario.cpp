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

void LarusVario::update(float thermalability, float varioReading, float thermallingRadius, float e0, float e1, float e2, float e3)
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
    float tas;
    _ahrs.airspeed_estimate(tas);
    tas = tas * _ahrs.get_EAS2TAS();

    // Save alpha 0 if we are in a level flight, not accelerating
    /*if (aspd > _alpha_0_min_aspd){
        if (abs(_ahrs.get_accel().z - 1.0) < 0.1 ) {
            _alpha_0[MIN(((uint8_t)aspd) - _alpha_0_min_aspd, 0)] = _alpha_0[MIN(((uint8_t)aspd) - _alpha_0_min_aspd, 0)] * 0.9 + _ahrs.get_pitch() * 0.1;
        }
    }

    _c_l = _ahrs.get_accel().z / powf(aspd, 2);    
    _alpha = _c_l + _alpha_0[MIN(((uint8_t)_aspd_filt) - _alpha_0_min_aspd, 0)];
*/
    //uint64_t now = AP_HAL::micros64();
    //float dt = (now - _prev_update_time) * 1.0e-6f;

    //_prev_update_time = now;
    // Logic borrowed from AP_TECS.cpp
    // Update and average speed rate of change
    // Get DCM
    //const Matrix3f &rotMat = _ahrs.get_rotation_body_to_ned();
    // Calculate speed rate of change
    //float temp = rotMat.c.x * GRAVITY_MSS + AP::ins().get_accel().x;

    Vector3f velned;
    Vector3f wind;
    //Vector2f groundspeed_vector = _ahrs.groundspeed_vector();

    if (_ahrs.get_velocity_NED(velned)) {

        wind = _ahrs.wind_estimate();
        //_ahrs.airspeed_vector_true(_aspd_vec);
        //float current_raw_tot_e = get_wind_compensation(_aspd_vec, wind) + velned.z + (double)_alt;
        //_raw_climb_rate = (current_raw_tot_e - _prev_raw_total_energy) / dt;
        //_prev_raw_total_energy = current_raw_tot_e;
        
    }
  /*
    float windCorrection = _ahrs.earth_to_body(_wind_filter.get() - wind).x * dt;
    _wind_filter.apply(wind);
    //float te_altitude = get_te_altitude(aspd);
    //_simple_climb_rate = (te_altitude - _prev_simple_tot_e) / dt;
    //_prev_simple_tot_e = te_altitude;

    
    //ap gps get singleton
    if(_ahrs.get_location(_loc) && AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
            _loc = AP::gps().location();
    }*/  
    /*
    //uart->printf("test print larus %.4f %.4f %.4f\r\n", (double)roll, (double)_height_baro, (double)aspd);
    _larus_variables.airspeed = aspd;   // 4B
    //_larus_variables.airspeed_filtered = _aspd_filt;
    _larus_variables.e0 = e0;   // 4B
    _larus_variables.e1 = e1;   // 4B
    _larus_variables.e2 = e2;   // 4B    
    _larus_variables.roll = (int16_t)((_ahrs.roll / M_PI) * (float)(0x8000)); // 2B
    
    _larus_variables.wind_vector_x = wind.x;   // 4B
    _larus_variables.wind_vector_y = wind.y;   // 4B
    _larus_variables.wind_vector_z = wind.z;   // 4B
    _larus_variables.height_gps = _loc.alt;   // 4B
    _larus_variables.pitch = (int16_t)((_ahrs.pitch / M_PI) * (float)(0x8000)); // 2B

    _larus_variables.latitude = _loc.lat;   // 4B
    _larus_variables.longitude = _loc.lng;   // 4B
    _larus_variables.e3 = e3;   // 4B
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
    _larus_variables.tasstate = (int16_t)(tas * 100.0);   // 2B
    _larus_variables.height_baro = get_altitude();

    _larus_variables.acc_x = (int16_t)(_ahrs.get_accel().x * 50);   // 2B
    _larus_variables.acc_y = (int16_t)(_ahrs.get_accel().y * 50);   // 2B
    _larus_variables.acc_z = (int16_t)(_ahrs.get_accel().z * 50);   // 2B
    _larus_variables.battery_voltage = roundf(AP::battery().voltage() * 100.0f);   // 2B
    _larus_variables.windCorrection = windCorrection;   // 4B
    _larus_variables.spedot = (int16_t)(_tecs->getSPEdot() * 50);
    _larus_variables.skedot = (int16_t)(_tecs->getSKEdot() * 50);
    _larus_variables.gps_status = AP::gps().status();

    // 128 / 5 = 25.6
    _larus_variables.newvario0 = (int8_t)(25 * MAX(MIN(velned.z + ((_tecs->getTASState() - windCorrection) * _tecs->getVelDot() / 9.81), 5.0), -5.0));
    _larus_variables.newvario1 = _larus_variables.newvario0;
    _larus_variables.newvario2 = _larus_variables.newvario0;
    _larus_variables.newvario3 = _larus_variables.newvario0;
    _larus_variables.newvario4 = _larus_variables.newvario0;
    _larus_variables.newvario5 = _larus_variables.newvario0;
    */

    /*struct PACKED fast_vario_larus_variables {
        int16_t wind_vector_x;
        int16_t wind_vector_y;
        int16_t windCorrection;
        int16_t airspeed;
        int16_t spedot;
        int16_t skedot;
        int16_t roll;
        int16_t pitch;
    } _fast_larus_variables;*/

    _fast_larus_variables.wind_vector_x = (int16_t)(wind.x * 500.0);
    _fast_larus_variables.wind_vector_y = (int16_t)(wind.y * 500.0);
    _fast_larus_variables.airspeed = (int16_t)(aspd * 500.0);
    _fast_larus_variables.spedot = (int16_t)(_tecs->getSPEdot() * 50);
    _fast_larus_variables.skedot = (int16_t)(_tecs->getSKEdot() * 50);
    _fast_larus_variables.roll = (int16_t)((_ahrs.roll / M_PI) * (float)(0x8000));
    _fast_larus_variables.pitch = (int16_t)((_ahrs.pitch / M_PI) * (float)(0x8000));
    _fast_larus_variables.yaw = (int16_t)((_ahrs.yaw / M_PI) * (float)(0x8000));
    _fast_larus_variables.vvv = 10;

    /*struct PACKED fast_aux_larus_variables {
        int16_t velned_velocity_x;//500
        int16_t velned_velocity_y;//500
        int16_t velned_velocity_z;//500
        int16_t acc_x;//50
        int16_t acc_y;//50
        int16_t acc_z;//50
        int16_t tas;//500
        int16_t varioreading;//500
        int8_t vvv;
    } _fast_aux_variables;
    */

    _fast_aux_variables.velned_velocity_x = (int16_t)(velned.x * 500);
    _fast_aux_variables.velned_velocity_y = (int16_t)(velned.y * 500);
    _fast_aux_variables.velned_velocity_z = (int16_t)(velned.z * 500);
    _fast_aux_variables.acc_x = (int16_t)(_ahrs.get_accel().x * 50);
    _fast_aux_variables.acc_y = (int16_t)(_ahrs.get_accel().y * 50);
    _fast_aux_variables.acc_z = (int16_t)(_ahrs.get_accel().z * 50);
    _fast_aux_variables.tas = (int16_t)(tas * 500);
    _fast_aux_variables.varioreading = (int16_t)(varioReading * 500);
    _fast_aux_variables.vvv = 11;


    /*struct PACKED slow_vario_larus_variables {
        int16_t battery_voltage;
        int32_t height_gps;
        float turn_radius;
        int8_t vvv; // indicator for app
    } _slow_larus_variables;
    */
    _slow_larus_variables.battery_voltage = roundf(AP::battery().voltage() * 100.0f);   // 2B
    _slow_larus_variables.height_gps = _loc.alt;   // 4B
    _slow_larus_variables.turn_radius = thermallingRadius;
    _slow_larus_variables.vvv = 20;

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
   
        AP::logger().WriteStreaming("LARU", "TimeUS,aspd,spedot,skedot", "Qfff",
                       AP_HAL::micros64(),
                       (double)_fast_larus_variables.airspeed,
                       (double)_fast_larus_variables.spedot,
                       (double)_fast_larus_variables.skedot);
    if (false) {
        uart->write((uint8_t*)&_larus_variables + (_ble_msg_count * _ble_msg_length), _ble_msg_length);
        uart->write((uint8_t)_ble_msg_count);
        
    } else {
        // send slow data every 100th packet (about 5 sec)
        if (fast_var_packet_counter < 100){
            fast_var_packet_counter++;
            if (fast_var_packet_counter % 2 == 0){
                uart->write((uint8_t*)&_fast_larus_variables, sizeof(_fast_larus_variables));
            } else {
                uart->write((uint8_t*)&_fast_aux_variables, sizeof(_fast_aux_variables));
            }
        } else {
            fast_var_packet_counter = 0;
            uart->write((uint8_t*)&_slow_larus_variables, sizeof(_slow_larus_variables));
        }
    }
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