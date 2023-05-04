
/* Variometer class by Samuel Tabor

Manages the estimation of aircraft total energy, drag and vertical air velocity.
*/
#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <Filter/AverageFilter.h>
#include <AP_Vehicle/AP_FixedWing.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BattMonitor/AP_BattMonitor.h>

#include <AP_GPS/AP_GPS.h>
#include <stdio.h>


class LarusVario {


    const AP_HAL::HAL& hal = AP_HAL::get_HAL();
    const AP_FixedWing &_aparm;
    const uint8_t _ble_msg_length = 18;
    AP_Airspeed *aspeed = AP::airspeed();
    const AP_BattMonitor &battery = AP::battery();
    uint8_t _ble_msg_count = 0;
    const uint8_t _num_messages = sizeof(larus_variables) / _ble_msg_length;

    // store time of last update
    uint64_t _prev_update_time;

    // store time of last log
    uint64_t _prev_log_time;

    // uart for the device
    AP_HAL::UARTDriver *uart;
    
    //uint8_t *_uart_buffer;

    struct PACKED larus_variables {
        float airspeed;
        float airspeed_vector_x;
        float airspeed_vector_y;
        float airspeed_vector_z;
        int16_t roll;
        
        float wind_vector_x;
        float wind_vector_y;
        float wind_vector_z;
        int32_t height_gps;
        int16_t pitch;

        float ground_course;
        int32_t latitude;
        int32_t longitude;
        float ground_speed;
        int16_t yaw;

        float turn_radius;
        int16_t ekf_ground_speed_x;
        int16_t ekf_ground_speed_y;
        float raw_climb_rate;
        float simple_climb_rate;
        int16_t reading;

        int16_t gps_velocity_x;
        int16_t gps_velocity_y;
        int16_t gps_velocity_z;
        int16_t velned_velocity_x;
        int16_t velned_velocity_y;
        int16_t velned_velocity_z;
        int16_t smoothed_climb_rate;
        float height_baro;

        int16_t acc_x;
        int16_t acc_y;
        int16_t acc_z;
        int16_t battery_voltage;
        uint32_t gps_time;
        float pres_temp;
        uint16_t gps_status;
        
        float dsp_bias;
    } _larus_variables;

    bool _uart_started = false;

    float _raw_climb_rate;
    
    float _simple_climb_rate;

    float _aspd_filt;

    float _expected_thermalling_sink;

    float _height_baro;

    float _prev_simple_tot_e;   // AHRS total energy with z axis projection

    float _prev_raw_total_energy;   // Wind compensated total energy with z axis projection

    float _c_l;

    float _alpha;

    const uint8_t _alpha_0_min_aspd = 16; // minimum airspeed for alpha 0 array
    float _alpha_0 [16];    // array to store alpha 0 values for different airspeeds: 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46 m/s

    int32_t _alt;

    Location _loc;
    
    Vector3f _prev_velned;

    Vector3f _prev_wind;


    Vector3f _aspd_vec;

    // declares a 5point average filter using floats
    //AverageFilterFloat_Size5 _vdot_filter;

    /*
     low pass filters for various purposes.
     */
    // Climb rate filter for monitoring progress in thermal.
    LowPassFilter<float> _climb_filter{1/60.0};

    // Longitudinal acceleration bias filter.
    LowPassFilter<float> _vdotbias_filter{1/60.0};

public:
    struct PolarParams {
        float K;
        float CD0;
        float B;
    };

    PolarParams _polarParams = {25.6, 0.027, 0.031};

    LarusVario(const AP_FixedWing &parms);
    //LarusVario(const AP_FixedWing &parms, const PolarParams &polarParams);
    
    
    float alt;
    float reading;

    void update();
    float calculate_aircraft_sinkrate(float phi) const;

    void start_uart(void);

    void send_uart(uint8_t *buffer, uint8_t buffer_len);

    float get_smoothed_climb(void);

    float get_aircraft_sink(void);

    // energy_height = Square(true_airspeed) * INVERSE_2G. Returns energy height in m
    float get_energy_height(float airspeed);

    // Returns altitude in m
    float get_altitude(void);

    // TE_altitude = nav_altitude + energy_height. Returns altitude in m
    float get_te_altitude(float airspeed);

    // Larus Part: Speed change - wind change
    float get_wind_compensation(Vector3f velned, Vector3f wind);

};

