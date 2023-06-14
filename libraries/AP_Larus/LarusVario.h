
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
#include <AP_TECS/AP_TECS.h>

#include <AP_GPS/AP_GPS.h>
#include <stdio.h>


class LarusVario {


    const AP_HAL::HAL& hal = AP_HAL::get_HAL();
    const uint8_t _ble_msg_length = 19;
    AP_Airspeed *aspeed = AP::airspeed();
    const AP_BattMonitor &battery = AP::battery();
    uint8_t _ble_msg_count = 0;
    const uint8_t _num_messages = sizeof(larus_variables) / _ble_msg_length;

    // store time of last update
    uint64_t _prev_update_time;

    uint16_t fast_var_packet_counter = 0;

    // store time of last log
    uint64_t _prev_log_time;

    // uart for the device
    AP_HAL::UARTDriver *uart;
    
    //uint8_t *_uart_buffer;

    struct PACKED larus_variables {
        float airspeed;
        float e0;
        float e1;
        float e2;
        int16_t roll;
        int8_t newvario0;
        
        float wind_vector_x;
        float wind_vector_y;
        float wind_vector_z;
        int32_t height_gps;
        int16_t pitch;
        int8_t newvario1;

        float ground_course;
        int32_t latitude;
        int32_t longitude;
        float e3;
        int16_t yaw;
        int8_t newvario2;

        float turn_radius;
        int16_t ekf_ground_speed_x;
        int16_t ekf_ground_speed_y;
        float raw_climb_rate;
        int16_t reading;
        float thermability;
        int8_t newvario3;

        int16_t gps_velocity_x;
        int16_t gps_velocity_y;
        int16_t gps_velocity_z;
        int16_t velned_velocity_x;
        int16_t velned_velocity_y;
        int16_t velned_velocity_z;
        int16_t tasstate;
        float height_baro;
        int8_t newvario4;

        int16_t acc_x;
        int16_t acc_y;
        int16_t acc_z;
        int16_t battery_voltage;
        float windCorrection;
        int16_t spedot;
        int16_t skedot;
        uint16_t gps_status;
        int8_t newvario5;
        
    } _larus_variables;

    struct PACKED fast_vario_larus_variables {
        int16_t wind_vector_x;
        int16_t wind_vector_y;
        int16_t windCorrection;
        int16_t airspeed;
        int16_t spedot;
        int16_t skedot;
        int16_t roll;
        int16_t pitch;
        int8_t vvv; // indicator for app
    } _fast_larus_variables;

    struct PACKED slow_vario_larus_variables {
        int16_t battery_voltage;
        int32_t height_gps;
        float turn_radius;
        int8_t vvv; // indicator for app
    } _slow_larus_variables;

    bool _uart_started = false;

    float _raw_climb_rate;
    
    //float _simple_climb_rate;

    //float _aspd_filt;

    //float _expected_thermalling_sink;

    float _height_baro;

    float _prev_simple_tot_e;   // AHRS total energy with z axis projection

    float _prev_raw_total_energy;   // Wind compensated total energy with z axis projection

    //float _c_l;

    //float _alpha;

    //const uint8_t _alpha_0_min_aspd = 16; // minimum airspeed for alpha 0 array
    //float _alpha_0 [16];    // array to store alpha 0 values for different airspeeds: 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46 m/s

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
    LowPassFilter<Vector3f> _wind_filter{1/2.0};

public:
    /*struct PolarParams {
        float K;
        float CD0;
        float B;
    };*/

    //PolarParams _polarParams = {25.6, 0.027, 0.031};

    //LarusVario(const AP_FixedWing &parms, const PolarParams &polarParams);
    //LarusVario(AP_AHRS &ahrs, const AP_TECS *tecs);
    LarusVario(AP_AHRS &ahrs, AP_TECS *tecs)
        : _ahrs(ahrs)
        , _tecs(tecs)
    {

    _ahrs.set_wind_estimation_enabled(true);
    uart = hal.serial(5);
    //_uart_buffer = new uint8_t[20];
    }

    // reference to the AHRS object
    AP_AHRS &_ahrs;

    // pointer to the SpdHgtControl object
    AP_TECS *_tecs;
    float alt;
    float reading;

    void update(float thermalability, float varioReading, float thermallingRadius, float e0, float e1, float e2, float e3);
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

