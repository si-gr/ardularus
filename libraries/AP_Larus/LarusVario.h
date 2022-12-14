
/* Variometer class by Samuel Tabor

Manages the estimation of aircraft total energy, drag and vertical air velocity.
*/
#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <Filter/AverageFilter.h>
#include <AP_Vehicle/AP_FixedWing.h>

class LarusVario {


    const AP_FixedWing &_aparm;

    // store time of last update
    uint64_t _prev_update_time;

    float _raw_climb_rate;

    float _aspd_filt;

    float _expected_thermalling_sink;

    float _height_baro;

    int32_t _alt;

    float _eheight;

    Location _loc;

    
    Vector3f _prev_velned;

    Vector3f _prev_wind;

    // declares a 5point average filter using floats
    AverageFilterFloat_Size5 _vdot_filter;

    AverageFilterFloat_Size5 _sp_filter;

    /*
     low pass filters for various purposes.
     */
    // Climb rate filter for monitoring progress in thermal.
    LowPassFilter<float> _climb_filter{1/60.0};

    // Longitudinal acceleration bias filter.
    LowPassFilter<float> _vdotbias_filter{1/60.0};

    // Speed to fly vario filter.
    LowPassFilter<float> _stf_filter{1/20.0};

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
    float tau;

    void update();
    float calculate_aircraft_sinkrate(float phi) const;

    float get_smoothed_climb(void);

    float get_aircraft_sink(void);

    // energy_height = Square(true_airspeed) * INVERSE_2G
    float get_energy_height(void);

    // write baro height, true if success
    void get_height_baro(void);

    // TE_altitude = nav_altitude + energy_height;
    float get_te_altitude(void);

    // Larus Part: Speed change - wind change
    float get_wind_compensation(Vector3f velned, Vector3f wind);

};

