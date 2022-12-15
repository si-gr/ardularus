/* Variometer class by Samuel Tabor

Manages the estimation of aircraft total energy, drag and vertical air velocity.
*/
#include "LarusVario.h"

#include <AP_Logger/AP_Logger.h>

LarusVario::LarusVario(const AP_FixedWing &parms) :
//LarusVario::LarusVario(const AP_FixedWing &parms, const PolarParams &polarParams) :
    _aparm(parms)
{
    AP::ahrs().set_wind_estimation_enabled(true);
}

void LarusVario::get_height_baro(void){
    if (AP::ahrs().get_location(_loc)){
        if(_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, _alt)){
            _height_baro = (double)_alt / 10.0f;
        }
    }
}

float LarusVario::get_energy_height(void){
    _eheight = (powf(_aspd_filt, 2) / GRAVITY_MSS) / 2;
    return _eheight;
}

float LarusVario::get_te_altitude(void){
    return _eheight + _height_baro;
}

float LarusVario::get_wind_compensation(Vector3f velned, Vector3f wind){
    return powf(velned.x - wind.x, 2) + powf(velned.y - wind.y, 2);
}

void LarusVario::update()
{
    const AP_AHRS &_ahrs = AP::ahrs();
    get_height_baro();
    float aspd = 0;
    if (!_ahrs.airspeed_estimate(aspd)) {
            aspd = _aparm.airspeed_cruise_cm * 0.01f;
    }

    _aspd_filt = _sp_filter.apply(aspd);

    get_energy_height();

    float dt = (float)(AP_HAL::micros64() - _prev_update_time)/1e6;

    // Logic borrowed from AP_TECS.cpp
    // Update and average speed rate of change
    // Get DCM
    const Matrix3f &rotMat = _ahrs.get_rotation_body_to_ned();
    // Calculate speed rate of change
    float temp = rotMat.c.x * GRAVITY_MSS + AP::ins().get_accel().x;
    // take 5 point moving average
    float dsp = _vdot_filter.apply(temp);

    // Now we need to high-pass this signal to remove bias.
    _vdotbias_filter.set_cutoff_frequency(30.0f);
    float dsp_bias = _vdotbias_filter.apply(temp, dt);
    
    float dsp_cor = dsp - dsp_bias;


    Vector3f velned;
    Vector3f wind;

    if(_prev_simple_tot_e < 5.0f){
        _prev_simple_tot_e = _alt;
    }
    if(_prev_raw_total_energy < 5.0f){
        _prev_raw_total_energy = _alt;
    }


    float raw_climb_rate = 0.0f;
    if (_ahrs.get_velocity_NED(velned)) {

        wind = _ahrs.wind_estimate();
        float current_raw_tot_e = get_wind_compensation(velned, wind) + velned.z + _alt;
        _raw_climb_rate = (current_raw_tot_e - _prev_raw_total_energy) / dt;
        _prev_raw_total_energy = current_raw_tot_e;
        float current_simple_tot_e = (velned - wind) * (velned - wind) + _alt;      // v^2 + h  simplified from 1 / 2 m v^2 + mgh
        _simple_climb_rate = (current_simple_tot_e - _prev_simple_tot_e) / dt;
        _prev_simple_tot_e = current_simple_tot_e;
    }
    
    _climb_filter.set_cutoff_frequency(5.0f);
    float smoothed_climb_rate = _climb_filter.apply(raw_climb_rate, dt);

    // Compute still-air sinkrate -- unused for now, only netto vario
    float roll = _ahrs.roll;
    //float sinkrate = calculate_aircraft_sinkrate(roll);

    reading = raw_climb_rate + dsp_cor*_aspd_filt/GRAVITY_MSS;
    
    // Update filters.

    _prev_update_time = AP_HAL::micros64();
    
        // Log at 1/10Hz
    if((float)(AP_HAL::micros64() - _prev_log_time)/1e6 > 10){
        _prev_log_time = AP_HAL::micros64();
    

// @LoggerMessage: VAR
// @Vehicles: Plane
// @Description: Variometer data
// @Field: TimeUS: Time since system startup
// @Field: aspd_raw
// @Field: aspd_filt: filtered
// @Field: roll: AHRS roll
// @Field: raw: estimated air vertical speed
// @Field: cl: raw climb rate
// @Field: fc: filtered climb rate
// @Field: dsp: average acceleration along X axis
// @Field: dspb: detected bias in average acceleration along X axis
// @Field: windX: wind along X axis
// @Field: windY: wind along Y axis
// @Field: windZ: wind along Z axis
// @Field: height_baro: height
    AP::logger().WriteStreaming("VAR", "TUS,aspr,aspf,rl,rw,cl,fc,dsp,dspb,windx,wy,wz,alt", "Qffffffffffff",
                       AP_HAL::micros64(),
                       (double)aspd,
                       (double)_aspd_filt,
                       (double)roll,
                       (double)reading,
                       (double)_raw_climb_rate,
                       (double)smoothed_climb_rate,
                       (double)dsp,
                       (double)dsp_bias,
                       (double)wind.x,
                       (double)wind.y,
                       (double)wind.z,
                       (double)_height_baro);
    printf("aspd: %f, aspd_filt: %f, roll: %f, reading: %f, raw_climb_rate: %f, smoothed_climb_rate: %f, dsp: %f, dsp_bias: %f, wind.x: %f, wind.y: %f, wind.z: %f, height_baro: %f, simple: %f\r\n",
                       (double)aspd,
                       (double)_aspd_filt,
                       (double)roll,
                       (double)reading,
                       (double)_raw_climb_rate,
                       (double)smoothed_climb_rate,
                       (double)dsp,
                       (double)dsp_bias,
                       (double)wind.x,
                       (double)wind.y,
                       (double)wind.z,
                       (double)_height_baro,
                       (double)_simple_climb_rate);
    }
}


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
}