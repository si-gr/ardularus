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

bool LarusVario::get_height_agl(void){
    return AP::ahrs().get_hagl(_agl);
}

float LarusVario::get_energy_height(void){
    _eheight = (powf(_aspd_filt, 2) / GRAVITY_MSS) / 2;
    return _eheight;
}

float LarusVario::get_te_altitude(void){
    return _eheight + _agl;
}

float LarusVario::get_wind_compensation(Vector3f velned, Vector3f wind){
    return powf(velned.x - wind.x, 2) + powf(velned.y - wind.y, 2) + get_te_altitude();
}

void LarusVario::update()
{
    const AP_AHRS &_ahrs = AP::ahrs();
    bool hagl_succ = get_height_agl();
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
    _vdotbias_filter.set_cutoff_frequency(1/(20*tau));
    float dsp_bias = _vdotbias_filter.apply(temp, dt);
    
    float dsp_cor = dsp - dsp_bias;


    Vector3f velned;
    Vector3f wind;
    Vector3f acceleration;


    float raw_climb_rate = 0.0f;
    if (_ahrs.get_velocity_NED(velned)) {

        wind = _ahrs.wind_estimate();
        acceleration = _ahrs.get_accel() - _ahrs.get_accel_bias();
        raw_climb_rate = get_wind_compensation(velned, wind);
    }
    
    _climb_filter.set_cutoff_frequency(1/(3*tau));
    float smoothed_climb_rate = _climb_filter.apply(raw_climb_rate, dt);

    // Compute still-air sinkrate -- unused for now, only netto vario
    float roll = _ahrs.roll;
    //float sinkrate = calculate_aircraft_sinkrate(roll);

    reading = raw_climb_rate + dsp_cor*_aspd_filt/GRAVITY_MSS;
    
    // Update filters.

    _prev_update_time = AP_HAL::micros64();

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
// @Field: accx: acc along X axis
// @Field: accy: wind along Y axis
// @Field: accz: wind along Z axis
// @Field: agl: agl height
// @Field: agl_succ: successful agl height
    AP::logger().WriteStreaming("VAR", "TimeUS,aspd_raw,aspd_filt,roll,raw,cl,fc,dsp,dspb,windx,windy,windz,accx,accy,accz,agl,agl_succ", "Qfffffffffff",
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
                       (double)acceleration.x,
                       (double)acceleration.y,
                       (double)acceleration.z,
                       (double)_agl,
                       hagl_succ);
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