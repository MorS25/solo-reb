/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand.h"
#include "AC_PrecLand_Backend.h"
#include "AC_PrecLand_Companion.h"
#include "AC_PrecLand_IRLock.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_PrecLand::var_info[] = {
    // @Param: ENABLED
    // @DisplayName: Precision Land enabled/disabled and behaviour
    // @Description: Precision Land enabled/disabled and behaviour
    // @Values: 0:Disabled, 1:Enabled Always Land, 2:Enabled Strict
    // @User: Advanced
    AP_GROUPINFO("ENABLED", 0, AC_PrecLand, _enabled, 0),

    // @Param: TYPE
    // @DisplayName: Precision Land Type
    // @Description: Precision Land Type
    // @Values: 0:None, 1:CompanionComputer, 2:IRLock
    // @User: Advanced
    AP_GROUPINFO("TYPE",    1, AC_PrecLand, _type, 0),

    // @Param: SPEED
    // @DisplayName: Precision Land horizontal speed maximum in cm/s
    // @Description: Precision Land horizontal speed maximum in cm/s
    // @Range: 0 500
    // @User: Advanced
    AP_GROUPINFO("SPEED",   2, AC_PrecLand, _speed_xy, AC_PRECLAND_SPEED_XY_DEFAULT),

     // @Param: VEL_P
     // @DisplayName: Precision landing velocity controller P gain
     // @Description: Precision landing velocity controller P gain
     // @Range: 0.100 5.000
     // @User: Advanced

     // @Param: VEL_I
     // @DisplayName: Precision landing velocity controller I gain
     // @Description: Precision landing velocity controller I gain
     // @Range: 0.100 5.000
     // @User: Advanced

     // @Param: VEL_IMAX
     // @DisplayName: Precision landing velocity controller I gain maximum
     // @Description: Precision landing velocity controller I gain maximum
     // @Range: 0 1000
     // @Units: cm/s
     // @User: Standard
     AP_SUBGROUPINFO(_pi_vel_xy, "VEL_", 3, AC_PrecLand, AC_PI_2D),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_PrecLand::AC_PrecLand(const AP_AHRS& ahrs, const AP_InertialNav& inav, float dt) :
    _ahrs(ahrs),
    _inav(inav),
    _pi_vel_xy(PRECLAND_P, PRECLAND_I, PRECLAND_IMAX, PRECLAND_FILT_HZ, dt),
    //_dt(dt),
    _size_rad(0.0f),
    _have_estimate(false),
    _backend(NULL)
{
    // set parameters to defaults
    AP_Param::setup_object_defaults(this, var_info);

    // other initialisation
    _backend_state.healthy = false;
}


// init - perform any required initialisation of backends
void AC_PrecLand::init()
{
    // exit immediately if init has already been run
    if (_backend != NULL) {
        return;
    }

    // default health to false
    _backend = NULL;
    _backend_state.healthy = false;

    // instantiate backend based on type parameter
    switch ((enum PrecLandType)(_type.get())) {
        // no type defined
        case PRECLAND_TYPE_NONE:
        default:
            return;
        // companion computer
        case PRECLAND_TYPE_COMPANION:
            _backend = new AC_PrecLand_Companion(*this, _backend_state);
            break;
        // IR Lock
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        case PRECLAND_TYPE_IRLOCK:
            _backend = new AC_PrecLand_IRLock(*this, _backend_state);
            break;
#endif
    }

    // init backend
    if (_backend != NULL) {
        _backend->init();
    }
}

// update - give chance to driver to get updates from sensor
void AC_PrecLand::update(float alt_above_terrain_cm)
{
    // run backend update
    if (_backend != NULL) {
        // read from sensor
        _backend->update();

        // calculate angles to target and position estimate
        calc_angles_and_pos(alt_above_terrain_cm);
    }
}

// INITIALISE
void AC_PrecLand::set_initial_vals()
{
	_prev_bf_roll_pos_offset = 0.0f;
	_prev_bf_pitch_pos_offset = 0.0f;
	_target_pos_offset.x = 0.0f;
	_target_pos_offset.y = 0.0f;
	_target_pos_offset.z = 0.0f;
	_missed_target_frames = 0;
	_integrator_roll_offset = 0.0f;
	_integrator_pitch_offset = 0.0f;
	_d_term_x = 0.0f;
	_d_term_y = 0.0f;
	_d_term_x_avg = 0.0f;
	_d_term_y_avg = 0.0f;
}

// get_target_shift - returns 3D vector of earth-frame position adjustments to target
Vector3f AC_PrecLand::get_target_shift(const Vector3f &orig_target)
{
    Vector3f shift; // default shift initialised to zero

    // do not shift target if not enabled or no position estimate
    if (_backend == NULL || !_have_estimate) {
        return shift;
    }

    // shift is target_offset - (original target - current position)
    Vector3f curr_offset_from_target = orig_target - _inav.get_position();
    shift = _target_pos_offset - curr_offset_from_target;
    shift.z = 0.0f;

    // record we have consumed this reading (perhaps there is a cleaner way to do this using timestamps)
    _have_estimate = false;

    // return adjusted target
    return shift;
}

// calc_angles_and_pos - converts sensor's body-frame angles to earth-frame angles and position estimate
//  raw sensor angles stored in _angle_to_target (might be in earth frame, or maybe body frame)
//  earth-frame angles stored in _ef_angle_to_target
//  position estimate is stored in _target_pos
void AC_PrecLand::calc_angles_and_pos(float alt_above_terrain_cm)
{
    // exit immediately if not enabled
    if (_backend == NULL) {
        _have_estimate = false;
        return;
    }

    // get angles to target from backend
    if (!_backend->get_angle_to_target(_angle_to_target.x, _angle_to_target.y, _size_rad)) {
        _have_estimate = false;
        return;
    }

    float x_rad;
    float y_rad;

    if(_backend->get_frame_of_reference() == MAV_FRAME_LOCAL_NED){
        //don't subtract vehicle lean angles
        x_rad = _angle_to_target.x;
        y_rad = -_angle_to_target.y;
    }else{ // assume MAV_FRAME_BODY_NED (i.e. a hard-mounted sensor)
        // subtract vehicle lean angles
        x_rad = _angle_to_target.x - _ahrs.roll;
        y_rad = -_angle_to_target.y + _ahrs.pitch;
    }

    // DON'T rotate to earth-frame angles
    //_ef_angle_to_target.x = y_rad*_ahrs.cos_yaw() - x_rad*_ahrs.sin_yaw();
    //_ef_angle_to_target.y = y_rad*_ahrs.sin_yaw() + x_rad*_ahrs.cos_yaw();

    // get current altitude (constrained to no lower than 1cm)
    float alt = MAX(alt_above_terrain_cm, 1.0f);

    // RE-PURPOSE this logged variable: Target Offset in roll/pitch directions
    //_ef_angle_to_target.x = alt*tanf(x_rad);
    //_ef_angle_to_target.y = alt*tanf(y_rad);

    /// DON'T update these here
    // convert earth-frame angles to earth-frame position offset
    ///_target_pos_offset.x = alt*tanf(_ef_angle_to_target.x);
    ///_target_pos_offset.y = alt*tanf(_ef_angle_to_target.y);
    ///_target_pos_offset.z = 0;  // not used

    _have_estimate = true;
}

// calc_angles_and_pos - reports back bf target pos
//  converts sensor's body-frame angles to earth-frame angles and position estimate
//  raw sensor angles stored in _angle_to_target (might be in earth frame, or maybe body frame)
//  earth-frame angles stored in _ef_angle_to_target
//  position estimate is stored in _target_pos
const Vector3f& AC_PrecLand::calc_angles_and_pos_out(float alt_above_terrain_cm, float p_gain, float d_gain_temp)
{
	float i_max = _pi_precland_xy.imax(); //
	float i_gain = _pi_precland_xy.kI(); // set to 0; default is 1
//	float d_gain = _pi_precland_xy.filt_hz(); // set to 100; previously 25
	float d_gain = 400.0f*d_gain_temp;
	float ctrl_max = _pi_precland_xy.imax(); // set to 2 (degrees)
//	float p_gain = _pi_precland_xy.kP();

    // exit immediately if not enabled
    if (_backend == NULL) {
        _have_estimate = false;
        _target_pos_offset.x = 0.0f;
        _target_pos_offset.y = 0.0f;
        _target_pos_offset.z = 0.0f;
        return _target_pos_offset;
    }

    if (!_backend->get_angle_to_target(_angle_to_target.x, _angle_to_target.y) && _missed_target_frames > 50) {
        _have_estimate = false;
        _missed_target_frames = 0;
        _target_pos_offset.x = 0.0f;
        _target_pos_offset.y = 0.0f;
        _target_pos_offset.z = 0.0f;
        _d_term_x_avg = 0.0f;
        _d_term_y_avg = 0.0f;
        _integrator_roll_offset = 0.0f;
        _integrator_pitch_offset = 0.0f;
        return _target_pos_offset;
    }

    // get angles to target from backend
    if (!_backend->get_angle_to_target(_angle_to_target.x, _angle_to_target.y)) {
        _have_estimate = false;
        _missed_target_frames++;
        //_target_pos_offset.x = 0.0f;
        //_target_pos_offset.y = 0.0f;
        //_target_pos_offset.z = 0.0f;
        return _target_pos_offset;
    }

    float x_rad;
    float y_rad;
    _missed_target_frames = 0;

    if(_backend->get_frame_of_reference() == MAV_FRAME_LOCAL_NED){
        //don't subtract vehicle lean angles
        x_rad = _angle_to_target.x;
        y_rad = -_angle_to_target.y;
    }else{ // assume MAV_FRAME_BODY_NED (i.e. a hard-mounted sensor)
        // subtract vehicle lean angles
        x_rad = _angle_to_target.x - _ahrs.roll;
        y_rad = -_angle_to_target.y + _ahrs.pitch;
    }

    // DON'T rotate to earth-frame angles
    //_ef_angle_to_target.x = y_rad*_ahrs.cos_yaw() - x_rad*_ahrs.sin_yaw();
    //_ef_angle_to_target.y = y_rad*_ahrs.sin_yaw() + x_rad*_ahrs.cos_yaw();

    // get current altitude (constrained to no lower than 20cm)
    float alt = MAX(alt_above_terrain_cm, 20.0f);

    // RE-PURPOSE this logged variable: Target Offset in roll/pitch directions
    //_ef_angle_to_target.x = alt*tanf(x_rad);
    //_ef_angle_to_target.y = alt*tanf(y_rad);

    /// RE-PURPOSE this logged variable: Control in roll/pitch directions (centi-degrees)
    // convert earth-frame angles to earth-frame position offset
    ///_target_pos_offset.x = alt*tanf(_ef_angle_to_target.x);
    ///_target_pos_offset.y = alt*tanf(_ef_angle_to_target.y);
    ///_target_pos_offset.z = 0;  // not used

    // OUTPUT
    float bf_roll_pos_offset = alt*tanf(x_rad);
    float bf_pitch_pos_offset = alt*tanf(y_rad);

    //_target_pos_offset.x = p_gain*bf_roll_pos_offset;
    //_target_pos_offset.y = -p_gain*bf_pitch_pos_offset;
    //_target_pos_offset.z = 0.0f;

    _d_term_x = d_gain*(bf_roll_pos_offset-_prev_bf_roll_pos_offset);
    _d_term_y = d_gain*(bf_pitch_pos_offset-_prev_bf_pitch_pos_offset);
    // PSEUDO-AVERAGING
    _d_term_x_avg = (0.005f*_d_term_x) + (1.0f - 0.005f)*_d_term_x_avg;
    _d_term_y_avg = (0.005f*_d_term_y) + (1.0f - 0.005f)*_d_term_y_avg;
    // LOG d_term
    _ef_angle_to_target.x = bf_roll_pos_offset;
    _ef_angle_to_target.y = bf_pitch_pos_offset;

    // ADD D-control
    _target_pos_offset.x = p_gain*bf_roll_pos_offset + _d_term_x_avg;
    _target_pos_offset.y = -p_gain*bf_pitch_pos_offset - _d_term_y_avg;
    _target_pos_offset.z = _d_term_y_avg;

    // PID, ADD I-control
    _integrator_roll_offset += bf_roll_pos_offset*i_gain;
    _integrator_pitch_offset += bf_pitch_pos_offset*i_gain;

    if (_integrator_roll_offset > i_max) {
    	_integrator_roll_offset = i_max;
    }
    if (_integrator_pitch_offset > i_max) {
    	_integrator_pitch_offset = i_max;
    }
    if (_integrator_roll_offset < -i_max) {
    	_integrator_roll_offset = -i_max;
    }
    if (_integrator_pitch_offset < -i_max) {
    	_integrator_pitch_offset = -i_max;
    }

    // PID, ADD I-control
//    _target_pos_offset.x = p_gain*bf_roll_pos_offset +
//    		_d_term_x + _integrator_roll_offset;
//    _target_pos_offset.y = -p_gain*bf_pitch_pos_offset -
//    		_d_term_y - _integrator_pitch_offset;
//    _target_pos_offset.z = 0.0f;

    // CTRL Max
    if (_target_pos_offset.x > ctrl_max) {
    	_target_pos_offset.x = ctrl_max;
        }
    if (_target_pos_offset.y > ctrl_max) {
    	_target_pos_offset.y = ctrl_max;
    }
    if (_target_pos_offset.y < -ctrl_max) {
    	_target_pos_offset.y = -ctrl_max;
        }
    if (_target_pos_offset.x < -ctrl_max) {
    	_target_pos_offset.x = -ctrl_max;
    }

    // STORE previous value for D-control application
    _prev_bf_roll_pos_offset = bf_roll_pos_offset;
    _prev_bf_pitch_pos_offset = bf_pitch_pos_offset;

    return _target_pos_offset;

    _have_estimate = true;
}

// handle_msg - Process a LANDING_TARGET mavlink message
void AC_PrecLand::handle_msg(mavlink_message_t* msg)
{
    // run backend update
    if (_backend != NULL) {
        _backend->handle_msg(msg);
    }
}
