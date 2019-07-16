#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_P.h>                   // P library
#include <AC_PID/AC_PID.h>                 // PID library
#include <AC_PID/AC_PI_2D.h>               // PI library (2-axis)
#include <AC_PID/AC_PID_2D.h>              // PID library (2-axis)
#include <AP_InertialNav/AP_InertialNav.h> // Inertial Navigation library
#include "AC_AttitudeControl.h"            // Attitude control library
#include <AP_Motors/AP_Motors.h>           // motors library
#include <AP_Vehicle/AP_Vehicle.h>         // common vehicle parameters
#include <AP_Trajectory/AP_Trajectory.h>

// position controller default definitions
#define TRAJTRACK_ACCELERATION_MIN 50.0f // minimum horizontal acceleration in cm/s/s - used for sanity checking acceleration in leash length calculation
#define TRAJTRACK_ACCEL_XY 100.0f        // default horizontal acceleration in cm/s/s.  This is overwritten by waypoint and loiter controllers
// #define TRAJTRACK_ACCEL_XY_MAX 980.0f           // max horizontal acceleration in cm/s/s that the position velocity controller will ask from the lower accel controller
#define TRAJTRACK_STOPPING_DIST_UP_MAX 300.0f   // max stopping distance (in cm) vertically while climbing
#define TRAJTRACK_STOPPING_DIST_DOWN_MAX 200.0f // max stopping distance (in cm) vertically while descending

#define TRAJTRACK_SPEED 500.0f       // default horizontal speed in cm/s
#define TRAJTRACK_SPEED_DOWN -150.0f // default descent rate in cm/s
#define TRAJTRACK_SPEED_UP 250.0f    // default climb rate in cm/s

#define TRAJTRACK_ACCEL_Z 250.0f // default vertical acceleration in cm/s/s.

#define TRAJTRACK_LEASH_LENGTH_MIN 100.0f // minimum leash lengths in cm

#define TRAJTRACK_DT_50HZ 0.02f    // time difference in seconds for 50hz update rate
#define TRAJTRACK_DT_400HZ 0.0025f // time difference in seconds for 400hz update rate

#define TRAJTRACK_ACTIVE_TIMEOUT_US 200000 // position controller is considered active if it has been called within the past 0.2 seconds

#define TRAJTRACK_VEL_ERROR_CUTOFF_FREQ 4.0f // low-pass filter on velocity error (unit: hz)
#define TRAJTRACK_THROTTLE_CUTOFF_FREQ 2.0f  // low-pass filter on accel error (unit: hz)
#define TRAJTRACK_ACCEL_FILTER_HZ 2.0f       // low-pass filter on acceleration (unit: hz)
#define TRAJTRACK_JERK_RATIO 1.0f            // Defines the time it takes to reach the requested acceleration

#define TRAJTRACK_OVERSPEED_GAIN_Z 2.0f // gain controlling rate at which z-axis speed is brought back within SPEED_UP and SPEED_DOWN range
#define TAKEOFF_LOITER_SECOND 10.0
#define LANDING_LOITER_SECOND 10.0

class AC_TrajTrack
{
public:
    /// Constructor
    AC_TrajTrack(const AP_AHRS_View &ahrs, const AP_InertialNav &inav,
                 const AP_Motors &motors, AC_AttitudeControl &attitude_control);

    ///
    /// initialisation functions
    ///
    void init();

    void update(Trajectory::reference reference);

    void attcontroller(Quaternion &ref_att, Vector3f &ref_acc, Quaternion &curr_att);

    Quaternion quatMultiplication(Quaternion &q, Quaternion &p);

    Quaternion acc2quaternion(Vector3f vector_acc, double yaw);
    /// set_dt - sets time delta in seconds for all controllers (i.e. 100hz = 0.01, 400hz = 0.0025)
    ///     updates z axis accel controller's D term filter
    Quaternion rot2Quaternion(Matrix3f R);
    void get_rate_throttle(Vector3f &rate_cmd_, double &throttle_des_);

    ///
    /// z position controller
    ///
    Matrix3f quat2RotMatrix(Quaternion q);

    /// set_max_speed_z - sets maximum climb and descent rates
    ///     speed_down can be positive or negative but will always be interpreted as a descent speed
    ///     leash length will be recalculat

    const Vector3f &get_desired_velocity() { return _vel_desired; }

    /// get_max_speed_up - accessor for current maximum up speed in cm/s
    float get_max_speed_up() const { return _speed_up_cms; }

    /// get_max_speed_down - accessors for current maximum down speed in cm/s.  Will be a negative number
    float get_max_speed_down() const { return _speed_down_cms; }

    /// get_vel_target_z - returns current vertical speed in cm/s
    float get_vel_target_z() const { return _vel_target.z; }

    /// get_max_accel_z - returns current maximum vertical acceleration in cm/s/s
    float get_max_accel_z() const { return _accel_z_cms; }

    /// calc_leash_length - calculates the vertical leash lengths from maximum speed, acceleration
    ///     called by update_z_controller if z-axis speed or accelerations are changed

    ///
    /// xy position controller
    ///

    /// get_lean_angle_max_cd - returns the maximum lean angle the autopilot may request
    float get_lean_angle_max_cd() const { return 0; }
    float get_max_accel_xy() const { return _accel_cms; }
    float get_max_speed_xy() const { return _speed_cms; }

    /// set_limit_accel_xy - mark that accel has been limited
    ///     this prevents integrator buildup
    // void set_limit_accel_xy(void) { _limit.accel_xy = true; }

    /// calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration
    ///     should be called whenever the speed, acceleration or position kP is modified

    const Vector3f &get_pos_target() const { return _pos_target; }

    /// set_pos_target in cm from home
    void set_pos_target(const Vector3f &position) {}

    /// set_xy_target in cm from home
    void set_xy_target(float x, float y) {}

    /// shift position target target in x, y axis

    bool is_active_xy() const { return true; }

    /// get desired roll, pitch which should be fed into stabilize controllers
    float get_roll() const { return _roll_target; }
    float get_pitch() const { return _pitch_target; }
    float get_yaw() const {return _yaw_target;}

    // get_leash_xy - returns horizontal leash length in cm
    float get_leash_xy() const { return _leash; }

    /// get pid controller

    /// accessors for reporting
    const Vector3f &get_vel_target() const { return _vel_target; }
    const Vector3f &get_accel_target() const { return _accel_target; }

    // lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s

    void write_log();

    // provide feedback on whether arming would be a good idea right now:
    bool pre_arm_checks(const char *param_prefix,
                        char *failure_msg,
                        const uint8_t failure_msg_len)
    {
        return true;
    }

    void set_traj_start_time(int takeoff_time)
    {
        traj_start_epoch_time = takeoff_time + traj_start_time.get();
    }

    uint64_t get_traj_start_time()
    {
        return traj_start_epoch_time * 1e6;
    }

    static const struct AP_Param::GroupInfo var_info[];

protected:
    ///
    /// z controller private methods
    ///

    // run position control for Z axis
    // target altitude should be set with one of these functions
    //          set_alt_target
    //          set_target_to_stopping_point_z
    //          init_takeoff

    ///
    /// xy controller private methods
    ///

    /// move velocity target using desired acceleration

    /// run horizontal position controller correcting position and velocity
    ///     converts position (_pos_target) to target velocity (_vel_target)
    ///     desired velocity (_vel_desired) is combined into final target velocity
    ///     converts desired velocities in lat/lon directions to accelerations in lat/lon frame
    ///     converts desired accelerations provided in lat/lon frame to roll/pitch angles

    /// initialise and check for ekf position reset

    // references to inertial nav and ahrs libraries
    const AP_AHRS_View &_ahrs;
    const AP_InertialNav &_inav;
    const AP_Motors &_motors;
    AC_AttitudeControl &_attitude_control;

    // parameters
    AP_Float kpxy; // XY acceleration filter cutoff frequency
    AP_Float kvxy; // Maximum autopilot commanded angle (in degrees). Set to zero for Angle Max
    AP_Float kpz;
    AP_Float kvz;
    AP_Float krp;
    AP_Float kyaw;
    AP_Float pxy_error_max;
    AP_Float vxy_error_max;
    AP_Float pz_error_max;
    AP_Float vz_error_max;
    AP_Float yaw_error_max;
    AP_Float k_drag_x;
    AP_Float k_drag_y;
    AP_Float k_drag_z;
    AP_Float k_thrust_horz;
    AP_Int32 traj_start_time;
    AP_Int8 velocity_yaw;
    AP_Float vxy_max;
    AP_Float vz_max;
    AP_Float axy_max;
    AP_Float az_max;
    AP_Float attctrl_tau;
    AP_Float norm_thrust_const;

    double throttle_des;
    Vector3f rate_cmd;
    uint64_t traj_start_epoch_time;

    // internal variables
    float _dt;                // time difference (in seconds) between calls from the main program
    uint64_t _last_update_us; // system time (in microseconds) since last update_xy_controller call
    float _speed_down_cms;    // max descent rate in cm/s
    float _speed_up_cms;      // max climb rate in cm/s
    float _speed_cms;         // max horizontal speed in cm/s
    float _accel_z_cms;       // max vertical acceleration in cm/s/s
    float _accel_last_z_cms;  // max vertical acceleration in cm/s/s
    float _accel_cms;         // max horizontal acceleration in cm/s/s
    float _leash;             // horizontal leash length in cm.  target will never be further than this distance from the vehicle
    float _leash_down_z;      // vertical leash down in cm.  target will never be further than this distance below the vehicle
    float _leash_up_z;        // vertical leash up in cm.  target will never be further than this distance above the vehicle

    // output from controller
    float _roll_target;  // desired roll angle in centi-degrees calculated by position controller
    float _pitch_target; // desired roll pitch in centi-degrees calculated by position controller
    float _yaw_target;

    // position controller internal variables
    Vector3f _pos_target;    // target location in cm from home
    Vector3f _pos_error;     // error between desired and actual position in cm
    Vector3f _vel_desired;   // desired velocity in cm/s
    Vector3f _vel_target;    // velocity target in cm/s calculated by pos_to_rate step
    Vector3f _vel_error;     // error between desired and actual acceleration in cm/s
    Vector3f _vel_last;      // previous iterations velocity in cm/s
    Vector3f _accel_desired; // desired acceleration in cm/s/s (feed forward)
    Vector3f _accel_target;  // acceleration target in cm/s/s
    Vector3f _accel_error;   // acceleration error in cm/s/s
    Vector3f _gravity;
    Vector2f _vehicle_horiz_vel; // velocity to use if _flags.vehicle_horiz_vel_override is set
    double _yaw_target;
    Matrix3f D_;
    Matrix3f Rot_ned2enu;
    LowPassFilterFloat _vel_error_filter; // low-pass-filter on z-axis velocity error

    LowPassFilterVector2f _accel_target_filter; // acceleration target filter

    // ekf reset handling
    uint32_t _ekf_xy_reset_ms; // system time of last recorded ekf xy position reset
    uint32_t _ekf_z_reset_ms;  // system time of last recorded ekf altitude reset
};
