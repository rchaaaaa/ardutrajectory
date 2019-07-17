#include <AP_HAL/AP_HAL.h>
#include "AC_TrajTrack.h"
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL &hal;

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
// default gains for Plane
#define TRAJTRACK_POS_Z_P 1.0f          // vertical position controller P gain default
#define TRAJTRACK_VEL_Z_P 5.0f          // vertical velocity controller P gain default
#define TRAJTRACK_ACC_Z_P 0.3f          // vertical acceleration controller P gain default
#define TRAJTRACK_ACC_Z_I 1.0f          // vertical acceleration controller I gain default
#define TRAJTRACK_ACC_Z_D 0.0f          // vertical acceleration controller D gain default
#define TRAJTRACK_ACC_Z_IMAX 800        // vertical acceleration controller IMAX gain default
#define TRAJTRACK_ACC_Z_FILT_HZ 10.0f   // vertical acceleration controller input filter default
#define TRAJTRACK_ACC_Z_DT 0.02f        // vertical acceleration controller dt default
#define TRAJTRACK_POS_XY_P 1.0f         // horizontal position controller P gain default
#define TRAJTRACK_VEL_XY_P 1.4f         // horizontal velocity controller P gain default
#define TRAJTRACK_VEL_XY_I 0.7f         // horizontal velocity controller I gain default
#define TRAJTRACK_VEL_XY_D 0.35f        // horizontal velocity controller D gain default
#define TRAJTRACK_VEL_XY_IMAX 1000.0f   // horizontal velocity controller IMAX gain default
#define TRAJTRACK_VEL_XY_FILT_HZ 5.0f   // horizontal velocity controller input filter
#define TRAJTRACK_VEL_XY_FILT_D_HZ 5.0f // horizontal velocity controller input filter for D
#elif APM_BUILD_TYPE(APM_BUILD_ArduSub)
// default gains for Sub
#define TRAJTRACK_POS_Z_P 3.0f          // vertical position controller P gain default
#define TRAJTRACK_VEL_Z_P 8.0f          // vertical velocity controller P gain default
#define TRAJTRACK_ACC_Z_P 0.5f          // vertical acceleration controller P gain default
#define TRAJTRACK_ACC_Z_I 0.1f          // vertical acceleration controller I gain default
#define TRAJTRACK_ACC_Z_D 0.0f          // vertical acceleration controller D gain default
#define TRAJTRACK_ACC_Z_IMAX 100        // vertical acceleration controller IMAX gain default
#define TRAJTRACK_ACC_Z_FILT_HZ 20.0f   // vertical acceleration controller input filter default
#define TRAJTRACK_ACC_Z_DT 0.0025f      // vertical acceleration controller dt default
#define TRAJTRACK_POS_XY_P 1.0f         // horizontal position controller P gain default
#define TRAJTRACK_VEL_XY_P 1.0f         // horizontal velocity controller P gain default
#define TRAJTRACK_VEL_XY_I 0.5f         // horizontal velocity controller I gain default
#define TRAJTRACK_VEL_XY_D 0.0f         // horizontal velocity controller D gain default
#define TRAJTRACK_VEL_XY_IMAX 1000.0f   // horizontal velocity controller IMAX gain default
#define TRAJTRACK_VEL_XY_FILT_HZ 5.0f   // horizontal velocity controller input filter
#define TRAJTRACK_VEL_XY_FILT_D_HZ 5.0f // horizontal velocity controller input filter for D
#else
// default gains for Copter / TradHeli
#define TRAJTRACK_KP_XY 10.0f        // vertical position controller P gain default
#define TRAJTRACK_KD_XY 4.0f         // vertical velocity controller P gain default
#define TRAJTRACK_KP_Z 15.0f         // vertical acceleration controller P gain default
#define TRAJTRACK_KD_Z 6.0f          // vertical acceleration controller I gain default
#define TRAJTRACK_K_RP 12.0f         // vertical acceleration controller D gain default
#define TRAJTRACK_K_YAW 5.0f         // vertical acceleration controller IMAX gain default
#define TRAJTRACK_PXY_ERROR_MAX 0.6f // vertical acceleration controller input filter default
#define TRAJTRACK_VXY_ERROR_MAX 1.0f // vertical acceleration controller dt default
#define TRAJTRACK_PZ_ERROR_MAX 0.3f  // horizontal position controller P gain default
#define TRAJTRACK_VZ_ERROR_MAX 0.75f // horizontal velocity controller P gain default
#define TRAJTRACK_YAW_ERROR_MAX 0.7f // horizontal velocity controller I gain default
#define TRAJTRACK_K_DRAG_X 0.0f      // horizontal velocity controller D gain default
#define TRAJTRACK_K_DRAG_Y 0.0f      // horizontal velocity controller IMAX gain default
#define TRAJTRACK_K_DRAG_Z 0.0f      // horizontal velocity controller input filter
#define TRAJTRACK_K_THRUST_HORZ 0.0f // horizontal velocity controller input filter for D
#define TRAJTRACK_VELOCITY_YAW 1
#define TRAJTRACK_VELOCITY_XY_MAX 5
#define TRAJTRACK_VELOCITY_Z_MAX 2
#define TRAJTRACK_ACCEL_XY_MAX 1
#define TRAJTRACK_ACCEL_Z_MAX 1
#define TRAJTRACK_ATT_TAO 0.1
#define TRAJTRACK_NORM_THR 0.05
#endif

const AP_Param::GroupInfo AC_TrajTrack::var_info[] = {
    // 0 was used for HOVER

    // @Param: _ACC_XY_FILT
    // @DisplayName: XY Acceleration filter cutoff frequency
    // @Description: Lower values will slow the response of the navigation controller and reduce twitchiness
    // @Units: Hz
    // @Range: 0.5 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_KP_XY", 0, AC_TrajTrack, kpxy, TRAJTRACK_KP_XY),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_KV_XY", 1, AC_TrajTrack, kvxy, TRAJTRACK_KD_XY),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_KP_Z", 2, AC_TrajTrack, kpz, TRAJTRACK_KP_Z),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_KV_Z", 3, AC_TrajTrack, kvz, TRAJTRACK_KD_Z),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_K_RP", 4, AC_TrajTrack, krp, TRAJTRACK_K_RP),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_K_YAW", 5, AC_TrajTrack, kyaw, TRAJTRACK_K_YAW),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_PXY_EMAX", 6, AC_TrajTrack, pxy_error_max, TRAJTRACK_PXY_ERROR_MAX),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_VXY_EMAX", 7, AC_TrajTrack, vxy_error_max, TRAJTRACK_VXY_ERROR_MAX),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_PZ_EMAX", 8, AC_TrajTrack, pz_error_max, TRAJTRACK_PZ_ERROR_MAX),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_VZ_EMAX", 9, AC_TrajTrack, vz_error_max, TRAJTRACK_VZ_ERROR_MAX),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_YAW_EMAX", 10, AC_TrajTrack, yaw_error_max, TRAJTRACK_YAW_ERROR_MAX),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_K_DRAG_X", 11, AC_TrajTrack, k_drag_x, TRAJTRACK_K_DRAG_X),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_K_DRAG_Y", 12, AC_TrajTrack, k_drag_y, TRAJTRACK_K_DRAG_Y),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_K_DRAG_Z", 13, AC_TrajTrack, k_drag_z, TRAJTRACK_K_DRAG_Z),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_K_THR", 14, AC_TrajTrack, k_thrust_horz, TRAJTRACK_K_THRUST_HORZ),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_VEL_YAW", 15, AC_TrajTrack, velocity_yaw, TRAJTRACK_VELOCITY_YAW),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_VXY_MAX", 16, AC_TrajTrack, vxy_max, TRAJTRACK_VELOCITY_XY_MAX),
    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_VZ_MAX", 17, AC_TrajTrack, vz_max, TRAJTRACK_VELOCITY_Z_MAX),
    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_AXY_MAX", 18, AC_TrajTrack, axy_max, TRAJTRACK_ACCEL_XY_MAX),
    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_AZ_MAX", 19, AC_TrajTrack, az_max, TRAJTRACK_ACCEL_Z_MAX),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_ATT_TAO", 20, AC_TrajTrack, attctrl_tau, TRAJTRACK_ATT_TAO),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_NORM_THR", 21, AC_TrajTrack, norm_thrust_const, TRAJTRACK_NORM_THR),
    AP_GROUPEND};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_TrajTrack::AC_TrajTrack(const AP_AHRS_View &ahrs, const AP_InertialNav &inav,
                           const AP_Motors &motors, AC_AttitudeControl &attitude_control) : _ahrs(ahrs),
                                                                                            _inav(inav),
                                                                                            _motors(motors),
                                                                                            _attitude_control(attitude_control)
{
    AP_Param::setup_object_defaults(this, var_info);
}

///
/// z-axis position controller
///

/// set_dt - sets time delta in seconds for all controllers (i.e. 100hz = 0.01, 400hz = 0.0025)
// void AC_TrajTrack::set_dt(float delta_sec)
// {
//     _dt = delta_sec;

//     // update PID controller dt
//     _pid_accel_z.set_dt(_dt);
//     _pid_vel_xy.set_dt(_dt);

//     // update rate z-axis velocity error and accel error filters
//     _vel_error_filter.set_cutoff_frequency(TRAJTRACK_VEL_ERROR_CUTOFF_FREQ);
// }
void AC_TrajTrack::init()
{
    _pos_target.zero();
    _pos_error.zero();
    _vel_desired.zero();   // desired velocity in cm/s
    _vel_target.zero();    // velocity target in cm/s calculated by pos_to_rate step
    _vel_error.zero();     // error between desired and actual acceleration in cm/s
    _vel_last.zero();      // previous iterations velocity in cm/s
    _accel_desired.zero(); // desired acceleration in cm/s/s (feed forward)
    _accel_target.zero();  // acceleration target in cm/s/s
    _accel_error.zero();
    _gravity(0, 0, -9.8);
    D_.zero();
    Rot_ned2enu(Vector3f(0, 1, 0),
                Vector3f(1, 0, 0),
                Vector3f(0, 0, -1));
}

void AC_TrajTrack::update(Trajectory::reference reference) //enu
{
    _pos_target = Rot_ned2enu * reference.pos;
    _vel_target = Rot_ned2enu * reference.vel;
    _accel_target = Rot_ned2enu * reference.acc;
    _yaw_target = reference.yaw;
    Vector3f pos, vel;
    pos = _inav.get_position()/100;
    vel = _inav.get_velocity()/100;
    pos.z = -pos.z;
    vel.z = -vel.z;
    _pos_error = Rot_ned2enu * pos - _pos_target;
    _vel_error = Rot_ned2enu * vel - _vel_target;
    Matrix3f att_rot = _ahrs.get_rotation_body_to_ned();
    att_rot = Rot_ned2enu * att_rot; //body to enu
    Quaternion att_quat = rot2Quaternion(att_rot);
    Quaternion q_ref = acc2quaternion(_accel_target - _gravity, _yaw_target);
    Matrix3f R_ref = quat2RotMatrix(q_ref);
    Vector3f Kpos(kpxy.get(), kpxy.get(), kpz.get());
    Vector3f Kvel(kvxy.get(), kvxy.get(), kvz.get());
    Vector3f a_fb, a_rd;
    a_fb.x = -Kpos.x * _pos_error.x - Kvel.x * _vel_error.x;
    a_fb.y = -Kpos.y * _pos_error.y - Kvel.y * _vel_error.y;
    a_fb.z = -Kpos.z * _pos_error.z - Kvel.z * _vel_error.z;
    if (a_fb.x > axy_max)
        a_fb.x = axy_max;
    else if (a_fb.x < -axy_max)
        a_fb.x = -axy_max;
    if (a_fb.y > axy_max)
        a_fb.y = axy_max;
    else if (a_fb.y < -axy_max)
        a_fb.y = -axy_max;
    if (a_fb.z > az_max)
        a_fb.z = az_max;
    else if (a_fb.z < -az_max)
        a_fb.z = -az_max;
    D_.a.x = k_drag_x;
    D_.b.y = k_drag_y;
    D_.c.z = k_drag_z;
    a_rd = R_ref * D_ * R_ref.transposed() * _vel_target;
    _accel_desired = a_fb + _accel_target - a_rd - _gravity;
    Quaternion q_des = acc2quaternion(_accel_desired, _yaw_target);
    Matrix3f R_body2enu_des = quat2RotMatrix(q_des);
    Matrix3f R_body2ned_des = Rot_ned2enu.transposed() * R_body2enu_des;
    float roll_target, pitch_target, yaw_target;
    R_body2enu_des.to_euler(&roll_target, &pitch_target, &yaw_target);
    _roll_target=roll_target;
    _pitch_target=pitch_target;
    _yaw_target=yaw_target;
    attcontroller(q_des, _accel_desired, att_quat);
}

void AC_TrajTrack::attcontroller(Quaternion &ref_att, Vector3f &ref_acc, Quaternion &curr_att)
{
    Quaternion qe, q_inv, inverse;
    Matrix3f rotmat;
    Vector3f zb;
    inverse(1.0, -1.0, -1.0, -1.0);
    q_inv(curr_att[0], -curr_att[1], -curr_att[2], -curr_att[3]);
    qe = quatMultiplication(q_inv, ref_att);
    rate_cmd[0] = (2.0 / attctrl_tau) * std::copysign(1.0, qe[0]) * qe[1];
    rate_cmd[1] = (2.0 / attctrl_tau) * std::copysign(1.0, qe[0]) * qe[2];
    rate_cmd[2] = (2.0 / attctrl_tau) * std::copysign(1.0, qe[0]) * qe[3];
    rotmat = quat2RotMatrix(curr_att);
    zb(rotmat.a.z, rotmat.b.z, rotmat.c.z);
    throttle_des = ref_acc[0] * zb[0] + ref_acc[1] * zb[1] + ref_acc[2] * zb[2];
    throttle_des = constrain_float(norm_thrust_const * throttle_des, 0.0, 1.0);
}

void AC_TrajTrack::get_rate_throttle(Vector3f &rate_cmd_, double &throttle_des_)
{
    rate_cmd_ = rate_cmd;
    throttle_des_ = throttle_des;
}

Quaternion AC_TrajTrack::quatMultiplication(Quaternion &q, Quaternion &p)
{
    Quaternion quat;
    quat(p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3],
         p[0] * q[1] + p[1] * q[0] - p[2] * q[3] + p[3] * q[2],
         p[0] * q[2] + p[1] * q[3] + p[2] * q[0] - p[3] * q[1],
         p[0] * q[3] - p[1] * q[2] + p[2] * q[1] + p[3] * q[0]);
    return quat;
}

Quaternion AC_TrajTrack::acc2quaternion(Vector3f vector_acc, double yaw)
{
    Quaternion quat;
    Vector3f zb_des, yb_des, xb_des, proj_xb_des;
    Matrix3f rotmat;

    if (velocity_yaw)
        proj_xb_des = _vel_target.normalized();
    else
        proj_xb_des(std::cos(yaw), std::sin(yaw), 0.0);
    zb_des = vector_acc / vector_acc.length();
    yb_des = zb_des % (proj_xb_des) / (zb_des % (proj_xb_des)).length();
    xb_des = yb_des % (zb_des) / (yb_des % (zb_des)).length();

    rotmat(Vector3f(xb_des[0], yb_des[0], zb_des[0]),
           Vector3f(xb_des[1], yb_des[1], zb_des[1]),
           Vector3f(xb_des[2], yb_des[2], zb_des[2]));
    quat = rot2Quaternion(rotmat);
    return quat;
}

Quaternion AC_TrajTrack::rot2Quaternion(Matrix3f R)
{
    Quaternion quat;
    double tr = R.a.x + R.b.y + R.c.z;
    if (tr > 0.0)
    {
        double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
        quat[0] = 0.25 * S;
        quat[1] = (R[2][1] - R[1][2]) / S;
        quat[2] = (R[0][2] - R[2][0]) / S;
        quat[3] = (R[1][0] - R[0][1]) / S;
    }
    else if ((R[0][0] > R[1][1]) & (R[0][0] > R[2][2]))
    {
        double S = sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0; // S=4*qx
        quat[0] = (R[2][1] - R[1][2]) / S;
        quat[1] = 0.25 * S;
        quat[2] = (R[0][1] + R[1][0]) / S;
        quat[3] = (R[0][2] + R[2][0]) / S;
    }
    else if (R[1][1] > R[2][2])
    {
        double S = sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0; // S=4*qy
        quat[0] = (R[0][2] - R[2][0]) / S;
        quat[1] = (R[0][1] + R[1][0]) / S;
        quat[2] = 0.25 * S;
        quat[3] = (R[1][2] + R[2][1]) / S;
    }
    else
    {
        double S = sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0; // S=4*qz
        quat[0] = (R[1][0] - R[0][1]) / S;
        quat[1] = (R[0][2] + R[2][0]) / S;
        quat[2] = (R[1][2] + R[2][1]) / S;
        quat[3] = 0.25 * S;
    }
    return quat;
}

Matrix3f AC_TrajTrack::quat2RotMatrix(Quaternion q)
{
    Matrix3f rotmat;
    rotmat(Vector3f(q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3],
                    2 * q[1] * q[2] - 2 * q[0] * q[3],
                    2 * q[0] * q[2] + 2 * q[1] * q[3]),

           Vector3f(2 * q[0] * q[3] + 2 * q[1] * q[2],
                    q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3],
                    2 * q[2] * q[3] - 2 * q[0] * q[1]),

           Vector3f(2 * q[1] * q[3] - 2 * q[0] * q[2],
                    2 * q[0] * q[1] + 2 * q[2] * q[3],
                    q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]));
    return rotmat;
}

/// set_max_accel_z - set the maximum vertical acceleration in cm/s/s
// void AC_TrajTrack::set_max_accel_z(float accel_cmss)
// {
//     if (fabsf(_accel_z_cms - accel_cmss) > 1.0f) {
//         _accel_z_cms = accel_cmss;
//         _flags.recalc_leash_z = true;
//         calc_leash_length_z();
//     }
// }

/// set_alt_target_with_slew - adjusts target towards a final altitude target
///     should be called continuously (with dt set to be the expected time between calls)
///     actual position target will be moved no faster than the speed_down and speed_up
///     target will also be stopped if the motors hit their limits or leash length is exceeded

/// set_alt_target_from_climb_rate - adjusts target up or down using a climb rate in cm/s
///     should be called continuously (with dt set to be the expected time between calls)
///     actual position target will be moved no faster than the speed_down and speed_up
///     target will also be stopped if the motors hit their limits or leash length is exceeded

/// set_target_to_stopping_point_z - returns reasonable stopping altitude in cm above home

/// initialise ekf xy position reset check

/// limit vector to a given length, returns true if vector was limited

/// Proportional controller with piecewise sqrt sections to constrain second derivative
