#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>

namespace Trajectory
{
struct reference
{
    reference(Vector3f pos_, Vector3f vel_, Vector3f acc_)
    {
        pos = pos_;
        vel = vel_;
        acc = acc_;
        yaw = atan2(vel_.y,vel_.x);
    }
    Vector3f pos;
    Vector3f vel;
    Vector3f acc;
    double yaw;
};

struct state
{
    Vector3f pos;
    Vector3f vel;
    Vector3f acc;
};

struct Color
{
    Color()
    {
        r = 0;
        g = 0;
        b = 0;
    }
    double r;
    double g;
    double b;
};

class BasicState
{
public:
    virtual void getPos(double elapsed_t, Vector3f &pos, Vector3f &vel, Vector3f &accel, Color &color) = 0;

    virtual bool checkTime(double elapsed_t) = 0;
};

class RotationState : public BasicState
{
public:
    RotationState(Vector3f posOrigin, Vector3f layerCenter) : posOrigin_(posOrigin),
                                                              layerCenter_(layerCenter) {}

    virtual void getPos(double elapsed_t, Vector3f &pos, Vector3f &vel, Vector3f &accel, Color &color)
    {
        elapsed_t -= rotStartTime_;
        if (rotateDuration_ <= 0)
        {
            pos[0] = finalPos_[0];
            pos[1] = finalPos_[1];
            pos[2] = finalPos_[2];
            vel.zero();
            accel.zero();
            color.r = 0;
            color.g = 0;
            color.b = 0;
            return;
        }
        if (elapsed_t <= rotateDuration_)
        {
            double rotateAngleCurrent = rotateRate_ * elapsed_t;
            Vector3f centerPositionInRotFrame, centerCurrentPos, currentPos, r_pos_origin_w, origin_pos, velocity_rot_w, accel_rot_w, velocity_pos_center_w, origin_velocity, origin_accel;
            centerPositionInRotFrame.x = rotateRadius_ * cos(rotateAngleCurrent);
            centerPositionInRotFrame.y = rotateRadius_ * sin(rotateAngleCurrent);
            centerPositionInRotFrame.z = 0;
            if (elapsed_t < positiveAccelTime_)
            {
                origin_pos = rotateOrigin_ + rotateOriginAcceleration_ * 0.5 * elapsed_t * elapsed_t;
                origin_velocity = rotateOriginAcceleration_ * elapsed_t;
                origin_accel = rotateOriginAcceleration_;
            }
            else if (elapsed_t < velTime_ && elapsed_t > positiveAccelTime_)
            {
                origin_pos = rotateOrigin_ + rotateOriginAcceleration_ * 0.5 * positiveAccelTime_ * positiveAccelTime_ + rotateOriginVelocity_ * (elapsed_t - positiveAccelTime_);
                origin_velocity = rotateOriginVelocity_;
                origin_accel.zero();
            }
            else
            {
                origin_pos = rotateOrigin_ + rotateOriginAcceleration_ * 0.5 * positiveAccelTime_ * positiveAccelTime_ + rotateOriginVelocity_ * (velTime_ - positiveAccelTime_) + rotateOriginVelocity_ * (elapsed_t - velTime_) + (-rotateOriginAcceleration_) * 0.5 * (elapsed_t - velTime_) * (elapsed_t - velTime_);
                origin_velocity = rotateOriginVelocity_ + (-rotateOriginAcceleration_) * (elapsed_t - velTime_);
                origin_accel = -rotateOriginAcceleration_;
            }
            centerCurrentPos = Twr_ * centerPositionInRotFrame + origin_pos;
            Tlr.a.x = cos(rotateAngleCurrent);
            Tlr.a.y = sin(rotateAngleCurrent);
            Tlr.a.z = 0;
            Tlr.b.x = -sin(rotateAngleCurrent);
            Tlr.b.y = cos(rotateAngleCurrent);
            Tlr.b.z = 0;
            Tlr.c.x = 0;
            Tlr.c.y = 0;
            Tlr.c.z = 1;
            currentPos = centerCurrentPos +
                         Twr_ * Tlr.transposed() * posInLayerFrame *
                             ((scale_ratio - 1) / rotateDuration_ * elapsed_t + 1);
            r_pos_origin_w = currentPos - origin_pos;
            velocity_rot_w = omegaHat * r_pos_origin_w;
            accel_rot_w = omegaHat * (omegaHat * r_pos_origin_w);
            velocity_pos_center_w = Twr_ * (Tlr.transposed() * centerRelativeVelocity_l);
            pos = currentPos;
            vel = velocity_rot_w + velocity_pos_center_w + origin_velocity;
            accel = origin_accel + accel_rot_w;
            color.r = begin_color[0] + (rotation_color[0] - begin_color[0]) * elapsed_t / rotateDuration_;
            color.g = begin_color[1] + (rotation_color[1] - begin_color[1]) * elapsed_t / rotateDuration_;
            color.b = begin_color[2] + (rotation_color[2] - begin_color[2]) * elapsed_t / rotateDuration_;
            return;
        }
        else
        {
            pos[0] = finalPos_[0];
            pos[1] = finalPos_[1];
            pos[2] = finalPos_[2];
            vel.zero();
            accel.zero();
            color.r = rotation_color[0] +
                      (hold_color[0] - rotation_color[0]) * (elapsed_t - rotateDuration_) / rotateHoldTime_;
            color.g = rotation_color[1] +
                      (hold_color[1] - rotation_color[1]) * (elapsed_t - rotateDuration_) / rotateHoldTime_;
            color.b = rotation_color[2] +
                      (hold_color[2] - rotation_color[2]) * (elapsed_t - rotateDuration_) / rotateHoldTime_;
            return;
            ;
        }
    }

    virtual bool checkTime(double elapsed_t)
    {
        if (elapsed_t >= rotStartTime_ && elapsed_t < rotEndTime_)
            return true;
        else
            return false;
    }

    void setRotate(Vector3f vec1, Vector3f vec2, double rotateDegrees, Vector3f rotateOriginPath,
                   double positiveAccelTime, double velTime, double negativeAccelTime, double rotateHoldTime, bool isAxisPassed, double scaleRatio, Color begin_clr, Color rotation_clr,
                   Color hold_clr, double rotationStartTime = 0)
    {
        rotStartTime_ = rotationStartTime;
        rotateDuration_ = negativeAccelTime;
        rotateHoldTime_ = rotateHoldTime;
        positiveAccelTime_ = positiveAccelTime;
        negativeAccelTime_ = negativeAccelTime;
        velTime_ = velTime;
        rotEndTime_ = rotStartTime_ + rotateDuration_ + rotateHoldTime_;
        rotateRadian_ = rotateDegrees / 180 * M_PI;
        scale_ratio = scaleRatio;
        if (rotateDuration_ <= 0)
        {
            finalPos_ = posOrigin_;
            return;
        }
        rotateRate_ = rotateRadian_ / rotateDuration_;
        if (isAxisPassed)
            axisOrientNormalized_ = vec2.normalized();
        else
            axisOrientNormalized_ = (vec2 - vec1).normalized();
        Vector3f omega;
        omega = axisOrientNormalized_ * rotateRate_;
        omegaHat.a.x = 0;
        omegaHat.a.y = -omega[2];
        omegaHat.a.z = omega[1];
        omegaHat.b.x = omega[2];
        omegaHat.b.y = 0;
        omegaHat.b.z = -omega[0];
        omegaHat.c.x = -omega[1];
        omegaHat.c.y = omega[0];
        omegaHat.c.z = 0;
        double t =
            -(vec1[0] * axisOrientNormalized_[0] - layerCenter_[0] * axisOrientNormalized_[0] +
              vec1[1] * axisOrientNormalized_[1] - layerCenter_[1] * axisOrientNormalized_[1] +
              vec1[2] * axisOrientNormalized_[2] - layerCenter_[2] * axisOrientNormalized_[2]) /
            (axisOrientNormalized_[0] * axisOrientNormalized_[0] +
             axisOrientNormalized_[1] * axisOrientNormalized_[1] +
             axisOrientNormalized_[2] * axisOrientNormalized_[2]);
        rotateOrigin_[0] = vec1[0] + axisOrientNormalized_[0] * t;
        rotateOrigin_[1] = vec1[1] + axisOrientNormalized_[1] * t;
        rotateOrigin_[2] = vec1[2] + axisOrientNormalized_[2] * t;
        rotateRadius_ = (layerCenter_ - rotateOrigin_).length();
        tangentialVelocity_ = rotateRate_ * rotateRadius_;
        if (rotateRadius_ > 0)
            rotFrameXinWorld_ = (layerCenter_ - rotateOrigin_) / rotateRadius_;
        else
        {
            for (int i = 0; i < 3; ++i)
            {
                if (axisOrientNormalized_[i] > 0 || axisOrientNormalized_[i] < 0)
                {
                    if (i == 0)
                    {
                        rotFrameXinWorld_.x = -(axisOrientNormalized_[1] + axisOrientNormalized_[2]) / axisOrientNormalized_[0];
                        rotFrameXinWorld_.y = 1;
                        rotFrameXinWorld_.z = 1;
                    }
                    else if (i == 1)
                    {
                        rotFrameXinWorld_.x = 1;
                        rotFrameXinWorld_.y = -(axisOrientNormalized_[0] + axisOrientNormalized_[2]) / axisOrientNormalized_[1];
                        rotFrameXinWorld_.z = 1;
                    }
                    else if (i == 2)
                    {
                        rotFrameXinWorld_.x = 1;
                        rotFrameXinWorld_.y = 1;
                        rotFrameXinWorld_.z = -(axisOrientNormalized_[0] + axisOrientNormalized_[1]) / axisOrientNormalized_[2];
                    }
                }
            }
        }
        rotFrameXinWorld_ /= rotFrameXinWorld_.length();
        rotFrameZinWorld_ = axisOrientNormalized_;
        Matrix3f rotFrameZinWorldHat(0, -rotFrameZinWorld_[2], rotFrameZinWorld_[1],
                                     rotFrameZinWorld_[2], 0, -rotFrameZinWorld_[0],
                                     -rotFrameZinWorld_[1], rotFrameZinWorld_[0], 0);
        rotFrameYinWorld_ = rotFrameZinWorldHat * rotFrameXinWorld_;
        rotFrameYinWorld_ /= rotFrameYinWorld_.length();
        Twr_.a.x = rotFrameXinWorld_[0];
        Twr_.a.y = rotFrameYinWorld_[0];
        Twr_.a.z = rotFrameZinWorld_[0];
        Twr_.b.x = rotFrameXinWorld_[1];
        Twr_.b.y = rotFrameYinWorld_[1];
        Twr_.b.z = rotFrameZinWorld_[1];
        Twr_.c.x = rotFrameXinWorld_[2];
        Twr_.c.y = rotFrameYinWorld_[2];
        Twr_.c.z = rotFrameZinWorld_[2];
        rotateOriginPath_ = rotateOriginPath;
        rotateOriginVelocity_ = rotateOriginPath_ / velTime;
        rotateOriginAcceleration_ = rotateOriginVelocity_ / positiveAccelTime;
        Vector3f centerFinalPosInRotFrame;
        centerFinalPosInRotFrame.x = rotateRadius_ * cos(rotateRadian_);
        centerFinalPosInRotFrame.y = rotateRadius_ * sin(rotateRadian_);
        centerFinalPosInRotFrame.z = 0;
        Vector3f centerFinalPos = Twr_ * centerFinalPosInRotFrame + rotateOrigin_ + rotateOriginPath_;
        posInLayerFrame = Twr_.transposed() * (posOrigin_ - layerCenter_);
        centerRelativeVelocity_l = posInLayerFrame * (scale_ratio - 1) / rotateDuration_;
        Matrix3f finalT_lr(cos(rotateRadian_), sin(rotateRadian_), 0,
                           -sin(rotateRadian_), cos(rotateRadian_), 0,
                           0, 0, 1);
        finalPos_ = centerFinalPos + Twr_ * finalT_lr.transposed() * posInLayerFrame * scale_ratio;
        rotation_color[0] = rotation_clr.r;
        rotation_color[1] = rotation_clr.g;
        rotation_color[2] = rotation_clr.b;
        hold_color[0] = hold_clr.r;
        hold_color[1] = hold_clr.g;
        hold_color[2] = hold_clr.b;
        begin_color[0] = begin_clr.r;
        begin_color[1] = begin_clr.g;
        begin_color[2] = begin_clr.b;
    }

    Vector3f getFinalPos()
    {
        return finalPos_;
    }

    void getFinalColor(double finalColor[3])
    {
        finalColor[0] = hold_color[0];
        finalColor[1] = hold_color[1];
        finalColor[2] = hold_color[2];
    }

    double getRotateDuration()
    {
        return rotateDuration_;
    }

    double getRotateHoldTime()
    {
        return rotateHoldTime_;
    }

    double getRotateStartTime()
    {
        return rotStartTime_;
    }

    double getRotateEndTime()
    {
        return rotEndTime_;
    }

    Vector3f rotateOrigin_;
    Vector3f rotateOriginVelocity_;
    Vector3f rotateOriginAcceleration_;
    Vector3f axisOrientNormalized_;
    Vector3f posOrigin_;
    Vector3f layerCenter_;
    Vector3f rotateOriginPath_;
    Vector3f rotFrameXinWorld_;
    Vector3f rotFrameYinWorld_;
    Vector3f rotFrameZinWorld_;
    double rotateDuration_;
    double rotateHoldTime_;
    double rotateRadius_;
    double rotateRadian_;
    double rotateRate_;
    double tangentialVelocity_;
    double rotStartTime_;
    double rotEndTime_;
    double positiveAccelTime_;
    double negativeAccelTime_;
    double velTime_;
    Matrix3f Twr_;
    Matrix3f Tlr;
    Vector3f finalPos_;
    Vector3f posInLayerFrame;
    Vector3f centerRelativeVelocity_l;
    double begin_color[3];
    double rotation_color[3];
    double hold_color[3];
    double scale_ratio;
    Matrix3f omegaHat;
};

class FlightState : public BasicState
{
public:
    FlightState() {}

    struct Segment
    {
        double *Px;
        double *Py;
        double *Pz;
        double *Pvx;
        double *Pvy;
        double *Pvz;
        double segment_start_time;
        double segment_end_time;
        int degreeD;

        // void setup(swarm_traj_msgs::Trajectory_Segment traj_seg, double sequence_start_time)
        // {
        //     degreeD = traj_seg.poly_order;
        //     segment_start_time = traj_seg.segment_start_time + sequence_start_time;
        //     segment_end_time = traj_seg.segment_end_time + sequence_start_time;
        //     Px = new double[degreeD + 1];
        //     Py = new double[degreeD + 1];
        //     Pz = new double[degreeD + 1];
        //     Pvx = new double[degreeD];
        //     Pvy = new double[degreeD];
        //     Pvz = new double[degreeD];
        //     for (int i = 0; i < degreeD + 1; ++i)
        //     {
        //         Px[i] = traj_seg.coef_x[i];
        //         Py[i] = traj_seg.coef_y[i];
        //         Pz[i] = traj_seg.coef_z[i];
        //     }
        //     for (int i = 0; i < degreeD; ++i)
        //     {
        //         Pvx[i] = traj_seg.coef_vx[i];
        //         Pvy[i] = traj_seg.coef_vy[i];
        //         Pvz[i] = traj_seg.coef_vz[i];
        //     }
        // }

        void getPos(double elapsed_t, Vector3f &pos, Vector3f &vel, Vector3f &accel, Color &color)
        {
            double tao = (elapsed_t - segment_start_time) / (segment_end_time - segment_start_time);
            double *Tao = new double[degreeD + 1];
            double tao_pow = 1;
            for (int k = 0; k < degreeD + 1; ++k)
            {
                Tao[k] = tao_pow;
                tao_pow *= tao;
            }
            pos[0] = 0;
            pos[1] = 0;
            pos[2] = 0;
            for (int k = 0; k < degreeD + 1; ++k)
            {
                pos[0] += Tao[k] * Px[k];
                pos[1] += Tao[k] * Py[k];
                pos[2] += Tao[k] * Pz[k];
            }
        }
    };

    // void setup(swarm_traj_msgs::Trajectory traj)
    // {
    //     segment_count = traj.segment_count;
    //     segments = new Segment[segment_count];
    //     traj_start_time = traj.trajectory_start_time;
    //     traj_end_time = traj.trajectory_end_time;
    //     for (int i = 0; i < segment_count; ++i)
    //     {
    //         segments[i].setup(traj.traj_segments[i], traj_start_time);
    //     }
    //     hold_start_time = segments[segment_count - 1].segment_end_time;
    //     all_flight_end_time = traj.trajectory_end_time - traj.time_to_hold_s;
    //     goal_pos[0] = traj.goal_x;
    //     goal_pos[1] = traj.goal_y;
    //     goal_pos[2] = traj.goal_z;
    //     flight_color[0] = traj.flight_color.r;
    //     flight_color[1] = traj.flight_color.g;
    //     flight_color[2] = traj.flight_color.b;
    //     hold_color[0] = traj.hold_color.r;
    //     hold_color[1] = traj.hold_color.g;
    //     hold_color[2] = traj.hold_color.b;
    //     flight_begin_color[0] = traj.begin_color.r;
    //     flight_begin_color[1] = traj.begin_color.g;
    //     flight_begin_color[2] = traj.begin_color.b;
    // }

    virtual bool checkTime(double elapsed_t)
    {
        if (elapsed_t >= traj_start_time && elapsed_t < traj_end_time)
            return true;
        else
            return false;
    }

    virtual void getPos(double elapsed_t, Vector3f &pos, Vector3f &vel, Vector3f &accel, Color &color)
    {
        if (elapsed_t >= hold_start_time && elapsed_t < traj_end_time)
        {
            pos[0] = goal_pos[0];
            pos[1] = goal_pos[1];
            pos[2] = goal_pos[2];
        }
        else
        {
            for (int i = 0; i < segment_count; ++i)
            {
                if (elapsed_t >= segments[i].segment_start_time && elapsed_t < segments[i].segment_end_time)
                {
                    segments[i].getPos(elapsed_t, pos, vel, accel, color);
                    break;
                }
            }
        }
        if (elapsed_t >= all_flight_end_time && elapsed_t < traj_end_time)
        {
            color.r = flight_color[0] + (hold_color[0] - flight_color[0]) * (elapsed_t - all_flight_end_time) /
                                            (traj_end_time - all_flight_end_time);
            color.g = flight_color[1] + (hold_color[1] - flight_color[1]) * (elapsed_t - all_flight_end_time) /
                                            (traj_end_time - all_flight_end_time);
            color.b = flight_color[2] + (hold_color[2] - flight_color[2]) * (elapsed_t - all_flight_end_time) /
                                            (traj_end_time - all_flight_end_time);
        }
        else
        {
            color.r = flight_begin_color[0] +
                      (flight_color[0] - flight_begin_color[0]) * (elapsed_t - traj_start_time) /
                          (all_flight_end_time - traj_start_time);
            color.g = flight_begin_color[1] +
                      (flight_color[1] - flight_begin_color[1]) * (elapsed_t - traj_start_time) /
                          (all_flight_end_time - traj_start_time);
            color.b = flight_begin_color[2] +
                      (flight_color[2] - flight_begin_color[2]) * (elapsed_t - traj_start_time) /
                          (all_flight_end_time - traj_start_time);
        }
    }

    void getFinalColor(double final_color[3])
    {
        final_color[0] = hold_color[0];
        final_color[1] = hold_color[1];
        final_color[2] = hold_color[2];
    }

    double getEndTime()
    {
        return traj_end_time;
    }

    int segment_count;
    double hold_start_time;
    double traj_start_time;
    double traj_end_time;
    double all_flight_end_time;
    double goal_pos[3];
    double flight_begin_color[3];
    double flight_color[3];
    double hold_color[3];
    Segment *segments;
};

class States
{
public:
    struct TrajAndRotsSingleSeq
    {
        struct RotMsg
        {
            // RotMsg(swarm_traj_msgs::Rotation rot_msg)
            // {
            //     rot_origin[0] = rot_msg.rotate_origin.x;
            //     rot_origin[1] = rot_msg.rotate_origin.y;
            //     rot_origin[2] = rot_msg.rotate_origin.z;
            //     rot_orient[0] = rot_msg.rotate_orient.x;
            //     rot_orient[1] = rot_msg.rotate_orient.y;
            //     rot_orient[2] = rot_msg.rotate_orient.z;
            //     rot_origin_path[0] = rot_msg.rotate_origin_path.x;
            //     rot_origin_path[1] = rot_msg.rotate_origin_path.y;
            //     rot_origin_path[2] = rot_msg.rotate_origin_path.z;
            //     pos_origin[0] = rot_msg.pos_origin.x;
            //     pos_origin[1] = rot_msg.pos_origin.y;
            //     pos_origin[2] = rot_msg.pos_origin.z;
            //     layer_center[0] = rot_msg.layer_center.x;
            //     layer_center[1] = rot_msg.layer_center.y;
            //     layer_center[2] = rot_msg.layer_center.z;
            //     rot_degrees = rot_msg.rotate_degree;
            //     rot_duration = rot_msg.rotate_duration;
            //     rot_hold_time = rot_msg.rotate_holdtime;
            //     rotation_clr.r = rot_msg.rotation_color.r;
            //     rotation_clr.g = rot_msg.rotation_color.g;
            //     rotation_clr.b = rot_msg.rotation_color.b;
            //     hold_clr.r = rot_msg.hold_color.r;
            //     hold_clr.g = rot_msg.hold_color.g;
            //     hold_clr.b = rot_msg.hold_color.b;
            //     scale_ratio = rot_msg.scale_ratio;
            // }
            Vector3f rot_origin;
            Vector3f rot_orient;
            Vector3f rot_origin_path;
            Vector3f pos_origin;
            Vector3f layer_center;
            double rot_degrees;
            double rot_duration;
            double rot_hold_time;
            double rot_start_time;
            Color begin_clr;
            Color rotation_clr;
            Color hold_clr;
            double scale_ratio;
        };

        // void setup(swarm_traj_msgs::TrajectoryWithRotation traj_with_rot_msg)
        // {
        //     rot_count = traj_with_rot_msg.rotation_count;
        //     sequence_id = traj_with_rot_msg.trajectory.sequence_id;
        //     sequence_start_time = traj_with_rot_msg.sequence_start_time;
        //     sequence_end_time = traj_with_rot_msg.sequence_end_time;
        //     flight = new FlightState;
        //     flight->setup(traj_with_rot_msg.trajectory);
        //     rotations = new RotationState *[rot_count];
        //     for (int i = 0; i < rot_count; ++i)
        //     {
        //         RotMsg rot_msg(traj_with_rot_msg.rotation.rotations_sguav[i]);
        //         if (i == 0)
        //         {
        //             rot_msg.rot_start_time = flight->traj_end_time;
        //             rot_msg.begin_clr.r = flight->hold_color[0];
        //             rot_msg.begin_clr.g = flight->hold_color[1];
        //             rot_msg.begin_clr.b = flight->hold_color[2];
        //         }
        //         else
        //         {
        //             rot_msg.rot_start_time = rotations[i - 1]->getRotateEndTime();
        //             rot_msg.begin_clr.r = rotations[i - 1]->hold_color[0];
        //             rot_msg.begin_clr.g = rotations[i - 1]->hold_color[1];
        //             rot_msg.begin_clr.b = rotations[i - 1]->hold_color[2];
        //         }
        //         rotations[i] = new RotationState(rot_msg.pos_origin, rot_msg.layer_center);
        //         rotations[i]->setRotate(rot_msg.rot_origin, rot_msg.rot_orient, rot_msg.rot_degrees,
        //                                 rot_msg.rot_origin_path, rot_msg.rot_duration, rot_msg.rot_hold_time, true,
        //                                 rot_msg.begin_clr, rot_msg.rotation_clr, rot_msg.hold_clr, rot_msg.scale_ratio,
        //                                 rot_msg.rot_start_time);
        //     }
        //     if (rot_count == 0)
        //     {
        //         sequence_end_color[0] = traj_with_rot_msg.trajectory.hold_color.r;
        //         sequence_end_color[1] = traj_with_rot_msg.trajectory.hold_color.g;
        //         sequence_end_color[2] = traj_with_rot_msg.trajectory.hold_color.b;
        //     }
        //     else
        //     {
        //         sequence_end_color[0] = traj_with_rot_msg.rotation.rotations_sguav[rot_count - 1].hold_color.r;
        //         sequence_end_color[1] = traj_with_rot_msg.rotation.rotations_sguav[rot_count - 1].hold_color.g;
        //         sequence_end_color[2] = traj_with_rot_msg.rotation.rotations_sguav[rot_count - 1].hold_color.b;
        //     }
        // }

        bool checkTime(double elapsed_t)
        {
            if (elapsed_t >= sequence_start_time && elapsed_t < sequence_end_time)
                return true;
            else
                return false;
        }

        void getPos(double elapsed_t, Vector3f &pos, Vector3f &vel, Vector3f &accel, Color &color)
        {
            if (elapsed_t >= flight->traj_start_time && elapsed_t < flight->traj_end_time)
            {
                flight->getPos(elapsed_t, pos, vel, accel, color);
                return;
            }
            else
            {
                for (int i = 0; i < rot_count; ++i)
                {
                    if (elapsed_t >= rotations[i]->getRotateStartTime() &&
                        elapsed_t < rotations[i]->getRotateEndTime())
                    {
                        rotations[i]->getPos(elapsed_t, pos, vel, accel, color);
                        return;
                    }
                }
            }
            if (rot_count > 0)
            {
                Vector3f pos_ = rotations[rot_count - 1]->getFinalPos();
                pos[0] = pos_[0];
                pos[1] = pos_[1];
                pos[2] = pos_[2];
                double rot_final_color[3];
                rotations[rot_count - 1]->getFinalColor(rot_final_color);
                double rot_final_time = rotations[rot_count - 1]->getRotateEndTime();
                color.r = rot_final_color[0] +
                          (sequence_end_color[0] - rot_final_color[0]) * (elapsed_t - rot_final_time) /
                              (sequence_end_time - rot_final_time);
                color.g = rot_final_color[1] +
                          (sequence_end_color[1] - rot_final_color[1]) * (elapsed_t - rot_final_time) /
                              (sequence_end_time - rot_final_time);
                color.b = rot_final_color[2] +
                          (sequence_end_color[2] - rot_final_color[2]) * (elapsed_t - rot_final_time) /
                              (sequence_end_time - rot_final_time);
            }
            else
            {
                pos[0] = flight->goal_pos[0];
                pos[1] = flight->goal_pos[1];
                pos[2] = flight->goal_pos[2];
                double traj_final_color[3];
                flight->getFinalColor(traj_final_color);
                double traj_end_time = flight->getEndTime();
                color.r = traj_final_color[0] +
                          (sequence_end_color[0] - traj_final_color[0]) * (elapsed_t - traj_end_time) /
                              (sequence_end_time - traj_end_time);
                color.g = traj_final_color[1] +
                          (sequence_end_color[1] - traj_final_color[1]) * (elapsed_t - traj_end_time) /
                              (sequence_end_time - traj_end_time);
                color.b = traj_final_color[2] +
                          (sequence_end_color[2] - traj_final_color[2]) * (elapsed_t - traj_end_time) /
                              (sequence_end_time - traj_end_time);
            }
        }

        FlightState *flight;
        RotationState **rotations;
        int rot_count;
        double sequence_start_time;
        double sequence_end_time;
        int sequence_id;
        double sequence_end_color[3];
    };

    States()
    {
        States(1, 0);
    }

    States(int uavid, int sequence_cnt)
    {
        uav_id = uavid;
        sequence_count = sequence_cnt;
        statesSeqs = new TrajAndRotsSingleSeq *[sequence_cnt];
    }

    // void addSequence()
    // {
    //     // int sequence_id = traj_with_rot_msg.trajectory.sequence_id;
    //     statesSeqs[sequence_id] = new TrajAndRotsSingleSeq;
    //     // statesSeqs[sequence_id]->setup(traj_with_rot_msg);
    // }

    void getPos(double elapsed_t, Vector3f &pos, Vector3f &vel, Vector3f &accel, Color &color)
    {
        for (int i = 0; i < sequence_count; ++i)
        {
            if (statesSeqs[i]->checkTime(elapsed_t))
            {
                statesSeqs[i]->getPos(elapsed_t, pos, vel, accel, color);
                return;
            }
        }
        pos[0] = statesSeqs[sequence_count - 1]->flight->goal_pos[0];
        pos[1] = statesSeqs[sequence_count - 1]->flight->goal_pos[1];
        pos[2] = statesSeqs[sequence_count - 1]->flight->goal_pos[2];
        color.r = 0;
        color.g = 0;
        color.b = 0;
    }

    int uav_id;
    int sequence_count;
    TrajAndRotsSingleSeq **statesSeqs;
};

} // namespace Trajectory

class AP_Trajectory
{
public:
    AP_Trajectory() {}
    ~AP_Trajectory() {}

    enum trajectory_state
    {
        TRAJECTORY_STOPPED = 0,
        TRAJECTORY_RUNNING = 1,
        TRAJECTORY_COMPLETE = 2
    };
    static AP_Trajectory *get_singleton()
    {
        return _singleton;
    }

    bool read_segment_from_storage(uint16_t index, Trajectory::FlightState::Segment &segment) const;
    bool read_rotation_from_storage(uint16_t index, Trajectory::States::TrajAndRotsSingleSeq::RotMsg &segment) const;
    void update();

private:
    static AP_Trajectory *_singleton;
    static StorageAccess _storage;
    struct Trajectory_Flags
    {
        trajectory_state state;
        uint8_t trajectory_loaded : 1; // true if a "navigation" command has been loaded into _nav_cmd
    } _flags;

    AP_Int16 _trajectory_total;
    Trajectory::States _trajectory;
};