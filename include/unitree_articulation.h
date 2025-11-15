// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "isaaclab/assets/articulation/articulation.h"
#include "FSM/FSMState.h"

namespace unitree
{

template <typename LowStatePtr, typename SportModeStatePtr, typename MotionCaptureStatePtr>
class BaseArticulation : public isaaclab::Articulation
{
public:
    BaseArticulation(LowStatePtr lowstate_, SportModeStatePtr sportmodestate_, MotionCaptureStatePtr motioncapturestate_)
    : lowstate(lowstate_), sportmodestate(sportmodestate_), motioncapturestate(motioncapturestate_)
    {
        data.joystick = &lowstate->joystick;
    }

    void update() override
    {
        Eigen::Quaternionf quat;
        Eigen::Vector3f root_lin_vel_w;
        if(FSMState::network_interface == "lo") {
            quat = Eigen::Quaternionf(
                lowstate->msg_.imu_state().quaternion()[0],
                lowstate->msg_.imu_state().quaternion()[1],
                lowstate->msg_.imu_state().quaternion()[2],
                lowstate->msg_.imu_state().quaternion()[3]
            );
            root_lin_vel_w = Eigen::Vector3f(
                sportmodestate->msg_.velocity()[0],
                sportmodestate->msg_.velocity()[1],
                sportmodestate->msg_.velocity()[2]
            );
        }
        else {
            quat = Eigen::Quaternionf(
                motioncapturestate->pose_msg_.pose.orientation.x,
                motioncapturestate->pose_msg_.pose.orientation.y,
                motioncapturestate->pose_msg_.pose.orientation.z,
                motioncapturestate->pose_msg_.pose.orientation.w
            );
            root_lin_vel_w = Eigen::Vector3f(
                motioncapturestate->vel_msg_.twist.linear.x,
                motioncapturestate->vel_msg_.twist.linear.y,
                motioncapturestate->vel_msg_.twist.linear.z
            );
        }
        Eigen::Vector3f root_lin_vel_b = quat.conjugate() * root_lin_vel_w;
        // base_linear_velocity
        for(int i(0); i<3; i++) {
            data.root_lin_vel_b[i] = root_lin_vel_b[i];
        }
        // base_angular_velocity
        for(int i(0); i<3; i++) {
            data.root_ang_vel_b[i] = lowstate->msg_.imu_state().gyroscope()[i];
        }
        // project_gravity_body
        data.projected_gravity_b = quat.conjugate() * data.GRAVITY_VEC_W;
        // joint positions and velocities
        for(int i(0); i< data.joint_ids_map.size(); i++) {
            data.joint_pos[i] = lowstate->msg_.motor_state()[data.joint_ids_map[i]].q();
            data.joint_vel[i] = lowstate->msg_.motor_state()[data.joint_ids_map[i]].dq();
        }
    }

    LowStatePtr lowstate;
    SportModeStatePtr sportmodestate;
    MotionCaptureStatePtr motioncapturestate;
};

}