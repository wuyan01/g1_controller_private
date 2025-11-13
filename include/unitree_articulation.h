// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "isaaclab/assets/articulation/articulation.h"

namespace unitree
{

template <typename LowStatePtr, typename SportModeStatePtr>
class BaseArticulation : public isaaclab::Articulation
{
public:
    BaseArticulation(LowStatePtr lowstate_, SportModeStatePtr sportmodestate_)
    : lowstate(lowstate_), sportmodestate(sportmodestate_)
    {
        data.joystick = &lowstate->joystick;
    }

    void update() override
    {
        Eigen::Quaternionf quat(
            lowstate->msg_.imu_state().quaternion()[0],
            lowstate->msg_.imu_state().quaternion()[1],
            lowstate->msg_.imu_state().quaternion()[2],
            lowstate->msg_.imu_state().quaternion()[3]
        );
        Eigen::Vector3f root_lin_vel_w(
            sportmodestate->msg_.velocity()[0],
            sportmodestate->msg_.velocity()[1],
            sportmodestate->msg_.velocity()[2]
        );
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
};

}