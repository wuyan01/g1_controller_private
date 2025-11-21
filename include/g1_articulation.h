// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "unitree_articulation.h"
#include "FSM/FSMState.h"


namespace unitree
{

template <typename LowStatePtr, typename SportModeStatePtr, typename MotionCaptureStatePtr>
class G1Articulation : public BaseArticulation<LowStatePtr>
{
public:
    using Base = BaseArticulation<LowStatePtr>;

    G1Articulation(LowStatePtr lowstate_, SportModeStatePtr sportmodestate_, MotionCaptureStatePtr motioncapturestate_)
    : Base(lowstate_), sportmodestate(sportmodestate_), motioncapturestate(motioncapturestate_)
    {
    }

    void update() override
    {
        std::scoped_lock lock(sportmodestate->mutex_,
                              motioncapturestate->mutex_);
        Base::update();

        Eigen::Vector3f root_lin_vel_w;
        Eigen::Vector3f root_lin_vel_b;
        if(FSMState::network_interface == "lo") {
            root_lin_vel_w = Eigen::Vector3f(
                sportmodestate->msg_.velocity()[0],
                sportmodestate->msg_.velocity()[1],
                sportmodestate->msg_.velocity()[2]
            );
            root_lin_vel_b = Base::data.root_quat_w.conjugate() * root_lin_vel_w;
        }
        else {
            root_lin_vel_b = Eigen::Vector3f(
                motioncapturestate->mocap_msg_.twist.twist.linear.x,
                motioncapturestate->mocap_msg_.twist.twist.linear.y,
                motioncapturestate->mocap_msg_.twist.twist.linear.z
            );
        }
        // base_linear_velocity
        for(int i(0); i<3; i++) {
            Base::data.root_lin_vel_b[i] = root_lin_vel_b[i];
        }
    }

    SportModeStatePtr sportmodestate;
    MotionCaptureStatePtr motioncapturestate;
};

}