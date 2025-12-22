// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "unitree_articulation.h"
#include "FSM/FSMState.h"


namespace unitree
{

template <typename LowStatePtr, typename BaseOdomPtr>
class G1Articulation : public BaseArticulation<LowStatePtr>
{
public:
    using Base = BaseArticulation<LowStatePtr>;

    G1Articulation(LowStatePtr lowstate_, BaseOdomPtr baseodom_)
    : Base(lowstate_), baseodom(baseodom_)
    {
    }

    void update() override
    {
        Base::update();

        Eigen::Vector3f root_lin_vel_b(
            baseodom->msg_.twist.twist.linear.x,
            baseodom->msg_.twist.twist.linear.y,
            baseodom->msg_.twist.twist.linear.z
        );
        // base_linear_velocity
        for(int i(0); i<3; i++) {
            Base::data.root_lin_vel_b[i] = root_lin_vel_b[i];
        }
    }

    BaseOdomPtr baseodom;
};

}