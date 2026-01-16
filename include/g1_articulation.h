// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "unitree_articulation.h"
#include "FSM/FSMState.h"


namespace unitree
{

template <typename LowStatePtr, typename OdometryFusionPtr, typename HeightMapHandlerPtr>
class G1Articulation : public BaseArticulation<LowStatePtr>
{
public:
    using Base = BaseArticulation<LowStatePtr>;

    G1Articulation(LowStatePtr lowstate_, OdometryFusionPtr torsoodometry_, HeightMapHandlerPtr heightmaphandler_)
    : Base(lowstate_), torsoodometryfusion(torsoodometry_), heightmaphandler(heightmaphandler_)
    {
        Base::data.height_map.resize(heightmaphandler->numPoints_);
    }

    void update() override
    {
        Base::update();

        Eigen::Vector3f root_lin_vel_b(
            torsoodometryfusion->msg_.twist.twist.linear.x,
            torsoodometryfusion->msg_.twist.twist.linear.y,
            torsoodometryfusion->msg_.twist.twist.linear.z
        );
        // base_linear_velocity
        for(int i(0); i<3; i++) {
            Base::data.root_lin_vel_b[i] = root_lin_vel_b[i];
        }

        Eigen::Quaterniond quat(
            torsoodometryfusion->msg_.pose.pose.orientation.w,
            torsoodometryfusion->msg_.pose.pose.orientation.x,
            torsoodometryfusion->msg_.pose.pose.orientation.y,
            torsoodometryfusion->msg_.pose.pose.orientation.z
        );
        heightmaphandler->basePosition_ <<
            torsoodometryfusion->msg_.pose.pose.position.x,
            torsoodometryfusion->msg_.pose.pose.position.y,
            torsoodometryfusion->msg_.pose.pose.position.z;
        Eigen::VectorXd MeasurementHeights = heightmaphandler->getMeasurementHeights(quat);
        for(int i(0); i<heightmaphandler->numPoints_; i++) {
            Base::data.height_map[i] = static_cast<float>(MeasurementHeights[i]);
        }
    }

    OdometryFusionPtr torsoodometryfusion;
    HeightMapHandlerPtr heightmaphandler;
};

}