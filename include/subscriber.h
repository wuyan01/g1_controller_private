#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mutex>


class MotionCaptureState
{
public:
    using SharedPtr = std::shared_ptr<MotionCaptureState>;

    MotionCaptureState() {}

    MotionCaptureState(ros::NodeHandle& nh, const std::string& mocap_topic = "/qualisys/base/odom")
    {
        mocap_sub_ = nh.subscribe(mocap_topic, 10, &MotionCaptureState::mocapCallback, this);
    }

    nav_msgs::Odometry mocap_msg_;
    std::mutex mutex_;

private:
    void mocapCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        mocap_msg_ = *msg;
    }

    ros::Subscriber mocap_sub_;
};
