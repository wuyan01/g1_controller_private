#pragma once

#include <ros/ros.h>
#include <g1_controller/LowState.h>
#include <g1_controller/LowCmd.h>
#include <mutex>


class LowCmdROS1
{
public:
    using SharedPtr = std::shared_ptr<LowCmdROS1>;

    LowCmdROS1() {}

    LowCmdROS1(ros::NodeHandle& nh, const std::string& lowcmd_topic = "rt/lowcmd", uint32_t queue_size = 10)
    {
        lowcmd_pub_ = nh.advertise<g1_controller::LowCmd>(lowcmd_topic, queue_size);
    }

    void publish()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lowcmd_pub_.publish(msg_);
    }

    g1_controller::LowCmd msg_;
    std::mutex mutex_;

private:

    ros::Publisher lowcmd_pub_;
};


class LowStateROS1
{
public:
    using SharedPtr = std::shared_ptr<LowStateROS1>;

    LowStateROS1() {}

    LowStateROS1(ros::NodeHandle& nh, const std::string& lowstate_topic = "rt/lowstate", uint32_t queue_size = 10)
    {
        lowstate_pub_ = nh.advertise<g1_controller::LowState>(lowstate_topic, queue_size);
    }

    void publish()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lowstate_pub_.publish(msg_);
    }

    g1_controller::LowState msg_;
    std::mutex mutex_;

private:

    ros::Publisher lowstate_pub_;
};
