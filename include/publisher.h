#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
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


class JointStateROS1
{
public:
    using SharedPtr = std::shared_ptr<JointStateROS1>;

    JointStateROS1() {}

    JointStateROS1(ros::NodeHandle& nh, const std::string& jointstate_topic = "rt/joint_states", uint32_t queue_size = 10)
    {
        jointstate_pub_ = nh.advertise<sensor_msgs::JointState>(jointstate_topic, queue_size);
    }

    void publish()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        jointstate_pub_.publish(msg_);
    }

    sensor_msgs::JointState msg_;
    std::mutex mutex_;

private:

    ros::Publisher jointstate_pub_;
};
