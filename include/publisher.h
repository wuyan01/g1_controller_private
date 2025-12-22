#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
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


class BaseOdomState
{
public:
    using SharedPtr = std::shared_ptr<BaseOdomState>;

    BaseOdomState() : tf_listener_(tf_buffer_) {}

    BaseOdomState(ros::NodeHandle& nh, const std::string& odom_topic = "rt/base_odom", uint32_t queue_size = 10)
        : tf_listener_(tf_buffer_)
    {
        odom_pub_ = nh.advertise<nav_msgs::Odometry>(odom_topic, queue_size);
    }

    void publish()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        odom_pub_.publish(msg_);
        publishOdomToPelvisTF();
    }

    nav_msgs::Odometry msg_;
    geometry_msgs::TransformStamped tf;
    std::mutex mutex_;

private:

    ros::Publisher odom_pub_;
    tf2_ros::TransformBroadcaster br_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void publishOdomToPelvisTF()
    {

        if (!tf_buffer_.canTransform("pelvis", "torso_link", ros::Time(0), ros::Duration(0.1)))
        {
            ROS_WARN_THROTTLE(1.0,
                "Waiting for TF pelvis -> torso_link");
            return;
        }

        geometry_msgs::TransformStamped pelvis_T_torso;

        pelvis_T_torso = tf_buffer_.lookupTransform("pelvis", "torso_link", ros::Time(0));

        tf2::Transform T_odom_torso;
        tf2::fromMsg(msg_.pose.pose, T_odom_torso);
        tf2::Transform T_pelvis_torso;
        tf2::fromMsg(pelvis_T_torso.transform, T_pelvis_torso);
        tf2::Transform T_torso_pelvis = T_pelvis_torso.inverse();
        tf2::Transform T_odom_pelvis = T_odom_torso * T_torso_pelvis;

        geometry_msgs::TransformStamped tf;
        tf.header.stamp = msg_.header.stamp;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "pelvis";
        tf.transform = tf2::toMsg(T_odom_pelvis);

        br_.sendTransform(tf);
    }

};
