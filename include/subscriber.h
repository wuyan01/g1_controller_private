#pragma once

#include "unitree/dds_wrapper/common/Subscription.h"
#include <unitree/idl/go2/HeightMap_.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <mutex>
#include <string>


class HeightMap : public unitree::robot::SubscriptionBase<unitree_go::msg::dds_::HeightMap_>
{
public:
    using SharedPtr = std::shared_ptr<HeightMap>;

    HeightMap(std::string topic = "rt/heightmap") : unitree::robot::SubscriptionBase<MsgType>(topic) {}

};


class MotionCaptureState
{
public:
    using SharedPtr = std::shared_ptr<MotionCaptureState>;

    MotionCaptureState() {}

    MotionCaptureState(rclcpp::Node::SharedPtr node, const std::string& mocap_topic = "/qualisys/base/odom")
    {
        mocap_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
            mocap_topic, 10, std::bind(&MotionCaptureState::mocapCallback, this, std::placeholders::_1));
        msg_.pose.pose.orientation.w = 1.0;
    }

    nav_msgs::msg::Odometry msg_;
    std::mutex mutex_;

private:
    void mocapCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        msg_ = *msg;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mocap_sub_;
};


class FastLioState
{
public:
    using SharedPtr = std::shared_ptr<FastLioState>;

    FastLioState(rclcpp::Node::SharedPtr node, const std::string& fastlio_topic = "/Odometry_loc")
        : node_(node)
    {
        fastlio_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = fastlio_cb_group_;

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        br_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
        
        fastlio_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
            fastlio_topic, 10, std::bind(&FastLioState::fastlioCallback, this, std::placeholders::_1), sub_options);
        msg_.pose.pose.orientation.x = 1.0; 
    }

    nav_msgs::msg::Odometry msg_;
    std::mutex mutex_;

private:
    void fastlioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        const std::string parent_frame = "torso_link";
        const std::string child_frame  = "mid360_link_inv";
        geometry_msgs::msg::TransformStamped torso_link_T_mid360_link_inv;
        
        if (!tf_buffer_) return;

        try
        {
            if (!tf_buffer_->canTransform(parent_frame, child_frame, msg->header.stamp, rclcpp::Duration::from_seconds(0.1)))
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Waiting for TF %s -> %s", parent_frame.c_str(), child_frame.c_str());
                return;
            }

            torso_link_T_mid360_link_inv = tf_buffer_->lookupTransform(parent_frame, child_frame, msg->header.stamp);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "TF lookup failed (%s -> %s): %s",
                              parent_frame.c_str(), child_frame.c_str(), ex.what());
            return;
        }
        tf2::Transform T_torso_link_mid360_link_inv;
        tf2::fromMsg(torso_link_T_mid360_link_inv.transform, T_torso_link_mid360_link_inv);

        tf2::Transform T_camera_init_body;
        tf2::fromMsg(msg->pose.pose, T_camera_init_body);
        tf2::Transform T_torso_link_body = T_torso_link_mid360_link_inv * T_camera_init_body;

        msg_.header.stamp = msg->header.stamp;
        msg_.pose.pose = transformToPoseMsg(T_torso_link_body);

        torso_link_T_mid360_link_inv.header.stamp = msg->header.stamp;
        torso_link_T_mid360_link_inv.header.frame_id = "odom";
        torso_link_T_mid360_link_inv.child_frame_id = "camera_init";
        
        if (br_) br_->sendTransform(torso_link_T_mid360_link_inv);
    }

    inline geometry_msgs::msg::Pose transformToPoseMsg(const tf2::Transform& T)
    {
        geometry_msgs::msg::Pose p;
        p.position.x = T.getOrigin().x();
        p.position.y = T.getOrigin().y();
        p.position.z = T.getOrigin().z();
        p.orientation = tf2::toMsg(T.getRotation());
        return p;
    }

    rclcpp::CallbackGroup::SharedPtr fastlio_cb_group_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fastlio_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Node::SharedPtr node_;
};
