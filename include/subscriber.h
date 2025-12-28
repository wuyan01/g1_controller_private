#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <mutex>


class MotionCaptureState
{
public:
    using SharedPtr = std::shared_ptr<MotionCaptureState>;

    MotionCaptureState() {}

    MotionCaptureState(ros::NodeHandle& nh, const std::string& mocap_topic = "/qualisys/base/odom")
    {
        mocap_sub_ = nh.subscribe(mocap_topic, 10, &MotionCaptureState::mocapCallback, this);
        msg_.pose.pose.orientation.w = 1.0;
    }

    nav_msgs::Odometry msg_;
    std::mutex mutex_;

private:
    void mocapCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        msg_ = *msg;
    }

    ros::Subscriber mocap_sub_;
};


class FastLioState
{
public:
    using SharedPtr = std::shared_ptr<FastLioState>;

    FastLioState() : tf_listener_(tf_buffer_) {}

    FastLioState(ros::NodeHandle& nh, const std::string& fastlio_topic = "/Odometry_loc")
        : tf_listener_(tf_buffer_)
    {
        fastlio_sub_ = nh.subscribe(fastlio_topic, 10, &FastLioState::fastlioCallback, this);
        msg_.pose.pose.orientation.x = 1.0;
        publishMid360LinkInv();
    }

    nav_msgs::Odometry msg_;
    std::mutex mutex_;

private:
    void fastlioCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        const std::string parent_frame = "torso_link";
        const std::string child_frame  = "mid360_link_inv";
        geometry_msgs::TransformStamped torso_link_T_mid360_link_inv;
        try
        {
            if (!tf_buffer_.canTransform(parent_frame, child_frame, msg->header.stamp, ros::Duration(0.1)))
            {
                ROS_WARN_THROTTLE(1.0, "Waiting for TF %s -> %s", parent_frame.c_str(), child_frame.c_str());
                return;
            }

            torso_link_T_mid360_link_inv = tf_buffer_.lookupTransform(parent_frame, child_frame, msg->header.stamp);
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN_THROTTLE(1.0, "TF lookup failed (%s -> %s): %s",
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
        br_.sendTransform(torso_link_T_mid360_link_inv);
    }

    void publishMid360LinkInv()
    {
        geometry_msgs::TransformStamped mid360_link_T_mid360_link_inv;
        mid360_link_T_mid360_link_inv.header.stamp = ros::Time::now();
        mid360_link_T_mid360_link_inv.header.frame_id = "mid360_link";
        mid360_link_T_mid360_link_inv.child_frame_id = "mid360_link_inv";

        mid360_link_T_mid360_link_inv.transform.translation.x = 0.0;
        mid360_link_T_mid360_link_inv.transform.translation.y = 0.0;
        mid360_link_T_mid360_link_inv.transform.translation.z = 0.0;
        mid360_link_T_mid360_link_inv.transform.rotation.x = 1.0;
        mid360_link_T_mid360_link_inv.transform.rotation.y = 0.0;
        mid360_link_T_mid360_link_inv.transform.rotation.z = 0.0;
        mid360_link_T_mid360_link_inv.transform.rotation.w = 0.0;

        static_br_.sendTransform(mid360_link_T_mid360_link_inv);
    }

    inline geometry_msgs::Pose transformToPoseMsg(const tf2::Transform& T)
    {
        geometry_msgs::Pose p;
        p.position.x = T.getOrigin().x();
        p.position.y = T.getOrigin().y();
        p.position.z = T.getOrigin().z();
        p.orientation = tf2::toMsg(T.getRotation());
        return p;
    }

    ros::Subscriber fastlio_sub_;
    tf2_ros::TransformBroadcaster br_;
    tf2_ros::StaticTransformBroadcaster static_br_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};
