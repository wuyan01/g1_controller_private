#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
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
        sub_msg_.pose.pose.orientation.w = 1.0;
        pub_msg_.pose.pose.orientation.w = 1.0;
    }

    nav_msgs::Odometry sub_msg_;
    nav_msgs::Odometry pub_msg_;
    std::mutex mutex_;

private:
    void fastlioCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sub_msg_ = *msg;

        geometry_msgs::TransformStamped odom_T_camera_init;
        odom_T_camera_init.header.stamp = msg->header.stamp;
        odom_T_camera_init.header.frame_id = "odom";
        odom_T_camera_init.child_frame_id = msg->header.frame_id;

        // identity transform
        odom_T_camera_init.transform.translation.x = 0.0;
        odom_T_camera_init.transform.translation.y = 0.0;
        odom_T_camera_init.transform.translation.z = 0.0;
        odom_T_camera_init.transform.rotation.x = 0.0;
        odom_T_camera_init.transform.rotation.y = 0.0;
        odom_T_camera_init.transform.rotation.z = 0.0;
        odom_T_camera_init.transform.rotation.w = 1.0;

        br_.sendTransform(odom_T_camera_init);


        geometry_msgs::TransformStamped body_T_mid360_link_inv;
        body_T_mid360_link_inv.header.stamp = msg->header.stamp;
        body_T_mid360_link_inv.header.frame_id = "body";
        body_T_mid360_link_inv.child_frame_id = "mid360_link_inv";

        // identity transform
        body_T_mid360_link_inv.transform.translation.x = 0.0;
        body_T_mid360_link_inv.transform.translation.y = 0.0;
        body_T_mid360_link_inv.transform.translation.z = 0.0;
        body_T_mid360_link_inv.transform.rotation.x = 1.0;
        body_T_mid360_link_inv.transform.rotation.y = 0.0;
        body_T_mid360_link_inv.transform.rotation.z = 0.0;
        body_T_mid360_link_inv.transform.rotation.w = 0.0;

        br_.sendTransform(body_T_mid360_link_inv);


        const std::string parent_frame = "odom";
        const std::string child_frame  = "mid360_link_inv";
        ros::Time t = ros::Time(0);
        geometry_msgs::TransformStamped odom_T_mid360_link_inv;
        try
        {
            if (!tf_buffer_.canTransform(parent_frame, child_frame, t, ros::Duration(0.1)))
            {
                ROS_WARN_THROTTLE(1.0, "Waiting for TF %s -> %s", parent_frame.c_str(), child_frame.c_str());
                return;
            }

            odom_T_mid360_link_inv = tf_buffer_.lookupTransform(parent_frame, child_frame, t);
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN_THROTTLE(1.0, "TF lookup failed (%s -> %s): %s",
                              parent_frame.c_str(), child_frame.c_str(), ex.what());
            return;
        }
        tf2::Transform T_odom_mid360_link_inv;
        tf2::fromMsg(odom_T_mid360_link_inv.transform, T_odom_mid360_link_inv);
        pub_msg_.pose.pose = transformToPoseMsg(T_odom_mid360_link_inv);

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
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};
