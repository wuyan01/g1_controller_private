#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mutex>
#include <string>


class JointState
{
public:
    using SharedPtr = std::shared_ptr<JointState>;

    JointState() {}

    JointState(rclcpp::Node::SharedPtr node, const std::string& jointstate_topic = "joint_states", uint32_t queue_size = 10)
    {
        clock_ = node->get_clock();
        jointstate_pub_ = node->create_publisher<sensor_msgs::msg::JointState>(jointstate_topic, queue_size);
    }

    void publish()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        msg_.header.stamp = clock_->now();
        jointstate_pub_->publish(msg_);
    }

    sensor_msgs::msg::JointState msg_;
    std::mutex mutex_;

private:
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointstate_pub_;
};


class Imu
{
public:
    using SharedPtr = std::shared_ptr<Imu>;

    Imu() {}

    Imu(rclcpp::Node::SharedPtr node, const std::string& imu_topic = "imu", uint32_t queue_size = 10)
    {
        clock_ = node->get_clock();
        imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>(imu_topic, queue_size);
    }

    void publish()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        msg_.header.stamp = clock_->now();
        imu_pub_->publish(msg_);
    }

    sensor_msgs::msg::Imu msg_;
    std::mutex mutex_;

private:
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};


class VelocityMarkers
{
public:
    using SharedPtr = std::shared_ptr<VelocityMarkers>;

    VelocityMarkers() {}

    VelocityMarkers(rclcpp::Node::SharedPtr node, const std::string& velocitymarkers_topic = "velocity_markers", uint32_t queue_size = 10)
    {
        clock_ = node->get_clock();
        marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(velocitymarkers_topic, queue_size);
    }

    void publish(const std::string& frame_id = "torso_link", double lifetime_seconds = 0.0)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        marker_array_.markers.clear();
        
        cmd_marker_.header.stamp = rclcpp::Time(0, 0, clock_->get_clock_type());
        cmd_marker_.header.frame_id = frame_id;
        cmd_marker_.lifetime = rclcpp::Duration::from_seconds(lifetime_seconds);
        cmd_marker_.ns = "cmd_vel";
        cmd_marker_.id = 0;
        cmd_marker_.type = visualization_msgs::msg::Marker::ARROW;
        cmd_marker_.action = visualization_msgs::msg::Marker::ADD;

        cmd_marker_.pose.orientation.x = 0.0;
        cmd_marker_.pose.orientation.y = 0.0;
        cmd_marker_.pose.orientation.z = 0.0;
        cmd_marker_.pose.orientation.w = 1.0;

        cmd_marker_.scale.x = 0.1;
        cmd_marker_.scale.y = 0.2;
        cmd_marker_.scale.z = 0.2;

        cmd_marker_.color.r = 0.0f;
        cmd_marker_.color.g = 1.0f;
        cmd_marker_.color.b = 0.0f;
        cmd_marker_.color.a = 0.9f;


        real_marker_.header.stamp = rclcpp::Time(0, 0, clock_->get_clock_type());
        real_marker_.header.frame_id = frame_id;
        real_marker_.lifetime = rclcpp::Duration::from_seconds(lifetime_seconds);
        real_marker_.ns = "real_vel";
        real_marker_.id = 1;
        real_marker_.type = visualization_msgs::msg::Marker::ARROW;
        real_marker_.action = visualization_msgs::msg::Marker::ADD;

        real_marker_.pose.orientation.x = 0.0;
        real_marker_.pose.orientation.y = 0.0;
        real_marker_.pose.orientation.z = 0.0;
        real_marker_.pose.orientation.w = 1.0;

        real_marker_.scale.x = 0.1;
        real_marker_.scale.y = 0.2;
        real_marker_.scale.z = 0.2;

        real_marker_.color.r = 0.0f;
        real_marker_.color.g = 0.0f;
        real_marker_.color.b = 1.0f;
        real_marker_.color.a = 0.9f;

        marker_array_.markers.push_back(cmd_marker_);
        marker_array_.markers.push_back(real_marker_);
        marker_pub_->publish(marker_array_);
    }

    visualization_msgs::msg::MarkerArray marker_array_;
    visualization_msgs::msg::Marker cmd_marker_;
    visualization_msgs::msg::Marker real_marker_;
    std::mutex mutex_;

private:
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};


class PoseSourceOdom
{
public:
    using SharedPtr = std::shared_ptr<PoseSourceOdom>;

    PoseSourceOdom() {}

    PoseSourceOdom(rclcpp::Node::SharedPtr node, const std::string& pose_source_odom_topic = "pose_source/odom", uint32_t queue_size = 10)
    {
        clock_ = node->get_clock();
        pose_source_odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>(pose_source_odom_topic, queue_size);
    }

    void publish()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        msg_.header.stamp = clock_->now();
        pose_source_odom_pub_->publish(msg_);
    }

    nav_msgs::msg::Odometry msg_;
    std::mutex mutex_;

private:
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_source_odom_pub_;
};

class OdometryFusion
{
public:
    using SharedPtr = std::shared_ptr<OdometryFusion>;

    OdometryFusion(rclcpp::Node::SharedPtr node,
                         const std::string& input_odom_topic  = "pose_source/odom",
                         const std::string& output_odom_frame = "pelvis",
                         uint32_t queue_size = 10)
        : output_odom_frame_(output_odom_frame), last_tf_stamp_(0, 0, RCL_ROS_TIME), node_(node)
    {
        odom_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = odom_cb_group_;

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        br_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);

        odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
            input_odom_topic,
            queue_size,
            std::bind(&OdometryFusion::odomCallback, this, std::placeholders::_1),
            sub_options);

        target_odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>(output_odom_frame + "_odom", queue_size);
        last_valid_ = false;
        last_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    nav_msgs::msg::Odometry msg_;
    std::mutex mutex_;

private:
    const std::string output_odom_frame_;
    rclcpp::CallbackGroup::SharedPtr odom_cb_group_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr target_odom_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Node::SharedPtr node_;

    bool last_valid_;
    rclcpp::Time last_stamp_;
    rclcpp::Time last_tf_stamp_;
    tf2::Transform last_T_odom_target_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::string parent_frame = msg->header.frame_id;
        if (parent_frame != "odom")
        {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                "Input odometry frame_id is not 'odom' (got '%s')",
                parent_frame.c_str());
            return;
        }

        const std::string& source_link = msg->child_frame_id;
        if (source_link.empty())
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Incoming odometry has empty child_frame_id");
            return;
        }
        tf2::Transform T_odom_target;
        if (source_link == output_odom_frame_)
        {
            msg_ = *msg;
            tf2::Transform T_odom_source;
            tf2::fromMsg(msg->pose.pose, T_odom_source);
            if (!msg->twist.twist.linear.x &&
                !msg->twist.twist.linear.y &&
                !msg->twist.twist.linear.z &&
                !msg->twist.twist.angular.x &&
                !msg->twist.twist.angular.y &&
                !msg->twist.twist.angular.z)
            {
                computeTwistFromPoseDifference(msg, T_odom_source, msg_.twist.twist);
            }
            target_odom_pub_->publish(msg_);
            T_odom_target = T_odom_source;
        }
        else
        {
            if (!tf_buffer_) return;

            // Wait for target <- source_link TF
            if (!tf_buffer_->canTransform(output_odom_frame_,
                                        source_link,
                                        msg->header.stamp,
                                        rclcpp::Duration::from_seconds(0.1)))
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                    "Waiting for TF %s -> %s",
                    output_odom_frame_.c_str(),
                    source_link.c_str());
                return;
            }
            geometry_msgs::msg::TransformStamped target_T_source;
            try
            {
                target_T_source =
                    tf_buffer_->lookupTransform(output_odom_frame_,
                                            source_link,
                                            msg->header.stamp);
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                    "TF lookup failed (%s -> %s): %s",
                    output_odom_frame_.c_str(),
                    source_link.c_str(),
                    ex.what());
                return;
            }
            // Convert odom -> source to tf2
            tf2::Transform T_odom_source;
            tf2::fromMsg(msg->pose.pose, T_odom_source);

            tf2::Transform T_target_source;
            tf2::fromMsg(target_T_source.transform, T_target_source);

            tf2::Transform T_source_target = T_target_source.inverse();
            T_odom_target = T_odom_source * T_source_target;

            // ----------------------------
            // Publish target odometry
            // ----------------------------
            msg_.header.stamp = msg->header.stamp;
            msg_.header.frame_id = "odom";
            msg_.child_frame_id  = output_odom_frame_;

            msg_.pose.pose = transformToPoseMsg(T_odom_target);

            if (!msg->twist.twist.linear.x &&
                !msg->twist.twist.linear.y &&
                !msg->twist.twist.linear.z &&
                !msg->twist.twist.angular.x &&
                !msg->twist.twist.angular.y &&
                !msg->twist.twist.angular.z)
            {
                computeTwistFromPoseDifference(msg, T_odom_target, msg_.twist.twist);
            }
            else
            {
                transformTwistChildToChild(msg, T_odom_source, T_odom_target, msg_.twist.twist);
            }
            target_odom_pub_->publish(msg_);
        }

        // ----------------------------
        // Publish TF: odom -> target
        // ----------------------------
        if (output_odom_frame_ == "pelvis" && rclcpp::Time(msg_.header.stamp) > last_tf_stamp_)
        {
            geometry_msgs::msg::TransformStamped tf;
            tf.header = msg_.header;
            tf.child_frame_id = output_odom_frame_;
            tf.transform = tf2::toMsg(T_odom_target);

            if (br_) br_->sendTransform(tf);
        }
        last_tf_stamp_ = msg_.header.stamp;
    }

    inline Eigen::Matrix3d quatToEigen(const tf2::Quaternion& q)
    {
        tf2::Matrix3x3 R_tf(q);
        Eigen::Matrix3d R;
        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                R(i,j) = R_tf[i][j];
        return R;
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

    void computeTwistFromPoseDifference(
        const nav_msgs::msg::Odometry::SharedPtr msg,
        const tf2::Transform& T_odom_target,
        geometry_msgs::msg::Twist& twist_out)
    {
        if (!last_valid_)
        {
            last_T_odom_target_ = T_odom_target;
            last_stamp_ = msg->header.stamp;
            last_valid_ = true;

            twist_out = geometry_msgs::msg::Twist();
            return;
        }

        const double dt = (rclcpp::Time(msg->header.stamp) - last_stamp_).seconds();
        if (dt <= 0.0) {
            twist_out = geometry_msgs::msg::Twist();
            return; // don't update on out-of-order
        }
        if (dt <= 1e-4) {
            twist_out = geometry_msgs::msg::Twist();
            last_T_odom_target_ = T_odom_target;
            last_stamp_ = msg->header.stamp;
            return;
        }
        if (dt > 1e-4) {
            // Linear velocity in odom frame
            tf2::Vector3 p_now  = T_odom_target.getOrigin();
            tf2::Vector3 p_prev = last_T_odom_target_.getOrigin();
            tf2::Vector3 v_odom = (p_now - p_prev) / dt;

            // Angular velocity in odom frame
            tf2::Quaternion q_now  = T_odom_target.getRotation();
            tf2::Quaternion q_prev = last_T_odom_target_.getRotation();

            tf2::Quaternion q_rel = q_prev.inverse() * q_now;
            q_rel.normalize();

            double angle = q_rel.getAngle();
            tf2::Vector3 w_odom(0.0, 0.0 ,0.0);
            if (angle > 1e-9)
            {
                if (angle > M_PI) angle -= 2*M_PI;
                tf2::Vector3 axis = q_rel.getAxis();
                w_odom = axis * (angle / dt);
            }

            // Convert twist from odom frame -> child frame
            tf2::Matrix3x3 R_odom_target(q_now);
            tf2::Matrix3x3 R_target_odom = R_odom_target.transpose();

            tf2::Vector3 v_child = R_target_odom * v_odom;
            tf2::Vector3 w_child = R_target_odom * w_odom;

            twist_out.linear.x = v_child.x();
            twist_out.linear.y = v_child.y();
            twist_out.linear.z = v_child.z();

            twist_out.angular.x = w_child.x();
            twist_out.angular.y = w_child.y();
            twist_out.angular.z = w_child.z();

            // Update last state
            last_T_odom_target_ = T_odom_target;
            last_stamp_ = msg->header.stamp;
        }
    }

    void transformTwistChildToChild(
        const nav_msgs::msg::Odometry::SharedPtr msg,
        const tf2::Transform& T_odom_source,
        const tf2::Transform& T_odom_target,
        geometry_msgs::msg::Twist& twist_out)
    {
        // source twist in source frame
        Eigen::Vector3d v_s(msg->twist.twist.linear.x,
                            msg->twist.twist.linear.y,
                            msg->twist.twist.linear.z);

        Eigen::Vector3d w_s(msg->twist.twist.angular.x,
                            msg->twist.twist.angular.y,
                            msg->twist.twist.angular.z);

        // rotation matrices
        Eigen::Matrix3d R_os = quatToEigen(T_odom_source.getRotation());
        Eigen::Matrix3d R_op = quatToEigen(T_odom_target.getRotation());
        Eigen::Matrix3d R_po = R_op.transpose();

        // twist expressed in odom frame
        Eigen::Vector3d v_o_s = R_os * v_s;
        Eigen::Vector3d w_o_s = R_os * w_s;

        // lever arm in odom frame
        Eigen::Vector3d p_o_s(T_odom_source.getOrigin().x(),
                            T_odom_source.getOrigin().y(),
                            T_odom_source.getOrigin().z());

        Eigen::Vector3d p_o_p(T_odom_target.getOrigin().x(),
                            T_odom_target.getOrigin().y(),
                            T_odom_target.getOrigin().z());

        Eigen::Vector3d r = p_o_p - p_o_s;  // vector from source origin -> target origin in odom frame

        // rigid-body velocity shift
        Eigen::Vector3d v_o_p = v_o_s + w_o_s.cross(r);
        Eigen::Vector3d w_o_p = w_o_s;

        // express in target frame (child convention)
        Eigen::Vector3d v_p = R_po * v_o_p;
        Eigen::Vector3d w_p = R_po * w_o_p;

        twist_out.linear.x = v_p.x();
        twist_out.linear.y = v_p.y();
        twist_out.linear.z = v_p.z();

        twist_out.angular.x = w_p.x();
        twist_out.angular.y = w_p.y();
        twist_out.angular.z = w_p.z();
    }
};
