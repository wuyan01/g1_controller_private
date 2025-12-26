#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
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

    LowCmdROS1(ros::NodeHandle& nh, const std::string& lowcmd_topic = "lowcmd", uint32_t queue_size = 10)
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

    LowStateROS1(ros::NodeHandle& nh, const std::string& lowstate_topic = "lowstate", uint32_t queue_size = 10)
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


class JointState
{
public:
    using SharedPtr = std::shared_ptr<JointState>;

    JointState() {}

    JointState(ros::NodeHandle& nh, const std::string& jointstate_topic = "joint_states", uint32_t queue_size = 10)
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


class VelocityMarkers
{
public:
    using SharedPtr = std::shared_ptr<VelocityMarkers>;

    VelocityMarkers() {}

    VelocityMarkers(ros::NodeHandle& nh, const std::string& velocitymarkers_topic = "velocity_markers", uint32_t queue_size = 10)
    {
        marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(velocitymarkers_topic, queue_size);
    }

    void publish()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        marker_array_.markers.clear();
        
        cmd_marker_.header.stamp = ros::Time::now();
        cmd_marker_.header.frame_id = "torso_link";
        cmd_marker_.ns = "cmd_vel";
        cmd_marker_.id = 0;
        cmd_marker_.type = visualization_msgs::Marker::ARROW;
        cmd_marker_.action = visualization_msgs::Marker::ADD;

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


        real_marker_.header.stamp = ros::Time::now();
        real_marker_.header.frame_id = "torso_link";
        real_marker_.ns = "real_vel";
        real_marker_.id = 1;
        real_marker_.type = visualization_msgs::Marker::ARROW;
        real_marker_.action = visualization_msgs::Marker::ADD;

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
        marker_pub_.publish(marker_array_);
    }

    visualization_msgs::MarkerArray marker_array_;
    visualization_msgs::Marker cmd_marker_;
    visualization_msgs::Marker real_marker_;
    std::mutex mutex_;

private:

    ros::Publisher marker_pub_;
};


class PoseSourceOdom
{
public:
    using SharedPtr = std::shared_ptr<PoseSourceOdom>;

    PoseSourceOdom() {}

    PoseSourceOdom(ros::NodeHandle& nh, const std::string& pose_source_odom_topic = "pose_source/odom", uint32_t queue_size = 10)
    {
        pose_source_odom_pub_ = nh.advertise<nav_msgs::Odometry>(pose_source_odom_topic, queue_size);
    }

    void publish()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pose_source_odom_pub_.publish(msg_);
    }

    nav_msgs::Odometry msg_;
    std::mutex mutex_;

private:

    ros::Publisher pose_source_odom_pub_;
};

class PelvisOdometryFusion
{
public:
    using SharedPtr = std::shared_ptr<PelvisOdometryFusion>;

    PelvisOdometryFusion() : tf_listener_(tf_buffer_) {}
    
    PelvisOdometryFusion(ros::NodeHandle& nh,
                         const std::string& input_odom_topic  = "pose_source/odom",
                         const std::string& output_odom_topic = "pelvis/odom",
                         uint32_t queue_size = 10)
        : tf_listener_(tf_buffer_)
    {
        odom_sub_ = nh.subscribe(input_odom_topic,
                                 queue_size,
                                 &PelvisOdometryFusion::odomCallback,
                                 this);

        pelvis_odom_pub_ = nh.advertise<nav_msgs::Odometry>(output_odom_topic,
                                                           queue_size);
    }

private:
    ros::Subscriber odom_sub_;
    ros::Publisher pelvis_odom_pub_;

    tf2_ros::TransformBroadcaster br_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        const std::string& source_link = msg->child_frame_id;

        if (source_link.empty())
        {
            ROS_WARN_THROTTLE(1.0, "Incoming odometry has empty child_frame_id");
            return;
        }

        // Wait for pelvis <- source_link TF
        if (!tf_buffer_.canTransform("pelvis",
                                     source_link,
                                     ros::Time(0),
                                     ros::Duration(0.1)))
        {
            ROS_WARN_THROTTLE(1.0,
                "Waiting for TF pelvis -> %s",
                source_link.c_str());
            return;
        }

        geometry_msgs::TransformStamped pelvis_T_source =
            tf_buffer_.lookupTransform("pelvis",
                                       source_link,
                                       ros::Time(0));

        // Convert odom -> source to tf2
        tf2::Transform T_odom_source;
        tf2::fromMsg(msg->pose.pose, T_odom_source);

        tf2::Transform T_pelvis_source;
        tf2::fromMsg(pelvis_T_source.transform, T_pelvis_source);

        tf2::Transform T_source_pelvis = T_pelvis_source.inverse();
        tf2::Transform T_odom_pelvis = T_odom_source * T_source_pelvis;

        // ----------------------------
        // Publish pelvis odometry
        // ----------------------------
        nav_msgs::Odometry pelvis_odom;
        pelvis_odom.header.stamp = msg->header.stamp;
        pelvis_odom.header.frame_id = msg->header.frame_id;
        pelvis_odom.child_frame_id  = "pelvis";

        pelvis_odom.pose.pose = transformToPoseMsg(T_odom_pelvis);
        transformTwistChildToChild(msg, T_odom_source, T_odom_pelvis, pelvis_odom.twist.twist);
        pelvis_odom.pose.covariance =
            transformPoseCovariance(msg->pose.covariance, T_odom_source, T_odom_pelvis);
        pelvis_odom.twist.covariance =
            transformTwistCovariance(msg->twist.covariance, T_odom_source, T_odom_pelvis);
                pelvis_odom_pub_.publish(pelvis_odom);

        // ----------------------------
        // Publish TF: odom -> pelvis
        // ----------------------------
        geometry_msgs::TransformStamped tf;
        tf.header = pelvis_odom.header;
        tf.child_frame_id = "pelvis";
        tf.transform = tf2::toMsg(T_odom_pelvis);

        br_.sendTransform(tf);
    }

    inline Eigen::Matrix3d skew(const Eigen::Vector3d& v)
    {
        Eigen::Matrix3d S;
        S <<     0, -v.z(),  v.y(),
            v.z(),     0, -v.x(),
            -v.y(),  v.x(),     0;
        return S;
    }

    inline Eigen::Matrix<double,6,6> covMsgToEigen(const boost::array<double,36>& cov)
    {
        Eigen::Matrix<double,6,6> C;
        for(int r=0;r<6;r++)
            for(int c=0;c<6;c++)
                C(r,c) = cov[r*6+c];
        return C;
    }

    inline boost::array<double,36> covEigenToMsg(const Eigen::Matrix<double,6,6>& C)
    {
        boost::array<double,36> cov;
        for(int r=0;r<6;r++)
            for(int c=0;c<6;c++)
                cov[r*6+c] = C(r*6+c);
        return cov;
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

    inline geometry_msgs::Pose transformToPoseMsg(const tf2::Transform& T)
    {
        geometry_msgs::Pose p;
        p.position.x = T.getOrigin().x();
        p.position.y = T.getOrigin().y();
        p.position.z = T.getOrigin().z();
        p.orientation = tf2::toMsg(T.getRotation());
        return p;
    }

    void transformTwistChildToChild(
        const nav_msgs::OdometryConstPtr& msg,
        const tf2::Transform& T_odom_source,
        const tf2::Transform& T_odom_pelvis,
        geometry_msgs::Twist& twist_out)
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
        Eigen::Matrix3d R_op = quatToEigen(T_odom_pelvis.getRotation());
        Eigen::Matrix3d R_po = R_op.transpose();

        // twist expressed in odom frame
        Eigen::Vector3d v_o_s = R_os * v_s;
        Eigen::Vector3d w_o_s = R_os * w_s;

        // lever arm in odom frame
        Eigen::Vector3d p_o_s(T_odom_source.getOrigin().x(),
                            T_odom_source.getOrigin().y(),
                            T_odom_source.getOrigin().z());

        Eigen::Vector3d p_o_p(T_odom_pelvis.getOrigin().x(),
                            T_odom_pelvis.getOrigin().y(),
                            T_odom_pelvis.getOrigin().z());

        Eigen::Vector3d r = p_o_p - p_o_s;  // vector from source origin -> pelvis origin in odom frame

        // rigid-body velocity shift
        Eigen::Vector3d v_o_p = v_o_s + w_o_s.cross(r);
        Eigen::Vector3d w_o_p = w_o_s;

        // express in pelvis frame (child convention)
        Eigen::Vector3d v_p = R_po * v_o_p;
        Eigen::Vector3d w_p = R_po * w_o_p;

        twist_out.linear.x = v_p.x();
        twist_out.linear.y = v_p.y();
        twist_out.linear.z = v_p.z();

        twist_out.angular.x = w_p.x();
        twist_out.angular.y = w_p.y();
        twist_out.angular.z = w_p.z();
    }

    boost::array<double,36> transformPoseCovariance(
        const boost::array<double,36>& cov_in,
        const tf2::Transform& T_odom_source,
        const tf2::Transform& T_odom_pelvis)
    {
        Eigen::Matrix<double,6,6> C_s = covMsgToEigen(cov_in);

        Eigen::Vector3d p_o_s(T_odom_source.getOrigin().x(),
                            T_odom_source.getOrigin().y(),
                            T_odom_source.getOrigin().z());

        Eigen::Vector3d p_o_p(T_odom_pelvis.getOrigin().x(),
                            T_odom_pelvis.getOrigin().y(),
                            T_odom_pelvis.getOrigin().z());

        Eigen::Vector3d r = p_o_p - p_o_s;

        Eigen::Matrix<double,6,6> J = Eigen::Matrix<double,6,6>::Identity();
        J.block<3,3>(0,3) = -skew(r);

        Eigen::Matrix<double,6,6> C_p = J * C_s * J.transpose();

        return covEigenToMsg(C_p);
    }

    boost::array<double,36> transformTwistCovariance(
        const boost::array<double,36>& cov_in,
        const tf2::Transform& T_odom_source,
        const tf2::Transform& T_odom_pelvis)
    {
        Eigen::Matrix<double,6,6> C_s = covMsgToEigen(cov_in);

        Eigen::Matrix3d R_os = quatToEigen(T_odom_source.getRotation());
        Eigen::Matrix3d R_op = quatToEigen(T_odom_pelvis.getRotation());
        Eigen::Matrix3d R_po = R_op.transpose();

        Eigen::Vector3d p_o_s(T_odom_source.getOrigin().x(),
                            T_odom_source.getOrigin().y(),
                            T_odom_source.getOrigin().z());

        Eigen::Vector3d p_o_p(T_odom_pelvis.getOrigin().x(),
                            T_odom_pelvis.getOrigin().y(),
                            T_odom_pelvis.getOrigin().z());

        Eigen::Vector3d r = p_o_p - p_o_s;

        Eigen::Matrix<double,6,6> A = Eigen::Matrix<double,6,6>::Zero();

        // build A = (pelvis<-odom) * (shift) * (odom<-source)
        Eigen::Matrix<double,6,6> R1 = Eigen::Matrix<double,6,6>::Zero();
        R1.block<3,3>(0,0) = R_os;
        R1.block<3,3>(3,3) = R_os;

        Eigen::Matrix<double,6,6> S = Eigen::Matrix<double,6,6>::Identity();
        S.block<3,3>(0,3) = skew(r);  // v += w × r

        Eigen::Matrix<double,6,6> R2 = Eigen::Matrix<double,6,6>::Zero();
        R2.block<3,3>(0,0) = R_po;
        R2.block<3,3>(3,3) = R_po;

        A = R2 * S * R1;

        Eigen::Matrix<double,6,6> C_p = A * C_s * A.transpose();
        return covEigenToMsg(C_p);
    }
};


class TorsoOdometry
{
public:
    using SharedPtr = std::shared_ptr<TorsoOdometry>;

    TorsoOdometry() : tf_listener_(tf_buffer_) {}

    TorsoOdometry(ros::NodeHandle& nh,
                  const std::string& input_odom_topic  = "pose_source/odom",
                  const std::string& output_odom_topic = "torso_link/odom",
                  uint32_t queue_size = 10)
        : tf_listener_(tf_buffer_)
    {
        odom_sub_ = nh.subscribe(input_odom_topic,
                                 queue_size,
                                 &TorsoOdometry::odomCallback,
                                 this);

        torso_odom_pub_ = nh.advertise<nav_msgs::Odometry>(output_odom_topic,
                                                           queue_size);

        last_valid_ = false;

        // Initialize msg_
        msg_.header.frame_id = "odom";
        msg_.child_frame_id  = "torso_link";
        msg_.pose.covariance.fill(0.0);
        msg_.twist.covariance.fill(0.0);
    }

    // Public odometry output (as requested)
    nav_msgs::Odometry msg_;
    std::mutex mutex_;

private:
    ros::Subscriber odom_sub_;
    ros::Publisher torso_odom_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    bool last_valid_;
    ros::Time last_stamp_;
    tf2::Transform last_T_odom_torso_;

    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        const std::string parent_frame = "odom";
        const std::string child_frame  = "torso_link";

        ros::Time t = ros::Time(0);

        if (msg->header.frame_id == parent_frame &&
            msg->child_frame_id  == child_frame)
        {
            torso_odom_pub_.publish(msg);
            return;
        }

        geometry_msgs::TransformStamped odom_T_torso_msg;
        try
        {
            if (!tf_buffer_.canTransform(parent_frame, child_frame, t, ros::Duration(0.1)))
            {
                ROS_WARN_THROTTLE(1.0, "Waiting for TF %s -> %s", parent_frame.c_str(), child_frame.c_str());
                return;
            }

            odom_T_torso_msg = tf_buffer_.lookupTransform(parent_frame, child_frame, t);
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN_THROTTLE(1.0, "TF lookup failed (%s -> %s): %s",
                              parent_frame.c_str(), child_frame.c_str(), ex.what());
            return;
        }

        // Convert to tf2::Transform
        tf2::Transform T_odom_torso;
        tf2::fromMsg(odom_T_torso_msg.transform, T_odom_torso);

        // ----------------------------
        // Fill msg_ header + pose
        // ----------------------------
        msg_.header.stamp    = odom_T_torso_msg.header.stamp;  // consistent TF timestamp
        msg_.header.frame_id = parent_frame;
        msg_.child_frame_id  = child_frame;

        msg_.pose.pose = transformToPoseMsg(T_odom_torso);

        // ----------------------------
        // Compute twist by differentiation (twist in child frame)
        // ----------------------------
        if (last_valid_)
        {
            const double dt = (msg_.header.stamp - last_stamp_).toSec();

            if (dt > 1e-4)
            {
                // Linear velocity in odom frame
                tf2::Vector3 p_now  = T_odom_torso.getOrigin();
                tf2::Vector3 p_prev = last_T_odom_torso_.getOrigin();
                tf2::Vector3 v_odom = (p_now - p_prev) / dt;

                // Angular velocity in odom frame
                tf2::Quaternion q_now  = T_odom_torso.getRotation();
                tf2::Quaternion q_prev = last_T_odom_torso_.getRotation();

                tf2::Quaternion q_rel = q_prev.inverse() * q_now;
                q_rel.normalize();

                double angle = q_rel.getAngle();
                tf2::Vector3 axis = q_rel.getAxis();

                // Ensure shortest rotation
                if (angle > M_PI)
                    angle -= 2.0 * M_PI;

                tf2::Vector3 w_odom = axis * (angle / dt);

                // Convert twist from odom frame -> child frame
                tf2::Matrix3x3 R_odom_torso(q_now);
                tf2::Matrix3x3 R_torso_odom = R_odom_torso.transpose();

                tf2::Vector3 v_child = R_torso_odom * v_odom;
                tf2::Vector3 w_child = R_torso_odom * w_odom;

                msg_.twist.twist.linear.x = v_child.x();
                msg_.twist.twist.linear.y = v_child.y();
                msg_.twist.twist.linear.z = v_child.z();

                msg_.twist.twist.angular.x = w_child.x();
                msg_.twist.twist.angular.y = w_child.y();
                msg_.twist.twist.angular.z = w_child.z();
            }
            else
            {
                msg_.twist.twist = geometry_msgs::Twist();
            }
        }
        else
        {
            msg_.twist.twist = geometry_msgs::Twist();
        }

        // Publish
        torso_odom_pub_.publish(msg_);

        // Update last state
        last_T_odom_torso_ = T_odom_torso;
        last_stamp_ = msg_.header.stamp;
        last_valid_ = true;
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

};
