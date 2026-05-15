// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSMState.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/terminations.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <mutex>

class State_RLBase : public FSMState
{
public:
    using steady_clock = std::chrono::steady_clock;

    State_RLBase(int state_mode, std::string state_string);
    
    void enter()
    {
        // set gain
        for (int i = 0; i < env->robot->data.joint_stiffness.size(); ++i)
        {
            lowcmd->msg_.motor_cmd()[i].kp() = env->robot->data.joint_stiffness[i];
            lowcmd->msg_.motor_cmd()[i].kd() = env->robot->data.joint_damping[i];
            lowcmd->msg_.motor_cmd()[i].dq() = 0;
            lowcmd->msg_.motor_cmd()[i].tau() = 0;
        }

        env->robot->update();
        if (has_goal_command_observation_) {
            env->reset_goal_position(Eigen::Vector3f(0.0f, 0.0f, env->robot->data.root_pos_w.z()));
            env->set_text_key(goal_idle_text_key_);
        } else {
            env->initialize_goal_to_robot_if_unset();
        }
        goal_reached_since_ = rclcpp::Time(0, 0, node_ ? node_->get_clock()->get_clock_type() : RCL_ROS_TIME);
        goal_reached_idle_sent_ = false;
        goal_distance_log_counter_ = 0;
        action_update_seq_ = 0;
        last_applied_action_seq_ = 0;
        latest_action_.assign(env->robot->data.joint_ids_map.size(), 0.0f);
        control_freq_sample_count_ = 0;
        control_freq_window_start_ = steady_clock::time_point{};

        if (getStateString() == "Physhuman" &&
            FSMState::lowstate &&
            FSMState::lowstate->joystick.LB.pressed &&
            FSMState::lowstate->joystick.LB.pressed_time >= 2.0f &&
            FSMState::lowstate->joystick.B.on_pressed) {
            env->set_text_key("apose");
        }

        // Start policy thread
        policy_thread_running = true;
        policy_thread = std::thread([this]{
            using clock = std::chrono::high_resolution_clock;
            const std::chrono::duration<double> desiredDuration(env->step_dt);
            const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);
            const auto dt_ms = std::chrono::duration<double, std::milli>(dt).count();

            // Initialize timing
            auto sleepTill = clock::now() + dt;
            env->reset();

            while (policy_thread_running)
            {
                const auto step_start = clock::now();
                env->step();
                const auto action_update_time = steady_clock::now();
                publish_latest_action_snapshot(
                    env->action_manager->processed_actions(),
                    action_update_time
                );
                const auto step_end = clock::now();

                if (policy_delay_debug_enabled_) {
                    policy_delay_debug_counter_++;
                    const auto step_ms = std::chrono::duration<double, std::milli>(step_end - step_start).count();
                    const auto overrun_ms = std::max(0.0, step_ms - dt_ms);
                    const bool should_log_interval =
                        (policy_delay_debug_counter_ % policy_delay_debug_interval_) == 0;
                    const bool should_log_overrun =
                        policy_delay_debug_warn_ms_ > 0.0 && step_ms >= policy_delay_debug_warn_ms_;
                    if (should_log_interval || should_log_overrun) {
                        spdlog::info(
                            "[PolicyDelay] step={} policy_step_ms={:.3f} budget_ms={:.3f} overrun_ms={:.3f}",
                            policy_delay_debug_counter_,
                            step_ms,
                            dt_ms,
                            overrun_ms
                        );
                    }
                }

                // Sleep
                std::this_thread::sleep_until(sleepTill);
                sleepTill += dt;
            }
        });
        if (measure_terrain_heights) {
            heightmaphandler->start();
        }
    }

    void run();
    
    void exit()
    {
        policy_thread_running = false;
        if (policy_thread.joinable()) {
            policy_thread.join();
        }
        if (action_csv_stream_.is_open()) {
            action_csv_stream_.flush();
            action_csv_stream_.close();
        }
    }

    void pre_run()
    {
        FSMState::pre_run();
        handle_text_commands();
        republish_velocitymarkers();
        monitor_goal();
        simulate_heights();
    }

    static std::shared_ptr<VelocityMarkers_t> velocitymarkers;
    static std::shared_ptr<OdometryFusion_t> torsoodometryfusion;
    static std::shared_ptr<HeightMap_t> heightmap;
    static std::shared_ptr<HeightMapHandler_t> heightmaphandler;
    static rclcpp::Node::SharedPtr node_;

private:
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;

    std::thread policy_thread;
    bool policy_thread_running = false;

    bool measure_terrain_heights = false;

    // ROS subscriber for real-time text commands
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_cmd_sub_;
    rclcpp::CallbackGroup::SharedPtr text_cmd_cb_group_;

    // ROS subscriber for goal-reaching commands
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::CallbackGroup::SharedPtr goal_pose_cb_group_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;

    // Action monitor (offline CSV + online ROS topic)
    bool action_csv_log_enabled_ = false;
    bool action_topic_pub_enabled_ = false;
    std::size_t action_monitor_decimation_ = 1;
    std::size_t action_monitor_counter_ = 0;
    std::ofstream action_csv_stream_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr action_pub_;

    bool policy_delay_debug_enabled_ = false;
    std::size_t policy_delay_debug_interval_ = 1;
    double policy_delay_debug_warn_ms_ = 0.0;
    std::size_t policy_delay_debug_counter_ = 0;
    mutable std::mutex action_update_mutex_;
    std::vector<float> latest_action_;
    steady_clock::time_point latest_action_update_time_{};
    std::size_t action_update_seq_ = 0;
    std::size_t last_applied_action_seq_ = 0;
    std::size_t control_freq_sample_count_ = 0;
    steady_clock::time_point control_freq_window_start_{};
    bool has_goal_command_observation_ = false;
    double goal_reached_distance_ = 0.3;
    double goal_reached_hold_time_ = 0.5;
    std::size_t goal_distance_log_interval_ = 50;
    std::string goal_marker_frame_id_ = "odom";
    std::string goal_idle_text_key_ = "stand";
    std::string goal_active_text_key_ = "walk";
    rclcpp::Time goal_reached_since_{0, 0, RCL_ROS_TIME};
    bool goal_reached_idle_sent_ = false;
    std::size_t goal_distance_log_counter_ = 0;

    void monitor_action(const std::vector<float>& action);
    void publish_latest_action_snapshot(const std::vector<float>& action,
                                        const steady_clock::time_point& update_time);
    bool consume_latest_action_snapshot(std::vector<float>& action,
                                        steady_clock::time_point& update_time,
                                        std::size_t& action_seq);
    void handle_text_commands();
    void handle_text_command_msg(const std::string& key);
    void monitor_goal();
    void publish_goal_marker(double distance_xy);
    double goal_distance_xy() const;
    void gauge_control_frequency(const steady_clock::time_point& action_time)
    {
        if (control_freq_sample_count_ == 0) {
            control_freq_window_start_ = action_time;
            control_freq_sample_count_ = 1;
            return;
        }

        control_freq_sample_count_++;
        constexpr std::size_t kLogSamples = 10;
        if (control_freq_sample_count_ < kLogSamples) {
            return;
        }

        const auto elapsed_s =
            std::chrono::duration<double>(action_time - control_freq_window_start_).count();
        if (elapsed_s > 0.0) {
            const double hz = static_cast<double>(control_freq_sample_count_ - 1) / elapsed_s;
            spdlog::info(
                "[ControlFreq] state={} target_action_hz={:.2f} measured_action_hz={:.2f}",
                getStateString(),
                1.0 / static_cast<double>(env->step_dt),
                hz
            );
        }

        control_freq_window_start_ = action_time;
        control_freq_sample_count_ = 1;
    }

    void republish_velocitymarkers()
    {
        if (!env->cfg["commands"].IsDefined() ||
            !env->cfg["commands"]["base_velocity"].IsDefined() ||
            !env->cfg["commands"]["base_velocity"]["ranges"].IsDefined()) {
            return;
        }
        auto & joystick = env->robot->data.joystick;

        const auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

        velocitymarkers->cmd_marker_.points.resize(2);
        velocitymarkers->real_marker_.points.resize(2);

        velocitymarkers->cmd_marker_.points[0].x = 0.0;
        velocitymarkers->cmd_marker_.points[0].y = 0.0;
        velocitymarkers->cmd_marker_.points[0].z = 0.0;
        velocitymarkers->cmd_marker_.points[1].x = std::clamp(joystick->ly(), cfg["lin_vel_x"][0].as<float>(), cfg["lin_vel_x"][1].as<float>());
        velocitymarkers->cmd_marker_.points[1].y = std::clamp(-joystick->lx(), cfg["lin_vel_y"][0].as<float>(), cfg["lin_vel_y"][1].as<float>());
        velocitymarkers->cmd_marker_.points[1].z = 0.0;

        velocitymarkers->real_marker_.points[0].x = 0.0;
        velocitymarkers->real_marker_.points[0].y = 0.0;
        velocitymarkers->real_marker_.points[0].z = 0.0;
        velocitymarkers->real_marker_.points[1].x = env->robot->data.root_lin_vel_b[0];
        velocitymarkers->real_marker_.points[1].y = env->robot->data.root_lin_vel_b[1];
        velocitymarkers->real_marker_.points[1].z = 0.0;

        velocitymarkers->publish();
    }

    void simulate_heights()
    {
        heightmaphandler->updateSimulationHeights(heightmap->msg_.data());
    }
};

REGISTER_FSM(State_RLBase)
