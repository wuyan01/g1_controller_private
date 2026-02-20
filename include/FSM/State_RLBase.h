// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSMState.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/terminations.h"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <fstream>
#include <filesystem>

class State_RLBase : public FSMState
{
public:
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
        // Start policy thread
        policy_thread_running = true;
        policy_thread = std::thread([this]{
            using clock = std::chrono::high_resolution_clock;
            const std::chrono::duration<double> desiredDuration(env->step_dt);
            const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);

            // Initialize timing
            auto sleepTill = clock::now() + dt;
            env->reset();

            while (policy_thread_running)
            {
                env->step();

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
        republish_velocitymarkers();
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

    // Action monitor (offline CSV + online ROS topic)
    bool action_csv_log_enabled_ = false;
    bool action_topic_pub_enabled_ = false;
    std::size_t action_monitor_decimation_ = 1;
    std::size_t action_monitor_counter_ = 0;
    std::ofstream action_csv_stream_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr action_pub_;

    void monitor_action(const std::vector<float>& action);

    void republish_velocitymarkers()
    {
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
