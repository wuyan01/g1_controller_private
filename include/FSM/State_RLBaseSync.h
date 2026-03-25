// Debug-only synchronous RL state: policy inference and motor command
// application run in a single thread with zero control delay.
// Use type: RLBaseSync in config.yaml to activate (instead of RLBase).

#pragma once

#include "FSMState.h"
#include "FSM/State_RLBase.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/terminations.h"
#include <std_msgs/msg/string.hpp>
#include <chrono>

class State_RLBaseSync : public FSMState
{
public:
    using steady_clock = std::chrono::steady_clock;

    State_RLBaseSync(int state_mode, std::string state_string);

    void enter()
    {
        for (int i = 0; i < env->robot->data.joint_stiffness.size(); ++i)
        {
            lowcmd->msg_.motor_cmd()[i].kp() = env->robot->data.joint_stiffness[i];
            lowcmd->msg_.motor_cmd()[i].kd() = env->robot->data.joint_damping[i];
            lowcmd->msg_.motor_cmd()[i].dq() = 0;
            lowcmd->msg_.motor_cmd()[i].tau() = 0;
        }

        env->robot->update();
        env->reset();
        env->step();
        step_counter_ = 0;
        control_freq_sample_count_ = 0;
        control_freq_window_start_ = steady_clock::time_point{};

        if (measure_terrain_heights) {
            State_RLBase::heightmaphandler->start();
        }
    }

    void run()
    {
        ++step_counter_;
        if (step_counter_ >= steps_per_policy_)
        {
            step_counter_ = 0;
            env->step();
            gauge_control_frequency(steady_clock::now());
        }

        auto action = env->action_manager->processed_actions();
        for (int i(0); i < env->robot->data.joint_ids_map.size(); i++) {
            lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
        }
    }

    void exit() {}

    void pre_run()
    {
        FSMState::pre_run();
        republish_velocitymarkers();
        simulate_heights();
    }

private:
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;

    bool measure_terrain_heights = false;
    int step_counter_ = 0;
    int steps_per_policy_ = 20;
    std::size_t control_freq_sample_count_ = 0;
    steady_clock::time_point control_freq_window_start_{};

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_cmd_sub_;
    rclcpp::CallbackGroup::SharedPtr text_cmd_cb_group_;

    void gauge_control_frequency(const steady_clock::time_point& action_time)
    {
        if (control_freq_sample_count_ == 0) {
            control_freq_window_start_ = action_time;
            control_freq_sample_count_ = 1;
            return;
        }

        control_freq_sample_count_++;
        constexpr std::size_t kLogSamples = 100;
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
                1000.0 / static_cast<double>(steps_per_policy_),
                hz
            );
        }

        control_freq_window_start_ = action_time;
        control_freq_sample_count_ = 1;
    }

    void republish_velocitymarkers()
    {
        auto & joystick = env->robot->data.joystick;
        const auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

        State_RLBase::velocitymarkers->cmd_marker_.points.resize(2);
        State_RLBase::velocitymarkers->real_marker_.points.resize(2);

        State_RLBase::velocitymarkers->cmd_marker_.points[0].x = 0.0;
        State_RLBase::velocitymarkers->cmd_marker_.points[0].y = 0.0;
        State_RLBase::velocitymarkers->cmd_marker_.points[0].z = 0.0;
        State_RLBase::velocitymarkers->cmd_marker_.points[1].x = std::clamp(joystick->ly(), cfg["lin_vel_x"][0].as<float>(), cfg["lin_vel_x"][1].as<float>());
        State_RLBase::velocitymarkers->cmd_marker_.points[1].y = std::clamp(-joystick->lx(), cfg["lin_vel_y"][0].as<float>(), cfg["lin_vel_y"][1].as<float>());
        State_RLBase::velocitymarkers->cmd_marker_.points[1].z = 0.0;

        State_RLBase::velocitymarkers->real_marker_.points[0].x = 0.0;
        State_RLBase::velocitymarkers->real_marker_.points[0].y = 0.0;
        State_RLBase::velocitymarkers->real_marker_.points[0].z = 0.0;
        State_RLBase::velocitymarkers->real_marker_.points[1].x = env->robot->data.root_lin_vel_b[0];
        State_RLBase::velocitymarkers->real_marker_.points[1].y = env->robot->data.root_lin_vel_b[1];
        State_RLBase::velocitymarkers->real_marker_.points[1].z = 0.0;

        State_RLBase::velocitymarkers->publish();
    }

    void simulate_heights()
    {
        State_RLBase::heightmaphandler->updateSimulationHeights(State_RLBase::heightmap->msg_.data());
    }
};

REGISTER_FSM(State_RLBaseSync)
