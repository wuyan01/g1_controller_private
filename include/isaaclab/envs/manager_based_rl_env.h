// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "isaaclab/manager/observation_manager.h"
#include "isaaclab/envs/steering_command.h"
#include "isaaclab/manager/action_manager.h"
#include "isaaclab/assets/articulation/articulation.h"
#include "isaaclab/algorithms/algorithms.h"
#include <iostream>
#include "isaaclab/utils/utils.h"
#include "cnpy.h"
#include <random>
#include <algorithm>
#include <fstream>
#include <mutex>

namespace isaaclab
{

class ObservationManager;
class ActionManager;

class ManagerBasedRLEnv
{
public:
    // Constructor
    ManagerBasedRLEnv(YAML::Node cfg, std::shared_ptr<Articulation> robot_)
    :cfg(cfg), robot(std::move(robot_))
    {
        // Parse configuration
        this->step_dt = cfg["step_dt"].as<float>();
        const auto observations_cfg = cfg["observations"];
        if (observations_cfg["steering_command"].IsDefined()) {
            const auto params = observations_cfg["steering_command"]["params"];
            steering_command_.configure(
                params["max_speed"].as<float>(1.0f),
                params["max_yaw_rate"].as<float>(1.0f),
                params["deadzone"].as<float>(0.1f)
            );
            steering_command_enabled_ = true;
        }
        robot->data.joint_ids_map = cfg["joint_ids_map"].as<std::vector<int>>();
        robot->data.joint_pos.resize(robot->data.joint_ids_map.size());
        robot->data.joint_vel.resize(robot->data.joint_ids_map.size());

        if (cfg["text_embedding_dict_path"].IsDefined() && cfg["text_embedding_dict_path"].IsScalar()) {
            std::filesystem::path text_embedding_dict_path = cfg["text_embedding_dict_path"].as<std::string>();
            if (text_embedding_dict_path.is_relative()) {
                text_embedding_dict_path = param::proj_dir / text_embedding_dict_path;
            }
            this->text_embedding_dict = cnpy::npz_load(text_embedding_dict_path);
        }

        // Initialize current text key from config (fallback to empty string)
        if (cfg["observations"]["text_embedding"].IsDefined() &&
            cfg["observations"]["text_embedding"]["params"]["text_key"].IsDefined()) {
            current_text_key_ = cfg["observations"]["text_embedding"]["params"]["text_key"].as<std::string>();
            spdlog::info("Initial text command: '{}'", current_text_key_);
        }

        { // default joint positions
            auto default_joint_pos = cfg["default_joint_pos"].as<std::vector<float>>();
            robot->data.default_joint_pos = Eigen::VectorXf(robot->data.joint_ids_map.size());
            for(int i(0); i< robot->data.joint_ids_map.size(); i++) {
                robot->data.default_joint_pos[i] = default_joint_pos[robot->data.joint_ids_map[i]];
            }
        }
        { // joint stiffness and damping
            robot->data.joint_stiffness = cfg["stiffness"].as<std::vector<float>>();
            robot->data.joint_damping = cfg["damping"].as<std::vector<float>>();
        }

        robot->update();
        reset_steering_command();
        initialize_goal_to_robot_if_unset();

        // load managers
        action_manager = std::make_unique<ActionManager>(cfg["actions"], this);
        observation_manager = std::make_unique<ObservationManager>(cfg["observations"], this);

        if(cfg["observations"]["base_lin_vel"].IsDefined()) {
            spdlog::warn("Observation base_lin_vel is used.");
            spdlog::warn("Please ensure the motion capture system is set correctly.");
        }

        if (cfg["observations"]["noise"].IsDefined()) {
            const auto noise_cfg = cfg["observations"]["noise"];
            const auto noise_params = noise_cfg["params"];
            noise_mode_ = noise_params["mode"].as<std::string>("gaussian");

            const unsigned int seed = noise_params["seed"].as<unsigned int>(42);
            noise_gen_.seed(seed);

            if (noise_mode_ == "fixed") {
                fixed_noise_ = generate_noise_observation();
            } else if (noise_mode_ != "gaussian" && noise_mode_ != "zero") {
                spdlog::warn("Unknown noise mode '{}', falling back to gaussian.", noise_mode_);
                noise_mode_ = "gaussian";
            }

            spdlog::info("Noise observation mode: {}", noise_mode_);
        }
    }

    void reset()
    {
        global_phase = 0;
        episode_length = 0;
        robot->update();
        reset_steering_command();
        initialize_goal_to_robot_if_unset();
        if (alg) {
            alg->reset();
        }
        // Preserve the action chunk size used to compute kActionRepeat.
        std::fill(action.begin(), action.end(), 0.0f);
        action_manager->reset();
        observation_manager->reset();
    }

    void step()
    {
        const int kActionRepeat = action.size() / robot->data.joint_pos.size();
        const int action_idx = static_cast<int>(episode_length % kActionRepeat);
        robot->update();
        update_steering_command();
        auto obs = observation_manager->compute();

        if (action_idx == 0) {
            action = alg->act(obs);
        }

        const std::size_t start = robot->data.joint_pos.size() * static_cast<std::size_t>(action_idx);
        std::vector<float> indexed_action(action.begin() + start,
                                        action.begin() + start + robot->data.joint_pos.size());
        action_manager->process_action(indexed_action);
        episode_length += 1;
    }

    // Thread-safe text key management for real-time text command updates
    bool set_text_key(const std::string& key) {
        std::lock_guard<std::mutex> lock(text_key_mutex_);
        if (text_embedding_dict.count(key)) {
            current_text_key_ = key;
            spdlog::info("Text command updated: '{}'", key);
            return true;
        }
        spdlog::warn("Text key '{}' not found in embedding dictionary. Ignoring.", key);
        return false;
    }

    bool has_text_key(const std::string& key) const {
        return text_embedding_dict.count(key) > 0;
    }

    std::string get_text_key() const {
        std::lock_guard<std::mutex> lock(text_key_mutex_);
        return current_text_key_;
    }

    void set_goal_position(const Eigen::Vector3f& goal_pos_w)
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        goal_pos_w_ = goal_pos_w;
        goal_initialized_ = true;
        goal_received_ = true;
    }

    void reset_goal_position(const Eigen::Vector3f& goal_pos_w)
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        goal_pos_w_ = goal_pos_w;
        goal_initialized_ = true;
        goal_received_ = false;
    }

    Eigen::Vector3f get_goal_position() const
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        return goal_pos_w_;
    }

    bool has_goal_received() const
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        return goal_received_;
    }

    void initialize_goal_to_robot_if_unset()
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (!goal_initialized_) {
            goal_pos_w_ = robot->data.root_pos_w;
            goal_initialized_ = true;
        }
    }

    std::vector<float> generate_noise_observation()
    {
        const auto noise_cfg = cfg["observations"]["noise"];
        const auto noise_params = noise_cfg["params"];
        const int num_future = noise_params["num_future_steps"].as<int>(1);
        const int state_dim = noise_params["state_dim"].as<int>();
        const int noise_dim = num_future * state_dim;
        const float clip_noise = noise_params["clip_noise"].as<float>(1.0f);
        const bool clip_enabled = noise_params["clip_enabled"].as<bool>(true);

        std::vector<float> noise(noise_dim, 0.0f);
        if (noise_mode_ == "zero") {
            return noise;
        }

        for (auto& v : noise) {
            v = noise_dist_(noise_gen_);
            if (clip_enabled) {
                v = std::max(-clip_noise, std::min(clip_noise, v));
            }
        }
        return noise;
    }

    std::vector<float> get_noise_observation()
    {
        if (noise_mode_ == "fixed") {
            return fixed_noise_;
        }
        return generate_noise_observation();
    }

    std::vector<float> get_steering_command() const
    {
        const auto& command = steering_command_.command();
        return std::vector<float>(command.begin(), command.end());
    }

    float step_dt;
    
    YAML::Node cfg;

    std::unique_ptr<ObservationManager> observation_manager;
    std::unique_ptr<ActionManager> action_manager;
    std::shared_ptr<Articulation> robot;
    std::unique_ptr<Algorithms> alg;
    long episode_length = 0;
    float global_phase = 0.0f;
    cnpy::npz_t text_embedding_dict;
    std::vector<float> action;

private:
    float robot_yaw() const
    {
        const auto q = robot->data.root_quat_w.normalized();
        return std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()),
                          1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
    }

    void reset_steering_command()
    {
        if (steering_command_enabled_) {
            // Mode entry holds the current heading and clears old stick commands.
            steering_command_.reset(robot_yaw());
        }
    }

    void update_steering_command()
    {
        if (!steering_command_enabled_) {
            return;
        }
        auto* joystick = robot->data.joystick;
        steering_command_.update(
            robot_yaw(), joystick ? joystick->ly() : 0.0f,
            joystick ? -joystick->lx() : 0.0f,
            joystick ? -joystick->rx() : 0.0f, step_dt
        );
    }

    SteeringCommand steering_command_;
    bool steering_command_enabled_ = false;
    std::string current_text_key_;
    mutable std::mutex text_key_mutex_;
    mutable std::mutex goal_mutex_;
    Eigen::Vector3f goal_pos_w_ = Eigen::Vector3f::Zero();
    bool goal_initialized_ = false;
    bool goal_received_ = false;
    std::string noise_mode_ = "gaussian";
    std::mt19937 noise_gen_{42};
    std::normal_distribution<float> noise_dist_{0.0f, 1.0f};
    std::vector<float> fixed_noise_;
};

};
