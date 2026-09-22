// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "onnxruntime_cxx_api.h"
#include <iostream>
#include <mutex>
#include <memory>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <yaml-cpp/yaml.h>

namespace isaaclab
{

class Algorithms
{
public:
    virtual std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs) = 0;
    virtual void reset() {}

    std::vector<float> get_action()
    {
        std::lock_guard<std::mutex> lock(act_mtx_);
        return action;
    }
    
    std::vector<float> action;
protected:
    std::mutex act_mtx_;
};

struct SoftGateRuntimeConfig
{
    bool enabled = false;
    bool use_projected_gravity = true;
    bool use_root_velocity = true;
    float threshold = 0.2f;
    float sharpness = 10.0f;
    float gate_floor = 0.0f;
    float initial_gate = 0.0f;
    bool ema_enabled = false;
    float ema_alpha = 0.9f;
    std::string ema_target = "risk";
    float projected_gravity_weight = 1.0f;
    float root_velocity_weight = 1.0f;
};

struct StatefulDebugConfig
{
    bool enabled = false;
    int interval = 50;
    int preview_dims = 3;
};

class StatefulOrtRunner : public Algorithms
{
public:
    StatefulOrtRunner(const std::string& model_path, YAML::Node deploy_cfg, std::string precision = "fp32")
    : deploy_cfg_(deploy_cfg)
    {
        env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "onnx_model");

        const auto provider = deploy_cfg_["policy_provider"].as<std::string>("auto");
        if (provider == "cuda") {
            session = create_session_cuda(model_path);
            if (!session) {
                throw std::runtime_error("Requested policy_provider=cuda could not initialize CUDA EP.");
            }
        } else if (provider == "cpu") {
            session = create_session_cpu(model_path);
            if (!session) {
                throw std::runtime_error("Requested policy_provider=cpu could not initialize CPU EP.");
            }
        } else if (provider == "auto") {
            #if defined(__aarch64__)
                session = create_session_cpu(model_path);
            #else
                session = create_session_tensorrt(model_path, precision);
                if (!session) {
                    spdlog::warn("[ORT-Stateful] Falling back to CUDA EP");
                    session = create_session_cuda(model_path);
                }
                if (!session) {
                    spdlog::warn("[ORT-Stateful] Falling back to CPU EP");
                    session = create_session_cpu(model_path);
                }
            #endif
            if (!session) {
                throw std::runtime_error("No ONNX execution provider could initialize the stateful policy.");
            }
        } else {
            throw std::runtime_error("Unknown policy_provider '" + provider + "'; expected auto, cuda, or cpu.");
        }

        for (size_t i = 0; i < session->GetInputCount(); ++i) {
            Ort::TypeInfo input_type = session->GetInputTypeInfo(i);
            input_shapes.push_back(input_type.GetTensorTypeAndShapeInfo().GetShape());
            input_sizes.push_back(product_of_shape(input_shapes.back()));
            auto input_name = session->GetInputNameAllocated(i, allocator);
            input_name_storage.emplace_back(input_name.get());
        }

        for (size_t i = 0; i < session->GetOutputCount(); ++i) {
            Ort::TypeInfo output_type = session->GetOutputTypeInfo(i);
            output_shapes.push_back(output_type.GetTensorTypeAndShapeInfo().GetShape());
            auto output_name = session->GetOutputNameAllocated(i, allocator);
            output_name_storage.emplace_back(output_name.get());
        }
        rebuild_name_views();

        uses_policy_core_outputs_ = output_name_storage.size() >= 6 &&
            output_name_storage[0] == "base_action" &&
            output_name_storage[1] == "residual_action_scaled" &&
            output_name_storage[2] == "predicted_state";

        const auto policy_input_dim = find_input_size("obs");
        if (policy_input_dim <= 0) {
            throw std::runtime_error("Stateful ONNX runner requires an input named 'obs'.");
        }
        if (uses_policy_core_outputs_) {
            action.resize(product_of_shape(output_shapes[0]), 0.0f);
        } else {
            action.resize(product_of_shape(output_shapes[0]), 0.0f);
        }

        const auto plan_cache_dim = find_input_size("plan_cache_flat");
        if (plan_cache_dim <= 0) {
            throw std::runtime_error("Stateful ONNX runner requires input 'plan_cache_flat'.");
        }
        plan_cache_flat_.assign(static_cast<std::size_t>(plan_cache_dim), 0.0f);
        plan_idx_ = {0};
        plan_valid_ = {0.0f};

        state_dim_ = infer_state_dim();
        soft_gate_cfg_ = parse_soft_gate_cfg(deploy_cfg_);
        soft_gate_cfg_.initial_gate = std::max(soft_gate_cfg_.initial_gate, soft_gate_cfg_.gate_floor);
        debug_cfg_ = parse_debug_cfg(deploy_cfg_);

        prev_predicted_state_.assign(static_cast<std::size_t>(state_dim_), 0.0f);
        prediction_valid_ = false;
        risk_ema_ = 0.0f;
        gate_ema_ = soft_gate_cfg_.initial_gate;
        current_gate_ = soft_gate_cfg_.initial_gate;

        spdlog::info("[ORT-Stateful] Loaded {} outputs, policy_core_outputs={}",
                     output_name_storage.size(), uses_policy_core_outputs_);
        if (uses_policy_core_outputs_) {
            spdlog::info("[ORT-Stateful] Soft gate runtime enabled={}, threshold={}, sharpness={}, gate_floor={}",
                         soft_gate_cfg_.enabled, soft_gate_cfg_.threshold, soft_gate_cfg_.sharpness, soft_gate_cfg_.gate_floor);
        }
    }

    // Called on the policy thread before a new episode starts.
    void reset() override
    {
        std::lock_guard<std::mutex> lock(act_mtx_);
        std::fill(action.begin(), action.end(), 0.0f);
        std::fill(plan_cache_flat_.begin(), plan_cache_flat_.end(), 0.0f);
        plan_idx_.assign(1, 0);
        plan_valid_.assign(1, 0.0f);
        std::fill(prev_predicted_state_.begin(), prev_predicted_state_.end(), 0.0f);
        prediction_valid_ = false;
        risk_ema_ = 0.0f;
        gate_ema_ = soft_gate_cfg_.initial_gate;
        current_gate_ = soft_gate_cfg_.initial_gate;
        last_risk_ = 0.0f;
        last_projected_gravity_dev_ = 0.0f;
        last_root_velocity_dev_ = 0.0f;
        last_gate_raw_ = 0.0f;
        last_base_action_norm_ = 0.0f;
        last_residual_action_scaled_norm_ = 0.0f;
        last_final_action_norm_ = 0.0f;
        last_action_preview_ = "[]";
        last_plan_idx_in_ = 0;
        last_plan_valid_in_ = 0.0f;
        last_plan_idx_out_ = 0;
        last_plan_valid_out_ = 0.0f;
        step_counter_ = 0;
    }

    std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs) override
    {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        if (obs.find("obs") == obs.end()) {
            throw std::runtime_error("Stateful ONNX runner requires observation group 'obs'.");
        }
        last_plan_idx_in_ = plan_idx_.empty() ? 0 : plan_idx_[0];
        last_plan_valid_in_ = plan_valid_.empty() ? 0.0f : plan_valid_[0];

        std::vector<Ort::Value> input_tensors;
        input_tensors.reserve(input_name_storage.size());

        for (std::size_t i = 0; i < input_name_storage.size(); ++i) {
            const std::string& name = input_name_storage[i];
            if (name == "obs") {
                auto& input_data = obs.at("obs");
                input_tensors.push_back(Ort::Value::CreateTensor<float>(
                    memory_info,
                    input_data.data(),
                    input_data.size(),
                    input_shapes[i].data(),
                    input_shapes[i].size()
                ));
            } else if (name == "plan_cache_flat") {
                input_tensors.push_back(Ort::Value::CreateTensor<float>(
                    memory_info,
                    plan_cache_flat_.data(),
                    plan_cache_flat_.size(),
                    input_shapes[i].data(),
                    input_shapes[i].size()
                ));
            } else if (name == "plan_idx") {
                input_tensors.push_back(Ort::Value::CreateTensor<int64_t>(
                    memory_info,
                    plan_idx_.data(),
                    plan_idx_.size(),
                    input_shapes[i].data(),
                    input_shapes[i].size()
                ));
            } else if (name == "plan_valid") {
                input_tensors.push_back(Ort::Value::CreateTensor<float>(
                    memory_info,
                    plan_valid_.data(),
                    plan_valid_.size(),
                    input_shapes[i].data(),
                    input_shapes[i].size()
                ));
            } else {
                throw std::runtime_error("Unsupported stateful ONNX input: " + name);
            }
        }

        auto output_tensors = session->Run(
            Ort::RunOptions{nullptr},
            input_name_ptrs.data(),
            input_tensors.data(),
            input_tensors.size(),
            output_name_ptrs.data(),
            output_name_ptrs.size()
        );

        if (uses_policy_core_outputs_) {
            update_action_from_policy_core(obs, output_tensors);
            copy_float_output(output_tensors[3], plan_cache_flat_);
            copy_int64_output(output_tensors[4], plan_idx_);
            copy_float_output(output_tensors[5], plan_valid_);
        } else {
            copy_float_output(output_tensors[0], action);
            copy_float_output(output_tensors[1], plan_cache_flat_);
            copy_int64_output(output_tensors[2], plan_idx_);
            copy_float_output(output_tensors[3], plan_valid_);
        }
        step_counter_ += 1;
        maybe_log_debug();

        std::lock_guard<std::mutex> lock(act_mtx_);
        return action;
    }

private:
    std::size_t product_of_shape(const std::vector<int64_t>& shape) const
    {
        std::size_t size = 1;
        for (const auto dim : shape) {
            if (dim < 0) {
                throw std::runtime_error("Dynamic ONNX dimensions are not supported in StatefulOrtRunner.");
            }
            size *= static_cast<std::size_t>(dim);
        }
        return size;
    }

    int64_t find_input_size(const std::string& name) const
    {
        for (std::size_t i = 0; i < input_name_storage.size(); ++i) {
            if (name == input_name_storage[i]) {
                return static_cast<int64_t>(input_sizes[i]);
            }
        }
        return -1;
    }

    SoftGateRuntimeConfig parse_soft_gate_cfg(const YAML::Node& cfg) const
    {
        SoftGateRuntimeConfig gate_cfg;
        const auto node = cfg["soft_gate"];
        if (!node || !node.IsDefined()) {
            return gate_cfg;
        }

        gate_cfg.enabled = node["enabled"].as<bool>(false);
        gate_cfg.threshold = node["threshold"].as<float>(0.2f);
        gate_cfg.sharpness = node["sharpness"].as<float>(10.0f);
        gate_cfg.gate_floor = std::clamp(node["gate_floor"].as<float>(0.0f), 0.0f, 1.0f);
        gate_cfg.initial_gate = node["initial_gate"].as<float>(gate_cfg.gate_floor);
        gate_cfg.ema_enabled = node["ema_enabled"].as<bool>(false);
        gate_cfg.ema_alpha = node["ema_alpha"].as<float>(0.9f);
        gate_cfg.ema_target = node["ema_target"].as<std::string>("risk");
        gate_cfg.projected_gravity_weight = node["projected_gravity_weight"].as<float>(1.0f);
        gate_cfg.root_velocity_weight = node["root_velocity_weight"].as<float>(1.0f);

        if (node["active_signals"] && node["active_signals"].IsSequence()) {
            gate_cfg.use_projected_gravity = false;
            gate_cfg.use_root_velocity = false;
            for (const auto& signal : node["active_signals"]) {
                const auto name = signal.as<std::string>();
                if (name == "projected_gravity") {
                    gate_cfg.use_projected_gravity = true;
                } else if (name == "root_velocity") {
                    gate_cfg.use_root_velocity = true;
                }
            }
        }
        return gate_cfg;
    }

    StatefulDebugConfig parse_debug_cfg(const YAML::Node& cfg) const
    {
        StatefulDebugConfig debug_cfg;
        const auto node = cfg["stateful_debug"];
        if (!node || !node.IsDefined()) {
            return debug_cfg;
        }
        debug_cfg.enabled = node["enabled"].as<bool>(false);
        debug_cfg.interval = std::max(1, node["interval"].as<int>(50));
        debug_cfg.preview_dims = std::max(0, node["preview_dims"].as<int>(3));
        return debug_cfg;
    }

    int infer_state_dim() const
    {
        if (uses_named_output("predicted_state")) {
            for (std::size_t i = 0; i < output_name_storage.size(); ++i) {
                if (output_name_storage[i] == "predicted_state") {
                    return static_cast<int>(product_of_shape(output_shapes[i]));
                }
            }
        }
        const auto deploy_noise = deploy_cfg_["observations"]["noise"];
        if (deploy_noise && deploy_noise["params"] && deploy_noise["params"]["state_dim"]) {
            return deploy_noise["params"]["state_dim"].as<int>() - static_cast<int>(action.size());
        }
        return 67;
    }

    bool uses_named_output(const std::string& name) const
    {
        return std::find(output_name_storage.begin(), output_name_storage.end(), name) != output_name_storage.end();
    }

    void rebuild_name_views()
    {
        input_name_ptrs.clear();
        input_name_ptrs.reserve(input_name_storage.size());
        for (const auto& name : input_name_storage) {
            input_name_ptrs.push_back(name.c_str());
        }

        output_name_ptrs.clear();
        output_name_ptrs.reserve(output_name_storage.size());
        for (const auto& name : output_name_storage) {
            output_name_ptrs.push_back(name.c_str());
        }
    }

    float compute_gate(const std::unordered_map<std::string, std::vector<float>>& obs)
    {
        if (!soft_gate_cfg_.enabled) {
            last_root_velocity_dev_ = 0.0f;
            last_projected_gravity_dev_ = 0.0f;
            last_risk_ = 0.0f;
            last_gate_raw_ = 1.0f;
            return 1.0f;
        }

        const int lin_vel_dim = std::max(state_dim_ - 64, 0);
        float risk = 0.0f;
        float root_velocity_dev = 0.0f;
        float projected_gravity_dev = 0.0f;

        if (soft_gate_cfg_.use_root_velocity && lin_vel_dim > 0) {
            const auto actual_root_velocity = current_term_slice_from_obs_group(obs, "base_lin_vel", lin_vel_dim);
            float sq_sum = 0.0f;
            for (int i = 0; i < lin_vel_dim; ++i) {
                const float diff = actual_root_velocity[i] - prev_predicted_state_[i];
                sq_sum += diff * diff;
            }
            root_velocity_dev = std::sqrt(sq_sum);
            risk += soft_gate_cfg_.root_velocity_weight * root_velocity_dev;
        }

        if (soft_gate_cfg_.use_projected_gravity) {
            const int gravity_offset = lin_vel_dim + 3;
            const auto actual_gravity = current_term_slice_from_obs_group(obs, "projected_gravity", 3);
            float sq_sum = 0.0f;
            for (int i = 0; i < 3; ++i) {
                const float diff = actual_gravity[i] - prev_predicted_state_[gravity_offset + i];
                sq_sum += diff * diff;
            }
            projected_gravity_dev = std::sqrt(sq_sum);
            risk += soft_gate_cfg_.projected_gravity_weight * projected_gravity_dev;
        }

        const float gate_raw = apply_gate_floor(sigmoid(soft_gate_cfg_.sharpness * (risk - soft_gate_cfg_.threshold)));
        last_root_velocity_dev_ = root_velocity_dev;
        last_projected_gravity_dev_ = projected_gravity_dev;
        last_risk_ = risk;
        last_gate_raw_ = gate_raw;
        if (!prediction_valid_) {
            return current_gate_;
        }
        if (!soft_gate_cfg_.ema_enabled) {
            risk_ema_ = risk;
            gate_ema_ = gate_raw;
            return gate_raw;
        }

        if (soft_gate_cfg_.ema_target == "gate") {
            gate_ema_ = soft_gate_cfg_.ema_alpha * gate_ema_ + (1.0f - soft_gate_cfg_.ema_alpha) * gate_raw;
            return gate_ema_;
        }

        risk_ema_ = soft_gate_cfg_.ema_alpha * risk_ema_ + (1.0f - soft_gate_cfg_.ema_alpha) * risk;
        gate_ema_ = apply_gate_floor(sigmoid(soft_gate_cfg_.sharpness * (risk_ema_ - soft_gate_cfg_.threshold)));
        return gate_ema_;
    }

    int infer_term_dim(const std::string& term_name, const YAML::Node& term_cfg) const
    {
        if (term_cfg["scale"] && !term_cfg["scale"].IsNull()) {
            return static_cast<int>(term_cfg["scale"].as<std::vector<float>>().size());
        }
        if (term_name == "base_lin_vel" || term_name == "base_ang_vel" || term_name == "projected_gravity") {
            return 3;
        }
        if ((term_name == "joint_pos_rel" || term_name == "joint_vel_rel" || term_name == "last_action")
            && deploy_cfg_["default_joint_pos"] && deploy_cfg_["default_joint_pos"].IsSequence()) {
            return static_cast<int>(deploy_cfg_["default_joint_pos"].size());
        }
        if (term_name == "noise" && term_cfg["params"]) {
            const int state_dim = term_cfg["params"]["state_dim"].as<int>(0);
            const int num_future_steps = term_cfg["params"]["num_future_steps"].as<int>(1);
            return state_dim * num_future_steps;
        }
        if (term_name == "text_embedding") {
            return 512;
        }
        throw std::runtime_error("Unable to infer observation dimension for term '" + term_name + "'.");
    }

    std::vector<float> current_term_slice_from_obs_group(const std::unordered_map<std::string, std::vector<float>>& obs,
                                                         const std::string& term_name,
                                                         int expected_dim) const
    {
        const auto it = obs.find("obs");
        if (it == obs.end()) {
            throw std::runtime_error("Required observation group 'obs' not found for temporal soft gate.");
        }

        const auto obs_cfg = deploy_cfg_["observations"];
        if (!obs_cfg || !obs_cfg.IsMap()) {
            throw std::runtime_error("Deployment config observations block is missing or invalid.");
        }

        const bool use_gym_history = obs_cfg["use_gym_history"].as<bool>(false);
        if (use_gym_history) {
            throw std::runtime_error("Temporal soft gate currently does not support use_gym_history=true.");
        }

        int offset = 0;
        for (auto term = obs_cfg.begin(); term != obs_cfg.end(); ++term) {
            const auto name = term->first.as<std::string>();
            if (name == "use_gym_history" || name == "scale_first") {
                continue;
            }

            const auto term_cfg = term->second;
            const int history_length = term_cfg["history_length"].as<int>(1);
            const int term_dim = infer_term_dim(name, term_cfg);

            if (name == term_name) {
                if (term_dim != expected_dim) {
                    throw std::runtime_error(
                        "Observation term '" + term_name + "' dim mismatch. Expected "
                        + std::to_string(expected_dim) + ", got " + std::to_string(term_dim) + "."
                    );
                }
                const int current_offset = offset + (history_length - 1) * term_dim;
                const auto& values = it->second;
                if (static_cast<int>(values.size()) < current_offset + term_dim) {
                    throw std::runtime_error(
                        "Observation group 'obs' is shorter than expected for term '" + term_name + "'."
                    );
                }
                return std::vector<float>(
                    values.begin() + current_offset,
                    values.begin() + current_offset + term_dim
                );
            }

            offset += history_length * term_dim;
        }

        throw std::runtime_error("Required observation term '" + term_name + "' not found in deploy config.");
    }

    float sigmoid(float x) const
    {
        return 1.0f / (1.0f + std::exp(-x));
    }

    float apply_gate_floor(float gate) const
    {
        return soft_gate_cfg_.gate_floor + (1.0f - soft_gate_cfg_.gate_floor) * gate;
    }

    void update_action_from_policy_core(
        const std::unordered_map<std::string, std::vector<float>>& obs,
        const std::vector<Ort::Value>& output_tensors)
    {
        std::vector<float> base_action_values;
        std::vector<float> residual_action_scaled_values;
        std::vector<float> predicted_state_values;
        copy_float_output(output_tensors[0], base_action_values);
        copy_float_output(output_tensors[1], residual_action_scaled_values);
        copy_float_output(output_tensors[2], predicted_state_values);
        last_base_action_norm_ = vector_norm(base_action_values);
        last_residual_action_scaled_norm_ = vector_norm(residual_action_scaled_values);

        const float gate = compute_gate(obs);
        action.resize(base_action_values.size());
        for (std::size_t i = 0; i < action.size(); ++i) {
            action[i] = base_action_values[i] + gate * residual_action_scaled_values[i];
        }
        last_final_action_norm_ = vector_norm(action);
        last_action_preview_ = preview_vector(action, debug_cfg_.preview_dims);
        last_plan_idx_out_ = plan_idx_.empty() ? 0 : plan_idx_[0];
        last_plan_valid_out_ = plan_valid_.empty() ? 0.0f : plan_valid_[0];

        prev_predicted_state_ = std::move(predicted_state_values);
        prediction_valid_ = std::all_of(prev_predicted_state_.begin(), prev_predicted_state_.end(), [](float v) {
            return std::isfinite(v);
        });
        current_gate_ = gate;
    }

    float vector_norm(const std::vector<float>& values) const
    {
        float sq_sum = 0.0f;
        for (const auto v : values) {
            sq_sum += v * v;
        }
        return std::sqrt(sq_sum);
    }

    std::string preview_vector(const std::vector<float>& values, int count) const
    {
        if (count <= 0 || values.empty()) {
            return "[]";
        }
        std::ostringstream oss;
        oss << "[";
        const int n = std::min<int>(count, static_cast<int>(values.size()));
        for (int i = 0; i < n; ++i) {
            if (i > 0) {
                oss << ", ";
            }
            oss << values[static_cast<std::size_t>(i)];
        }
        if (static_cast<int>(values.size()) > n) {
            oss << ", ...";
        }
        oss << "]";
        return oss.str();
    }

    void maybe_log_debug()
    {
        if (!debug_cfg_.enabled || step_counter_ % static_cast<std::size_t>(debug_cfg_.interval) != 0) {
            return;
        }

        const auto plan_idx_in = last_plan_idx_in_;
        const auto plan_valid_in = last_plan_valid_in_;
        const auto plan_idx_out = plan_idx_.empty() ? 0 : plan_idx_[0];
        const auto plan_valid_out = plan_valid_.empty() ? 0.0f : plan_valid_[0];

        if (uses_policy_core_outputs_) {
            spdlog::info(
                "[ORT-Stateful][step {}] cache in(idx={}, valid={:.0f}) out(idx={}, valid={:.0f}) "
                "gate={:.3f} risk={:.3f} grav_dev={:.3f} root_vel_dev={:.3f} "
                "base_norm={:.3f} residual_scaled_norm={:.3f} final_norm={:.3f} pred_valid={} action={}",
                step_counter_,
                plan_idx_in,
                plan_valid_in,
                plan_idx_out,
                plan_valid_out,
                current_gate_,
                last_risk_,
                last_projected_gravity_dev_,
                last_root_velocity_dev_,
                last_base_action_norm_,
                last_residual_action_scaled_norm_,
                last_final_action_norm_,
                prediction_valid_,
                last_action_preview_
            );
        } else {
            spdlog::info(
                "[ORT-Stateful][step {}] cache in(idx={}, valid={:.0f}) out(idx={}, valid={:.0f}) "
                "final_norm={:.3f} action={}",
                step_counter_,
                plan_idx_in,
                plan_valid_in,
                plan_idx_out,
                plan_valid_out,
                vector_norm(action),
                preview_vector(action, debug_cfg_.preview_dims)
            );
        }
    }

    void copy_float_output(const Ort::Value& value, std::vector<float>& dst) const
    {
        const auto info = value.GetTensorTypeAndShapeInfo();
        const auto size = static_cast<std::size_t>(info.GetElementCount());
        const auto* data = value.GetTensorData<float>();
        dst.assign(data, data + size);
    }

    void copy_int64_output(const Ort::Value& value, std::vector<int64_t>& dst) const
    {
        const auto info = value.GetTensorTypeAndShapeInfo();
        const auto size = static_cast<std::size_t>(info.GetElementCount());
        const auto* data = value.GetTensorData<int64_t>();
        dst.assign(data, data + size);
    }

    std::unique_ptr<Ort::Session> create_session_tensorrt(const std::string& model_path, const std::string& precision) {
        try {
            Ort::SessionOptions so;
            so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
            so.SetIntraOpNumThreads(1);
            so.SetInterOpNumThreads(1);
            so.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
            enable_transformer_optimizations(so);
            so.AddConfigEntry("memory.optimize_memory", "1");
            so.AddConfigEntry("enable_cpu_mem_arena", "1");
            Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_Tensorrt(so, 0));
            if (precision == "fp16") {
                so.AddConfigEntry("trt_fp16_enable", "1");
            } else if (precision == "int8") {
                so.AddConfigEntry("trt_int8_enable", "1");
            }
            so.AddConfigEntry("trt_timing_cache_enable", "1");
            so.AddConfigEntry("trt_max_workspace_size", "1073741824");
            so.AddConfigEntry("trt_engine_cache_enable", "1");
            so.AddConfigEntry("trt_engine_cache_path", "./trt_cache");
            auto sess = std::make_unique<Ort::Session>(env, model_path.c_str(), so);
            spdlog::info("[ORT-Stateful] Using TensorRT EP");
            return sess;
        } catch (const Ort::Exception& e) {
            spdlog::warn("[ORT-Stateful] TensorRT EP failed: {}", e.what());
            return nullptr;
        }
    }

    std::unique_ptr<Ort::Session> create_session_cuda(const std::string& model_path) {
        try {
            Ort::SessionOptions so;
            so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
            const auto& api = Ort::GetApi();
            OrtCUDAProviderOptionsV2* raw_cuda = nullptr;
            Ort::ThrowOnError(api.CreateCUDAProviderOptions(&raw_cuda));
            const auto release_cuda = [](OrtCUDAProviderOptionsV2* value) {
                Ort::GetApi().ReleaseCUDAProviderOptions(value);
            };
            std::unique_ptr<OrtCUDAProviderOptionsV2, decltype(release_cuda)> cuda(raw_cuda, release_cuda);
            // TF32 drift can cross a decoder FSQ rounding boundary. Preserve FP32.
            // Keep the default SIZE_MAX arena limit; zero bytes cannot allocate.
            const char* keys[] = {"use_tf32"};
            const char* values[] = {"0"};
            Ort::ThrowOnError(api.UpdateCUDAProviderOptions(cuda.get(), keys, values, 1));
            so.AppendExecutionProvider_CUDA_V2(*cuda);
            auto sess = std::make_unique<Ort::Session>(env, model_path.c_str(), so);
            spdlog::info("[ORT-Stateful] Using CUDA EP (TF32 disabled)");
            return sess;
        } catch (const Ort::Exception& e) {
            spdlog::warn("[ORT-Stateful] CUDA EP failed: {}", e.what());
            return nullptr;
        }
    }

    std::unique_ptr<Ort::Session> create_session_cpu(const std::string& model_path) {
        try {
            Ort::SessionOptions so;
            so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
            auto sess = std::make_unique<Ort::Session>(env, model_path.c_str(), so);
            spdlog::info("[ORT-Stateful] Using CPU EP");
            return sess;
        } catch (const Ort::Exception& e) {
            spdlog::warn("[ORT-Stateful] CPU EP failed: {}", e.what());
            return nullptr;
        }
    }

    void enable_transformer_optimizations(Ort::SessionOptions& so) {
        so.AddConfigEntry("optimization.enable_layer_norm", "1");
        so.AddConfigEntry("optimization.enable_attention", "1");
        so.AddConfigEntry("optimization.enable_embed_layer_norm", "1");
        so.AddConfigEntry("optimization.enable_skip_layer_norm", "1");
        so.AddConfigEntry("optimization.enable_bias_gelu", "1");
        so.AddConfigEntry("optimization.enable_gelu", "1");
        so.AddConfigEntry("optimization.enable_memory_efficient_attention", "1");
        so.AddConfigEntry("optimization.enable_shape_optimization", "1");
        so.AddConfigEntry("optimization.enable_packed_qkv", "1");
        so.AddConfigEntry("optimization.enable_gpu_attention", "1");
    }

private:
    YAML::Node deploy_cfg_;
    Ort::Env env;
    std::unique_ptr<Ort::Session> session;
    Ort::AllocatorWithDefaultOptions allocator;

    std::vector<std::string> input_name_storage;
    std::vector<std::string> output_name_storage;
    std::vector<const char*> input_name_ptrs;
    std::vector<const char*> output_name_ptrs;
    std::vector<std::vector<int64_t>> input_shapes;
    std::vector<std::vector<int64_t>> output_shapes;
    std::vector<std::size_t> input_sizes;

    std::vector<float> plan_cache_flat_;
    std::vector<int64_t> plan_idx_;
    std::vector<float> plan_valid_;

    SoftGateRuntimeConfig soft_gate_cfg_;
    StatefulDebugConfig debug_cfg_;
    std::vector<float> prev_predicted_state_;
    bool prediction_valid_ = false;
    float risk_ema_ = 0.0f;
    float gate_ema_ = 0.0f;
    float current_gate_ = 0.0f;
    float last_risk_ = 0.0f;
    float last_projected_gravity_dev_ = 0.0f;
    float last_root_velocity_dev_ = 0.0f;
    float last_gate_raw_ = 0.0f;
    float last_base_action_norm_ = 0.0f;
    float last_residual_action_scaled_norm_ = 0.0f;
    float last_final_action_norm_ = 0.0f;
    std::string last_action_preview_ = "[]";
    int64_t last_plan_idx_in_ = 0;
    float last_plan_valid_in_ = 0.0f;
    int64_t last_plan_idx_out_ = 0;
    float last_plan_valid_out_ = 0.0f;
    std::size_t step_counter_ = 0;
    int state_dim_ = 67;
    bool uses_policy_core_outputs_ = false;
};

class OrtRunner : public Algorithms
{
public:
    OrtRunner(std::string model_path, std::string precision="fp32")
    {
        // Init Model
        env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "onnx_model");

        #if defined(__aarch64__)
            session = create_session_cpu(model_path);
        #else
            session = create_session_tensorrt(model_path, precision);
            if (!session) {
                spdlog::warn("[ORT] Falling back to CUDA EP");
                session = create_session_cuda(model_path);
            }
            if (!session) {
                spdlog::warn("[ORT] Falling back to CPU EP");
                session = create_session_cpu(model_path);
            }
        #endif

        for (size_t i = 0; i < session->GetInputCount(); ++i) {
            Ort::TypeInfo input_type = session->GetInputTypeInfo(i);
            input_shapes.push_back(input_type.GetTensorTypeAndShapeInfo().GetShape());
            auto input_name = session->GetInputNameAllocated(i, allocator);
            input_names.push_back(input_name.release());
        }

        for (const auto& shape : input_shapes) {
            size_t size = 1;
            for (const auto& dim : shape) {
                size *= dim;
            }
            input_sizes.push_back(size);
        }

        // Get output shape
        Ort::TypeInfo output_type = session->GetOutputTypeInfo(0);
        output_shape = output_type.GetTensorTypeAndShapeInfo().GetShape();
        auto output_name = session->GetOutputNameAllocated(0, allocator);
        output_names.push_back(output_name.release());

        action.resize(output_shape[1]);
    }

    std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs)
    {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

        // make sure all input names are in obs
        for (const auto& name : input_names) {
            if (obs.find(name) == obs.end()) {
                throw std::runtime_error("Input name " + std::string(name) + " not found in observations.");
            }
        }

        // Create input tensors
        std::vector<Ort::Value> input_tensors;
        for(int i(0); i<input_names.size(); ++i)
        {
            const std::string name_str(input_names[i]);
            auto& input_data = obs.at(name_str);
            auto input_tensor = Ort::Value::CreateTensor<float>(memory_info, input_data.data(), input_sizes[i], input_shapes[i].data(), input_shapes[i].size());
            input_tensors.push_back(std::move(input_tensor));
        }

        // Run the model
        auto output_tensor = session->Run(Ort::RunOptions{nullptr}, input_names.data(), input_tensors.data(), input_tensors.size(), output_names.data(), 1);

        // Copy output data
        auto floatarr = output_tensor.front().GetTensorMutableData<float>();
        std::lock_guard<std::mutex> lock(act_mtx_);
        std::memcpy(action.data(), floatarr, output_shape[1] * sizeof(float));
        return action;
    }

private:
// Add these config entries for transformer models
    void enable_transformer_optimizations(Ort::SessionOptions& so) {
        // Attention mechanism optimizations
        // so.AddConfigEntry("optimization.enable_gelu_approximation", "1");
        so.AddConfigEntry("optimization.enable_layer_norm", "1");
        so.AddConfigEntry("optimization.enable_attention", "1");
        
        // Memory optimization for attention layers
        so.AddConfigEntry("optimization.enable_embed_layer_norm", "1");
        so.AddConfigEntry("optimization.enable_skip_layer_norm", "1");
        
        // Kernel optimizations
        so.AddConfigEntry("optimization.enable_bias_gelu", "1");
        so.AddConfigEntry("optimization.enable_gelu", "1");

        // Enable memory-efficient attention (similar to FlashAttention)
        so.AddConfigEntry("optimization.enable_memory_efficient_attention", "1");
        
        // Enable cross attention optimizations (if your model uses it)
        so.AddConfigEntry("optimization.enable_shape_optimization", "1");
        
        // Packed QKV optimization (fuses Q, K, V projections)
        so.AddConfigEntry("optimization.enable_packed_qkv", "1");
        
        // Enable GPU attention optimizations (when using CUDA/TensorRT)
        so.AddConfigEntry("optimization.enable_gpu_attention", "1");

        // TensorRT specific fused multi-head attention
        // so.AddConfigEntry("trt_fused_multihead_attention", "1");
    }
    // ---- EP constructors ----

    // TensorRT via legacy C API (device_id only). Works with your headers.
    std::unique_ptr<Ort::Session> create_session_tensorrt(const std::string& model_path, const std::string& precision) {
        try {
            Ort::SessionOptions so;
            // so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

            // Basic optimizations
            so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
            so.SetIntraOpNumThreads(1);
            so.SetInterOpNumThreads(1);
            so.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
            
            // Transformer-specific optimizations
            enable_transformer_optimizations(so);
            
            // Memory optimizations
            so.AddConfigEntry("memory.optimize_memory", "1");
            so.AddConfigEntry("enable_cpu_mem_arena", "1");


            // Use device 0; if you need to select another GPU, change this int.
            Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_Tensorrt(so, /*device_id*/ 0));

            // Set precision and other options via config entries
            if (precision == "fp16") {
                so.AddConfigEntry("trt_fp16_enable", "1");
                spdlog::info("[ORT] Using TensorRT EP with FP16 quantization");
            } 
            else if (precision == "int8") {
                so.AddConfigEntry("trt_int8_enable", "1");
                spdlog::info("[ORT] Using TensorRT EP with INT8 quantization");
                // Note: For INT8, you may need to set the calibration table path as well.
                // so.AddConfigEntry("trt_int8_calibration_table_name", "path_to_calibration_table");
            }

            // TensorRT attention-specific settings
            so.AddConfigEntry("trt_timing_cache_enable", "1");
            // so.AddConfigEntry("trt_context_memory_sharing_enable", "1");

            so.AddConfigEntry("trt_max_workspace_size", "1073741824");
            so.AddConfigEntry("trt_engine_cache_enable", "1");
            so.AddConfigEntry("trt_engine_cache_path", "./trt_cache");

            auto sess = std::make_unique<Ort::Session>(env, model_path.c_str(), so);
            spdlog::info("[ORT] Using TensorRT EP");
            return sess;
        } catch (const Ort::Exception& e) {
            spdlog::warn("[ORT] TensorRT EP failed: {}", e.what());
            return nullptr;
        }
    }

    std::unique_ptr<Ort::Session> create_session_cuda(const std::string& model_path) {
        try {
            Ort::SessionOptions so;
            so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

            OrtCUDAProviderOptions cuda{};
            cuda.device_id = 0;
            cuda.arena_extend_strategy = 0;
            cuda.gpu_mem_limit = 0;
            cuda.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchExhaustive;
            cuda.do_copy_in_default_stream = 1;

            so.AppendExecutionProvider_CUDA(cuda);

            auto sess = std::make_unique<Ort::Session>(env, model_path.c_str(), so);
            spdlog::info("[ORT] Using CUDA EP");
            return sess;
        } catch (const Ort::Exception& e) {
            spdlog::warn("[ORT] CUDA EP failed: {}", e.what());
            return nullptr;
        }
    }

    std::unique_ptr<Ort::Session> create_session_cpu(const std::string& model_path) {
        try {
            Ort::SessionOptions so;
            so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
            auto sess = std::make_unique<Ort::Session>(env, model_path.c_str(), so);
            spdlog::info("[ORT] Using CPU EP");
            return sess;
        } catch (const Ort::Exception& e) {
            spdlog::warn("[ORT] CPU EP failed: {}", e.what());
            return nullptr;
        }
    }

private:
    Ort::Env env;
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;
    Ort::AllocatorWithDefaultOptions allocator;

    std::vector<const char*> input_names;
    std::vector<const char*> output_names;

    std::vector<std::vector<int64_t>> input_shapes;
    std::vector<int64_t> input_sizes;
    std::vector<int64_t> output_shape;
};
};
