// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "onnxruntime_cxx_api.h"
#include <iostream>
#include <mutex>

namespace isaaclab
{

class Algorithms
{
public:
    virtual std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs) = 0;

    std::vector<float> get_action()
    {
        std::lock_guard<std::mutex> lock(act_mtx_);
        return action;
    }
    
    std::vector<float> action;
protected:
    std::mutex act_mtx_;
};

class OrtRunner : public Algorithms
{
public:
    OrtRunner(std::string model_path)
    {
        // Init Model
        env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "onnx_model");

        #if defined(__aarch64__)
            session = create_session_cpu(model_path);
        #else
            session = create_session_tensorrt(model_path);
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
    // ---- EP constructors ----

    // TensorRT via legacy C API (device_id only). Works with your headers.
    std::unique_ptr<Ort::Session> create_session_tensorrt(const std::string& model_path) {
        try {
            Ort::SessionOptions so;
            so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

            // Use device 0; if you need to select another GPU, change this int.
            Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_Tensorrt(so, /*device_id*/ 0));

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