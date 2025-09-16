// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "onnxruntime_cxx_api.h"
#include "onnxruntime_c_api.h"
#include <mutex>

namespace isaaclab
{

class Algorithms
{
public:
    virtual std::vector<float> act(std::vector<float> obs) = 0;
    
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
                std::cerr << "[ORT] Falling back to CUDA EP\n";
                session = create_session_cuda(model_path);
            }
            if (!session) {
                std::cerr << "[ORT] Falling back to CPU EP\n";
                session = create_session_cpu(model_path);
            }
        #endif

        Ort::TypeInfo input_type = session->GetInputTypeInfo(0);
        input_shape = input_type.GetTensorTypeAndShapeInfo().GetShape();
        Ort::TypeInfo output_type = session->GetOutputTypeInfo(0);
        output_shape = output_type.GetTensorTypeAndShapeInfo().GetShape();

        action.resize(output_shape[1]);
    }

    std::vector<float> act(std::vector<float> obs)
    {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        auto input_tensor = Ort::Value::CreateTensor<float>(memory_info, obs.data(), obs.size(), input_shape.data(), input_shape.size());
        auto output_tensor = session->Run(Ort::RunOptions{nullptr}, input_names.data(), &input_tensor, 1, output_names.data(), 1);
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
            std::cerr << "[ORT] Using TensorRT EP\n";
            return sess;
        } catch (const Ort::Exception& e) {
            std::cerr << "[ORT] TensorRT EP failed: " << e.what() << "\n";
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
            std::cerr << "[ORT] Using CUDA EP\n";
            return sess;
        } catch (const Ort::Exception& e) {
            std::cerr << "[ORT] CUDA EP failed: " << e.what() << "\n";
            return nullptr;
        }
    }

    std::unique_ptr<Ort::Session> create_session_cpu(const std::string& model_path) {
        try {
            Ort::SessionOptions so;
            so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
            auto sess = std::make_unique<Ort::Session>(env, model_path.c_str(), so);
            std::cerr << "[ORT] Using CPU EP\n";
            return sess;
        } catch (const Ort::Exception& e) {
            std::cerr << "[ORT] CPU EP failed: " << e.what() << "\n";
            return nullptr;
        }
    }

private:
    Ort::Env env;
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;
    Ort::AllocatorWithDefaultOptions allocator;

    const std::vector<const char*> input_names = {"obs"};
    const std::vector<const char*> output_names = {"actions"};

    std::vector<int64_t> input_shape;
    std::vector<int64_t> output_shape;
};

};