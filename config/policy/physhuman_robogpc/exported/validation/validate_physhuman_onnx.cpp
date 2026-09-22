// Standalone CPU ONNX Runtime check for the physhuman_robogpc export.
// It exercises the existing StatefulOrtRunner's four positional outputs and
// recurrent tensor wiring without starting ROS or commanding a robot.
// Requires C++17 and an unpacked ONNX Runtime C/C++ distribution (ORT_DIR).
// Build from the protomotions-uniphys repository root:
//   ORT_DIR=/path/to/onnxruntime-linux-x64-1.22.0
//   g++ -std=c++17 -O2 -I"$ORT_DIR/include" \
//     deployment/validate_physhuman_onnx.cpp \
//     "$ORT_DIR/lib/libonnxruntime.so.1.22.0" \
//     -Wl,-rpath,"$ORT_DIR/lib" -o /tmp/validate_physhuman_onnx
// Run:
//   /tmp/validate_physhuman_onnx \
//     output/onnx/physhuman_robogpc/exported/policy.onnx
// The fixture holds measured joints at nominal pose and updates last_action;
// this verifies the runtime contract, not closed-loop physical performance.

#include <onnxruntime_cxx_api.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

static std::size_t count(const std::vector<int64_t>& shape) {
    std::size_t n = 1;
    for (auto dimension : shape) {
        if (dimension <= 0) throw std::runtime_error("All dimensions must be static and positive");
        n *= static_cast<std::size_t>(dimension);
    }
    return n;
}

class Runner {
    Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "physhuman-contract-audit"};
    Ort::Session session_{nullptr};
    std::vector<std::string> input_storage_, output_storage_;
    std::vector<const char*> input_names_, output_names_;
    std::vector<std::vector<int64_t>> shapes_;
    std::vector<float> cache_;
    std::array<int64_t, 1> index_{};
    std::array<float, 1> valid_{};
public:
    explicit Runner(const char* model) {
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);
        options.SetInterOpNumThreads(1);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session_ = Ort::Session(env_, model, options);
        Ort::AllocatorWithDefaultOptions allocator;
        if (session_.GetInputCount() != 4 || session_.GetOutputCount() != 4)
            throw std::runtime_error("Expected four stateful inputs and four positional outputs");
        for (std::size_t i = 0; i < session_.GetInputCount(); ++i) {
            input_storage_.emplace_back(session_.GetInputNameAllocated(i, allocator).get());
            shapes_.push_back(session_.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
            const auto elements = count(shapes_.back());
            if (input_storage_.back() == "plan_cache_flat") cache_.assign(elements, 0.0f);
            if (input_storage_.back() == "obs" && (shapes_.back() != std::vector<int64_t>{1, 589}))
                throw std::runtime_error("Expected obs float32[1,589]");
        }
        if (cache_.size() != 80) throw std::runtime_error("Expected 5x16 latent cache");
        for (std::size_t i = 0; i < session_.GetOutputCount(); ++i) {
            output_storage_.emplace_back(session_.GetOutputNameAllocated(i, allocator).get());
        }
        for (const auto& name : input_storage_) input_names_.push_back(name.c_str());
        for (const auto& name : output_storage_) output_names_.push_back(name.c_str());
    }
    void reset() {
        std::fill(cache_.begin(), cache_.end(), 0.0f);
        index_[0] = 0;
        valid_[0] = 0;
    }
    std::vector<float> act(std::vector<float>& obs) {
        auto memory = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        std::vector<Ort::Value> inputs;
        for (std::size_t i = 0; i < input_names_.size(); ++i) {
            const auto& name = input_storage_[i];
            if (name == "plan_idx") {
                inputs.push_back(Ort::Value::CreateTensor<int64_t>(memory, index_.data(), index_.size(), shapes_[i].data(), shapes_[i].size()));
            } else {
                float* data = nullptr;
                std::size_t size = 0;
                if (name == "obs") { data = obs.data(); size = obs.size(); }
                else if (name == "plan_cache_flat") { data = cache_.data(); size = cache_.size(); }
                else if (name == "plan_valid") { data = valid_.data(); size = valid_.size(); }
                else throw std::runtime_error("Unsupported stateful input " + name);
                inputs.push_back(Ort::Value::CreateTensor<float>(memory, data, size, shapes_[i].data(), shapes_[i].size()));
            }
        }
        auto outputs = session_.Run(Ort::RunOptions{nullptr}, input_names_.data(), inputs.data(), inputs.size(), output_names_.data(), output_names_.size());
        if (outputs[0].GetTensorTypeAndShapeInfo().GetShape() != std::vector<int64_t>{1,29})
            throw std::runtime_error("Expected rank-2 actions[1,29] as first output");
        if (outputs[1].GetTensorTypeAndShapeInfo().GetElementCount() != cache_.size())
            throw std::runtime_error("Cache output/input widths differ");
        const auto* action_data = outputs[0].GetTensorData<float>();
        std::vector<float> actions(action_data, action_data + 29);
        const auto* cache_data = outputs[1].GetTensorData<float>();
        cache_.assign(cache_data, cache_data + cache_.size());
        index_[0] = outputs[2].GetTensorData<int64_t>()[0];
        valid_[0] = outputs[3].GetTensorData<float>()[0];
        for (auto x : actions) if (!std::isfinite(x)) throw std::runtime_error("Nonfinite action");
        for (auto x : cache_) if (!std::isfinite(x)) throw std::runtime_error("Nonfinite cache");
        std::cout << "  cache_count=" << index_[0] << " valid=" << valid_[0] << " action_linf=";
        float maximum = 0;
        for (auto x : actions) maximum = std::max(maximum, std::abs(x));
        std::cout << maximum;
        return actions;
    }
};

int main(int argc, char** argv) {
    try {
        if (argc != 2) throw std::runtime_error("Usage: validate_physhuman_onnx policy.onnx");
        std::cout << "ONNX Runtime " << OrtGetApiBase()->GetVersionString() << " CPU\n";
        Runner runner(argv[1]);
        std::vector<std::vector<float>> first_run;
        std::vector<double> latencies;
        double max_error = 0;
        for (int episode = 0; episode < 2; ++episode) {
            runner.reset();
            std::vector<float> obs(589, 0.0f);
            for (int h = 0; h < 5; ++h) obs[30 + 3*h + 2] = -1.0f;
            for (int tick = 0; tick < 8; ++tick) {
                for (int n = 0; n < 109; ++n) obs[480+n] = 0.5f * std::sin(float(n + 17*tick));
                std::cout << "episode=" << episode << " tick=" << tick;
                const auto start = std::chrono::steady_clock::now();
                auto action = runner.act(obs);
                const auto stop = std::chrono::steady_clock::now();
                const auto milliseconds = std::chrono::duration<double, std::milli>(stop-start).count();
                std::cout << " ms=" << milliseconds << '\n';
                latencies.push_back(milliseconds);
                if (episode == 0) first_run.push_back(action);
                else for (int j = 0; j < 29; ++j) max_error = std::max(max_error, double(std::abs(action[j] - first_run[tick][j])));
                // Advance last_action history exactly as ObservationManager does.
                std::move(obs.begin()+335+29, obs.begin()+480, obs.begin()+335);
                std::copy(action.begin(), action.end(), obs.begin()+451);
            }
        }
        if (max_error > 1e-6) throw std::runtime_error("Reset/replay differs");
        std::sort(latencies.begin(), latencies.end());
        double total_ms = 0;
        for (auto ms : latencies) total_ms += ms;
        std::cout << "Latency single-thread CPU: min_ms=" << latencies.front()
                  << " median_ms=" << (latencies[7] + latencies[8]) / 2
                  << " mean_ms=" << total_ms / latencies.size()
                  << " max_ms=" << latencies.back() << '\n';
        std::cout << "PASS: 16 calls, static deployment I/O, finite actions/cache, two episodes, reset replay max_abs_error=" << max_error << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
