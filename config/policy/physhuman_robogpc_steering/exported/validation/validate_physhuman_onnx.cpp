/* Standalone CPU ONNX Runtime check for the physhuman_robogpc export.
 * It exercises the existing StatefulOrtRunner's four positional outputs and
 * recurrent tensor wiring without starting ROS or commanding a robot.
 * Requires C++17 and an unpacked ONNX Runtime C/C++ distribution (ORT_DIR).
 * Build from the protomotions-uniphys repository root:
 *   ORT_DIR=/path/to/onnxruntime-linux-x64-1.22.0
 *   g++ -std=c++17 -O2 -I"$ORT_DIR/include" \
 *     deployment/validate_physhuman_onnx.cpp \
 *     "$ORT_DIR/lib/libonnxruntime.so.1.22.0" \
 *     -Wl,-rpath,"$ORT_DIR/lib" -o /tmp/validate_physhuman_onnx
 * Run:
 *   /tmp/validate_physhuman_onnx \
 *     output/onnx/physhuman_robogpc/exported/policy.onnx
 * Supports exec_steps=1/5 base policies and the exec_steps=5 steering policy.
 * The fixture varies measured joint state and updates last_action;
 * this verifies the runtime contract, not closed-loop physical performance.
 */

#include <onnxruntime_cxx_api.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
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
    std::size_t obs_dim_ = 0;
    std::size_t exec_steps_ = 0;
    bool steering_ = false;
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
        const std::array<std::string, 4> expected_inputs{
            "obs", "plan_cache_flat", "plan_idx", "plan_valid"};
        const std::array<std::string, 4> expected_outputs{
            "actions", "plan_cache_flat_out", "plan_idx_out", "plan_valid_out"};
        for (std::size_t i = 0; i < session_.GetInputCount(); ++i) {
            input_storage_.emplace_back(session_.GetInputNameAllocated(i, allocator).get());
            if (input_storage_.back() != expected_inputs[i])
                throw std::runtime_error("Unexpected stateful input name/order");
            shapes_.push_back(session_.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
            const auto elements = count(shapes_.back());
            const auto expected_type = i == 2 ? ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64
                                               : ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT;
            if (session_.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetElementType() != expected_type)
                throw std::runtime_error("Unexpected stateful input dtype");
            if ((i < 2 && (shapes_.back().size() != 2 || shapes_.back()[0] != 1))
                || (i >= 2 && shapes_.back() != std::vector<int64_t>{1}))
                throw std::runtime_error("Expected batch-one observations/cache and one-element index/valid");
            if (input_storage_.back() == "plan_cache_flat") cache_.assign(elements, 0.0f);
            if (input_storage_.back() == "obs") obs_dim_ = elements;
        }
        steering_ = obs_dim_ == 593;
        if (steering_) {
            exec_steps_ = 5;
        } else {
            if (obs_dim_ <= 480 || (obs_dim_ - 480) % 109 != 0)
                throw std::runtime_error("Expected base obs589/1025 or steering obs593");
            exec_steps_ = (obs_dim_ - 480) / 109;
        }
        if (exec_steps_ != 1 && exec_steps_ != 5)
            throw std::runtime_error("This fixture supports exec_steps=1 or exec_steps=5");
        const auto expected_cache = exec_steps_ == 1 ? 80 : 80 + 16 * exec_steps_;
        if (cache_.size() != expected_cache)
            throw std::runtime_error("Cache width does not match latent history plus execution plan");
        for (std::size_t i = 0; i < session_.GetOutputCount(); ++i) {
            output_storage_.emplace_back(session_.GetOutputNameAllocated(i, allocator).get());
            if (output_storage_.back() != expected_outputs[i])
                throw std::runtime_error("Unexpected stateful output name/order");
            const auto expected_type = i == 2 ? ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64
                                               : ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT;
            if (session_.GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetElementType() != expected_type)
                throw std::runtime_error("Unexpected stateful output dtype");
            const auto shape = session_.GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
            const auto expected_shape = i == 0 ? std::vector<int64_t>{1, 29} : shapes_[i];
            if (shape != expected_shape)
                throw std::runtime_error("Unexpected stateful output shape");
        }
        for (const auto& name : input_storage_) input_names_.push_back(name.c_str());
        for (const auto& name : output_storage_) output_names_.push_back(name.c_str());
    }
    std::size_t obs_dim() const { return obs_dim_; }
    std::size_t exec_steps() const { return exec_steps_; }
    bool steering() const { return steering_; }
    std::size_t noise_offset() const { return steering_ ? 484 : 480; }
    const std::vector<float>& cache() const { return cache_; }
    int64_t index() const { return index_[0]; }
    void reset() {
        std::fill(cache_.begin(), cache_.end(), 0.0f);
        index_[0] = 0;
        valid_[0] = 0;
    }
    std::vector<float> act(std::vector<float>& obs) {
        if (obs.size() != obs_dim_) throw std::runtime_error("Observation width differs from model");
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
        if (valid_[0] != 1.0f) throw std::runtime_error("Expected initialized output cache");
        for (auto x : actions) if (!std::isfinite(x)) throw std::runtime_error("Nonfinite action");
        for (auto x : cache_) if (!std::isfinite(x)) throw std::runtime_error("Nonfinite cache");
        return actions;
    }
};

static void print_latencies(const char* phase, std::vector<double> latencies) {
    if (latencies.empty()) return;
    std::sort(latencies.begin(), latencies.end());
    double total_ms = 0;
    for (auto ms : latencies) total_ms += ms;
    const auto middle = latencies.size() / 2;
    const auto median = latencies.size() % 2 == 0
        ? (latencies[middle - 1] + latencies[middle]) / 2 : latencies[middle];
    std::cout << "Latency single-thread CPU phase=" << phase
              << " calls=" << latencies.size()
              << " min_ms=" << latencies.front()
              << " median_ms=" << median
              << " mean_ms=" << total_ms / latencies.size()
              << " max_ms=" << latencies.back() << '\n';
}

int main(int argc, char** argv) {
    try {
        if (argc != 2) throw std::runtime_error("Usage: validate_physhuman_onnx policy.onnx");
        std::cout << "ONNX Runtime " << OrtGetApiBase()->GetVersionString() << " CPU\n";
        Runner runner(argv[1]);
        std::cout << "obs_dim=" << runner.obs_dim() << " cache_dim=" << runner.cache().size()
                  << " exec_steps=" << runner.exec_steps() << '\n';
        std::vector<std::vector<float>> first_run;
        std::vector<std::vector<float>> first_caches;
        std::vector<double> latencies, bootstrap_latencies, replan_latencies, cached_latencies;
        double max_error = 0, max_cache_error = 0;
        bool saw_unbounded_epsilon = false;
        for (int episode = 0; episode < 2; ++episode) {
            runner.reset();
            std::mt19937 generator(42);
            std::normal_distribution<float> gaussian(0.0f, 1.0f);
            std::vector<float> obs(runner.obs_dim(), 0.0f);
            for (int h = 0; h < 5; ++h) obs[30 + 3*h + 2] = -1.0f;
            for (int tick = 0; tick < 17; ++tick) {
                for (std::size_t n = runner.noise_offset(); n < runner.obs_dim(); ++n) {
                    obs[n] = runner.steering() ? gaussian(generator)
                        : 0.5f * std::sin(float(n - 480 + 17*tick));
                    saw_unbounded_epsilon |= runner.steering() && std::abs(obs[n]) > 1.0f;
                }
                if (runner.steering()) {
                    const float angle = 0.15f * tick;
                    obs[480] = 0.4f * std::cos(angle);
                    obs[481] = 0.2f * std::sin(angle);
                    obs[482] = std::cos(angle);
                    obs[483] = std::sin(angle);
                }
                // Measured joint position/velocity change even on cached-plan ticks.
                for (const auto offset : {45, 190}) {
                    std::move(obs.begin()+offset+29, obs.begin()+offset+145, obs.begin()+offset);
                    for (int joint = 0; joint < 29; ++joint)
                        obs[offset+116+joint] = 0.02f * std::sin(float(joint + tick));
                }
                const auto start = std::chrono::steady_clock::now();
                auto action = runner.act(obs);
                const auto stop = std::chrono::steady_clock::now();
                const auto milliseconds = std::chrono::duration<double, std::milli>(stop-start).count();
                const bool bootstrap = tick < 5;
                const bool replan = !bootstrap && (tick - 5) % runner.exec_steps() == 0;
                const auto expected_index = bootstrap ? tick + 1
                    : 5 + (tick - 5 + 1) % runner.exec_steps();
                if (runner.index() != static_cast<int64_t>(expected_index))
                    throw std::runtime_error("Unexpected bootstrap/plan index transition");
                const auto phase = bootstrap ? "bootstrap" : replan ? "replan" : "cached";
                std::cout << "episode=" << episode << " tick=" << tick << " phase=" << phase
                          << " plan_idx=" << runner.index() << " ms=" << milliseconds << '\n';
                latencies.push_back(milliseconds);
                (bootstrap ? bootstrap_latencies : replan ? replan_latencies : cached_latencies)
                    .push_back(milliseconds);
                if (episode == 0) {
                    first_run.push_back(action);
                    first_caches.push_back(runner.cache());
                } else {
                    for (int j = 0; j < 29; ++j)
                        max_error = std::max(max_error, double(std::abs(action[j] - first_run[tick][j])));
                    for (std::size_t j = 0; j < runner.cache().size(); ++j)
                        max_cache_error = std::max(max_cache_error,
                            double(std::abs(runner.cache()[j] - first_caches[tick][j])));
                }
                // Advance last_action history exactly as ObservationManager does.
                std::move(obs.begin()+335+29, obs.begin()+480, obs.begin()+335);
                std::copy(action.begin(), action.end(), obs.begin()+451);
            }
        }
        if (max_error > 1e-6 || max_cache_error > 1e-6)
            throw std::runtime_error("Reset/replay differs");
        if (runner.steering() && !saw_unbounded_epsilon)
            throw std::runtime_error("Steering fixture failed to exercise unclipped Gaussian epsilon");
        print_latencies("all", latencies);
        print_latencies("bootstrap", bootstrap_latencies);
        print_latencies("replan", replan_latencies);
        print_latencies("cached", cached_latencies);
        std::cout << "PASS: 34 calls, static deployment I/O, finite actions/cache, expected index sequence, "
                     "two episodes, reset replay action_max_abs_error=" << max_error
                  << " cache_max_abs_error=" << max_cache_error << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
