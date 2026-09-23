#pragma once

#include "FSM/FSMState.h"
#include "scalebfm/controller.hpp"
#include "isaaclab/envs/external_steering.h"
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

// Separate FSM path: the ONNX graph owns normalization/action scaling and the
// ScaleBFM core owns history/reference generation. No generic RL manager runs.
class State_ScaleBFM : public FSMState {
 public:
  State_ScaleBFM(int state, std::string name);
  ~State_ScaleBFM() { exit(); if (fault_dump_thread_.joinable()) fault_dump_thread_.join(); }
  void enter() override;
  void run() override;
  void exit() override;

 private:
  using Clock = std::chrono::steady_clock;
  struct Frame {
    scalebfm::State state;
    Eigen::Vector3f joystick = Eigen::Vector3f::Zero();
    Eigen::Vector3f pelvis_linear_velocity_b = Eigen::Vector3f::Zero();
    bool sampled = false;
  };
  struct TraceTick {
    Frame frame;
    Eigen::Vector3f command = Eigen::Vector3f::Zero();
    Eigen::Vector3f root_reference = Eigen::Vector3f::Zero();
    Eigen::Vector2f effective_velocity_world = Eigen::Vector2f::Zero();
    scalebfm::Inputs inputs;
    scalebfm::PDDiagnostics pd;
    double monotonic_s = 0, step_ms = 0, inference_ms = 0;
    double scheduled_wake_monotonic_s = 0, next_wake_monotonic_s = 0;
    double actual_start_gap_ms = 0, sample_interval_ms = 0;
    double wake_lateness_ms = 0, window_wake_delay_ms = 0, full_work_ms = 0;
    bool timing_available = false, first_control_tick = false, timing_accepted = false;
    bool timing_guard_faulted = false;
    bool sample_interval_available = false;
    bool core_snapshot = false;
    std::string phase, fault;
  };
  Frame capture(Frame* diagnostic = nullptr);
  void policy_loop();
  void fail(const std::string& reason);
  void set_pending_trace(const TraceTick& trace);
  std::size_t complete_trace(TraceTick trace, Clock::time_point start, bool core_snapshot);
  void finish_trace_timing(std::size_t index, const TraceTick& trace);
  void dump_fault_async(const std::string& reason);
  void passive_damping();
  void wrist_command(geometry_msgs::msg::PoseArray::ConstSharedPtr message);
  std::unique_ptr<scalebfm::Controller> controller_;
  rclcpp::Node::SharedPtr node_;
  std::atomic<bool> running_{false}, fault_{true};
  std::atomic<std::uint64_t> wrist_generation_{0};
  std::thread policy_thread_;
  std::thread fault_dump_thread_;
  std::shared_ptr<std::atomic<bool>> fault_dump_done_ = std::make_shared<std::atomic<bool>>(true);
  isaaclab::ExternalSteering steering_;
  bool ros_steering_ = false, wrists_enabled_ = false;
  float state_timeout_s_ = .2f, action_timeout_s_ = .1f;
  float target_rate_notice_rad_s_ = 30.f, max_target_lead_rad_ = 1.5f;
  std::mutex action_mutex_, wrist_mutex_;
  std::mutex trace_mutex_;
  std::array<TraceTick, 100> trace_ring_{};
  std::size_t trace_next_ = 0, trace_count_ = 0;
  std::optional<TraceTick> pending_trace_;
  scalebfm::Joints latest_targets_{};
  Clock::time_point action_time_{};
  scalebfm::WristTargets wrists_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr steering_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr wrists_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
};

REGISTER_FSM(State_ScaleBFM)
