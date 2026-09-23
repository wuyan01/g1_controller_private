#include "FSM/State_ScaleBFM.h"
#include "FSM/State_RLBase.h"
#include "scalebfm/entry_guard.hpp"
#include "scalebfm/control_schedule.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <unistd.h>

namespace {
template <class Values>
YAML::Node sequence(const Values& values) {
  YAML::Node node(YAML::NodeType::Sequence);
  for (std::size_t i = 0; i < static_cast<std::size_t>(values.size()); ++i)
    node.push_back(values[i]);
  node.SetStyle(YAML::EmitterStyle::Flow);
  return node;
}
YAML::Node quaternion(const Eigen::Quaternionf& value) {
  return sequence(std::array<float, 4>{value.w(), value.x(), value.y(), value.z()});
}
}  // namespace

State_ScaleBFM::State_ScaleBFM(int state, std::string name) : FSMState(state, name) {
  node_ = State_RLBase::node_;
  if (!node_) throw std::runtime_error("ScaleBFM requires the controller ROS node");
  const auto config = param::config["FSM"][name];
  bool simulation = false;
  node_->get_parameter_or("simulation", simulation, false);
  if (!simulation && !config["allow_hardware"].as<bool>(false))
    throw std::runtime_error("ScaleBFM hardware is unqualified; this profile permits simulation only");
  if (network_interface != "lo" && !config["allow_hardware"].as<bool>(false))
    throw std::runtime_error("ScaleBFM unqualified profile requires the loopback simulator interface");
  // This initial deployment uses the SDK pelvis pose and IMU in the same world
  // frame. An external localization frame needs an explicit IMU-heading transform.
  const auto topics = node_->get_node_topics_interface();
  if (topics->resolve_topic_name("pose_source/odom") != topics->resolve_topic_name("sport_mode/odom"))
    throw std::runtime_error("ScaleBFM currently requires sport_mode/odom; external world-frame alignment is unsupported");
  lowstate->set_timeout_ms(200);
  sportmodestate->set_timeout_ms(200);
  const auto root = std::filesystem::path(ament_index_cpp::get_package_share_directory("g1_controller"));
  const auto policy_dir = root / config["policy_dir"].as<std::string>();
  scalebfm::Options options;
  options.inference_threads = config["inference_threads"].as<int>(2);
  options.max_target_lead_rad = config["max_target_lead_rad"].as<float>(1.5f);
  if (config["target_rate_notice_rad_s"]) {
    options.target_rate_notice_rad_s = config["target_rate_notice_rad_s"].as<float>();
  } else if (config["max_target_slew_rad_s"]) {
    options.target_rate_notice_rad_s = config["max_target_slew_rad_s"].as<float>();
    spdlog::warn("[ScaleBFM] legacy max_target_slew_rad_s is now a diagnostic notice threshold only; "
                 "rename it to target_rate_notice_rad_s. Physical and timing guards remain active.");
  }
  target_rate_notice_rad_s_ = options.target_rate_notice_rad_s;
  max_target_lead_rad_ = options.max_target_lead_rad;
  controller_ = std::make_unique<scalebfm::Controller>(
      (policy_dir / "exported/policy.onnx").string(),
      (policy_dir / "exported/deployment.yaml").string(), options);
  const auto configured_names = param::config["robot"]["joint_names"].as<std::vector<std::string>>();
  if (configured_names != controller_->metadata().sdk_joint_names)
    throw std::runtime_error("ScaleBFM SDK joint names disagree with robot configuration");
  ros_steering_ = node_->get_parameter("steering_ros_enabled").as_bool();
  wrists_enabled_ = config["enable_wrist_targets"].as<bool>(false);
  ready_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/codex/steering_ready", 1);
  steering_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/codex/steering_command", rclcpp::QoS(1).best_effort(),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        const auto& t = msg->twist;
        const double age = (node_->now() - rclcpp::Time(msg->header.stamp)).seconds();
        if (msg->header.frame_id != "heading" || !std::isfinite(age) || age < 0 || age > .2 ||
            !std::isfinite(t.linear.x) || !std::isfinite(t.linear.y) || !std::isfinite(t.angular.z) ||
            t.linear.z != 0 || t.angular.x != 0 || t.angular.y != 0) {
          steering_.receive(0, 0, 0);
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                              "ScaleBFM rejects steering: require a fresh heading-frame planar twist");
          return;
        }
        steering_.receive(t.linear.x, t.linear.y, t.angular.z);
      });
  wrists_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
      "/scalebfm/wrist_targets", 1,
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg) { wrist_command(msg); });
  clear_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      "/scalebfm/clear_wrist_targets", 1, [this](std_msgs::msg::Empty::ConstSharedPtr) {
        std::lock_guard<std::mutex> lock(wrist_mutex_); wrists_ = {};
      });
  // A controller fault is checked before user-requested transitions, including
  // stale/invalid inputs and output guards. The current run also damps at once.
  registered_checks.insert(registered_checks.begin(), {
      [this] { return fault_.load(); }, FSMStringMap.right.at("Passive")});
  spdlog::info("[ScaleBFM] adapter=bounded_timing_v4 actor ONNX={} checkpoint={} control_hz=50 physics_history=3 "
               "wrist_servo=false wrist_targets={} lead_limit_rad={} target_rate_notice_rad_s={}",
               controller_->metadata().onnx_sha256, controller_->metadata().checkpoint_sha256,
               wrists_enabled_, options.max_target_lead_rad, options.target_rate_notice_rad_s);
  spdlog::info("[ScaleBFM] schedule=actual_start_plus_20ms no_catchup=true "
               "max_start_gap_ms=40 max_rolling_wake_delay_ms=50 wake_delay_window_ms=1000 "
               "wake_notice_ms=5 max_full_work_ms=20 max_action_age_ms=100 max_state_age_ms=200");
}

State_ScaleBFM::Frame State_ScaleBFM::capture(Frame* diagnostic) {
  if (lowstate->isTimeout()) throw std::runtime_error("ScaleBFM LowState timed out");
  // The ROS republisher stamps cached SportModeState each FSM tick. Check the
  // underlying DDS freshness as well as the ROS stamp below.
  if (sportmodestate->isTimeout()) throw std::runtime_error("ScaleBFM native pelvis position timed out");
  Frame frame;
  {
    std::lock_guard<std::mutex> lock(lowstate->mutex_);
    const auto& imu = lowstate->msg_.imu_state();
    frame.state.pelvis_quat_w = Eigen::Quaternionf(imu.quaternion()[0], imu.quaternion()[1],
                                                   imu.quaternion()[2], imu.quaternion()[3]);
    for (int i = 0; i < 3; ++i) frame.state.pelvis_ang_vel_b[i] = imu.gyroscope()[i];
    for (int i = 0; i < 29; ++i) {
      frame.state.joint_positions_sdk[i] = lowstate->msg_.motor_state()[i].q();
      frame.state.joint_velocities_sdk[i] = lowstate->msg_.motor_state()[i].dq();
    }
    frame.joystick = Eigen::Vector3f(lowstate->joystick.ly(), -lowstate->joystick.lx(),
                                    -lowstate->joystick.rx());
  }
  {
    std::lock_guard<std::mutex> lock(pelvisodometryfusion->mutex_);
    const auto& odom = pelvisodometryfusion->msg_;
    const double age = (node_->now() - rclcpp::Time(odom.header.stamp)).seconds();
    if (odom.child_frame_id != "pelvis" || odom.header.frame_id != "odom" ||
        !std::isfinite(age) || age < 0 || age > state_timeout_s_)
      throw std::runtime_error("ScaleBFM requires fresh odom-frame pelvis position");
    frame.state.pelvis_position_w = Eigen::Vector3f(odom.pose.pose.position.x,
        odom.pose.pose.position.y, odom.pose.pose.position.z);
    frame.pelvis_linear_velocity_b = Eigen::Vector3f(odom.twist.twist.linear.x,
        odom.twist.twist.linear.y, odom.twist.twist.linear.z);
  }
  frame.sampled = true;
  if (diagnostic) *diagnostic = frame;
  const auto& q = frame.state.pelvis_quat_w;
  if (!q.coeffs().allFinite() || std::abs(q.norm() - 1.f) > .01f ||
      !frame.state.pelvis_position_w.allFinite() || !frame.joystick.allFinite() ||
      frame.state.pelvis_position_w.z() < .5f ||
      (q * Eigen::Vector3f::UnitZ()).z() < std::cos(.65f))
    throw std::runtime_error("ScaleBFM invalid state, low pelvis or excessive tilt");
  return frame;
}

void State_ScaleBFM::enter() {
  exit();
  if (fault_dump_thread_.joinable() && fault_dump_done_->load(std::memory_order_acquire))
    fault_dump_thread_.join();
  {
    std::lock_guard<std::mutex> lock(trace_mutex_);
    trace_next_ = trace_count_ = 0;
    pending_trace_.reset();
  }
  fault_ = false;
  TraceTick entry;
  entry.phase = "entry";
  const auto entry_start = Clock::now();
  try {
    const auto initial = capture(&entry.frame);
    scalebfm::validate_entry(initial.state, initial.pelvis_linear_velocity_b,
        controller_->metadata(), controller_->forward_kinematics(initial.state));
    controller_->reset(initial.state);
    spdlog::info("[ScaleBFM] entry pelvis_xyz=({:.4f},{:.4f},{:.4f}) "
                 "pelvis_quat_wxyz=({:.4f},{:.4f},{:.4f},{:.4f}) "
                 "right_ankle_q={:.5f} right_ankle_dq={:.5f}",
                 initial.state.pelvis_position_w.x(), initial.state.pelvis_position_w.y(),
                 initial.state.pelvis_position_w.z(), initial.state.pelvis_quat_w.w(),
                 initial.state.pelvis_quat_w.x(), initial.state.pelvis_quat_w.y(),
                 initial.state.pelvis_quat_w.z(), initial.state.joint_positions_sdk[10],
                 initial.state.joint_velocities_sdk[10]);
    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      latest_targets_ = initial.state.joint_positions_sdk;
      action_time_ = Clock::now();
    }
    {
      std::lock_guard<std::mutex> lock(wrist_mutex_); wrists_ = {};
    }
    const auto kp = controller_->metadata().kp_sdk(), kd = controller_->metadata().kd_sdk();
    for (int i = 0; i < 29; ++i) {
      auto& motor = lowcmd->msg_.motor_cmd()[i];
      motor.kp() = kp[i]; motor.kd() = kd[i]; motor.dq() = 0; motor.tau() = 0;
      motor.q() = initial.state.joint_positions_sdk[i];
    }
    steering_.activate(ros_steering_);
    running_ = true;
    policy_thread_ = std::thread(&State_ScaleBFM::policy_loop, this);
  } catch (const std::exception& e) {
    entry.fault = e.what();
    complete_trace(entry, entry_start, false);
    passive_damping(); fail(e.what());
  }
}

void State_ScaleBFM::policy_loop() {
  scalebfm::ControlSchedule schedule(Clock::now());
  auto last_notice = Clock::time_point{};
  auto last_wake_notice = Clock::time_point{};
  std::optional<Clock::time_point> previous_sample;
  std::uint64_t wake_notice_ticks = 0;
  std::uint64_t notice_ticks = 0;
  float maximum_noticed_rate = 0;
  while (running_ && !fault_) {
    const auto start = Clock::now();
    TraceTick trace;
    trace.phase = "before_capture_timing_check";
    trace.scheduled_wake_monotonic_s =
        std::chrono::duration<double>(schedule.next_wake().time_since_epoch()).count();
    trace.monotonic_s = std::chrono::duration<double>(start.time_since_epoch()).count();
    set_pending_trace(trace);
    bool core_step_called = false;
    std::optional<std::size_t> trace_index;
    const auto copy_timing = [&] {
      const auto& timing = schedule.timing();
      trace.timing_available = true;
      trace.first_control_tick = timing.first_tick;
      trace.timing_accepted = timing.accepted;
      trace.timing_guard_faulted = schedule.faulted();
      trace.actual_start_gap_ms = timing.gap_ms;
      trace.wake_lateness_ms = timing.lateness_ms;
      trace.window_wake_delay_ms = timing.window_delay_ms;
      trace.full_work_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
      trace.next_wake_monotonic_s =
          std::chrono::duration<double>(schedule.next_wake().time_since_epoch()).count();
    };
    try {
      schedule.begin(start);
      copy_timing();
      trace.phase = "capture";
      set_pending_trace(trace);
      auto frame = capture(&trace.frame);
      const auto sampled_at = Clock::now();
      if (previous_sample) {
        trace.sample_interval_ms = std::chrono::duration<double, std::milli>(sampled_at - *previous_sample).count();
        trace.sample_interval_available = true;
      }
      previous_sample = sampled_at;
      // Joystick has a deadband before physical-unit scaling; moving it latches
      // takeover through the shared ExternalSteering implementation.
      for (int i = 0; i < 3; ++i) if (std::abs(frame.joystick[i]) <= .1f) frame.joystick[i] = 0;
      Eigen::Vector3f velocity = frame.joystick.cwiseProduct(Eigen::Vector3f(.3f, .3f, .6f));
      if (velocity.head<2>().norm() > .3f) velocity.head<2>() *= .3f / velocity.head<2>().norm();
      if (ros_steering_) {
        const bool moved = frame.joystick.head<2>().norm() > .1f || std::abs(frame.joystick.z()) > .1f;
        const auto external = steering_.sample(moved);
        if (steering_.ready()) velocity = Eigen::Vector3f(external[0], external[1], external[2]);
      }
      scalebfm::WristTargets goals;
      { std::lock_guard<std::mutex> lock(wrist_mutex_); goals = wrists_; }
      trace.command = velocity;
      trace.phase = "policy_in_progress";
      set_pending_trace(trace);
      core_step_called = true;
      auto targets = controller_->step(frame.state, velocity, goals);
      const auto end = Clock::now();
      trace.phase = "pre_publish_timing_check";
      schedule.finish(end);
      { std::lock_guard<std::mutex> lock(action_mutex_); latest_targets_ = targets; action_time_ = start; }
      std_msgs::msg::Bool ready; ready.data = steering_.ready(); ready_pub_->publish(ready);
      if (schedule.timing().noticed) {
        ++wake_notice_ticks;
        if (last_wake_notice == Clock::time_point{} || start - last_wake_notice >= std::chrono::seconds(2)) {
          spdlog::info("[ScaleBFM] wake-delay notice: gap_ms={:.3f} lateness_ms={:.3f} "
                       "rolling_delay_ms={:.3f}/50.000 noticed_ticks={}; bounded schedule continues",
                       trace.actual_start_gap_ms, trace.wake_lateness_ms,
                       trace.window_wake_delay_ms, wake_notice_ticks);
          last_wake_notice = start;
        }
      }
      const auto& diagnostic = controller_->last_pd_diagnostics();
      if (diagnostic.target_rate_notice_exceeded) {
        ++notice_ticks;
        maximum_noticed_rate = std::max(maximum_noticed_rate, diagnostic.max_abs_target_rate_rad_s);
        if (last_notice == Clock::time_point{} || start - last_notice >= std::chrono::seconds(2)) {
          spdlog::info("[ScaleBFM] PD target-rate notice: tick={} threshold_rad_s={:.3f} "
                       "current_max_rad_s={:.3f} noticed_ticks={} session_max_rad_s={:.3f}; "
                       "diagnostic only, physical and timing guards remain active",
                       controller_->control_steps(), target_rate_notice_rad_s_,
                       diagnostic.max_abs_target_rate_rad_s, notice_ticks, maximum_noticed_rate);
          last_notice = start;
        }
      }
      if (controller_->control_steps() == 1 || controller_->control_steps() % 250 == 0) {
        spdlog::info("[ScaleBFM] tick={} inference_ms={:.3f} step_ms={:.3f} "
                     "right_ankle_q={:.5f} right_ankle_q_des={:.5f} "
                     "right_ankle_requested_torque_nm={:.3f}",
                     controller_->control_steps(), controller_->inference_ms(),
                     std::chrono::duration<double, std::milli>(end - start).count(),
                     diagnostic.measured_q_sdk[10], diagnostic.requested_targets_sdk[10],
                     diagnostic.estimated_pd_torque_sdk[10]);
      }
      trace.phase = "full_work_timing_check";
      copy_timing();
      trace_index = complete_trace(trace, start, true);
      // Include target publication, notices, telemetry and the trace copy in the
      // work deadline. The earlier check also rejects slow inference before use.
      schedule.finish(Clock::now());
      copy_timing();
      trace.phase = "completed";
      finish_trace_timing(*trace_index, trace);
      trace.phase = "final_trace_timing_check";
      schedule.finish(Clock::now());  // Also bound the final timing annotation.
      // Every period starts at the actual accepted start; there is no catch-up.
      std::this_thread::sleep_until(schedule.next_wake());
    } catch (const std::exception& e) {
      copy_timing();
      trace.fault = e.what();
      if (!trace_index) trace_index = complete_trace(trace, start, core_step_called);
      finish_trace_timing(*trace_index, trace);
      fail(e.what());
    }
  }
}

void State_ScaleBFM::fail(const std::string& reason) {
  const bool first_fault = !fault_.exchange(true);
  running_ = false;
  steering_.activate(false);
  ready_pub_->publish(std_msgs::msg::Bool{});
  if (first_fault) {
    spdlog::error("[ScaleBFM] fault -> Passive: {}", reason);
    dump_fault_async(reason);
  }
}

void State_ScaleBFM::set_pending_trace(const TraceTick& trace) {
  std::lock_guard<std::mutex> lock(trace_mutex_);
  pending_trace_ = trace;
}

std::size_t State_ScaleBFM::complete_trace(TraceTick trace, Clock::time_point start,
                                   bool core_snapshot) {
  trace.monotonic_s = std::chrono::duration<double>(start.time_since_epoch()).count();
  trace.step_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
  trace.core_snapshot = core_snapshot;
  if (core_snapshot) {
    trace.inputs = controller_->last_inputs();
    trace.pd = controller_->last_pd_diagnostics();
    trace.root_reference = controller_->root_target();
    trace.effective_velocity_world = controller_->effective_velocity_world();
    trace.inference_ms = controller_->inference_ms();
  }
  std::lock_guard<std::mutex> lock(trace_mutex_);
  const auto index = trace_next_;
  trace_ring_[index] = std::move(trace);
  trace_next_ = (trace_next_ + 1) % trace_ring_.size();
  trace_count_ = std::min(trace_count_ + 1, trace_ring_.size());
  pending_trace_.reset();
  return index;
}

void State_ScaleBFM::finish_trace_timing(std::size_t index, const TraceTick& trace) {
  std::lock_guard<std::mutex> lock(trace_mutex_);
  auto& recorded = trace_ring_[index];
  recorded.phase = trace.phase;
  recorded.fault = trace.fault;
  recorded.step_ms = trace.full_work_ms;
  recorded.full_work_ms = trace.full_work_ms;
  recorded.scheduled_wake_monotonic_s = trace.scheduled_wake_monotonic_s;
  recorded.next_wake_monotonic_s = trace.next_wake_monotonic_s;
  recorded.actual_start_gap_ms = trace.actual_start_gap_ms;
  recorded.sample_interval_ms = trace.sample_interval_ms;
  recorded.sample_interval_available = trace.sample_interval_available;
  recorded.wake_lateness_ms = trace.wake_lateness_ms;
  recorded.window_wake_delay_ms = trace.window_wake_delay_ms;
  recorded.timing_available = trace.timing_available;
  recorded.first_control_tick = trace.first_control_tick;
  recorded.timing_accepted = trace.timing_accepted;
  recorded.timing_guard_faulted = trace.timing_guard_faulted;
}

void State_ScaleBFM::dump_fault_async(const std::string& reason) {
  bool starting_writer = false;
  try {
    // A previous immutable snapshot may still be writing when the user retries.
    // Diagnostics never delay entry or block control; retain at most one writer.
    if (fault_dump_thread_.joinable()) {
      if (!fault_dump_done_->load(std::memory_order_acquire)) {
        spdlog::warn("[ScaleBFM] previous fault snapshot is still writing; skipping this diagnostic dump");
        return;
      }
      fault_dump_thread_.join();
    }
    std::vector<TraceTick> ticks;
    std::optional<TraceTick> pending;
    {
      std::lock_guard<std::mutex> lock(trace_mutex_);
      ticks.reserve(trace_count_);
      const auto first = (trace_next_ + trace_ring_.size() - trace_count_) % trace_ring_.size();
      for (std::size_t i = 0; i < trace_count_; ++i)
        ticks.push_back(trace_ring_[(first + i) % trace_ring_.size()]);
      pending = pending_trace_;
    }
    const auto metadata = controller_->metadata();
    const float notice_threshold = target_rate_notice_rad_s_, lead_limit = max_target_lead_rad_;
    const auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    // Serialization and file I/O run independently of both control threads.
    // The lambda owns its snapshot and never reads this FSM or controller again.
    starting_writer = true;
    fault_dump_done_->store(false, std::memory_order_release);
    fault_dump_thread_ = std::thread([ticks = std::move(ticks), pending = std::move(pending),
        metadata, reason, notice_threshold, lead_limit, timestamp, done = fault_dump_done_] {
      struct Completion {
        std::shared_ptr<std::atomic<bool>> done;
        ~Completion() { done->store(true, std::memory_order_release); }
      } completion{done};
      try {
        auto encode = [](const TraceTick& tick) {
          YAML::Node item;
          item["phase"] = tick.phase;
          item["fault"] = tick.fault;
          item["monotonic_seconds"] = tick.monotonic_s;
          item["step_ms"] = tick.step_ms;
          item["inference_ms"] = tick.inference_ms;
          item["state_sample_available"] = tick.frame.sampled;
          if (!tick.frame.sampled)
            item["sample_note"] = "No current robot sample was captured for this attempt";
          item["timing_available"] = tick.timing_available;
          if (tick.timing_available) {
            auto timing = item["control_timing"];
            timing["scheduled_wake_monotonic_seconds"] = tick.scheduled_wake_monotonic_s;
            timing["next_wake_monotonic_seconds"] = tick.next_wake_monotonic_s;
            timing["actual_start_gap_ms"] = tick.actual_start_gap_ms;
            timing["first_control_tick"] = tick.first_control_tick;
            timing["wake_lateness_ms"] = tick.wake_lateness_ms;
            timing["rolling_wake_delay_ms"] = tick.window_wake_delay_ms;
            timing["full_work_ms"] = tick.full_work_ms;
            timing["begin_accepted"] = tick.timing_accepted;
            timing["guard_faulted"] = tick.timing_guard_faulted;
            timing["actual_sample_interval_available"] = tick.sample_interval_available;
            if (tick.sample_interval_available)
              timing["actual_sample_interval_ms"] = tick.sample_interval_ms;
          }
          item["command_heading_vx_vy_yaw_rate"] = sequence(tick.command);
          if (tick.frame.sampled) {
            const auto& state = tick.frame.state;
            item["joint_positions_sdk"] = sequence(state.joint_positions_sdk);
            item["joint_velocities_sdk"] = sequence(state.joint_velocities_sdk);
            item["pelvis_position_world"] = sequence(state.pelvis_position_w);
            item["pelvis_quaternion_wxyz"] = quaternion(state.pelvis_quat_w);
            item["pelvis_gyro_local"] = sequence(state.pelvis_ang_vel_b);
            item["pelvis_linear_velocity_local"] = sequence(tick.frame.pelvis_linear_velocity_b);
            item["joystick_ly_minus_lx_minus_rx"] = sequence(tick.frame.joystick);
          }
          item["core_snapshot_available"] = tick.core_snapshot;
          if (tick.core_snapshot) {
            item["root_reference_world"] = sequence(tick.root_reference);
            item["effective_velocity_world"] = sequence(tick.effective_velocity_world);
            item["mode_index"] = tick.inputs.mode[0];
            const auto& pd = tick.pd;
            auto diagnostics = item["pd_diagnostics"];
            diagnostics["tick"] = pd.tick;
            diagnostics["fault"] = pd.fault;
            diagnostics["has_policy_output"] = pd.has_policy_output;
            diagnostics["measured_q_sdk"] = sequence(pd.measured_q_sdk);
            diagnostics["measured_dq_sdk"] = sequence(pd.measured_dq_sdk);
            diagnostics["requested_targets_sdk"] = sequence(pd.requested_targets_sdk);
            diagnostics["estimated_unsaturated_pd_torque_sdk"] = sequence(pd.estimated_pd_torque_sdk);
            diagnostics["target_lead_sdk"] = sequence(pd.target_lead_sdk);
            diagnostics["target_rate_sdk"] = sequence(pd.target_rate_sdk);
            diagnostics["target_rate_notice_exceeded"] = pd.target_rate_notice_exceeded;
            diagnostics["max_abs_target_rate_rad_s"] = pd.max_abs_target_rate_rad_s;
            auto inputs = item["onnx_inputs_flat"];
            inputs["root_quat_buffer"] = sequence(tick.inputs.root_rotation);
            inputs["base_ang_vel_buffer"] = sequence(tick.inputs.root_ang_vel);
            inputs["dof_pos_buffer"] = sequence(tick.inputs.dof_pos);
            inputs["dof_vel_buffer"] = sequence(tick.inputs.dof_vel);
            inputs["last_action_buffer"] = sequence(tick.inputs.actions);
            inputs["target_body_pos_future_to_robot_base"] = sequence(tick.inputs.reference_pos);
            inputs["target_body_rot_future_to_robot_base"] = sequence(tick.inputs.reference_rot);
            inputs["time_offsets"] = sequence(tick.inputs.future_idx);
            inputs["mode_index"] = sequence(tick.inputs.mode);
          }
          return item;
        };
        YAML::Node report;
        report["schema_version"] = 2;
        report["adapter_revision"] = "bounded_timing_v4";
        report["fault"] = reason;
        report["process_id"] = static_cast<long>(::getpid());
        report["unix_timestamp_ns"] = std::to_string(timestamp);
        report["checkpoint_sha256"] = metadata.checkpoint_sha256;
        report["onnx_sha256"] = metadata.onnx_sha256;
        report["sdk_joint_names"] = metadata.sdk_joint_names;
        report["actor_joint_names"] = metadata.joint_names;
        report["control_period_seconds"] = metadata.control_dt;
        auto timing_limits = report["timing_limits"];
        timing_limits["schedule"] = "next wake equals actual accepted start plus 20 ms; no catch-up";
        timing_limits["max_actual_start_gap_ms"] = 40;
        timing_limits["max_rolling_positive_wake_delay_ms"] = 50;
        timing_limits["wake_delay_window_ms"] = 1000;
        timing_limits["wake_notice_ms"] = 5;
        timing_limits["max_full_work_ms"] = 20;
        timing_limits["max_action_age_ms"] = 100;
        timing_limits["max_state_age_ms"] = 200;
        report["target_rate_notice_rad_s"] = notice_threshold;
        report["max_target_lead_rad"] = lead_limit;
        report["kp_sdk"] = sequence(metadata.kp_sdk());
        report["kd_sdk"] = sequence(metadata.kd_sdk());
        report["history_order"] = "oldest_to_newest";
        report["effort_semantics"] = "unsaturated PD request at sampled state; native simulator enforces actuator limits";
        report["ring_capacity_ticks"] = 100;
        report["ticks"] = YAML::Node(YAML::NodeType::Sequence);
        for (const auto& tick : ticks) report["ticks"].push_back(encode(tick));
        if (pending) report["in_progress_at_fault"] = encode(*pending);
        YAML::Emitter emitter;
        emitter.SetFloatPrecision(std::numeric_limits<float>::max_digits10);
        emitter.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
        emitter << report;
        if (!emitter.good()) throw std::runtime_error("YAML serialization failed");
        std::string pattern = "/tmp/scalebfm_" + std::to_string(::getpid()) + "_" +
            std::to_string(timestamp) + "_XXXXXX.yaml";
        std::vector<char> filename(pattern.begin(), pattern.end()); filename.push_back('\0');
        const int descriptor = ::mkstemps(filename.data(), 5);
        if (descriptor < 0) throw std::runtime_error("could not create unique fault snapshot");
        FILE* file = ::fdopen(descriptor, "w");
        if (!file) { ::close(descriptor); throw std::runtime_error("could not open fault snapshot stream"); }
        const auto length = std::strlen(emitter.c_str());
        const bool written = std::fwrite(emitter.c_str(), 1, length, file) == length;
        const bool closed = std::fclose(file) == 0;
        if (!written || !closed) throw std::runtime_error("fault snapshot write failed");
        spdlog::error("[ScaleBFM] fault snapshot: {} ({} completed/attempted ticks)", filename.data(), ticks.size());
      } catch (const std::exception& error) {
        spdlog::error("[ScaleBFM] could not write fault snapshot: {}", error.what());
      }
    });
    starting_writer = false;
  } catch (const std::exception& error) {
    if (starting_writer) fault_dump_done_->store(true, std::memory_order_release);
    spdlog::error("[ScaleBFM] could not start fault snapshot writer: {}", error.what());
  }
}

void State_ScaleBFM::passive_damping() {
  const auto kd = param::config["FSM"]["Passive"]["kd"].as<std::vector<float>>();
  for (int i = 0; i < 29; ++i) {
    auto& motor = lowcmd->msg_.motor_cmd()[i];
    motor.kp() = 0; motor.kd() = kd.at(i); motor.dq() = 0; motor.tau() = 0;
  }
}

void State_ScaleBFM::run() {
  if (fault_ || lowstate->isTimeout()) { passive_damping(); fail("stale LowState or latched fault"); return; }
  scalebfm::Joints targets;
  { std::lock_guard<std::mutex> lock(action_mutex_);
    if (std::chrono::duration<float>(Clock::now() - action_time_).count() > action_timeout_s_) {
      passive_damping(); fail("stale policy target"); return;
    }
    targets = latest_targets_;
  }
  for (int i = 0; i < 29; ++i) lowcmd->msg_.motor_cmd()[i].q() = targets[i];
}

void State_ScaleBFM::exit() {
  running_ = false;
  {
    std::lock_guard<std::mutex> lock(wrist_mutex_);
    ++wrist_generation_;
    wrists_ = {};
  }
  if (policy_thread_.joinable()) policy_thread_.join();
  steering_.activate(false);
  if (ready_pub_) ready_pub_->publish(std_msgs::msg::Bool{});
}

void State_ScaleBFM::wrist_command(geometry_msgs::msg::PoseArray::ConstSharedPtr message) {
  const auto generation = wrist_generation_.load();
  if (!running_ || fault_ || !wrists_enabled_) {
    RCLCPP_WARN(node_->get_logger(), "ScaleBFM wrist targets rejected: state inactive or enable_wrist_targets=false");
    return;
  }
  try {
    const auto age = (node_->now() - rclcpp::Time(message->header.stamp)).seconds();
    if (message->header.frame_id != "odom" || !std::isfinite(age) || age < 0 || age > .2 || message->poses.size() != 2)
      throw std::runtime_error("need fresh odom PoseArray [left_wrist_yaw_link,right_wrist_yaw_link]");
    scalebfm::WristTargets goals;
    std::optional<scalebfm::Pose>* destinations[] = {&goals.left, &goals.right};
    const auto measured = capture().state;
    for (int i = 0; i < 2; ++i) {
      const auto& p = message->poses[i];
      scalebfm::Pose goal;
      goal.position = Eigen::Vector3f(p.position.x, p.position.y, p.position.z);
      goal.orientation = Eigen::Quaternionf(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
      if (!goal.position.allFinite() || !goal.orientation.coeffs().allFinite() ||
          std::abs(goal.orientation.norm() - 1.f) > .001f ||
          (goal.position - measured.pelvis_position_w).norm() > 1.f)
        throw std::runtime_error("invalid wrist pose or reference more than 1 m from pelvis");
      *destinations[i] = goal;
    }
    std::lock_guard<std::mutex> lock(wrist_mutex_);
    if (!running_ || fault_ || generation != wrist_generation_.load())
      throw std::runtime_error("controller reset while accepting wrist target; submit again");
    wrists_ = goals;
  } catch (const std::exception& e) {
    RCLCPP_WARN(node_->get_logger(), "ScaleBFM wrist target rejected: %s", e.what());
  }
}
