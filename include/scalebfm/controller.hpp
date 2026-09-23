#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace scalebfm {
constexpr std::size_t kJoints = 29, kHistory = 3, kFuture = 6, kBodies = 14;
using Joints = std::array<float, kJoints>;
struct State {
  Eigen::Quaternionf pelvis_quat_w = Eigen::Quaternionf::Identity();
  Eigen::Vector3f pelvis_ang_vel_b = Eigen::Vector3f::Zero();
  Eigen::Vector3f pelvis_position_w = Eigen::Vector3f::Zero();
  Joints joint_positions_sdk{}, joint_velocities_sdk{};
};
struct Pose {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
};
// Poses refer to wrist_yaw_link, never a palm or fingertip. Explicitly clearing
// both fields releases mode 2. This is the native actor, without a wrist servo.
struct WristTargets { std::optional<Pose> left, right; };
struct Body {
  std::string name;
  int parent = -1, joint_actor = -1;
  Eigen::Vector3f translation = Eigen::Vector3f::Zero();
  Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
  Eigen::Vector3f axis = Eigen::Vector3f::Zero();
  float joint_reference = 0;
};
struct Metadata {
  std::vector<std::string> joint_names, sdk_joint_names, selected_body_names;
  Joints default_q_actor{}, kp_actor{}, kd_actor{}, lower_actor{}, upper_actor{};
  Joints action_scale_actor{};
  std::array<int, kJoints> actor_to_sdk{};
  std::array<int, kBodies> selected_body_indices{};
  std::array<float, kFuture> future_idx{};
  std::vector<Body> bodies;
  float control_dt = .02f;
  std::string checkpoint_sha256, onnx_sha256;
  static Metadata load(const std::string& path);
  void validate();
  Joints actor_to_sdk_values(const Joints& values) const;
  Joints kp_sdk() const { return actor_to_sdk_values(kp_actor); }
  Joints kd_sdk() const { return actor_to_sdk_values(kd_actor); }
  Joints default_q_sdk() const { return actor_to_sdk_values(default_q_actor); }
  Joints lower_sdk() const { return actor_to_sdk_values(lower_actor); }
  Joints upper_sdk() const { return actor_to_sdk_values(upper_actor); }
};
struct Inputs {
  std::array<float, 12> root_rotation{};
  std::array<float, 9> root_ang_vel{};
  std::array<float, 87> dof_pos{}, dof_vel{}, actions{};
  std::array<float, 252> reference_pos{};
  std::array<float, 336> reference_rot{};
  std::array<std::int64_t, 6> future_idx{};
  std::array<std::int64_t, 1> mode{};
};
struct Output { Joints targets_actor{}, raw_actions{}; };
class Policy {
 public:
  virtual ~Policy() = default;
  virtual Output infer(const Inputs& inputs) = 0;
};
class OnnxPolicy final : public Policy {
 public:
  explicit OnnxPolicy(const std::string& model_path, int threads = 2,
                      const std::string& expected_sha256 = "");
  ~OnnxPolicy();
  Output infer(const Inputs&) override;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
struct Options {
  float max_target_lead_rad = 1.5f;
  // A sampled PD equilibrium can change faster than the physical joint. This
  // threshold produces telemetry only; it never rejects or clips actor output.
  float target_rate_notice_rad_s = 30.f;
  int inference_threads = 2;
  // Omit to use the exported training gains exactly. Changed waist gains use
  // the documented compensation; changed non-waist gains are rejected.
  std::optional<Joints> native_kp_sdk, native_kd_sdk;
};
// Values describe one attempted policy tick in SDK order. Torque is the
// unsaturated PD request at the sampled state, not measured/applied effort.
// The simulator's native actuator saturates effort at every physics step.
struct PDDiagnostics {
  std::uint64_t tick = 0;
  bool has_policy_output = false;
  bool target_rate_notice_exceeded = false;
  float max_abs_target_rate_rad_s = 0.f;
  Joints measured_q_sdk{}, measured_dq_sdk{}, requested_targets_sdk{};
  Joints estimated_pd_torque_sdk{}, target_lead_sdk{}, target_rate_sdk{};
  std::string fault;
};
class Controller {
 public:
  Controller(const std::string& model_path, const std::string& metadata_path,
             Options options = {});
  Controller(Metadata metadata, std::unique_ptr<Policy> policy, Options options = {});
  void reset(const State& state);
  // One invocation per fresh control sample at nominal 50 Hz; the caller owns
  // bounded clock-jitter and stale-state checks and must not run catch-up ticks.
  // Velocity XY is in the measured pelvis heading frame;
  // Z is yaw rate, bounded to 0.6 rad/s. A failed tick latches until reset.
  Joints step(const State& state, const Eigen::Vector3f& heading_velocity,
              const WristTargets& wrists = {});
  const Metadata& metadata() const { return metadata_; }
  const Joints& raw_actions() const { return raw_; }
  const Inputs& last_inputs() const { return inputs_; }
  const PDDiagnostics& last_pd_diagnostics() const { return pd_diagnostics_; }
  float inference_ms() const { return inference_ms_; }
  std::uint64_t control_steps() const { return steps_; }
  bool faulted() const { return faulted_; }
  const Eigen::Vector3f& root_target() const { return root_target_; }
  const Eigen::Vector2f& effective_velocity_world() const { return velocity_; }
  std::vector<Pose> forward_kinematics(const State&) const;
 private:
  void validate_state(const State&) const;
  std::string joint_diagnostic(std::size_t actor_index, const State&,
                               std::optional<float> requested_target = {}) const;
  void update_state_history(const State&, bool reset);
  void references(const State&, const Eigen::Vector3f&, const WristTargets&);
  Metadata metadata_;
  std::unique_ptr<Policy> policy_;
  Options options_;
  Inputs inputs_{};
  PDDiagnostics pd_diagnostics_{};
  Joints raw_{}, previous_targets_{}, kp_sdk_{}, kd_sdk_{};
  std::array<Pose, kBodies> neutral_{};
  std::array<Pose, 2> held_wrists_{};
  std::array<int, 2> wrist_slots_{};
  Eigen::Vector3f root_target_ = Eigen::Vector3f::Zero();
  Eigen::Vector2f velocity_ = Eigen::Vector2f::Zero();
  float yaw_ = 0, inference_ms_ = 0;
  std::uint64_t steps_ = 0;
  bool ready_ = false, faulted_ = false, wrist_mode_ = false;
};
}  // namespace scalebfm
