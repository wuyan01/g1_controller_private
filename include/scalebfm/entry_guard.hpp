#pragma once

#include "scalebfm/controller.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace scalebfm {
// This deployment profile assumes a horizontal floor at odom z=0. These
// checks reject unsuitable reset snapshots before commands are applied.
// Geometric proximity is not a contact measurement or proof of unassisted
// balance: an elastic-band-supported robot can also satisfy these checks.
struct EntryLimits {
  static constexpr float min_pelvis_height = .65f;
  static constexpr float max_pelvis_height = .85f;
  static constexpr float max_tilt_rad = .2f;
  static constexpr float max_angular_speed = .5f;
  static constexpr float max_joint_speed = 1.f;
  static constexpr float max_horizontal_speed = .10f;
  static constexpr float max_vertical_speed = .10f;
  static constexpr float min_ankle_height = .015f;
  static constexpr float max_ankle_height = .10f;
  static constexpr float max_ankle_height_difference = .05f;
};

inline void validate_entry(const State& state,
                           const Eigen::Vector3f& pelvis_linear_velocity_b,
                           const Metadata& metadata,
                           const std::vector<Pose>& body_poses) {
  const auto require = [](bool condition, const char* message) {
    if (!condition) throw std::runtime_error(std::string("ScaleBFM entry: ") + message);
  };
  require(state.pelvis_quat_w.coeffs().allFinite() &&
              std::abs(state.pelvis_quat_w.norm() - 1.f) < .01f,
          "need a finite unit pelvis quaternion");
  require(state.pelvis_position_w.allFinite() && state.pelvis_ang_vel_b.allFinite() &&
              pelvis_linear_velocity_b.allFinite(),
          "nonfinite pelvis state");
  require(std::all_of(state.joint_positions_sdk.begin(), state.joint_positions_sdk.end(),
                      [](float x) { return std::isfinite(x); }) &&
              std::all_of(state.joint_velocities_sdk.begin(), state.joint_velocities_sdk.end(),
                      [](float x) { return std::isfinite(x); }),
          "nonfinite joint state");
  const auto rotation = state.pelvis_quat_w.normalized();
  require((rotation * Eigen::Vector3f::UnitZ()).z() >= std::cos(EntryLimits::max_tilt_rad),
          "pelvis tilt exceeds 0.2 rad; enter from a supported upright stance");
  require(state.pelvis_position_w.z() >= EntryLimits::min_pelvis_height &&
              state.pelvis_position_w.z() <= EntryLimits::max_pelvis_height,
          "pelvis height must be 0.65 to 0.85 m above the odom z=0 floor");
  require(state.pelvis_ang_vel_b.norm() <= EntryLimits::max_angular_speed,
          "pelvis angular speed exceeds 0.5 rad/s");
  for (const float speed : state.joint_velocities_sdk)
    require(std::abs(speed) <= EntryLimits::max_joint_speed,
            "joint speed exceeds 1 rad/s; wait for the supported pose to settle");
  // Convert the child-frame twist to world coordinates before separating
  // horizontal and vertical speeds, including when the entry pose is tilted.
  const Eigen::Vector3f velocity_w = rotation * pelvis_linear_velocity_b;
  require(velocity_w.head<2>().norm() <= EntryLimits::max_horizontal_speed &&
              std::abs(velocity_w.z()) <= EntryLimits::max_vertical_speed,
          "pelvis speed exceeds 0.10 m/s horizontally or vertically");
  require(body_poses.size() == metadata.bodies.size(), "FK body count mismatch");
  std::array<float, 2> ankle_height{};
  const std::array<std::string, 2> ankle_names{
      "left_ankle_roll_link", "right_ankle_roll_link"};
  for (std::size_t side = 0; side < 2; ++side) {
    const auto it = std::find_if(metadata.bodies.begin(), metadata.bodies.end(),
        [&](const Body& body) { return body.name == ankle_names[side]; });
    require(it != metadata.bodies.end(), "missing ankle frame in FK metadata");
    const auto& pose = body_poses[static_cast<std::size_t>(it - metadata.bodies.begin())];
    require(pose.position.allFinite(), "nonfinite ankle FK position");
    ankle_height[side] = pose.position.z();
    require(ankle_height[side] >= EntryLimits::min_ankle_height &&
                ankle_height[side] <= EntryLimits::max_ankle_height,
            "both ankle origins must be 0.015 to 0.10 m above the odom z=0 floor");
  }
  require(std::abs(ankle_height[0] - ankle_height[1]) <=
              EntryLimits::max_ankle_height_difference,
          "ankle height difference exceeds 0.05 m; only flat-floor entry is supported");
}
}  // namespace scalebfm
