// Joystick command adapter for the RoboGPC noise-steering policy.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace isaaclab
{

class SteeringCommand
{
public:
    void configure(float max_speed, float max_yaw_rate, float deadzone)
    {
        if (!std::isfinite(max_speed) || max_speed < 0.0f ||
            !std::isfinite(max_yaw_rate) || max_yaw_rate < 0.0f ||
            !std::isfinite(deadzone) || deadzone < 0.0f || deadzone >= 1.0f) {
            throw std::invalid_argument("Invalid steering joystick limits or deadzone");
        }
        max_speed_ = max_speed;
        max_yaw_rate_ = max_yaw_rate;
        deadzone_ = deadzone;
    }

    void reset(float measured_yaw)
    {
        heading_target_ = wrap(measured_yaw);
        command_ = {0.0f, 0.0f, 1.0f, 0.0f};
    }

    // Inputs are forward, left, and counterclockwise turn, in [-1, 1].
    // Call exactly once per policy tick, including cached latent-plan ticks.
    void update(float measured_yaw, float forward, float left, float turn_left, float dt)
    {
        if (!std::isfinite(measured_yaw) || !std::isfinite(forward) ||
            !std::isfinite(left) || !std::isfinite(turn_left) ||
            !std::isfinite(dt) || dt <= 0.0f) {
            throw std::invalid_argument("Nonfinite steering input or invalid control period");
        }

        const float magnitude = std::hypot(forward, left);
        float scale = 0.0f;
        if (magnitude > deadzone_) {
            // Radial scaling prevents diagonal input from exceeding max_speed.
            const float speed_fraction = (std::min(magnitude, 1.0f) - deadzone_) /
                                         (1.0f - deadzone_);
            scale = max_speed_ * speed_fraction / magnitude;
        }
        command_[0] = forward * scale;
        command_[1] = left * scale;

        const float turn_magnitude = std::min(std::abs(turn_left), 1.0f);
        const float turn_fraction = turn_magnitude > deadzone_
            ? std::copysign((turn_magnitude - deadzone_) / (1.0f - deadzone_), turn_left)
            : 0.0f;
        heading_target_ = wrap(heading_target_ + turn_fraction * max_yaw_rate_ * dt);
        const float heading_error = heading_target_ - measured_yaw;
        command_[2] = std::cos(heading_error);
        command_[3] = std::sin(heading_error);
    }

    const std::array<float, 4>& command() const { return command_; }

    // Physical units, without joystick deadzone. An idle ROS command cancels
    // any outstanding turn rather than holding a previous desired heading.
    void update_velocity(float yaw, float vx, float vy, float wz, float dt)
    {
        if (!std::isfinite(yaw) || !std::isfinite(vx) || !std::isfinite(vy) ||
            !std::isfinite(wz) || !std::isfinite(dt) || dt <= 0.0f) {
            throw std::invalid_argument("Invalid external steering command");
        }
        if (vx == 0.0f && vy == 0.0f && wz == 0.0f) {
            reset(yaw);
            return;
        }
        const float speed = std::hypot(vx, vy);
        const float scale = speed > max_speed_ ? max_speed_ / speed : 1.0f;
        command_[0] = vx * scale;
        command_[1] = vy * scale;
        heading_target_ = wrap(heading_target_ + std::clamp(wz, -max_yaw_rate_, max_yaw_rate_) * dt);
        command_[2] = std::cos(heading_target_ - yaw);
        command_[3] = std::sin(heading_target_ - yaw);
    }

private:
    static float wrap(float angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

    float max_speed_ = 1.0f;
    float max_yaw_rate_ = 1.0f;
    float deadzone_ = 0.1f;
    float heading_target_ = 0.0f;
    std::array<float, 4> command_{0.0f, 0.0f, 1.0f, 0.0f};
};

} // namespace isaaclab
