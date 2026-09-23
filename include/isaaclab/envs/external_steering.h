#pragma once

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>

namespace isaaclab
{
// Shared by the ROS callback and policy thread. Deadlines use monotonic time.
class ExternalSteering
{
public:
    using Clock = std::chrono::steady_clock;
    void activate(bool active)
    {
        std::lock_guard<std::mutex> guard(mutex_);
        active_ = active;
        manual_ = false;
        valid_ = false;
    }
    bool receive(float vx, float vy, float wz, Clock::time_point now = Clock::now())
    {
        std::lock_guard<std::mutex> guard(mutex_);
        valid_ = active_ && !manual_ && std::isfinite(vx) && std::isfinite(vy) && std::isfinite(wz);
        command_ = valid_ ? std::array<float, 3>{vx, vy, wz} : std::array<float, 3>{};
        received_ = now;
        return valid_;
    }
    std::array<float, 3> sample(bool joystick_moved, Clock::time_point now = Clock::now())
    {
        std::lock_guard<std::mutex> guard(mutex_);
        manual_ = manual_ || joystick_moved;
        if (manual_ || !active_ || !valid_ || now < received_ ||
            now - received_ >= std::chrono::milliseconds(200)) {
            valid_ = false;
            return {};
        }
        return command_;
    }
    bool ready() const
    {
        std::lock_guard<std::mutex> guard(mutex_);
        return active_ && !manual_;
    }
    // VELOCITY policy input: physical forward/left/yaw-rate, with existing
    // per-axis training limits. Zero is literally [0, 0, 0] in this mode.
    std::array<float, 3> select_velocity(bool ros_enabled,
        const std::array<float, 3>& joystick, const std::array<float, 3>& lower,
        const std::array<float, 3>& upper, Clock::time_point now = Clock::now())
    {
        auto velocity = joystick;
        if (ros_enabled) {
            const bool moved = std::hypot(joystick[0], joystick[1]) > 0.1f ||
                               std::abs(joystick[2]) > 0.1f;
            const auto external = sample(moved, now);
            if (ready()) {
                velocity = external;
            }
        }
        for (std::size_t i = 0; i < velocity.size(); ++i) {
            velocity[i] = std::clamp(velocity[i], lower[i], upper[i]);
        }
        return velocity;
    }
private:
    mutable std::mutex mutex_;
    bool active_ = false, manual_ = false, valid_ = false;
    std::array<float, 3> command_{};
    Clock::time_point received_{};
};
} // namespace isaaclab
