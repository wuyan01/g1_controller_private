#include "isaaclab/envs/external_steering.h"
#include "isaaclab/envs/steering_command.h"
#include <cassert>
#include <limits>

int main()
{
    using Clock = isaaclab::ExternalSteering::Clock;
    const auto t = Clock::now();
    isaaclab::ExternalSteering input;
    assert(!input.receive(.1f, 0, 0, t));
    input.activate(true);
    assert(input.receive(.1f, 0, .3f, t));
    assert(input.sample(false, t)[0] == .1f);
    assert(input.sample(false, t + std::chrono::milliseconds(200))[0] == 0);
    input.receive(.1f, 0, 0, t);
    input.sample(true, t);
    assert(!input.ready());
    assert(!input.receive(.1f, 0, 0, t));
    input.sample(false, t);
    assert(!input.ready()); // Stick release never resumes ROS.
    input.activate(true);
    assert(input.ready());
    assert(input.sample(false, t)[0] == 0); // Reentry clears commands.
    assert(!input.receive(std::numeric_limits<float>::quiet_NaN(), 0, 0, t));
    const std::array<float, 3> lower{-1, -.5f, -1}, upper{1, .5f, 1}, zero{};
    input.receive(.15f, -.1f, .2f, t);
    const auto ros = input.select_velocity(true, zero, lower, upper, t);
    assert(ros[0] == .15f && ros[1] == -.1f && ros[2] == .2f);
    assert(input.select_velocity(true, zero, lower, upper, t + std::chrono::milliseconds(201)) == zero);
    input.receive(.15f, 0, .2f, t);
    const auto manual = input.select_velocity(true, {0, -.8f, .3f}, lower, upper, t);
    assert(manual[1] == -.5f && manual[2] == .3f);
    assert(!input.ready());
    assert(!input.receive(.15f, 0, 0, t));
    assert(input.select_velocity(true, zero, lower, upper, t) == zero);
    input.activate(true);
    assert(input.select_velocity(true, zero, lower, upper, t) == zero);
    const auto original = input.select_velocity(false, {.6f, -.8f, .4f}, lower, upper, t);
    assert(original[0] == .6f && original[1] == -.5f && original[2] == .4f);
    isaaclab::SteeringCommand command;
    command.configure(.25f, .4f, .1f);
    command.reset(0);
    command.update_velocity(0, .01f, 0, .4f, .02f);
    assert(command.command()[0] == .01f); // No joystick deadzone.
    assert(command.command()[3] > 0);
    command.update_velocity(.3f, 0, 0, 0, .02f);
    assert(command.command()[2] == 1 && command.command()[3] == 0);
    command.update_velocity(.3f, 1, 1, 0, .02f);
    assert(std::abs(std::hypot(command.command()[0], command.command()[1])-.25f) < 1e-6f);
}
