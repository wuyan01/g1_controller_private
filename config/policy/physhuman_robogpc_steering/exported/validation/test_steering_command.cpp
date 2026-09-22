#include "isaaclab/envs/steering_command.h"

#include <cassert>
#include <cmath>
#include <iostream>

static void near(float actual, float expected, float tolerance = 1e-5f)
{
    assert(std::abs(actual - expected) < tolerance);
}

int main()
{
    isaaclab::SteeringCommand command;
    command.configure(1.0f, 1.0f, 0.1f);
    command.reset(0.7f);
    near(command.command()[0], 0.0f);
    near(command.command()[2], 1.0f);
    near(command.command()[3], 0.0f);

    // Repeated observation reads leave the heading target untouched.
    for (int i = 0; i < 10; ++i) {
        near(command.command()[2], 1.0f);
    }

    // Centered joystick holds world heading while the robot turns underneath.
    command.update(1.2f, 0.0f, 0.0f, 0.0f, 0.02f);
    near(command.command()[2], std::cos(-0.5f));
    near(command.command()[3], std::sin(-0.5f));

    // Translation axes use forward/left and cap the total diagonal speed.
    command.update(0.7f, 1.0f, 1.0f, 0.0f, 0.02f);
    near(command.command()[0], std::sqrt(0.5f));
    near(command.command()[1], std::sqrt(0.5f));
    near(std::hypot(command.command()[0], command.command()[1]), 1.0f);
    command.update(0.7f, -1.0f, 0.0f, 0.0f, 0.02f);
    near(command.command()[0], -1.0f);
    command.update(0.7f, 0.05f, 0.05f, 0.05f, 0.02f);
    near(command.command()[0], 0.0f);
    near(command.command()[1], 0.0f);
    near(command.command()[2], 1.0f);

    // Positive turn input turns left; native Xbox rx is negated by the adapter.
    command.reset(0.0f);
    command.update(0.0f, 0.0f, 0.0f, 1.0f, 0.02f);
    near(command.command()[2], std::cos(0.02f));
    near(command.command()[3], std::sin(0.02f));
    command.update(0.02f, 0.0f, 0.0f, -1.0f, 0.02f);
    near(command.command()[3], std::sin(-0.02f));

    // Crossing +/-pi must remain a small relative heading error.
    constexpr float pi = 3.14159265358979323846f;
    command.reset(pi - 0.01f);
    command.update(-pi + 0.01f, 0.0f, 0.0f, 1.0f, 0.02f);
    near(command.command()[2], 1.0f);
    near(command.command()[3], 0.0f);

    // Mode reentry clears a previous movement and a previous facing target.
    command.update(-pi, 1.0f, 0.0f, -1.0f, 0.02f);
    command.reset(-0.8f);
    near(command.command()[0], 0.0f);
    near(command.command()[1], 0.0f);
    near(command.command()[2], 1.0f);
    near(command.command()[3], 0.0f);

    bool rejected = false;
    try { command.configure(1.0f, 1.0f, 1.0f); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);

    std::cout << "Steering command checks passed\n";
}
