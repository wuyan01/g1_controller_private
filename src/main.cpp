#include "unitree/dds_wrapper/robots/go2/go2.h"

#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"

#include <ros/ros.h>

void init_fsm_state(ros::NodeHandle& nh)
{
    auto lowcmd_sub = std::make_shared<unitree::robot::g1::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        unitree::robot::go2::shutdown();
        // exit(0);
    }
    FSMState::lowcmd = std::make_unique<unitree::robot::g1::publisher::LowCmd>();
    FSMState::lowstate = std::make_shared<unitree::robot::g1::subscription::LowState>();
    FSMState::sportmodestate = std::make_shared<unitree::robot::g1::subscription::SportModeState>();
    FSMState::motioncapturestate = std::make_shared<unitree::robot::g1::subscription::MotionCaptureState>(
        nh,
        "/qualisys/base/pose",
        "/qualisys/base/velocity"
    );
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

enum FSMMode
{
    Passive = 1,
    FixStand = 2,
    Velocity = 3,
};

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "g1_controller");
    ros::NodeHandle nh("~");

    // Get parameters
    std::string network;
    nh.param<std::string>("network", network, std::string("lo"));

    // Load parameters
    auto vm = param::helper(argc, argv);

    std::cout << " --- Unitree Robotics --- " << std::endl;
    std::cout << "     G1-29dof Controller  " << std::endl;

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(0, network);

    init_fsm_state(nh);

    FSMState::lowcmd->msg_.mode_machine() = 5; // 29dof
    FSMState::network_interface = network;

    // Initialize FSM
    auto & joy = FSMState::lowstate->joystick;
    auto fsm = std::make_unique<CtrlFSM>(new State_Passive(FSMMode::Passive));

    fsm->states.back()->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return joy.start.on_pressed; }, // Start
            (int)FSMMode::FixStand
        )
    );
    fsm->add(new State_FixStand(FSMMode::FixStand));
    fsm->states.back()->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return joy.A.on_pressed; }, // A
            FSMMode::Velocity
        )
    );
    fsm->add(new State_RLBase(FSMMode::Velocity, "Velocity"));
    fsm->states.back()->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return joy.back.on_pressed; }, // Select
            FSMMode::Passive
        )
    );

    spdlog::info("--- FSM States Initialized ---");
    spdlog::info("Press [Start] to enter FixStand mode.");
    spdlog::info("And then press [A] to start controlling the robot.");
    spdlog::info("Press [Select] to enter Passive mode.");

    ros::spin();
    
    return 0;
}

