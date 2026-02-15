#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "State_Mimic.h"
#include <rclcpp/rclcpp.hpp>


std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = std::make_shared<Keyboard>();
std::shared_ptr<SportModeState_t> FSMState::sportmodestate = nullptr;
std::shared_ptr<MotionCaptureState_t> FSMState::motioncapturestate = nullptr;
std::shared_ptr<FastLioState_t> FSMState::fastliostate = nullptr;
std::shared_ptr<JointState_t> FSMState::jointstate = nullptr;
std::shared_ptr<Imu_t> FSMState::imu = nullptr;
std::shared_ptr<VelocityMarkers_t> State_RLBase::velocitymarkers = nullptr;
std::shared_ptr<PoseSourceOdom_t> FSMState::sportmodeodom = nullptr;
std::shared_ptr<PoseSourceOdom_t> FSMState::motioncaptureodom = nullptr;
std::shared_ptr<PoseSourceOdom_t> FSMState::lidarodom = nullptr;
std::shared_ptr<OdometryFusion_t> FSMState::pelvisodometryfusion = nullptr;
std::shared_ptr<OdometryFusion_t> State_RLBase::torsoodometryfusion = nullptr;
std::shared_ptr<HeightMap_t> State_RLBase::heightmap = nullptr;
std::shared_ptr<HeightMapHandler_t> State_RLBase::heightmaphandler = nullptr;
rclcpp::Node::SharedPtr State_RLBase::node_ = nullptr;
std::string FSMState::network_interface = "lo";

void init_fsm_state(rclcpp::Node::SharedPtr node)
{
    auto lowcmd_sub = std::make_shared<unitree::robot::g1::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        unitree::robot::go2::shutdown();
        // exit(0);
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    FSMState::sportmodestate = std::make_shared<SportModeState_t>();
    FSMState::motioncapturestate = std::make_shared<MotionCaptureState_t>(node, "/qualisys/base/odom");
    FSMState::fastliostate = std::make_shared<FastLioState_t>(node, "/Odometry_loc");
    FSMState::jointstate = std::make_shared<JointState_t>(node, "joint_states");
    FSMState::imu = std::make_shared<Imu_t>(node, "imu");
    State_RLBase::velocitymarkers = std::make_shared<VelocityMarkers_t>(node, "velocity_markers");
    FSMState::sportmodeodom = std::make_shared<PoseSourceOdom_t>(node, "sport_mode/odom");
    FSMState::motioncaptureodom = std::make_shared<PoseSourceOdom_t>(node, "motion_capture/odom");
    FSMState::lidarodom = std::make_shared<PoseSourceOdom_t>(node, "lidar/odom");
    FSMState::pelvisodometryfusion = std::make_shared<OdometryFusion_t>(node, "pose_source/odom", "pelvis");
    State_RLBase::torsoodometryfusion = std::make_shared<OdometryFusion_t>(node, "pose_source/odom", "torso_link");
    State_RLBase::heightmap = std::make_shared<HeightMap_t>();
    State_RLBase::heightmaphandler = std::make_shared<HeightMapHandler_t>(node);
    State_RLBase::node_ = node;
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("g1_controller");

    // Get parameters
    std::string network = node->declare_parameter<std::string>("network", "lo");
    std::cout << "Using network interface: " << network << std::endl;

    // Load parameters
    auto vm = param::helper(argc, argv);

    std::cout << " --- Unitree Robotics --- " << std::endl;
    std::cout << "     G1-29dof Controller  " << std::endl;

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(0, network);

    init_fsm_state(node);

    FSMState::lowcmd->msg_.mode_machine() = 5; // 29dof
    if(!FSMState::lowcmd->check_mode_machine(FSMState::lowstate)) {
        spdlog::critical("Unmatched robot type.");
        exit(-1);
    }
    FSMState::network_interface = network;

    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    spdlog::info("--- FSM States Initialized ---");
    spdlog::info("Press [Start] to enter FixStand mode.");
    spdlog::info("And then press [A] to start controlling the robot.");
    spdlog::info("Press [Select] to enter Passive mode.");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    
    std::quick_exit(0);
}
