#include "FSM/State_RLBase.h"
#include "g1_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include <unordered_map>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace isaaclab
{
// keyboard velocity commands example
// change "velocity_commands" observation name in policy deploy.yaml to "keyboard_velocity_commands"
REGISTER_OBSERVATION(keyboard_velocity_commands)
{
    std::string key = FSMState::keyboard->key();
    static auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

    static std::unordered_map<std::string, std::vector<float>> key_commands = {
        {"w", {1.0f, 0.0f, 0.0f}},
        {"s", {-1.0f, 0.0f, 0.0f}},
        {"a", {0.0f, 1.0f, 0.0f}},
        {"d", {0.0f, -1.0f, 0.0f}},
        {"q", {0.0f, 0.0f, 1.0f}},
        {"e", {0.0f, 0.0f, -1.0f}}
    };
    std::vector<float> cmd = {0.0f, 0.0f, 0.0f};
    if (key_commands.find(key) != key_commands.end())
    {
        // TODO: smooth and limit the velocity commands
        cmd = key_commands[key];
    }
    return cmd;
}

}

State_RLBase::State_RLBase(int state_mode, std::string state_string)
: FSMState(state_mode, state_string) 
{
    auto cfg = param::config["FSM"][state_string];
    std::filesystem::path package_path = ament_index_cpp::get_package_share_directory("g1_controller");
    auto policy_dir = package_path / cfg["policy_dir"].as<std::string>();

    auto deploy_cfg = YAML::LoadFile(policy_dir / "params" / "deploy.yaml");
    measure_terrain_heights = deploy_cfg["height_map"]["measure_terrain_heights"].as<bool>();
    if (measure_terrain_heights) {
        heightmaphandler->init(deploy_cfg["height_map"]);
    }

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        deploy_cfg,
        std::make_shared<unitree::G1Articulation<LowState_t::SharedPtr, OdometryFusion_t::SharedPtr, HeightMapHandler_t::SharedPtr>>(FSMState::lowstate, State_RLBase::torsoodometryfusion, State_RLBase::heightmaphandler)
    );
    auto precision = env->cfg["policy_precision"].as<std::string>("fp32");
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx", precision);
    env->action.resize(env->alg->action.size(), 0.0f);

    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
            FSMStringMap.right.at("Passive")
        )
    );

    // Create ROS subscriber for real-time text commands
    if (node_ && env->text_embedding_dict.size() > 0) {
        text_cmd_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = text_cmd_cb_group_;
        text_cmd_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/text_command", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                env->set_text_key(msg->data);
            },
            sub_options
        );
        spdlog::info("Subscribed to /text_command for real-time text input.");
    }
}

void State_RLBase::run()
{
    auto action = env->action_manager->processed_actions();
    for(int i(0); i < env->robot->data.joint_ids_map.size(); i++) {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }
}