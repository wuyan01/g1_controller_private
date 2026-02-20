#include "FSM/State_RLBase.h"
#include "g1_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include <unordered_map>
#include <algorithm>
#include <iomanip>
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

    if (deploy_cfg["action_monitor"].IsDefined()) {
        const auto action_monitor_cfg = deploy_cfg["action_monitor"];
        action_csv_log_enabled_ = action_monitor_cfg["csv_log"]["enabled"].as<bool>(false);
        action_topic_pub_enabled_ = action_monitor_cfg["topic_pub"]["enabled"].as<bool>(false);
        action_monitor_decimation_ = std::max<std::size_t>(1, action_monitor_cfg["decimation"].as<std::size_t>(1));

        if (action_csv_log_enabled_) {
            std::filesystem::path csv_path =
                action_monitor_cfg["csv_log"]["path"].as<std::string>("log/policy_actions.csv");
            if (csv_path.is_relative()) {
                csv_path = std::filesystem::current_path() / csv_path;
            }
            std::filesystem::create_directories(csv_path.parent_path());
            action_csv_stream_.open(csv_path, std::ios::out | std::ios::trunc);
            if (!action_csv_stream_.is_open()) {
                spdlog::warn("Failed to open action CSV log path: {}", csv_path.string());
                action_csv_log_enabled_ = false;
            } else {
                action_csv_stream_ << "stamp_sec";
                auto action_dim = env->robot->data.joint_ids_map.size();
                for (std::size_t i = 0; i < action_dim; ++i) {
                    action_csv_stream_ << ",action_" << i;
                }
                action_csv_stream_ << '\n';
                action_csv_stream_ << std::fixed << std::setprecision(6);
                spdlog::info("Action CSV logging enabled: {}", csv_path.string());
            }
        }

        if (action_topic_pub_enabled_ && node_) {
            const auto topic_name = action_monitor_cfg["topic_pub"]["topic"].as<std::string>("/policy_action");
            const auto queue_size = action_monitor_cfg["topic_pub"]["queue_size"].as<int>(10);
            action_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(topic_name, queue_size);
            spdlog::info("Action topic publishing enabled: {}", topic_name);
        }
    }
}

void State_RLBase::run()
{
    auto action = env->action_manager->processed_actions();
    monitor_action(action);
    for(int i(0); i < env->robot->data.joint_ids_map.size(); i++) {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }
}

void State_RLBase::monitor_action(const std::vector<float>& action)
{
    action_monitor_counter_++;
    if (action_monitor_counter_ % action_monitor_decimation_ != 0) {
        return;
    }

    if (action_csv_log_enabled_ && action_csv_stream_.is_open() && node_) {
        const double stamp_sec = node_->now().seconds();
        action_csv_stream_ << stamp_sec;
        for (const auto value : action) {
            action_csv_stream_ << ',' << value;
        }
        action_csv_stream_ << '\n';
    }

    if (action_topic_pub_enabled_ && action_pub_ && node_) {
        std_msgs::msg::Float32MultiArray msg;
        msg.data = action;
        action_pub_->publish(msg);
    }
}
