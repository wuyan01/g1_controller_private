#include "FSM/State_RLBaseSync.h"
#include "g1_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>

State_RLBaseSync::State_RLBaseSync(int state_mode, std::string state_string)
: FSMState(state_mode, state_string)
{
    spdlog::warn("[RLBaseSync] Using ZERO-DELAY debug mode (single-threaded inference).");

    auto cfg = param::config["FSM"][state_string];
    std::filesystem::path package_path = ament_index_cpp::get_package_share_directory("g1_controller");
    auto policy_dir = package_path / cfg["policy_dir"].as<std::string>();

    auto deploy_cfg = YAML::LoadFile(policy_dir / "params" / "deploy.yaml");
    measure_terrain_heights = deploy_cfg["height_map"]["measure_terrain_heights"].as<bool>();
    if (measure_terrain_heights) {
        State_RLBase::heightmaphandler->init(deploy_cfg["height_map"]);
    }

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        deploy_cfg,
        std::make_shared<unitree::G1Articulation<LowState_t::SharedPtr, OdometryFusion_t::SharedPtr, HeightMapHandler_t::SharedPtr>>(FSMState::lowstate, State_RLBase::torsoodometryfusion, State_RLBase::heightmaphandler)
    );
    auto precision = env->cfg["policy_precision"].as<std::string>("fp32");
    const auto policy_runner = env->cfg["policy_runner"].as<std::string>("onnx");
    const auto policy_file = env->cfg["policy_file"].as<std::string>("policy.onnx");
    const auto policy_path = policy_dir / "exported" / policy_file;
    if (policy_runner == "onnx_stateful") {
        env->alg = std::make_unique<isaaclab::StatefulOrtRunner>(policy_path, deploy_cfg, precision);
    } else {
        env->alg = std::make_unique<isaaclab::OrtRunner>(policy_path, precision);
    }
    env->action.resize(env->alg->action.size(), 0.0f);

    constexpr double kFsmDt = 0.001;
    steps_per_policy_ = static_cast<int>(std::round(env->step_dt / kFsmDt));
    spdlog::info("[RLBaseSync] Policy steps every {} FSM ticks ({:.3f}s)",
                 steps_per_policy_, env->step_dt);

    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
            FSMStringMap.right.at("Passive")
        )
    );

    if (State_RLBase::node_ && env->text_embedding_dict.size() > 0) {
        text_cmd_cb_group_ = State_RLBase::node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = text_cmd_cb_group_;
        text_cmd_sub_ = State_RLBase::node_->create_subscription<std_msgs::msg::String>(
            "/text_command", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                env->set_text_key(msg->data);
            },
            sub_options
        );
        spdlog::info("[RLBaseSync] Subscribed to /text_command for real-time text input.");
    }
}
