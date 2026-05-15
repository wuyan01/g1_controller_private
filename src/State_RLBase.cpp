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
    has_goal_command_observation_ = deploy_cfg["observations"]["goal_command"].IsDefined();
    if (deploy_cfg["goal_monitor"].IsDefined()) {
        const auto goal_monitor_cfg = deploy_cfg["goal_monitor"];
        goal_reached_distance_ = goal_monitor_cfg["reached_distance"].as<double>(goal_reached_distance_);
        goal_reached_hold_time_ = goal_monitor_cfg["reached_hold_time"].as<double>(goal_reached_hold_time_);
        goal_distance_log_interval_ =
            std::max<std::size_t>(1, goal_monitor_cfg["log_interval"].as<std::size_t>(goal_distance_log_interval_));
        goal_marker_frame_id_ = goal_monitor_cfg["marker_frame_id"].as<std::string>(goal_marker_frame_id_);
        goal_idle_text_key_ = goal_monitor_cfg["idle_text_key"].as<std::string>(goal_idle_text_key_);
        goal_active_text_key_ = goal_monitor_cfg["active_text_key"].as<std::string>(goal_active_text_key_);
    } else if (deploy_cfg["observations"]["text_embedding"]["params"]["text_key"].IsDefined()) {
        goal_active_text_key_ =
            deploy_cfg["observations"]["text_embedding"]["params"]["text_key"].as<std::string>(goal_active_text_key_);
    }
    if (measure_terrain_heights) {
        heightmaphandler->init(deploy_cfg["height_map"]);
    }
    if (deploy_cfg["policy_delay_debug"].IsDefined()) {
        const auto policy_delay_debug_cfg = deploy_cfg["policy_delay_debug"];
        policy_delay_debug_enabled_ = policy_delay_debug_cfg["enabled"].as<bool>(false);
        policy_delay_debug_interval_ =
            std::max<std::size_t>(1, policy_delay_debug_cfg["interval"].as<std::size_t>(1));
        policy_delay_debug_warn_ms_ = policy_delay_debug_cfg["warn_ms"].as<double>(0.0);
        if (policy_delay_debug_enabled_) {
            spdlog::info("Policy delay debug enabled: interval={}, warn_ms={:.3f}",
                         policy_delay_debug_interval_, policy_delay_debug_warn_ms_);
        }
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
    if (has_goal_command_observation_) {
        env->set_text_key(goal_idle_text_key_);
    }

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
                handle_text_command_msg(msg->data);
            },
            sub_options
        );
        spdlog::info("Subscribed to /text_command for real-time text input.");
    }

    if (node_ && has_goal_command_observation_) {
        goal_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "/goal_marker",
            rclcpp::QoS(1).transient_local()
        );
        goal_pose_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = goal_pose_cb_group_;
        goal_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                Eigen::Vector3f goal_pos_w(
                    static_cast<float>(msg->pose.position.x),
                    static_cast<float>(msg->pose.position.y),
                    static_cast<float>(msg->pose.position.z)
                );
                env->set_goal_position(goal_pos_w);
                env->set_text_key(goal_active_text_key_);
                goal_reached_since_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
                goal_reached_idle_sent_ = false;
                spdlog::info("Goal pose updated: frame='{}', xy=({:.3f}, {:.3f}), text='{}'",
                             msg->header.frame_id,
                             goal_pos_w.x(),
                             goal_pos_w.y(),
                             goal_active_text_key_);
            },
            sub_options
        );
        spdlog::info("Subscribed to /goal_pose for goal-reaching input.");
        spdlog::info("Publishing persistent goal marker on /goal_marker.");
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
    std::vector<float> action;
    State_RLBase::steady_clock::time_point action_time;
    std::size_t action_seq = 0;
    if (!consume_latest_action_snapshot(action, action_time, action_seq)) {
        return;
    }

    if (action_seq == last_applied_action_seq_) {
        return;
    }

    last_applied_action_seq_ = action_seq;
    gauge_control_frequency(action_time);
    monitor_action(action);
    for(int i(0); i < env->robot->data.joint_ids_map.size(); i++) {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }
}

void State_RLBase::publish_latest_action_snapshot(const std::vector<float>& action,
                                                  const State_RLBase::steady_clock::time_point& update_time)
{
    std::lock_guard<std::mutex> lock(action_update_mutex_);
    latest_action_ = action;
    latest_action_update_time_ = update_time;
    ++action_update_seq_;
}

bool State_RLBase::consume_latest_action_snapshot(std::vector<float>& action,
                                                  State_RLBase::steady_clock::time_point& update_time,
                                                  std::size_t& action_seq)
{
    std::lock_guard<std::mutex> lock(action_update_mutex_);
    if (latest_action_.empty()) {
        return false;
    }

    action = latest_action_;
    update_time = latest_action_update_time_;
    action_seq = action_update_seq_;
    return true;
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

void State_RLBase::handle_text_commands()
{
    const auto state = getStateString();
    if (state != "Physhuman" && state != "Physhuman_Steer") {
        return;
    }

    if (FSMState::lowstate && FSMState::lowstate->joystick.Y.on_pressed) {
        env->set_text_key("apose");
    }
}

void State_RLBase::handle_text_command_msg(const std::string& key)
{
    if (!has_goal_command_observation_) {
        env->set_text_key(key);
        return;
    }

    if (!env->has_text_key(key)) {
        env->set_text_key(key);
        return;
    }

    if (key == goal_idle_text_key_) {
        env->set_text_key(key);
        return;
    }

    goal_active_text_key_ = key;
    if (env->has_goal_received() && !goal_reached_idle_sent_) {
        env->set_text_key(goal_active_text_key_);
        spdlog::info("Goal active text command updated online: '{}'", goal_active_text_key_);
    } else {
        spdlog::info("Goal active text command queued for next goal: '{}'", goal_active_text_key_);
    }
}

double State_RLBase::goal_distance_xy() const
{
    const Eigen::Vector3f goal_w = env->get_goal_position();
    const Eigen::Vector3f root_w = env->robot->data.root_pos_w;
    return std::hypot(
        static_cast<double>(goal_w.x() - root_w.x()),
        static_cast<double>(goal_w.y() - root_w.y())
    );
}

void State_RLBase::publish_goal_marker(double distance_xy)
{
    if (!goal_marker_pub_) {
        return;
    }

    const Eigen::Vector3f goal_w = env->get_goal_position();
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = goal_marker_frame_id_;
    marker.header.stamp = node_ ? node_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
    marker.ns = "physhuman_goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = goal_w.x();
    marker.pose.position.y = goal_w.y();
    marker.pose.position.z = goal_w.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.r = distance_xy <= goal_reached_distance_ ? 0.1f : 1.0f;
    marker.color.g = distance_xy <= goal_reached_distance_ ? 0.9f : 0.2f;
    marker.color.b = 0.1f;
    marker.color.a = 0.9f;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    goal_marker_pub_->publish(marker);
}

void State_RLBase::monitor_goal()
{
    if (!has_goal_command_observation_) {
        return;
    }

    env->robot->update();
    const double distance_xy = goal_distance_xy();
    publish_goal_marker(distance_xy);

    goal_distance_log_counter_++;
    if (goal_distance_log_counter_ % goal_distance_log_interval_ == 0) {
        const Eigen::Vector3f goal_w = env->get_goal_position();
        spdlog::info(
            "[Goal] state={} distance_xy={:.3f}m target_xy=({:.3f}, {:.3f}) reached_threshold={:.3f}m",
            getStateString(),
            distance_xy,
            goal_w.x(),
            goal_w.y(),
            goal_reached_distance_
        );
    }

    if (goal_reached_idle_sent_) {
        return;
    }

    if (!env->has_goal_received()) {
        return;
    }

    if (distance_xy > goal_reached_distance_) {
        goal_reached_since_ = rclcpp::Time(0, 0, node_ ? node_->get_clock()->get_clock_type() : RCL_ROS_TIME);
        return;
    }

    if (!node_) {
        return;
    }

    const auto now = node_->now();
    if (goal_reached_since_.nanoseconds() == 0) {
        goal_reached_since_ = now;
        return;
    }

    if ((now - goal_reached_since_).seconds() >= goal_reached_hold_time_) {
        env->set_text_key(goal_idle_text_key_);
        goal_reached_idle_sent_ = true;
        spdlog::info(
            "[Goal] reached: distance_xy={:.3f}m for {:.2f}s. Text command set to '{}'.",
            distance_xy,
            goal_reached_hold_time_,
            goal_idle_text_key_
        );
    }
}
