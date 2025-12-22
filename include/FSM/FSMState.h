#pragma once

#include "Types.h"
#include "param.h"
#include "FSM/BaseState.h"
#include "isaaclab/devices/keyboard/keyboard.h"
#include "unitree_joystick_dsl.hpp"

class FSMState : public BaseState
{
public:
    FSMState(int state, std::string state_string) 
    : BaseState(state, state_string) 
    {
        spdlog::info("Initializing State_{} ...", state_string);

        auto transitions = param::config["FSM"][state_string]["transitions"];
        joint_names = param::config["robot"]["joint_names"].as<std::vector<std::string>>();

        if(transitions)
        {
            auto transition_map = transitions.as<std::map<std::string, std::string>>();

            for(auto it = transition_map.begin(); it != transition_map.end(); ++it)
            {
                std::string target_fsm = it->first;
                if(!FSMStringMap.right.count(target_fsm))
                {
                    spdlog::warn("FSM State_'{}' not found in FSMStringMap!", target_fsm);
                    continue;
                }

                int fsm_id = FSMStringMap.right.at(target_fsm);

                std::string condition = it->second;
                unitree::common::dsl::Parser p(condition);
                auto ast = p.Parse();
                auto func = unitree::common::dsl::Compile(*ast);
                registered_checks.emplace_back(
                    std::make_pair(
                        [func]()->bool{ return func(FSMState::lowstate->joystick); },
                        fsm_id
                    )
                );
            }
        }

        // register for all states
        registered_checks.emplace_back(
            std::make_pair(
                []()->bool{ return lowstate->isTimeout(); },
                FSMStringMap.right.at("Passive")
            )
        );
    }

    void pre_run()
    {
        lowstate->update();
        if(keyboard) keyboard->update();
        republish_lowstate_ros1();
        republish_jointstate();
        republish_baseodom();
    }

    void post_run()
    {
        lowcmd->unlockAndPublish();
        republish_lowcmd_ros1();
    }

    static std::unique_ptr<LowCmd_t> lowcmd;
    static std::shared_ptr<LowState_t> lowstate;
    static std::shared_ptr<Keyboard> keyboard;
    static std::shared_ptr<SportModeState_t> sportmodestate;
    static std::shared_ptr<MotionCaptureState_t> motioncapturestate;
    static std::shared_ptr<LowCmdROS1_t> lowcmd_ros1;
    static std::shared_ptr<LowStateROS1_t> lowstate_ros1;
    static std::shared_ptr<JointState_t> jointstate;
    static std::shared_ptr<BaseOdom_t> baseodom;
    static std::string network_interface;

private:

    std::vector<std::string> joint_names;

    void republish_lowstate_ros1()
    {
        /* ---------------- version ---------------- */
        for (int i = 0; i < 2; ++i) {
            lowstate_ros1->msg_.version[i] =
                lowstate->msg_.version()[i];
        }

        /* ---------------- modes ---------------- */
        lowstate_ros1->msg_.mode_pr =
            lowstate->msg_.mode_pr();

        lowstate_ros1->msg_.mode_machine =
            lowstate->msg_.mode_machine();

        lowstate_ros1->msg_.tick =
            lowstate->msg_.tick();

        /* ---------------- imu_state ---------------- */
        for (int i = 0; i < 4; ++i) {
            lowstate_ros1->msg_.imu_state.quaternion[i] =
                lowstate->msg_.imu_state().quaternion()[i];
        }

        for (int i = 0; i < 3; ++i) {
            lowstate_ros1->msg_.imu_state.gyroscope[i] =
                lowstate->msg_.imu_state().gyroscope()[i];

            lowstate_ros1->msg_.imu_state.accelerometer[i] =
                lowstate->msg_.imu_state().accelerometer()[i];

            lowstate_ros1->msg_.imu_state.rpy[i] =
                lowstate->msg_.imu_state().rpy()[i];
        }

        lowstate_ros1->msg_.imu_state.temperature =
            lowstate->msg_.imu_state().temperature();

        /* ---------------- motor_state ---------------- */
        for (int i = 0; i < 35; ++i) {
            lowstate_ros1->msg_.motor_state[i].mode =
                lowstate->msg_.motor_state()[i].mode();

            lowstate_ros1->msg_.motor_state[i].q =
                lowstate->msg_.motor_state()[i].q();

            lowstate_ros1->msg_.motor_state[i].dq =
                lowstate->msg_.motor_state()[i].dq();

            lowstate_ros1->msg_.motor_state[i].ddq =
                lowstate->msg_.motor_state()[i].ddq();

            lowstate_ros1->msg_.motor_state[i].tau_est =
                lowstate->msg_.motor_state()[i].tau_est();

            for (int j = 0; j < 2; ++j) {
                lowstate_ros1->msg_.motor_state[i].temperature[j] =
                    lowstate->msg_.motor_state()[i].temperature()[j];
            }

            lowstate_ros1->msg_.motor_state[i].vol =
                lowstate->msg_.motor_state()[i].vol();

            for (int j = 0; j < 2; ++j) {
                lowstate_ros1->msg_.motor_state[i].sensor[j] =
                    lowstate->msg_.motor_state()[i].sensor()[j];
            }

            lowstate_ros1->msg_.motor_state[i].motorstate =
                lowstate->msg_.motor_state()[i].motorstate();

            for (int j = 0; j < 4; ++j) {
                lowstate_ros1->msg_.motor_state[i].reserve[j] =
                    lowstate->msg_.motor_state()[i].reserve()[j];
            }
        }

        /* ---------------- wireless_remote ---------------- */
        for (int i = 0; i < 40; ++i) {
            lowstate_ros1->msg_.wireless_remote[i] =
                lowstate->msg_.wireless_remote()[i];
        }

        /* ---------------- reserve ---------------- */
        for (int i = 0; i < 4; ++i) {
            lowstate_ros1->msg_.reserve[i] =
                lowstate->msg_.reserve()[i];
        }

        /* ---------------- crc ---------------- */
        lowstate_ros1->msg_.crc =
            lowstate->msg_.crc();

        lowstate_ros1->publish();
    }

    void republish_jointstate()
    {
        jointstate->msg_.header.stamp = ros::Time::now();
        jointstate->msg_.name = joint_names;
        jointstate->msg_.position.resize(29);
        jointstate->msg_.velocity.resize(29);
        jointstate->msg_.effort.resize(29);

        for (int i = 0; i < 29; ++i) {
            jointstate->msg_.position[i] =
                lowstate->msg_.motor_state()[i].q();

            jointstate->msg_.velocity[i] =
                lowstate->msg_.motor_state()[i].dq();

            jointstate->msg_.effort[i] =
                lowstate->msg_.motor_state()[i].tau_est();
        }

        jointstate->publish();
    }

    void republish_baseodom()
    {
        baseodom->msg_.header.stamp = ros::Time::now();

        if(FSMState::network_interface == "lo") {
            // position
            baseodom->msg_.pose.pose.position.x =
                sportmodestate->msg_.position()[0];
            baseodom->msg_.pose.pose.position.y =
                sportmodestate->msg_.position()[1];
            baseodom->msg_.pose.pose.position.z =
                sportmodestate->msg_.position()[2];

            // orientation
            baseodom->msg_.pose.pose.orientation.w =
                lowstate->msg_.imu_state().quaternion()[0];
            baseodom->msg_.pose.pose.orientation.x =
                lowstate->msg_.imu_state().quaternion()[1];
            baseodom->msg_.pose.pose.orientation.y =
                lowstate->msg_.imu_state().quaternion()[2];
            baseodom->msg_.pose.pose.orientation.z =
                lowstate->msg_.imu_state().quaternion()[3];

            // linear velocity
            Eigen::Vector3f root_lin_vel_w(
                sportmodestate->msg_.velocity()[0],
                sportmodestate->msg_.velocity()[1],
                sportmodestate->msg_.velocity()[2]
            );
            Eigen::Quaternionf root_quat_w(
                lowstate->msg_.imu_state().quaternion()[0],
                lowstate->msg_.imu_state().quaternion()[1],
                lowstate->msg_.imu_state().quaternion()[2],
                lowstate->msg_.imu_state().quaternion()[3]
            );
            Eigen::Vector3f root_lin_vel_b;
            root_lin_vel_b = root_quat_w.conjugate() * root_lin_vel_w;

            baseodom->msg_.twist.twist.linear.x =
                root_lin_vel_b[0];
            baseodom->msg_.twist.twist.linear.y =
                root_lin_vel_b[1];
            baseodom->msg_.twist.twist.linear.z =
                root_lin_vel_b[2];

            // angular velocity
            baseodom->msg_.twist.twist.angular.x =
                lowstate->msg_.imu_state().gyroscope()[0];
            baseodom->msg_.twist.twist.angular.y =
                lowstate->msg_.imu_state().gyroscope()[1];
            baseodom->msg_.twist.twist.angular.z =
                lowstate->msg_.imu_state().gyroscope()[2];
        }
        else {
            baseodom->msg_.pose = motioncapturestate->msg_.pose;
            baseodom->msg_.twist = motioncapturestate->msg_.twist;
        }

        baseodom->publish();
    }

    void republish_lowcmd_ros1()
    {
        /* ---------------- modes ---------------- */
        lowcmd_ros1->msg_.mode_pr =
            lowcmd->msg_.mode_pr();

        lowcmd_ros1->msg_.mode_machine =
            lowcmd->msg_.mode_machine();

        /* ---------------- motor_cmd ---------------- */
        for (int i = 0; i < 35; ++i) {
            lowcmd_ros1->msg_.motor_cmd[i].mode =
                lowcmd->msg_.motor_cmd()[i].mode();

            lowcmd_ros1->msg_.motor_cmd[i].q =
                lowcmd->msg_.motor_cmd()[i].q();

            lowcmd_ros1->msg_.motor_cmd[i].dq =
                lowcmd->msg_.motor_cmd()[i].dq();

            lowcmd_ros1->msg_.motor_cmd[i].tau =
                lowcmd->msg_.motor_cmd()[i].tau();

            lowcmd_ros1->msg_.motor_cmd[i].kp =
                lowcmd->msg_.motor_cmd()[i].kp();

            lowcmd_ros1->msg_.motor_cmd[i].kd =
                lowcmd->msg_.motor_cmd()[i].kd();

            lowcmd_ros1->msg_.motor_cmd[i].reserve =
                lowcmd->msg_.motor_cmd()[i].reserve();
        }

        /* ---------------- reserve ---------------- */
        for (int i = 0; i < 4; ++i) {
            lowcmd_ros1->msg_.reserve[i] =
                lowcmd->msg_.reserve()[i];
        }

        /* ---------------- crc ---------------- */
        lowcmd_ros1->msg_.crc =
            lowcmd->msg_.crc();

        lowcmd_ros1->publish();
    }

};