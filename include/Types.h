#pragma once

#include "unitree/dds_wrapper/robots/go2/go2.h"
#include "unitree/dds_wrapper/robots/g1/g1.h"
#include "subscriber.h"
#include "publisher.h"

using LowCmd_t = unitree::robot::g1::publisher::LowCmd;
using LowState_t = unitree::robot::g1::subscription::LowState;
using SportModeState_t = unitree::robot::go2::subscription::SportModeState;
using MotionCaptureState_t = MotionCaptureState;
using LowCmdROS1_t = LowCmdROS1;
using LowStateROS1_t = LowStateROS1;
using JointState_t = JointState;
using VelocityMarkers_t = VelocityMarkers;
using TorsoLinkOdometry_t = TorsoLinkOdometry;
