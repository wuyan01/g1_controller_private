#include "scalebfm/controller.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <sstream>
#include <iomanip>
#include <stdexcept>

namespace scalebfm {
namespace {
void require(bool good, const std::string& message) {
  if (!good) throw std::runtime_error("ScaleBFM: " + message);
}
bool finite(float x) { return std::isfinite(x); }
template <class T> bool all_finite(const T& a) {
  return std::all_of(a.begin(), a.end(), [](auto v) { return std::isfinite(v); });
}
template <size_t N> std::array<float, N> array(const YAML::Node& n, const char* name) {
  require(n && n.IsSequence() && n.size() == N, std::string("metadata shape: ") + name);
  std::array<float, N> out{};
  for (size_t i = 0; i < N; ++i) out[i] = n[i].as<float>();
  require(all_finite(out), std::string("nonfinite metadata: ") + name);
  return out;
}
Eigen::Vector3f vector(const YAML::Node& n) {
  const auto v = array<3>(n, "vector3"); return {v[0], v[1], v[2]};
}
Eigen::Quaternionf quaternion(const YAML::Node& n) {
  const auto q = array<4>(n, "quaternion_wxyz");
  Eigen::Quaternionf result(q[0], q[1], q[2], q[3]);
  require(std::abs(result.norm() - 1.f) < .01f, "metadata quaternion is not unit length");
  return result.normalized();
}
float yaw(const Eigen::Quaternionf& q) {
  return std::atan2(2.f * (q.w()*q.z() + q.x()*q.y()),
                    1.f - 2.f * (q.y()*q.y() + q.z()*q.z()));
}
float wrap(float x) { return std::atan2(std::sin(x), std::cos(x)); }
Eigen::Quaternionf heading(float angle) {
  return Eigen::Quaternionf(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
}
void check_pose(const Pose& pose) {
  require(pose.position.allFinite() && pose.orientation.coeffs().allFinite(), "nonfinite wrist pose");
  require(std::abs(pose.orientation.norm() - 1.f) < .01f, "wrist quaternion must be unit WXYZ");
}
template<size_t N, size_t D> void append(std::array<float,N>& history, const std::array<float,D>& value, bool reset) {
  static_assert(N == 3*D);
  if (reset) {
    for(size_t row=0; row<3; ++row) std::copy(value.begin(),value.end(),history.begin()+row*D);
  } else {
    for(size_t i=0; i<2*D; ++i) history[i] = history[i+D];
    std::copy(value.begin(),value.end(),history.begin()+2*D);
  }
}
}  // namespace

Metadata Metadata::load(const std::string& path) {
  const auto node = YAML::LoadFile(path);
  require(node["schema_version"].as<int>() == 1, "unsupported metadata schema");
  Metadata m;
  m.joint_names = node["joint_names"].as<std::vector<std::string>>();
  require(node["action_names"].as<std::vector<std::string>>() == m.joint_names,
          "action_names must exactly match joint_names");
  m.sdk_joint_names = node["sdk_joint_names"].as<std::vector<std::string>>();
  m.selected_body_names = node["selected_body_names"].as<std::vector<std::string>>();
  require(node["history_buffer_size"].as<int>() == 3, "history must have three frames");
  m.default_q_actor = array<29>(node["default_dof_pos"], "default_dof_pos");
  m.kp_actor = array<29>(node["stiffness"], "stiffness");
  m.kd_actor = array<29>(node["damping"], "damping");
  m.action_scale_actor = array<29>(node["action_scale"], "action_scale");
  m.future_idx = array<6>(node["future_idx"], "future_idx");
  m.control_dt = node["control_dt"].as<float>();
  m.checkpoint_sha256 = node["checkpoint_sha256"].as<std::string>();
  m.onnx_sha256 = node["onnx_sha256"].as<std::string>();
  require(node["joint_limits"].size() == 29, "29 joint limits required");
  for (size_t i=0;i<29;++i) {
    auto limits=array<2>(node["joint_limits"][i],"joint_limits");
    m.lower_actor[i]=limits[0]; m.upper_actor[i]=limits[1];
  }
  for(const auto& n: node["kinematics"]) {
    Body b;
    b.name=n["name"].as<std::string>(); b.parent=n["parent"].as<int>();
    b.translation=vector(n["translation"]); b.rotation=quaternion(n["rotation_wxyz"]);
    b.axis=vector(n["axis"]); b.joint_reference=n["joint_reference"].as<float>();
    const auto joint=n["joint_name"].as<std::string>();
    if(!joint.empty()) {
      auto it=std::find(m.joint_names.begin(),m.joint_names.end(),joint);
      require(it!=m.joint_names.end(),"unknown FK joint "+joint);
      b.joint_actor=static_cast<int>(it-m.joint_names.begin());
    }
    m.bodies.push_back(b);
  }
  m.validate(); return m;
}
void Metadata::validate() {
  require(joint_names.size()==29 && sdk_joint_names.size()==29 && selected_body_names.size()==14,
          "unsupported joint/body count");
  require(std::set<std::string>(joint_names.begin(),joint_names.end()).size()==29 &&
          std::set<std::string>(sdk_joint_names.begin(),sdk_joint_names.end()).size()==29,
          "duplicate joint names");
  require(std::set<std::string>(selected_body_names.begin(),selected_body_names.end()).size()==14,
          "duplicate selected body names");
  const auto valid_sha256=[](const std::string& h) {
    return h.size()==64 && std::all_of(h.begin(),h.end(),[](char c){return (c>='0' && c<='9') || (c>='a' && c<='f');});
  };
  require(valid_sha256(checkpoint_sha256) && valid_sha256(onnx_sha256),"missing or invalid artifact SHA256");
  require(finite(control_dt) && std::abs(control_dt-.02f)<1e-7f,"control period must be 20 ms");
  for(size_t i=0;i<29;++i) {
    auto it=std::find(sdk_joint_names.begin(),sdk_joint_names.end(),joint_names[i]);
    require(it!=sdk_joint_names.end(),"missing SDK joint "+joint_names[i]);
    actor_to_sdk[i]=static_cast<int>(it-sdk_joint_names.begin());
    require(finite(kp_actor[i]) && kp_actor[i]>0 && finite(kd_actor[i]) && kd_actor[i]>=0,
            "invalid actuator gains");
    require(finite(lower_actor[i]) && finite(upper_actor[i]) && lower_actor[i]<upper_actor[i],"invalid joint limits");
    require(finite(default_q_actor[i]) && default_q_actor[i]>=lower_actor[i] && default_q_actor[i]<=upper_actor[i],"invalid default pose");
    require(finite(action_scale_actor[i]) && action_scale_actor[i]>0,"invalid action scaling");
  }
  for(auto f: future_idx) require(finite(f) && f>=0 && f==std::floor(f),"future offsets must be nonnegative steps");
  require(bodies.size()==30 && bodies[0].name=="pelvis" && bodies[0].parent==-1 && bodies[0].joint_actor==-1,"expected pelvis-rooted 30-body FK");
  std::set<int> used; std::set<std::string> names;
  for(size_t i=0;i<bodies.size();++i) {
    auto& b=bodies[i];
    require(names.insert(b.name).second,"duplicate FK body");
    require(b.translation.allFinite() && b.rotation.coeffs().allFinite() && finite(b.joint_reference),"invalid FK transform");
    require(std::abs(b.rotation.norm()-1.f)<.01f,"nonunit FK quaternion"); b.rotation.normalize();
    if(i) {
      require(b.parent>=0 && b.parent<static_cast<int>(i),"FK parent must precede child");
      require(b.joint_actor>=0 && b.joint_actor<29 && used.insert(b.joint_actor).second,"FK joints must be unique");
      require(b.axis.allFinite() && std::abs(b.axis.norm()-1.f)<.01f,"nonunit FK hinge axis"); b.axis.normalize();
    }
  }
  for(size_t i=0;i<14;++i) {
    auto it=std::find_if(bodies.begin(),bodies.end(),[&](auto& b){return b.name==selected_body_names[i];});
    require(it!=bodies.end(),"missing selected FK body"); selected_body_indices[i]=static_cast<int>(it-bodies.begin());
  }
}
Joints Metadata::actor_to_sdk_values(const Joints& a) const {
  Joints result{}; for(size_t i=0;i<29;++i) result[actor_to_sdk[i]]=a[i]; return result;
}
Controller::Controller(const std::string& model, const std::string& metadata, Options options)
  : Controller(Metadata::load(metadata),std::make_unique<OnnxPolicy>(model,options.inference_threads,Metadata::load(metadata).onnx_sha256),options) {}
Controller::Controller(Metadata metadata,std::unique_ptr<Policy> policy,Options options)
  : metadata_(std::move(metadata)),policy_(std::move(policy)),options_(options) {
  metadata_.validate(); require(policy_!=nullptr,"policy required");
  require(finite(options.max_target_lead_rad) && options.max_target_lead_rad>0 &&
          finite(options.target_rate_notice_rad_s) && options.target_rate_notice_rad_s>0,"invalid PD guard settings");
  require(options.native_kp_sdk.has_value()==options.native_kd_sdk.has_value(),"both native gain arrays must be supplied");
  kp_sdk_=options.native_kp_sdk.value_or(metadata_.kp_sdk());
  kd_sdk_=options.native_kd_sdk.value_or(metadata_.kd_sdk());
  for(size_t i=0;i<29;++i) {
    auto sdk=metadata_.actor_to_sdk[i];
    require(finite(kp_sdk_[sdk]) && kp_sdk_[sdk]>0 && finite(kd_sdk_[sdk]) && kd_sdk_[sdk]>=0,"invalid native gains");
    if(metadata_.joint_names[i].rfind("waist_",0)!=0)
      require(std::abs(kp_sdk_[sdk]-metadata_.kp_actor[i])<1e-4 && std::abs(kd_sdk_[sdk]-metadata_.kd_actor[i])<1e-4,
              "non-waist gains must match the exported training gains");
  }
  for(size_t side=0;side<2;++side) {
    const auto name=std::string(side==0?"left":"right")+"_wrist_yaw_link";
    const auto it=std::find(metadata_.selected_body_names.begin(),metadata_.selected_body_names.end(),name);
    require(it!=metadata_.selected_body_names.end(),"missing wrist reference slot");
    wrist_slots_[side]=static_cast<int>(it-metadata_.selected_body_names.begin());
  }
  for(size_t i=0;i<6;++i) inputs_.future_idx[i]=static_cast<int64_t>(metadata_.future_idx[i]);
}
std::string Controller::joint_diagnostic(std::size_t i, const State& s,
                                         std::optional<float> requested_target) const {
  const auto sdk = metadata_.actor_to_sdk[i];
  const float q = s.joint_positions_sdk[sdk];
  const float dq = s.joint_velocities_sdk[sdk];
  std::ostringstream out;
  out << std::setprecision(9) << "tick=" << pd_diagnostics_.tick
      << " joint=" << metadata_.joint_names[i] << " sdk_index=" << sdk
      << " q=" << q << " dq=" << dq
      << " measured_bounds=[" << metadata_.lower_actor[i] << ',' << metadata_.upper_actor[i]
      << "] measured_range_tolerance=0.05";
  if (requested_target) {
    const float value = *requested_target;
    out << " q_des=" << value << " previous_q_des=" << previous_targets_[sdk]
        << " target_lead=" << value - q << " lead_limit=" << options_.max_target_lead_rad
        << " target_rate=" << (value - previous_targets_[sdk]) / metadata_.control_dt
        << " rate_notice_threshold=" << options_.target_rate_notice_rad_s
        << " estimated_pd_torque_Nm=" << kp_sdk_[sdk] * (value - q) - kd_sdk_[sdk] * dq;
  } else {
    out << " q_des=unavailable estimated_pd_torque_Nm=unavailable";
  }
  return out.str();
}
void Controller::validate_state(const State& s) const {
  require(s.pelvis_quat_w.coeffs().allFinite() && std::abs(s.pelvis_quat_w.norm()-1.f)<.01f,"invalid pelvis quaternion");
  require(s.pelvis_position_w.allFinite() && s.pelvis_ang_vel_b.allFinite(),"nonfinite pelvis state");
  for (size_t i = 0; i < 29; ++i) {
    const auto sdk = metadata_.actor_to_sdk[i];
    const float q = s.joint_positions_sdk[sdk], dq = s.joint_velocities_sdk[sdk];
    if (!finite(q) || !finite(dq))
      require(false, "nonfinite measured joint state: " + joint_diagnostic(i, s));
    if (q < metadata_.lower_actor[i] - .05f || q > metadata_.upper_actor[i] + .05f)
      require(false, "measured joint exceeds mechanical range: " + joint_diagnostic(i, s));
  }
}
std::vector<Pose> Controller::forward_kinematics(const State& s) const {
  validate_state(s);
  std::vector<Pose> poses(metadata_.bodies.size());
  poses[0]={s.pelvis_position_w,s.pelvis_quat_w.normalized()};
  for(size_t i=1;i<poses.size();++i) {
    const auto& b=metadata_.bodies[i]; const auto& parent=poses[b.parent];
    const auto q=s.joint_positions_sdk[metadata_.actor_to_sdk[b.joint_actor]]-b.joint_reference;
    poses[i].position=parent.position+parent.orientation*b.translation;
    poses[i].orientation=(parent.orientation*b.rotation*Eigen::Quaternionf(Eigen::AngleAxisf(q,b.axis))).normalized();
  }
  return poses;
}
void Controller::update_state_history(const State& s,bool reset_history) {
  auto q=s.pelvis_quat_w.normalized();
  // Quaternion signs are physically equivalent; continuity avoids input jumps.
  if(!reset_history && q.w()*inputs_.root_rotation[8]+q.x()*inputs_.root_rotation[9]+
       q.y()*inputs_.root_rotation[10]+q.z()*inputs_.root_rotation[11]<0) q.coeffs()*=-1;
  append(inputs_.root_rotation,std::array<float,4>{q.w(),q.x(),q.y(),q.z()},reset_history);
  append(inputs_.root_ang_vel,std::array<float,3>{s.pelvis_ang_vel_b.x(),s.pelvis_ang_vel_b.y(),s.pelvis_ang_vel_b.z()},reset_history);
  Joints pos{},vel{};
  for(size_t i=0;i<29;++i) {pos[i]=s.joint_positions_sdk[metadata_.actor_to_sdk[i]];vel[i]=s.joint_velocities_sdk[metadata_.actor_to_sdk[i]];}
  append(inputs_.dof_pos,pos,reset_history);append(inputs_.dof_vel,vel,reset_history);
  if(reset_history) inputs_.actions.fill(0); else append(inputs_.actions,raw_,false);
}
void Controller::reset(const State& s) {
  ready_=false; faulted_=true; pd_diagnostics_={}; validate_state(s);
  update_state_history(s,true);raw_.fill(0);previous_targets_=s.joint_positions_sdk;
  root_target_=s.pelvis_position_w;velocity_.setZero();yaw_=yaw(s.pelvis_quat_w.normalized());
  const auto poses=forward_kinematics(s);const auto inverse=s.pelvis_quat_w.normalized().conjugate();
  for(size_t i=0;i<14;++i) {
    const auto& p=poses[metadata_.selected_body_indices[i]];
    neutral_[i]={inverse*(p.position-s.pelvis_position_w),(inverse*p.orientation).normalized()};
  }
  wrist_mode_=false;steps_=0;inference_ms_=0;inputs_.mode[0]=0;ready_=true;faulted_=false;
}
void Controller::references(const State& s,const Eigen::Vector3f& command,const WristTargets& wrists) {
  require(command.allFinite(),"nonfinite velocity command");
  require(command.head<2>().norm()<=.300001f && std::abs(command.z())<=.600001f,"command exceeds 0.3 m/s or 0.6 rad/s envelope");
  const bool active=wrists.left.has_value() || wrists.right.has_value();
  if(wrists.left) check_pose(*wrists.left);
  if(wrists.right) check_pose(*wrists.right);
  if(active && !wrist_mode_) {
    const auto poses=forward_kinematics(s);
    for(size_t side=0;side<2;++side) held_wrists_[side]=poses[metadata_.selected_body_indices[wrist_slots_[side]]];
    root_target_=s.pelvis_position_w;velocity_.setZero();
  }
  wrist_mode_=active;
  const float dt=metadata_.control_dt;
  const Eigen::Vector3f desired_world=heading(yaw(s.pelvis_quat_w.normalized()))*Eigen::Vector3f(command.x(),command.y(),0);
  // Bound acceleration by its vector norm. Independent world-axis clipping
  // would permit sqrt(2) faster diagonal acceleration and introduce a lateral
  // reference when the same forward command is issued at a different yaw.
  Eigen::Vector2f velocity_delta = desired_world.head<2>() - velocity_;
  const float delta_norm = velocity_delta.norm();
  if (delta_norm > dt) velocity_delta *= dt / delta_norm;
  velocity_ += velocity_delta;
  root_target_.head<2>()+=dt*velocity_;
  const Eigen::Vector2f lead=root_target_.head<2>()-s.pelvis_position_w.head<2>();
  if(lead.norm()>.18f) root_target_.head<2>()=s.pelvis_position_w.head<2>()+lead.normalized()*.18f;
  yaw_=wrap(yaw_+dt*command.z());
  const auto rotation=heading(yaw_);const auto inverse=s.pelvis_quat_w.normalized().conjugate();
  for(size_t t=0;t<6;++t) for(size_t b=0;b<14;++b) {
    Pose p{rotation*neutral_[b].position+root_target_,(rotation*neutral_[b].orientation).normalized()};
    p.position.head<2>()+=metadata_.future_idx[t]*dt*velocity_;
    if(active) for(size_t side=0;side<2;++side) if(static_cast<int>(b)==wrist_slots_[side]) {
      const auto& target=side==0?wrists.left:wrists.right;
      p=target.value_or(held_wrists_[side]);
    }
    const auto local_position=(inverse*(p.position-s.pelvis_position_w)).eval();
    const Eigen::Quaternionf local_rotation=(inverse*p.orientation.normalized()).normalized();
    const size_t j=t*14+b;
    for(size_t k=0;k<3;++k) inputs_.reference_pos[j*3+k]=local_position[k];
    inputs_.reference_rot[j*4]=local_rotation.w();inputs_.reference_rot[j*4+1]=local_rotation.x();
    inputs_.reference_rot[j*4+2]=local_rotation.y();inputs_.reference_rot[j*4+3]=local_rotation.z();
  }
  inputs_.mode[0]=active?2:0;
}
Joints Controller::step(const State& s, const Eigen::Vector3f& command,
                        const WristTargets& wrists) {
  require(ready_ && !faulted_, "reset required before control (or after a fault)");
  pd_diagnostics_ = {};
  pd_diagnostics_.tick = steps_;
  pd_diagnostics_.measured_q_sdk = s.joint_positions_sdk;
  pd_diagnostics_.measured_dq_sdk = s.joint_velocities_sdk;
  try {
    validate_state(s);
    update_state_history(s, steps_ == 0);
    references(s, command, wrists);
    const auto start = std::chrono::steady_clock::now();
    const auto result = policy_->infer(inputs_);
    inference_ms_ = std::chrono::duration<float, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    Joints target{};
    for (size_t i = 0; i < 29; ++i) {
      const auto sdk = metadata_.actor_to_sdk[i];
      const float q = s.joint_positions_sdk[sdk], dq = s.joint_velocities_sdk[sdk];
      float value = result.targets_actor[i];
      // Targets already include checkpoint scales and defaults. Never apply
      // action-manager scaling. Previous-action history remains raw.
      if (metadata_.joint_names[i].rfind("waist_", 0) == 0) {
        value = q + metadata_.kp_actor[i] / kp_sdk_[sdk] * (value - q)
            + (kd_sdk_[sdk] - metadata_.kd_actor[i]) / kp_sdk_[sdk] * dq;
      }
      target[sdk] = value;
      pd_diagnostics_.requested_targets_sdk[sdk] = value;
      pd_diagnostics_.estimated_pd_torque_sdk[sdk] = kp_sdk_[sdk] * (value - q) - kd_sdk_[sdk] * dq;
      pd_diagnostics_.target_lead_sdk[sdk] = value - q;
      pd_diagnostics_.target_rate_sdk[sdk] = (value - previous_targets_[sdk]) / metadata_.control_dt;
      pd_diagnostics_.max_abs_target_rate_rad_s = std::max(
          pd_diagnostics_.max_abs_target_rate_rad_s,
          std::abs(pd_diagnostics_.target_rate_sdk[sdk]));
    }
    pd_diagnostics_.target_rate_notice_exceeded =
        pd_diagnostics_.max_abs_target_rate_rad_s > options_.target_rate_notice_rad_s;
    pd_diagnostics_.has_policy_output = true;
    // q_des is a PD spring equilibrium, not a measured joint angle. The
    // upstream actor can request q_des outside the physical joint range to
    // produce bounded effort while q is inside it. Preserve the actor target;
    // native actuators enforce effort saturation at each physics step. A
    // target clamp here would change the policy's behavior over the 20 ms hold.
    for (size_t i = 0; i < 29; ++i) {
      const auto sdk = metadata_.actor_to_sdk[i];
      const auto reject = [&](const char* reason) {
        require(false, std::string(reason) + joint_diagnostic(i, s, target[sdk]));
      };
      if (!finite(result.raw_actions[i])) reject("nonfinite raw actor output: ");
      if (!finite(target[sdk]) || !finite(pd_diagnostics_.estimated_pd_torque_sdk[sdk]))
        reject("nonfinite PD request: ");
      if (std::abs(pd_diagnostics_.target_lead_sdk[sdk]) > options_.max_target_lead_rad)
        reject("PD target lead guard: ");
      // Target rate is telemetry, not joint velocity. Upstream applies the
      // native actor's discrete PD targets directly; rejecting or clamping a
      // target step can remove an intended balance correction.

    }
    raw_ = result.raw_actions;
    previous_targets_ = target;
    ++steps_;
    return target;
  } catch (const std::exception& error) {
    faulted_ = true;
    pd_diagnostics_.fault = error.what();
    throw;
  } catch (...) {
    faulted_ = true;
    pd_diagnostics_.fault = "Unknown policy/control failure";
    throw;
  }
}
} // namespace scalebfm
