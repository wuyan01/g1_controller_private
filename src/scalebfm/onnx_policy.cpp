#include "scalebfm/controller.hpp"
#include <onnxruntime_cxx_api.h>
#include <openssl/evp.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace scalebfm {
namespace {
constexpr std::array<const char*,9> input_names{
    "root_quat_buffer", "base_ang_vel_buffer", "dof_pos_buffer", "dof_vel_buffer",
    "last_action_buffer", "target_body_pos_future_to_robot_base",
    "target_body_rot_future_to_robot_base", "mode_index", "time_offsets"};
constexpr std::array<const char*,2> output_names{"joint_position_targets", "raw_actions"};
const std::array<std::vector<int64_t>,9> shapes{{
    {1,3,4},{1,3,3},{1,3,29},{1,3,29},{1,3,29},{1,6,14,3},{1,6,14,4},{1},{1,6,1}}};
void require(bool ok,const std::string& message) {
  if(!ok) throw std::runtime_error("ScaleBFM ONNX: "+message);
}
} // namespace
struct OnnxPolicy::Impl {
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING,"scalebfm"};
  Ort::SessionOptions options;
  Ort::Session session{nullptr};
  Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);
  explicit Impl(const std::string& path,int threads) {
    require(threads>=1,"inference threads must be positive");
    options.SetIntraOpNumThreads(threads);options.SetInterOpNumThreads(1);
    options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    // Deliberately CPU by default. CUDA/TensorRT require a separate measured
    // latency qualification and provider-specific installation.
    session=Ort::Session(env,path.c_str(),options);
    require(session.GetInputCount()==9 && session.GetOutputCount()==2,"expected nine named inputs and two outputs");
    Ort::AllocatorWithDefaultOptions allocator;
    for(size_t i=0;i<9;++i) {
      const auto name=session.GetInputNameAllocated(i,allocator);
      require(std::string(name.get())==input_names[i],"unexpected input order/name at "+std::to_string(i));
      const auto type_info=session.GetInputTypeInfo(i);
      const auto info=type_info.GetTensorTypeAndShapeInfo();
      require(info.GetShape()==shapes[i],"input shape mismatch: "+std::string(input_names[i]));
      require(info.GetElementType()==(i>=7?ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT),"input dtype mismatch");
    }
    for(size_t i=0;i<2;++i) {
      const auto name=session.GetOutputNameAllocated(i,allocator);
      require(std::string(name.get())==output_names[i],"output name mismatch");
      const auto type_info=session.GetOutputTypeInfo(i);
      const auto info=type_info.GetTensorTypeAndShapeInfo();
      require(info.GetShape()==std::vector<int64_t>({1,29}) && info.GetElementType()==ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT,"output shape/dtype mismatch");
    }
  }
};
OnnxPolicy::OnnxPolicy(const std::string& path,int threads,const std::string& expected_sha256) {
  if(!expected_sha256.empty()) {
    require(expected_sha256.size()==64,"invalid expected SHA256");
    std::ifstream stream(path,std::ios::binary);require(stream.good(),"cannot open model for hashing");
    std::unique_ptr<EVP_MD_CTX,decltype(&EVP_MD_CTX_free)> context(EVP_MD_CTX_new(),EVP_MD_CTX_free);
    require(context && EVP_DigestInit_ex(context.get(),EVP_sha256(),nullptr)==1,"SHA256 initialization failed");
    std::array<char,65536> buffer{};
    while(stream) {stream.read(buffer.data(),buffer.size());require(EVP_DigestUpdate(context.get(),buffer.data(),stream.gcount())==1,"SHA256 update failed");}
    require(stream.eof(),"model read failed");unsigned char digest[EVP_MAX_MD_SIZE];unsigned int size=0;
    require(EVP_DigestFinal_ex(context.get(),digest,&size)==1,"SHA256 finalization failed");
    std::ostringstream hex;hex<<std::hex<<std::setfill('0');
    for(unsigned int i=0;i<size;++i) hex<<std::setw(2)<<static_cast<unsigned int>(digest[i]);
    require(hex.str()==expected_sha256,"model SHA256 does not match deployment metadata");
  }
  impl_=std::make_unique<Impl>(path,threads);
}
OnnxPolicy::~OnnxPolicy()=default;
Output OnnxPolicy::infer(const Inputs& input) {
  // ORT's borrowed tensor API takes mutable memory. A local copy prevents any
  // runtime/kernel implementation from modifying retained controller history.
  Inputs data=input;
  std::vector<Ort::Value> values;values.reserve(9);
  auto add_float=[&](auto& a,size_t i){values.push_back(Ort::Value::CreateTensor<float>(impl_->memory,a.data(),a.size(),shapes[i].data(),shapes[i].size()));};
  add_float(data.root_rotation,0);add_float(data.root_ang_vel,1);add_float(data.dof_pos,2);
  add_float(data.dof_vel,3);add_float(data.actions,4);add_float(data.reference_pos,5);add_float(data.reference_rot,6);
  values.push_back(Ort::Value::CreateTensor<int64_t>(impl_->memory,data.mode.data(),data.mode.size(),shapes[7].data(),shapes[7].size()));
  values.push_back(Ort::Value::CreateTensor<int64_t>(impl_->memory,data.future_idx.data(),data.future_idx.size(),shapes[8].data(),shapes[8].size()));
  auto result=impl_->session.Run(Ort::RunOptions{nullptr},input_names.data(),values.data(),values.size(),output_names.data(),output_names.size());
  Output output;
  std::copy_n(result[0].GetTensorData<float>(),29,output.targets_actor.begin());
  std::copy_n(result[1].GetTensorData<float>(),29,output.raw_actions.begin());
  return output;
}
} // namespace scalebfm
