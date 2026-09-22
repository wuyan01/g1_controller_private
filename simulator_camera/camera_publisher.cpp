#include "camera_publisher.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {
constexpr int width = 640;
constexpr int height = 480;
constexpr double fovy = 58.0;  // Ideal pinhole approximation, not hardware calibration.
constexpr float near_clip = 0.01f;
constexpr float far_clip = 10.0f;
constexpr char optical_frame[] = "camera_color_optical_frame";

class CameraPublisher {
 public:
  CameraPublisher() {
    // No ROS CLI arguments: the vendor executable owns its argument parser.
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("mujoco_camera");
    buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    color_ = node_->create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw", qos);
    depth_ = node_->create_publisher<sensor_msgs::msg::Image>(
        "/camera/aligned_depth_to_color/image_raw", qos);
    color_info_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/camera/color/camera_info", qos);
    depth_info_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/camera/aligned_depth_to_color/camera_info", qos);
    cloud_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", qos);
    mjv_defaultScene(&scene_);
    mjv_defaultOption(&option_);
    mjv_defaultCamera(&camera_);
    rgb_.resize(width * height * 3);
    zbuffer_.resize(width * height);
    RCLCPP_INFO(node_->get_logger(), "MuJoCo RGB-D camera ready (640x480, up to 30 Hz); "
                "waiting for subscribers and torso_link -> %s TF", optical_frame);
  }

  void reset() {
    mjv_freeScene(&scene_);
    mjv_defaultScene(&scene_);
    allocated_ = false;
    ready_ = false;
  }

  void update(const mjModel* model, mjData* data) {
    ready_ = false;
    if (!rclcpp::ok() || !model || !data) return;
    auto now = std::chrono::steady_clock::now();
    if (now < next_frame_) return;
    next_frame_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                           std::chrono::duration<double>(1.0 / 30.0));
    if (!(color_->get_subscription_count() || depth_->get_subscription_count() ||
          color_info_->get_subscription_count() || depth_info_->get_subscription_count() ||
          cloud_->get_subscription_count())) return;
    int torso = mj_name2id(model, mjOBJ_BODY, "torso_link");
    if (torso < 0) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                          "Camera requires a torso_link body in the MuJoCo model");
      return;
    }
    geometry_msgs::msg::TransformStamped mount;
    try {
      mount = buffer_->lookupTransform("torso_link", optical_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException& error) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                          "Waiting for camera mount TF: %s", error.what());
      return;
    }
    const auto& q = mount.transform.rotation;
    const auto& t = mount.transform.translation;
    tf2::Transform body_to_optical(tf2::Quaternion(q.x, q.y, q.z, q.w),
                                   tf2::Vector3(t.x, t.y, t.z));
    const mjtNum* mat = data->xmat + 9 * torso;
    const mjtNum* pos = data->xpos + 3 * torso;
    tf2::Transform world_to_body(
        tf2::Matrix3x3(mat[0], mat[1], mat[2], mat[3], mat[4], mat[5],
                       mat[6], mat[7], mat[8]), tf2::Vector3(pos[0], pos[1], pos[2]));
    auto world_to_optical = world_to_body * body_to_optical;
    const auto eye = world_to_optical.getOrigin();
    const auto forward = world_to_optical.getBasis().getColumn(2);
    const auto up = -world_to_optical.getBasis().getColumn(1);
    if (!allocated_) {
      mjv_makeScene(model, &scene_, 10000);
      allocated_ = true;
    }
    // mjv_updateScene copies geometry while the physics mutex is held. The
    // renderer below only reads this independent scene, never live mjData.
    mjv_updateScene(model, data, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);
    scene_.stereo = mjSTEREO_NONE;
    for (auto& cam : scene_.camera) {
      for (int i = 0; i < 3; ++i) {
        cam.pos[i] = eye[i];
        cam.forward[i] = forward[i];
        cam.up[i] = up[i];
      }
      cam.orthographic = 0;
      cam.frustum_center = 0;
      cam.frustum_near = near_clip;
      cam.frustum_far = far_clip;
      cam.frustum_top = near_clip * std::tan(fovy * std::acos(-1.0) / 360.0);
      cam.frustum_bottom = -cam.frustum_top;
      cam.frustum_width = 2 * cam.frustum_top * width / height;
    }
    stamp_ = node_->now();  // Matches the existing controller's ROS wall-clock TF.
    ready_ = true;
  }

  void render(mjrContext* context) {
    if (!ready_) return;
    ready_ = false;
    const int previous_buffer = context->currentBuffer;
    const int previous_depth = context->readDepthMap;
    if (context->offWidth < width || context->offHeight < height)
      mjr_resizeOffscreen(std::max(width, context->offWidth),
                          std::max(height, context->offHeight), context);
    mjr_setBuffer(mjFB_OFFSCREEN, context);
    if (context->currentBuffer != mjFB_OFFSCREEN) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "MuJoCo offscreen framebuffer is unavailable");
      mjr_setBuffer(previous_buffer, context);
      return;
    }
    context->readDepthMap = mjDEPTH_ZERONEAR;
    const mjrRect viewport{0, 0, width, height};
    mjr_render(viewport, &scene_, context);
    mjr_readPixels(rgb_.data(), zbuffer_.data(), viewport, context);
    context->readDepthMap = previous_depth;
    mjr_setBuffer(previous_buffer, context);

    sensor_msgs::msg::Image color;
    color.header.stamp = stamp_;
    color.header.frame_id = optical_frame;
    color.width = width;
    color.height = height;
    color.encoding = "rgb8";
    color.step = width * 3;
    color.data.resize(width * height * 3);
    sensor_msgs::msg::Image depth;
    depth.header = color.header;
    depth.width = width;
    depth.height = height;
    depth.encoding = "16UC1";  // Millimeters, zero invalid, like RealSense.
    depth.step = width * 2;
    depth.data.resize(width * height * 2);
    const uint16_t endian_probe = 1;
    depth.is_bigendian = *reinterpret_cast<const uint8_t*>(&endian_probe) == 0;
    std::vector<float> meters(width * height);
    for (int y = 0; y < height; ++y) {
      int source_row = height - 1 - y;  // OpenGL reads from the bottom left.
      std::memcpy(color.data.data() + y * color.step,
                  rgb_.data() + source_row * color.step, color.step);
      for (int x = 0; x < width; ++x) {
        float raw = zbuffer_[source_row * width + x];
        float z = near_clip * far_clip / (far_clip - raw * (far_clip - near_clip));
        bool valid = raw < 1.0f && std::isfinite(z) && z < far_clip;
        uint16_t mm = valid ? static_cast<uint16_t>(std::lround(z * 1000)) : 0;
        int index = y * width + x;
        std::memcpy(depth.data.data() + index * 2, &mm, 2);
        meters[index] = valid ? z : std::numeric_limits<float>::quiet_NaN();
      }
    }
    sensor_msgs::msg::CameraInfo info;
    info.header = color.header;
    info.width = width;
    info.height = height;
    info.distortion_model = "plumb_bob";
    info.d.assign(5, 0.0);
    double focal = height / (2 * std::tan(fovy * std::acos(-1.0) / 360.0));
    double cx = (width - 1) / 2.0, cy = (height - 1) / 2.0;
    info.k = {focal, 0, cx, 0, focal, cy, 0, 0, 1};
    info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    info.p = {focal, 0, cx, 0, 0, focal, cy, 0, 0, 0, 1, 0};
    color_->publish(color);
    depth_->publish(depth);
    color_info_->publish(info);
    depth_info_->publish(info);
    if (cloud_->get_subscription_count()) {
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header = color.header;
      cloud.width = width;
      cloud.height = height;
      cloud.is_dense = false;
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
      modifier.resize(width * height);
      sensor_msgs::PointCloud2Iterator<float> px(cloud, "x"), py(cloud, "y"), pz(cloud, "z");
      sensor_msgs::PointCloud2Iterator<uint8_t> red(cloud, "r"), green(cloud, "g"), blue(cloud, "b");
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x, ++px, ++py, ++pz, ++red, ++green, ++blue) {
          int index = y * width + x;
          *pz = meters[index];
          *px = (x - cx) * *pz / focal;
          *py = (y - cy) * *pz / focal;
          *red = color.data[index * 3];
          *green = color.data[index * 3 + 1];
          *blue = color.data[index * 3 + 2];
        }
      }
      cloud_->publish(cloud);
    }
  }

 private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_, depth_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_, depth_info_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  mjvScene scene_{};
  mjvOption option_{};
  mjvCamera camera_{};
  bool allocated_ = false, ready_ = false;
  std::chrono::steady_clock::time_point next_frame_{};
  rclcpp::Time stamp_{0};
  std::vector<unsigned char> rgb_;
  std::vector<float> zbuffer_;
};

CameraPublisher& publisher() {
  static CameraPublisher camera;
  return camera;
}
}  // namespace

void g1_camera_reset() { publisher().reset(); }
void g1_camera_update(const mjModel* model, mjData* data) { publisher().update(model, data); }
void g1_camera_render(mjrContext* context) { publisher().render(context); }
