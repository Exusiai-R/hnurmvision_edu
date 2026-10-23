#include "io_context.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <hnurm_interfaces/msg/vision_recv_data.hpp>
#include <hnurm_interfaces/msg/vision_send_data.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Geometry>
#include <cmath>
#include <thread>
#include <mutex>
#include <atomic>
#include <optional>

namespace training::io {

namespace {
#ifndef M_PI
constexpr double M_PI = 3.14159265358979323846;
#endif

// 把 sensor_msgs::Image 转成 cv::Mat(BGR)。仿真器发布的是 rgb8。
cv::Mat to_mat(const sensor_msgs::msg::Image& msg) {
  const int w = static_cast<int>(msg.width);
  const int h = static_cast<int>(msg.height);
  if (w <= 0 || h <= 0 || msg.data.empty()) return {};
  const std::string& enc = msg.encoding;
  const auto* d = msg.data.data();
  if (enc == "rgb8") {
    cv::Mat rgb(h, w, CV_8UC3, const_cast<uint8_t*>(d), msg.step);
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    return bgr;
  }
  if (enc == "bgr8") {
    return cv::Mat(h, w, CV_8UC3, const_cast<uint8_t*>(d), msg.step).clone();
  }
  if (enc == "rgba8") {
    cv::Mat rgba(h, w, CV_8UC4, const_cast<uint8_t*>(d), msg.step);
    cv::Mat bgr;
    cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
    return bgr;
  }
  if (enc == "bgra8") {
    cv::Mat bgra(h, w, CV_8UC4, const_cast<uint8_t*>(d), msg.step);
    cv::Mat bgr;
    cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
    return bgr;
  }
  if (enc == "mono8" || enc == "8UC1") {
    cv::Mat gray(h, w, CV_8UC1, const_cast<uint8_t*>(d), msg.step);
    cv::Mat bgr;
    cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
  }
  return {};
}

// 单例桥接：保存最新图像与云台下位机数据，并在后台线程 spin ROS2。
struct RosBridge {
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<hnurm_interfaces::msg::VisionSendData>::SharedPtr cmd_pub;
  std::mutex mutex;
  cv::Mat latest_image;
  hnurm_interfaces::msg::VisionRecvData latest_recv;
  bool has_image = false;
  bool has_recv = false;
  std::thread spin_thread;
  std::atomic<bool> running{false};

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat img = to_mat(*msg);
    if (img.empty()) return;
    std::lock_guard<std::mutex> lk(mutex);
    latest_image = img.clone();
    has_image = true;
  }
  void on_recv(const hnurm_interfaces::msg::VisionRecvData::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex);
    latest_recv = *msg;
    has_recv = true;
  }
};

RosBridge g_bridge;
}  // namespace

void IoContext::open() {
  if (!rclcpp::ok()) {
    rclcpp::InitOptions opts;
    opts.shutdown_on_signal = false;  // 不抢占 main 的 SIGINT 处理
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv, opts);
  }
  g_bridge.node = std::make_shared<rclcpp::Node>("training_vision");
  g_bridge.node->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10,
      [](const sensor_msgs::msg::Image::SharedPtr m) { g_bridge.on_image(m); });
  g_bridge.node->create_subscription<hnurm_interfaces::msg::VisionRecvData>(
      "/vision_receive_data", 10,
      [](const hnurm_interfaces::msg::VisionRecvData::SharedPtr m) { g_bridge.on_recv(m); });
  g_bridge.cmd_pub = g_bridge.node->create_publisher<hnurm_interfaces::msg::VisionSendData>(
      "/vision_send_data", 10);
  g_bridge.running = true;
  g_bridge.spin_thread = std::thread([]() { rclcpp::spin(g_bridge.node); });
}

std::optional<SensorData> IoContext::read() {
  if (!rclcpp::ok() || !g_bridge.node) return std::nullopt;
  std::lock_guard<std::mutex> lk(g_bridge.mutex);
  if (!g_bridge.has_image) return std::nullopt;

  SensorData sd;
  sd.image = g_bridge.latest_image.clone();
  sd.timestamp = std::chrono::steady_clock::now();

  if (g_bridge.has_recv) {
    // 云台当前姿态（度），构造四元数备用（瞄准中相对几何使其不影响结果）。
    const double yaw = g_bridge.latest_recv.yaw * M_PI / 180.0;
    const double pitch = g_bridge.latest_recv.pitch * M_PI / 180.0;
    const double roll = g_bridge.latest_recv.roll * M_PI / 180.0;
    const Eigen::Quaterniond qy(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()));
    const Eigen::Quaterniond qx(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()));
    const Eigen::Quaterniond qz(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()));
    sd.imu_q = qy * qx * qz;
    sd.bullet_speed = static_cast<double>(g_bridge.latest_recv.bullet_speed.data);
  } else {
    sd.imu_q = Eigen::Quaterniond::Identity();
    sd.bullet_speed = 0.0;
  }
  return sd;
}

void IoContext::write(const Command& command) {
  if (!g_bridge.cmd_pub || !g_bridge.node) return;
  hnurm_interfaces::msg::VisionSendData msg;
  msg.header.stamp = g_bridge.node->get_clock()->now();
  msg.header.frame_id = "gimbal_link";

  if (command.control) {
    // 仿真器：target_state==0 或 target_distance==-1 会被忽略；2=FIRE,1=CONVERGING。
    // 最终是否真正发射仍受 config.allow_fire 门控。
    const bool may_fire = command.shoot && config.allow_fire;
    msg.target_state.data = may_fire ? 2u : 1u;
    // 仿真器内部对 pitch 取反（pitch = -cmd.pitch），故这里对上正的 pitch 取负，
    // 使云台实际向上；yaw 为正（左）直接发送。
    msg.yaw = static_cast<float>(command.yaw * 180.0 / M_PI);
    msg.pitch = static_cast<float>(-command.pitch * 180.0 / M_PI);
    msg.target_distance = 1.0f;  // 占位正数，避免被当作无效目标
    msg.control_id = 0.0f;
  } else {
    msg.target_state.data = 0u;  // LOST -> 云台忽略
    msg.target_distance = -1.0f;
    msg.yaw = 0.0f;
    msg.pitch = 0.0f;
  }
  g_bridge.cmd_pub->publish(msg);
}

void IoContext::close() {
  g_bridge.running = false;
  if (rclcpp::ok()) rclcpp::shutdown();
  if (g_bridge.spin_thread.joinable()) g_bridge.spin_thread.join();
}

}  // namespace training::io
