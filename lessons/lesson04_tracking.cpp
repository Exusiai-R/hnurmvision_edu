#include "lessons.hpp"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>
#include <optional>

namespace training {
#ifndef M_PI
constexpr double M_PI = 3.14159265358979323846;
#endif

// 装甲板真实尺寸（米）。与检测端的大小板判定保持一致。
constexpr double kSmallArmorW = 0.135;
constexpr double kLargeArmorW = 0.230;
constexpr double kArmorH = 0.055;

// 第四讲：位姿、坐标与基础跟踪
// 流程：取最显著装甲板 -> solvePnP 求相机系位姿 -> 相机系转云台系 ->
//       转 IMU 稳定系（相对云台原点）并做帧间速度估计。
std::optional<Target> lesson04_tracking(const std::vector<Armor>& armors, TimePoint timestamp,
    const Eigen::Quaterniond& imu_q, const CameraCalibration& calibration, TrackingState& state) {
  if (armors.empty() || !calibration.valid()) return std::nullopt;

  // 选取面积最大的装甲板作为本帧目标（最近/最显著）。
  const Armor* best = nullptr;
  double best_area = -1.0;
  for (const auto& a : armors) {
    const double w = cv::norm(a.corners[1] - a.corners[0]);
    const double h = cv::norm(a.corners[2] - a.corners[1]);
    const double area = w * h;
    if (area > best_area) {
      best_area = area;
      best = &a;
    }
  }
  if (!best) return std::nullopt;

  const double aw = best->large_armor ? kLargeArmorW : kSmallArmorW;
  const double ah = kArmorH;
  // 物体点：装甲板平面 z=0，与角点顺序一致（左上、右上、右下、左下）。
  std::vector<cv::Point3f> object = {
      cv::Point3f(-aw / 2, ah / 2, 0),
      cv::Point3f(aw / 2, ah / 2, 0),
      cv::Point3f(aw / 2, -ah / 2, 0),
      cv::Point3f(-aw / 2, -ah / 2, 0),
  };
  std::vector<cv::Point2f> image = {
      best->corners[0], best->corners[1], best->corners[2], best->corners[3]};

  cv::Mat rvec, tvec;
  const bool ok = cv::solvePnP(object, image, calibration.matrix, calibration.distortion, rvec,
      tvec, false, cv::SOLVEPNP_IPPE);
  if (!ok) return std::nullopt;

  cv::Mat tcam;
  tvec.convertTo(tcam, CV_64F);
  const Eigen::Vector3d p_camera(tcam.at<double>(0), tcam.at<double>(1), tcam.at<double>(2));

  // 相机系 -> 云台系（固定外参）。
  const Eigen::Vector3d p_gimbal =
      calibration.camera_to_gimbal * p_camera + calibration.camera_to_gimbal_translation;

  // 帧间速度估计与指数平滑。
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  if (state.last_valid) {
    const double dt = std::chrono::duration<double>(timestamp - state.last_time).count();
    if (dt > 1e-3) {
      velocity = (p_gimbal - state.last_position) / dt;
      state.velocity = 0.6 * state.velocity + 0.4 * velocity;
    }
  }
  state.last_position = p_gimbal;
  state.last_time = timestamp;
  state.last_valid = true;

  // 转 IMU 稳定系（相对云台原点）。训练场景下底盘静止，imu_q 近似单位阵，
  // 该变换与下游命令映射自洽；远动目标下仍提供一致的世界系向量。
  Target target;
  target.position = imu_q * p_gimbal;
  target.velocity = imu_q * state.velocity;
  return target;
}
}  // namespace training
