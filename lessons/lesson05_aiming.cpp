#include "lessons.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <optional>

namespace training {
#ifndef M_PI
constexpr double M_PI = 3.14159265358979323846;
#endif

constexpr double kGravity = 9.81;  // m/s^2

// 第五讲：弹道解算与瞄准
// 输入：目标（IMU 稳定系，相对云台原点，x 前 / y 左 / z 上，单位 m 与 m/s）。
// 输出：云台命令。yaw/pitch 经推导为「左正、上正」，并按下述约定送仿真器：
//   仿真器内部 yaw = cmd.yaw(rad)，pitch = -cmd.pitch(rad)，故 cmd.pitch 取负以得到上仰。
Command lesson05_aiming(const Target& target, TimePoint /*timestamp*/, double bullet_speed) {
  Command cmd{};
  if (!std::isfinite(bullet_speed) || bullet_speed <= 0) return cmd;  // 无安全弹道解

  const Eigen::Vector3d p = target.position;
  const Eigen::Vector3d v = target.velocity;
  const double dist = p.norm();
  if (dist < 1e-3) return cmd;

  // 飞行时间预估 + 一次迭代，给出带提前量的预测落点（相对云台原点）。
  double horiz = std::sqrt(p.x() * p.x() + p.y() * p.y());
  double t_flight = horiz / bullet_speed;
  Eigen::Vector3d p_pred = p + v * t_flight;
  horiz = std::sqrt(p_pred.x() * p_pred.x() + p_pred.y() * p_pred.y());
  t_flight = horiz / bullet_speed;
  p_pred = p + v * t_flight;

  const double x = p_pred.x();
  const double y = p_pred.y();
  const double z = p_pred.z();
  const double D = std::sqrt(x * x + y * y);  // 水平距离
  if (D < 1e-3) return cmd;

  // 抛物线解算：tan(theta) = (v^2 - sqrt(v^4 - g(g D^2 + 2 z v^2))) / (g D)，取近支（直接命中）。
  const double v2 = bullet_speed * bullet_speed;
  const double disc = v2 * v2 - kGravity * (kGravity * D * D + 2.0 * z * v2);
  double pitch = 0.0;
  if (disc < 0) {
    pitch = 0.5 * M_PI - 1e-3;  // 超出射程，给最大仰角尽力而为
  } else {
    pitch = std::atan((v2 - std::sqrt(disc)) / (kGravity * D));
  }

  const double yaw = std::atan2(y, x);  // 左为正

  cmd.control = true;
  cmd.shoot = true;  // 请求发射；最终是否开火由 IO 层 allow_fire 门控
  cmd.yaw = yaw;
  cmd.pitch = pitch;
  return cmd;
}
}  // namespace training
