#pragma once
#include "types.hpp"
#include <Eigen/Geometry>

namespace training {
// 第四讲使用的跨帧跟踪状态。
// 记录上一帧在「云台固定系」下的目标位置与速度，用于帧间速度估计与平滑。
struct TrackingState {
  bool last_valid = false;                                  // 上一帧是否有有效观测
  Eigen::Vector3d last_position = Eigen::Vector3d::Zero();   // 上一帧目标位置（云台系，单位 m）
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();        // 平滑后的目标速度（云台系，单位 m/s）
  TimePoint last_time{};                                    // 上一帧时间戳
};
}  // namespace training
