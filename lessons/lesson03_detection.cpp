#include "lessons.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

namespace training {
#ifndef M_PI
constexpr double M_PI = 3.14159265358979323846;
#endif

// 装甲板真实尺寸（米）。小板约 135x55，大板约 230x55（宽 x 高，含两灯条外缘）。
constexpr double kSmallArmorW = 0.135;
constexpr double kLargeArmorW = 0.230;
constexpr double kArmorH = 0.055;

// 单根灯条
struct LightBar {
  cv::Point2f center{};
  cv::Point2f dir{};    // 单位方向，指向「上」(图像 y 负方向)
  float length = 0;     // 主轴长度（像素）
  float width = 0;      // 短轴宽度（像素）
};

// 第三讲：装甲板检测
// 输入：CV_8UC3 BGR 图像。输出：角点顺序 左上、右上、右下、左下（像素）。
// 策略：亮度阈值找发光灯条 -> 轮廓拟合旋转矩形 -> 按平行/等长/水平配对 -> 取四角。
std::vector<Armor> lesson03_detection(const cv::Mat& image) {
  std::vector<Armor> armors;
  if (image.empty()) return armors;

  cv::Mat gray;
  if (image.channels() == 3)
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  else
    gray = image.clone();

  // 1. 亮度阈值（灯条是场景中最亮的部分），Otsu 自适应。
  cv::Mat bin;
  cv::threshold(gray, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(bin, bin, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, kernel);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<LightBar> lights;
  for (const auto& c : contours) {
    if (c.size() < 4) continue;
    if (cv::contourArea(c) < 20) continue;
    cv::RotatedRect rect = cv::minAreaRect(c);
    const float w = rect.size.width;
    const float h = rect.size.height;
    const float len = std::max(w, h);
    const float wid = std::min(w, h);
    if (len <= 0) continue;
    const float aspect = len / wid;  // 灯条细长
    if (aspect < 2.0f || aspect > 12.0f) continue;
    if (wid < 2.0f || wid > 30.0f) continue;

    float angle = rect.angle;
    cv::Point2f dir;
    if (w > h)
      dir = cv::Point2f(std::cos(angle * M_PI / 180.0f), std::sin(angle * M_PI / 180.0f));
    else
      dir = cv::Point2f(-std::sin(angle * M_PI / 180.0f), std::cos(angle * M_PI / 180.0f));
    if (dir.y > 0) dir = -dir;  // 统一指向上

    LightBar lb;
    lb.center = rect.center;
    lb.dir = dir;
    lb.length = len;
    lb.width = wid;
    lights.push_back(lb);
  }

  // 2. 配对：平行、等长、水平排列的两根灯条构成一块装甲板。
  for (size_t i = 0; i < lights.size(); ++i) {
    for (size_t j = i + 1; j < lights.size(); ++j) {
      const LightBar& a = lights[i];
      const LightBar& b = lights[j];
      if (std::abs(a.length - b.length) / std::max(a.length, b.length) > 0.4f) continue;
      if (std::abs(a.dir.dot(b.dir)) < 0.9f) continue;  // 两灯条方向平行
      cv::Point2f d = b.center - a.center;
      const float dist = cv::norm(d);
      if (dist < a.length * 0.5f || dist > a.length * 5.0f) continue;
      cv::Point2f perp(-a.dir.y, a.dir.x);  // 垂直于灯条（指向右）
      if (std::abs(d.dot(a.dir)) > dist * 0.5f) continue;  // 中心连线应垂直于灯条
      const bool swap_lr = d.dot(perp) < 0;                 // 保证 left 为左、right 为右
      const LightBar& left = swap_lr ? b : a;
      const LightBar& right = swap_lr ? a : b;

      // 3. 构造四角（图像坐标：左上、右上、右下、左下；原点在左上，y 向下）。
      //    dir 指向图像上方（-y），故顶边 = center + dir*half。
      cv::Point2f lh = left.dir * (left.length * 0.5f);
      cv::Point2f rh = right.dir * (right.length * 0.5f);
      Armor armor;
      armor.corners[0] = left.center + lh;   // 左上
      armor.corners[1] = right.center + rh;  // 右上
      armor.corners[2] = right.center - rh;  // 右下
      armor.corners[3] = left.center - lh;   // 左下

      // 大小板判定：外缘宽度 / 灯条长度，与距离无关。
      const float outer = dist + (left.width + right.width) * 0.5f;
      armor.large_armor = (outer / a.length) > 2.8f;
      armor.class_id = -1;  // 编号识别超出本讲范围
      armor.confidence = 0.6f;
      armors.push_back(armor);
    }
  }
  return armors;
}
}  // namespace training
