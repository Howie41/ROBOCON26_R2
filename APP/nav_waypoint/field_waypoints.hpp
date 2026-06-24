#pragma once

#include <cstdint>

namespace field {

struct StairPose {
  int16_t x;
  int16_t y;
  int16_t yaw;
};

// 当前楼梯直线测试坐标
constexpr StairPose kStairFrontPose{1300, 1470, 0};      // center0
constexpr StairPose kStairClosePose{2180, 1470, 0};      // close1
constexpr StairPose kStairHighDrivePose{2820, 1470, 0};  // lower-trigger1
constexpr StairPose kStairCenterPose{3100, 1470, 0};     // center1
constexpr int16_t kStairSpanMm = 1200;
constexpr uint8_t kStairMaxLevel = 3;

}  // namespace field
