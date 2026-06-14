#pragma once

#include <cstdint>

namespace field {

struct StairPose {
  int16_t x;
  int16_t y;
  int16_t yaw;
};

// 当前楼梯直线测试坐标
constexpr StairPose kStairFrontPose{1880, 1600, 0};      // center0
constexpr StairPose kStairClosePose{2230, 1600, 0};      // close1
constexpr StairPose kStairHighDrivePose{2920, 1600, 0};  // high_drive1
constexpr StairPose kStairCenterPose{3220, 1600, 0};     // center1
constexpr int16_t kStairSpanMm = 1340;
constexpr uint8_t kStairMaxLevel = 3;

}  // namespace field
