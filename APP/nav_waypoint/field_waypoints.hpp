#pragma once

#include <cstdint>

namespace field {

struct StairPose {
  int16_t x;
  int16_t y;
  int16_t yaw;
};

// 当前楼梯直线测试坐标
constexpr StairPose kStairFrontPose{2000, 1770, 0};      // stair-front
constexpr StairPose kStairClosePose{2365, 1770, 0};      // stair-close
constexpr StairPose kStairHighDrivePose{2975, 1770, 0};  // center - 280 mm
constexpr StairPose kStairCenterPose{3255, 1770, 0};     // stair-center
constexpr int16_t kStairSpanMm = 1200;
constexpr uint8_t kStairMaxLevel = 3;

}  // namespace field
