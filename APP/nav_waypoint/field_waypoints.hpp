#pragma once

#include <cstdint>

namespace field {

struct StairPose {
  int16_t x;
  int16_t y;
  int16_t yaw;
};

// Merlin local coordinates, all relative to config.origin_x/y.
constexpr StairPose kStairFrontPose{2144, 1496, 0};      // stair-front
constexpr StairPose kStairClosePose{2509, 1496, 0};      // stair-close
constexpr StairPose kStairHighDrivePose{3119, 1496, 0};  // center - 280 mm
constexpr StairPose kStairCenterPose{3399, 1496, 0};     // stair-center
constexpr int16_t kStairSpanMm = 1200;
constexpr uint8_t kStairMaxLevel = 3;

}  // namespace field
