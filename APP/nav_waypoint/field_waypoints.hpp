#pragma once

#include <cstdint>

namespace field {

struct StairPose {
  int16_t x;
  int16_t y;
  int16_t yaw;
};

constexpr int16_t kStairSpanMm = 1200;
constexpr uint8_t kStairMaxLevel = 3;

StairPose stairFrontPose();
StairPose stairClosePose();
StairPose stairHighDrivePose();
StairPose stairCenterPose();
StairPose merlinEntryPoseByCol(uint8_t col);

}  // namespace field
