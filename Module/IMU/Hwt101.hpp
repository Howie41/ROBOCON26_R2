/**
 * @file Hwt101.hpp
 * @author YE
 * @brief HWT101 IMU传感器驱动头文件
 * @version 0.1
 * @date 2026-05-12
 *
 */
#pragma once

#include <cstdint>

class Hwt101Parser {
public:
  bool processByte(uint8_t byte);

  float yawDeg() const { return yaw_deg_; }
  //float pitchDeg() const { return pitch_deg_; }
  //float rollDeg() const { return roll_deg_; }
  uint32_t frameCount() const { return frame_count_; }

private:
  static constexpr uint8_t kFrameSize = 11;

  void reset();
  void parseAngleFrame();

  uint8_t frame_[kFrameSize]{};
  uint8_t index_{0};
  float yaw_deg_{0.0f};
 // float pitch_deg_{0.0f};
  //float roll_deg_{0.0f};
  uint32_t frame_count_{0};
};

