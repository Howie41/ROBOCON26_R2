#pragma once

#include <cstdint>

namespace field {

/// @brief 2006 高位驱动距离 (mm)，沿当前 yaw 方向前进
constexpr float STAIR_CLIMB_DIST = 400.0f;

/// @brief 降下后 omni 去中心的距离 (mm)
constexpr float STAIR_CENTER_OFFSET = 200.0f;

/// @brief 航点表: 128 个预设航点，索引 0 保留为未使用
struct Wp {
  int16_t x, y, yaw;
};

constexpr Wp LIST[128] = {
    {0, 0, 0},  // [0] 保留
    // 用户自行填充 [1] ~ [127]
};

}  // namespace field
