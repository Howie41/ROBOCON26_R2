#pragma once

#include <cstdint>

namespace field {

/// @brief 2006 高位驱动距离 (mm)，沿当前 yaw 方向前进
constexpr float STAIR_CLIMB_DIST = 660.0f;

/// @brief 降下后 omni 去中心的距离 (mm)
constexpr float STAIR_CENTER_OFFSET = 350.0f;

/// @brief 下台阶: 2006 高位后退距离 (mm)
constexpr float STAIR_CLIMB_DOWN_DIST = 640.0f;

/// @brief 航点表: 128 个预设航点，索引 0 保留为未使用
struct Wp {
  int16_t x, y, yaw;
};
enum WpID : uint8_t {
  WP_NONE = 0,
  WP_SHR1 =1,
  WP_SHR2 =2,
  WP_SHR3 =3,
  WP_MF_Entrance = 4,
};

constexpr Wp LIST[128] = {
    {0, 0, 0},  
    
};


}  // namespace field
 