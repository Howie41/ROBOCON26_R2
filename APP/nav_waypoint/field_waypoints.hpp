#pragma once

namespace field {

/// @brief 2006 高位驱动距离 (mm)，沿当前 yaw 方向前进
constexpr float STAIR_CLIMB_DIST = 660.0f;

/// @brief 降下后 omni 去中心的距离 (mm)
constexpr float STAIR_CENTER_OFFSET = 350.0f;

/// @brief 下台阶 2006 高位后退距离 (mm)
constexpr float STAIR_CLIMB_DOWN_DIST = 640.0f;

}  // namespace field