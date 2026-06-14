#pragma once

#include <cstdint>

/// @brief 楼梯状态机上下文
struct StairContext {
  bool active = false;   // 状态机运行中
  bool is_up = false;    // true=上楼梯, false=下楼梯
  uint8_t phase = 0;     // 0=IDLE, 1=LIFT_UP, 2=HIGH_DRIVE, 3=LIFT_DOWN, 4=DRIVE_CENTER, 5=DONE
};

extern StairContext g_stair_ctx;
inline bool stairSMIdle() { return !g_stair_ctx.active; }
// ---- 事件码: 下位机→上位机 ----
#define STAIR_EVT_HD_START  0x0207  // 2006 高位驱动开始
#define STAIR_EVT_HD_DONE   0x0208  // 2006 高位驱动到达
#define STAIR_EVT_CD_START  0x0209  // 去中心驱动开始
#define STAIR_EVT_CD_DONE   0x020A  // 去中心驱动到达

/// @brief 启动楼梯状态机 (PC 收到 0x0103/0x0104 时调用)
void stairSMStart(bool is_up);

/// @brief 每周期执行一次，由 NavControlTask 调用
/// @return true 表示 SM 正在运行（调用方应跳过正常导航逻辑）
bool runStairSM();
