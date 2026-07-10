#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

extern osThreadId_t NavControlTaskHandle;

void NavControlTask(void *argument);

extern volatile float g_nav_cruise_speed_mps;
extern volatile float g_nav_brake_safety_scale;
extern volatile float g_nav_max_accel_mps2;
extern volatile float g_nav_max_decel_mps2;
extern volatile float g_nav_blend_dist_mm;
extern volatile float g_nav_pid_max_xy_speed_mps;
extern volatile float g_nav_pid_max_omega_radps;
extern volatile float g_nav_max_omega_radps;
extern volatile float g_nav_min_omega_radps;
extern volatile float g_nav_omega_slowdown_deg;
extern volatile float g_nav_max_omega_accel_radps2;
extern volatile float g_nav_yaw_slowdown_start_deg;
extern volatile float g_nav_yaw_slowdown_min_scale;
extern volatile float g_nav_arrive_dist_mm;
extern volatile float g_nav_arrive_yaw_deg;
extern volatile uint8_t g_nav_arrive_hold_count_target;

// 根据融合位姿计算的位置和航向误差。
extern volatile float g_ozone_nav_dist_mm;         // 到目标点的距离，单位：mm。
extern volatile float g_ozone_nav_yaw_err_deg;     // 目标航向角误差，单位：deg。
extern volatile float g_ozone_nav_path_err_mm;     // 沿路径方向误差，单位：mm。
extern volatile float g_ozone_nav_lateral_err_mm;  // 路径横向误差，单位：mm。

// 规划速度与 PID 输出的混合状态：0 表示完全使用 PID，1 表示完全使用巡航规划。
extern volatile float g_ozone_nav_blend;
extern volatile float g_ozone_nav_plan_speed_mps;

// 当前实现中实际发布给底盘的速度参考值。
extern volatile float g_ozone_nav_v_ref_mps;        // XY 合速度大小。
extern volatile float g_ozone_nav_vx_ref_mps;       // 底盘坐标系 X 速度，单位：m/s。
extern volatile float g_ozone_nav_vy_ref_mps;       // 底盘坐标系 Y 速度，单位：m/s。
extern volatile float g_ozone_nav_omega_ref_radps;  // 角速度，单位：rad/s。

// 与规划速度混合前，经过限幅的位置和航向 PID 分量。
extern volatile float g_ozone_nav_pid_vx_mps;       // 沿路径方向 PID 分量。
extern volatile float g_ozone_nav_pid_vy_mps;       // 横向 PID 分量。
extern volatile float g_ozone_nav_pid_omega_radps;  // 航向 PID 分量。

// 针对当前目标建立的世界坐标系固定路径切向量和法向量。
extern volatile float g_ozone_nav_path_tx;
extern volatile float g_ozone_nav_path_ty;
extern volatile float g_ozone_nav_path_nx;
extern volatile float g_ozone_nav_path_ny;

// 内部加减速斜坡状态，目前仅用于观测；底盘实际使用上方的 *_ref_* 值。
extern volatile float g_ozone_nav_vx_cmd_mps;
extern volatile float g_ozone_nav_vy_cmd_mps;
extern volatile float g_ozone_nav_omega_cmd_radps;
extern volatile float g_ozone_nav_cmd_speed_mps;
extern volatile float g_ozone_nav_brake_dist_mm;  // 根据斜坡速度计算的制动距离，单位：mm。

// 同时满足位置和航向到达条件的连续 10 ms 控制周期计数。
extern volatile uint8_t g_ozone_nav_arrive_hold_count;

namespace nav_control {

extern int16_t current_x;  // 供旧接口使用的融合世界 X 位置取整值，单位：mm。
extern int16_t current_y;  // 供旧接口使用的融合世界 Y 位置取整值，单位：mm。
extern int16_t current_yaw;
extern int16_t pc_reported_yaw;  // 雷达或上位机航向角，仅用于记录。
extern int16_t target_x;
extern int16_t target_y;
extern int16_t target_yaw;
extern bool auto_enabled;
extern bool arrived;
extern bool target_active;
extern bool arrival_reported;
extern bool high_mode_active;
// 最近一帧真实雷达位置的系统时刻，轮式里程计更新不会刷新该值。
extern TickType_t g_last_position_update_tick;

void submitRadarPosition(int16_t x, int16_t y);
void updatePositionTimestamp();
void resetAllPIDs();

}  // namespace nav_control

class NavProtocol {
 public:
  enum class CmdType {
    NONE = 0,
    TARGET,
    POSITION,
    QUERY,
    STOP,
  };

  struct NavCmd {
    CmdType type;
    int16_t param1;
    int16_t param2;
    int16_t param3;
  };

  bool processByte(uint8_t byte, NavCmd &out_cmd);

  static void buildResponse(const NavCmd &cmd, char *buf, size_t buf_size);

 private:
  void reset();

  char line_buf_[32]{};
  uint8_t index_{0};
};
