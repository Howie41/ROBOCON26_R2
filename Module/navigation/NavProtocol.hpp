#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

extern osThreadId_t NavControlTaskHandle;

void NavControlTask(void *argument);

namespace nav_control {

extern int16_t current_x;
extern int16_t current_y;
extern int16_t current_yaw;
extern int16_t pc_reported_yaw;
extern int16_t target_x;
extern int16_t target_y;
extern int16_t target_yaw;
extern bool auto_enabled;
extern bool arrived;
extern bool target_active;
extern bool arrival_reported;
extern bool high_mode_active;
extern TickType_t g_last_position_update_tick;

void updatePositionTimestamp();
void resetAllPIDs();

}  // namespace nav_control

// ---- Low-mode holonomic navigation tuning ----
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

// ---- Low-mode navigation debug globals for Ozone / VOFA ----
extern volatile float g_ozone_nav_dist_mm;
extern volatile float g_ozone_nav_yaw_err_deg;
extern volatile float g_ozone_nav_blend;
extern volatile float g_ozone_nav_plan_speed_mps;
extern volatile float g_ozone_nav_v_ref_mps;
extern volatile float g_ozone_nav_vx_ref_mps;
extern volatile float g_ozone_nav_vy_ref_mps;
extern volatile float g_ozone_nav_omega_ref_radps;
extern volatile float g_ozone_nav_pid_vx_mps;
extern volatile float g_ozone_nav_pid_vy_mps;
extern volatile float g_ozone_nav_pid_omega_radps;
extern volatile float g_ozone_nav_vx_cmd_mps;
extern volatile float g_ozone_nav_vy_cmd_mps;
extern volatile float g_ozone_nav_omega_cmd_radps;
extern volatile float g_ozone_nav_cmd_speed_mps;
extern volatile float g_ozone_nav_brake_dist_mm;
extern volatile uint8_t g_ozone_nav_arrive_hold_count;

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
