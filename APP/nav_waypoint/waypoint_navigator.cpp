#include "waypoint_navigator.hpp"
#include "field_waypoints.hpp"
#include "NavProtocol.hpp"
#include "lift_task.h"
#include "topic_pool.h"
#include "topics.hpp"

#include <cmath>

extern TypedTopicPublisher<pc_nav_event_t> pc_nav_event_pub;

StairContext g_stair_ctx;

static void publishEvent(uint16_t event_code) {
  pc_nav_event_t evt{event_code};
  pc_nav_event_pub.Publish(evt);
}

void stairSMStart(bool is_up) {
  if (g_stair_ctx.active) return;  // 不可打断

  g_stair_ctx.active = true;
  g_stair_ctx.is_up = is_up;
  g_stair_ctx.phase = 1;  // → LIFT_UP

  nav_control::auto_enabled = false;
  liftRequestHigh();  // 请求升到高位
}

bool runStairSM() {
  if (!g_stair_ctx.active) return false;

  const float sign = g_stair_ctx.is_up ? 1.0f : -1.0f;

  switch (g_stair_ctx.phase) {

    // ---- Phase 1: 等待升到高位 ----
    case 1:
      if (nav_control::high_mode_active) {
        // 计算目标: 当前坐标 + sign * STAIR_CLIMB_DIST * (cos, sin)(yaw)
        const float yaw_rad =
            static_cast<float>(nav_control::current_yaw) * 3.14159f / 180.0f;
        nav_control::target_x = nav_control::current_x +
            static_cast<int16_t>(sign * field::STAIR_CLIMB_DIST * cosf(yaw_rad));
        nav_control::target_y = nav_control::current_y +
            static_cast<int16_t>(sign * field::STAIR_CLIMB_DIST * sinf(yaw_rad));
        nav_control::target_yaw = nav_control::current_yaw;
        nav_control::auto_enabled = true;
        nav_control::arrived = false;
        nav_control::target_active = true;
        nav_control::arrival_reported = false;
        nav_control::resetAllPIDs();
        publishEvent(STAIR_EVT_HD_START);
        g_stair_ctx.phase = 2;  // → HIGH_DRIVE
      }
      break;

    // ---- Phase 2: 等待 2006 到达目标 ----
    case 2:
      if (nav_control::arrived) {
        publishEvent(STAIR_EVT_HD_DONE);
        liftRequestLow();  // 请求降到低位
        g_stair_ctx.phase = 3;  // → LIFT_DOWN
      }
      break;

    // ---- Phase 3: 等待降到位 ----
    case 3:
      if (!nav_control::high_mode_active) {
        // 计算去中心目标: 当前坐标 + STAIR_CENTER_OFFSET (沿yaw方向)
        const float yaw_rad =
            static_cast<float>(nav_control::current_yaw) * 3.14159f / 180.0f;
        nav_control::target_x = nav_control::current_x +
            static_cast<int16_t>(field::STAIR_CENTER_OFFSET * cosf(yaw_rad));
        nav_control::target_y = nav_control::current_y +
            static_cast<int16_t>(field::STAIR_CENTER_OFFSET * sinf(yaw_rad));
        nav_control::target_yaw = nav_control::current_yaw;
        nav_control::auto_enabled = true;
        nav_control::arrived = false;
        nav_control::target_active = true;
        nav_control::arrival_reported = false;
        nav_control::resetAllPIDs();
        publishEvent(STAIR_EVT_CD_START);
        g_stair_ctx.phase = 4;  // → DRIVE_CENTER
      }
      break;

    // ---- Phase 4: 等待到达中心 ----
    case 4:
      if (nav_control::arrived) {
        publishEvent(STAIR_EVT_CD_DONE);
        g_stair_ctx.phase = 5;  // → DONE
      }
      break;

    // ---- Phase 5: 完成，回到 IDLE ----
    case 5:
      g_stair_ctx.active = false;
      g_stair_ctx.phase = 0;
      nav_control::auto_enabled = false;  // 释放2006控制权回手动摇杆
      break;

    default:
      break;
  }

  return true;
}
