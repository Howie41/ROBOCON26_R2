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

  if (is_up) {
    // ---- 上楼梯: 升到高位 → 2006前进650mm → 降下 → omni前进350mm ----
    g_stair_ctx.phase = 1;
    nav_control::auto_enabled = false;
    liftRequestHigh();
  } else {
    // ---- 下楼梯: 低位omni后退350mm → 升到高位 → 2006后退640mm → 降下 ----
    g_stair_ctx.phase = 11;
    const float yaw_rad =
        static_cast<float>(nav_control::current_yaw) * 3.14159f / 180.0f;
    nav_control::target_x = nav_control::current_x -
        static_cast<int16_t>(field::STAIR_CENTER_OFFSET * cosf(yaw_rad));
    nav_control::target_y = nav_control::current_y -
        static_cast<int16_t>(field::STAIR_CENTER_OFFSET * sinf(yaw_rad));
    nav_control::target_yaw = nav_control::current_yaw;
    nav_control::auto_enabled = true;
    nav_control::arrived = false;
    nav_control::target_active = true;
    nav_control::arrival_reported = false;
    nav_control::resetAllPIDs();
    publishEvent(STAIR_EVT_CD_START);
  }
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
        nav_control::auto_enabled = false; 
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

    // ---- Phase 5: 上楼梯完成 ----
    case 5:
      publishEvent(0x0204);  // nav_climb_up_ok
      g_stair_ctx.active = false;
      g_stair_ctx.phase = 0;
      nav_control::auto_enabled = false;
      break;

    // ===== 下楼梯: Phase 11~15 =====
    // Phase 11: omni后退350mm到达 → 升到高位
    case 11:
      if (nav_control::arrived) {
        publishEvent(STAIR_EVT_CD_DONE);
        nav_control::auto_enabled = false;
        liftRequestHigh();
        g_stair_ctx.phase = 12;
      }
      break;

    // Phase 12: 等待升到高位 → 2006后退640mm
    case 12:
      if (nav_control::high_mode_active) {
        const float yaw_rad =
            static_cast<float>(nav_control::current_yaw) * 3.14159f / 180.0f;
        nav_control::target_x = nav_control::current_x -
            static_cast<int16_t>(field::STAIR_CLIMB_DOWN_DIST * cosf(yaw_rad));
        nav_control::target_y = nav_control::current_y -
            static_cast<int16_t>(field::STAIR_CLIMB_DOWN_DIST * sinf(yaw_rad));
        nav_control::target_yaw = nav_control::current_yaw;
        nav_control::auto_enabled = true;
        nav_control::arrived = false;
        nav_control::target_active = true;
        nav_control::arrival_reported = false;
        nav_control::resetAllPIDs();
        publishEvent(STAIR_EVT_HD_START);
        g_stair_ctx.phase = 13;
      }
      break;

    // Phase 13: 2006后退640mm到达 → 降到低位
    case 13:
      if (nav_control::arrived) {
        publishEvent(STAIR_EVT_HD_DONE);
        nav_control::auto_enabled = false;
        liftRequestLow();
        g_stair_ctx.phase = 14;
      }
      break;

    // Phase 14: 等待降到低位 → 完成
    case 14:
      if (!nav_control::high_mode_active) {
        g_stair_ctx.phase = 15;
      }
      break;

    // Phase 15: 下楼梯完成
    case 15:
      publishEvent(0x0205);  // nav_climb_down_ok
      g_stair_ctx.active = false;
      g_stair_ctx.phase = 0;
      nav_control::auto_enabled = false;
      break;

    default:
      break;
  }

  return (g_stair_ctx.phase != 2 && g_stair_ctx.phase != 4 &&
          g_stair_ctx.phase != 11 && g_stair_ctx.phase != 13);
}
