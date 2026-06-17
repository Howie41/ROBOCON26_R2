#include "waypoint_navigator.hpp"

#include "cmsis_os2.h"

#include <atomic>
#include <cmath>

#include "NavProtocol.hpp"
#include "field_waypoints.hpp"
#include "lift_task.h"
#include "stair_assist.h"
#include "topic_pool.h"
#include "topics.hpp"

namespace { 

std::atomic<uint8_t> g_stair_waypoint_step{0};
std::atomic<uint8_t> g_stair_waypoint_level{0};
std::atomic<bool> g_stair_waypoint_armed{false};

TypedTopicPublisher<pub_high_nav_cmd> stair_high_nav_pub("high_nav_cmd");
constexpr float kDescendLaserSeekSpeedRpm = -30.0f;
constexpr float kClimbLaserSeekSpeedRpm = 30.0f;
constexpr float kDescendEdgeSeekSpeedMps = -0.12f;
constexpr float kPassDistanceMm = 200.0f;
constexpr uint32_t kPassHoldMs = 1000U;
constexpr TickType_t kPassFreshWindowTicks = pdMS_TO_TICKS(200);
constexpr uint32_t kPosePollDelayMs = 10U;

template <typename T>
void wait_until(T &&condition, uint32_t delay_ms = 100U) {
  while (!condition()) {
    osDelay(delay_ms);
  }
}

field::StairPose offsetPoseX(const field::StairPose &base, int32_t delta_x) {
  return field::StairPose{
      static_cast<int16_t>(base.x + delta_x),
      base.y,
      base.yaw,
  };
}

int32_t stairLevelOffsetX(uint8_t level) {
  return static_cast<int32_t>(level) * field::kStairSpanMm;
}

field::StairPose stairStandbyPoseForLevel(uint8_t level) {
  return offsetPoseX(field::kStairFrontPose, stairLevelOffsetX(level));
}

field::StairPose stairClosePoseForLevel(uint8_t level) {
  return offsetPoseX(field::kStairClosePose, stairLevelOffsetX(level));
}

field::StairPose stairHighDrivePoseForLevel(uint8_t level) {
  return offsetPoseX(field::kStairHighDrivePose, stairLevelOffsetX(level));
}

field::StairPose stairCenterPoseForLevel(uint8_t level) {
  if (level == 0U) {
    return field::kStairFrontPose;
  }
  return offsetPoseX(field::kStairCenterPose, stairLevelOffsetX(level - 1U));
}

bool hasFreshPoseSample(TickType_t now) {
  return (nav_control::g_last_position_update_tick != 0U) &&
         ((now - nav_control::g_last_position_update_tick) <=
          kPassFreshWindowTicks);
}

float poseDistanceMm(const field::StairPose &pose) {
  const float error_x =
      static_cast<float>(pose.x - nav_control::current_x);
  const float error_y =
      static_cast<float>(pose.y - nav_control::current_y);
  return sqrtf(error_x * error_x + error_y * error_y);
}

void stop_auto_nav() {
  nav_control::auto_enabled = false;
  nav_control::target_active = false;
  nav_control::arrived = false;
  nav_control::arrival_reported = false;
}

void publishManualChassisCmd(float linear_x, float linear_y, float omega) {
  pub_chassis_cmd cmd{};
  cmd.linear_x_ = linear_x;
  cmd.linear_y_ = linear_y;
  cmd.omega_ = omega;
  cmd.nav_mode_ = false;
  TypedTopicPublisher<pub_chassis_cmd> chassis_pub("chassis_cmd");
  if (chassis_pub.IsValid()) {
    chassis_pub.Publish(cmd);
  }
}

void stop_manual_chassis_motion() {
  publishManualChassisCmd(0.0f, 0.0f, 0.0f);
}

bool move_to_pose(const field::StairPose &pose, bool allow_pass = false) {
  taskENTER_CRITICAL();
  nav_control::target_x = pose.x;
  nav_control::target_y = pose.y;
  nav_control::target_yaw = pose.yaw;
  nav_control::auto_enabled = true;
  nav_control::arrived = false;
  nav_control::target_active = true;
  nav_control::arrival_reported = false;
  nav_control::resetAllPIDs();
  taskEXIT_CRITICAL();

  TickType_t entered_pass_tick = 0U;

  while (true) {
    if (nav_control::arrived) {
      return true;
    }

    const TickType_t now = osKernelGetTickCount();
    if (allow_pass && hasFreshPoseSample(now)) {
      if (poseDistanceMm(pose) <= kPassDistanceMm) {
        if (entered_pass_tick == 0U) {
          entered_pass_tick = now;
        } else if ((now - entered_pass_tick) >= pdMS_TO_TICKS(kPassHoldMs)) {
          return true;
        }
      } else {
        entered_pass_tick = 0U;
      }
    } else {
      entered_pass_tick = 0U;
    }

    osDelay(kPosePollDelayMs);
  }
}

}  // namespace

void stairWaypointGoToFront() {
  g_stair_waypoint_step.store(21);
  const field::StairPose front_pose = stairStandbyPoseForLevel(0U);
  move_to_pose(front_pose, true);
  g_stair_waypoint_level.store(0U);
  g_stair_waypoint_armed.store(true);
  g_stair_waypoint_step.store(0);
  stop_auto_nav();
}

void stairWaypointRunUp() {
  const uint8_t current_level = g_stair_waypoint_level.load();
  if (current_level >= field::kStairMaxLevel) {
    g_stair_waypoint_step.store(0);
    stop_auto_nav();
    return;
  }

  const uint8_t next_level = current_level + 1U;
  const field::StairPose standby_pose = stairCenterPoseForLevel(current_level);
  const field::StairPose close_pose = stairClosePoseForLevel(current_level);
  const field::StairPose high_drive_pose =
      stairHighDrivePoseForLevel(current_level);
  const field::StairPose center_pose = stairCenterPoseForLevel(next_level);

  stairAssistSetMode(StairAssistMode::ClimbUp);
  stairAssistSetAutoLowerEnabled(true);
  stairAssistSetEnabled(true);

  g_stair_waypoint_step.store(1);
  move_to_pose(standby_pose, true);

  g_stair_waypoint_step.store(2);
  move_to_pose(close_pose, false);

  g_stair_waypoint_step.store(21);
  stop_auto_nav();
  publishManualChassisCmd(0.12f, 0.0f, 0.0f);
  wait_until([]() {
    stairAssistUpdate();
    return stairAssistSuggestClimbUp();
  }, 10U);
  stop_manual_chassis_motion();

  g_stair_waypoint_step.store(3);
  stop_auto_nav();
  liftRequestHigh();
  wait_until([]() { return nav_control::high_mode_active; }, 10U);

  g_stair_waypoint_step.store(4);
  move_to_pose(high_drive_pose, true);

  g_stair_waypoint_step.store(5);
  stop_auto_nav();
  {
    pub_high_nav_cmd crawl_cmd{};
    crawl_cmd.active = true;
    crawl_cmd.allow_without_auto = true;
    crawl_cmd.forward_speed = kClimbLaserSeekSpeedRpm;
    crawl_cmd.omega = 0.0f;
    stair_high_nav_pub.Publish(crawl_cmd);
  }
  wait_until([]() {
    stairAssistUpdate();
    return stairAssistShouldLowerAfterClimbAdvance();
  }, 10U);
  {
    pub_high_nav_cmd stop_cmd{};
    stop_cmd.active = false;
    stop_cmd.allow_without_auto = false;
    stop_cmd.forward_speed = 0.0f;
    stop_cmd.omega = 0.0f;
    stair_high_nav_pub.Publish(stop_cmd);
  }
  liftRequestLow();
  wait_until([]() { return !nav_control::high_mode_active; }, 10U);
  stairAssistSetEnabled(false);
  stairAssistSetAutoLowerEnabled(false);

  g_stair_waypoint_step.store(6);
  move_to_pose(center_pose, true);

  g_stair_waypoint_level.store(next_level);
  g_stair_waypoint_armed.store(true);
  g_stair_waypoint_step.store(0);
  stop_auto_nav();
}

void stairWaypointRunDown() {
  const uint8_t current_level = g_stair_waypoint_level.load();
  if (current_level == 0U) {
    g_stair_waypoint_step.store(0);
    stop_auto_nav();
    return;
  }

  const field::StairPose current_center_pose =
      stairCenterPoseForLevel(current_level);
  const field::StairPose high_drive_pose =
      stairHighDrivePoseForLevel(current_level - 1U);
  const field::StairPose close_pose =
      stairClosePoseForLevel(current_level - 1U);
  const field::StairPose lower_center_pose =
      stairCenterPoseForLevel(current_level - 1U);

  g_stair_waypoint_step.store(11);
  stairAssistSetMode(StairAssistMode::Descend);
  stairAssistSetAutoLowerEnabled(true);
  stairAssistSetEnabled(true);
  move_to_pose(current_center_pose, true);

  g_stair_waypoint_step.store(12);
  move_to_pose(high_drive_pose, true);

  g_stair_waypoint_step.store(13);
  stop_auto_nav();
  publishManualChassisCmd(kDescendEdgeSeekSpeedMps, 0.0f, 0.0f);
  wait_until([]() {
    stairAssistUpdate();
    return stairAssistSuggestDescendHighMode();
  }, 10U);
  stop_manual_chassis_motion();
  liftRequestHigh();
  wait_until([]() { return nav_control::high_mode_active; }, 10U);

  g_stair_waypoint_step.store(14);
  move_to_pose(close_pose, false);

  g_stair_waypoint_step.store(15);
  stop_auto_nav();
  {
    pub_high_nav_cmd crawl_cmd{};
    crawl_cmd.active = true;
    crawl_cmd.allow_without_auto = true;
    crawl_cmd.forward_speed = kDescendLaserSeekSpeedRpm;
    crawl_cmd.omega = 0.0f;
    stair_high_nav_pub.Publish(crawl_cmd);
  }
  wait_until([]() {
    stairAssistUpdate();
    return stairAssistShouldLowerAfterDescendRetreat();
  }, 10U);
  {
    pub_high_nav_cmd stop_cmd{};
    stop_cmd.active = false;
    stop_cmd.allow_without_auto = false;
    stop_cmd.forward_speed = 0.0f;
    stop_cmd.omega = 0.0f;
    stair_high_nav_pub.Publish(stop_cmd);
  }
  stairAssistSetEnabled(false);
  stairAssistSetAutoLowerEnabled(false);

  g_stair_waypoint_step.store(16);
  stop_auto_nav();
  liftRequestLow();
  wait_until([]() { return !nav_control::high_mode_active; }, 10U);

  g_stair_waypoint_step.store(17);
  move_to_pose(lower_center_pose, true);

  g_stair_waypoint_level.store(current_level - 1U);
  g_stair_waypoint_armed.store(true);
  g_stair_waypoint_step.store(0);
  stop_auto_nav();
}

uint8_t stairWaypointStep() { return g_stair_waypoint_step.load(); }

uint8_t stairWaypointLevel() { return g_stair_waypoint_level.load(); }

bool stairWaypointArmed() { return g_stair_waypoint_armed.load(); }
