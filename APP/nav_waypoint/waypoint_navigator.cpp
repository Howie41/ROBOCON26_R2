#include "waypoint_navigator.hpp"

#include "cmsis_os2.h"

#include <atomic>
#include <cmath>

#include "NavProtocol.hpp"
#include "field_waypoints.hpp"
#include "lift_task.h"
#include "merlin_map/merlin_map.h"
#include "stair_assist.h"
#include "topic_pool.h"
#include "topics.hpp"

namespace { 

std::atomic<uint8_t> g_stair_waypoint_step{0};
std::atomic<uint8_t> g_stair_waypoint_level{0};
std::atomic<bool> g_stair_waypoint_armed{false};

TypedTopicPublisher<pub_high_nav_cmd> stair_high_nav_pub("high_nav_cmd");
constexpr float kDescendLaserSeekSpeedRpm = -50.0f;
constexpr float kClimbLaserSeekSpeedRpm = 100.0f;
constexpr float kDescendEdgeSeekSpeedMps = -0.2f;
constexpr int16_t kR1ClimbYawDeg = -90;
constexpr int16_t kR1PostLowAdvanceMm = 0;
constexpr int16_t kClimbAdvanceToLowerMm = 670;
constexpr int16_t kClimbAdvanceToCenterMm = 950;
constexpr int16_t kDescendRetreatToHighMm = 280;
constexpr int16_t kDescendRetreatToLowerMm = 950;
constexpr float kPassDistanceMm = 200.0f;
constexpr uint32_t kPassHoldMs = 1000U;
constexpr TickType_t kPassFreshWindowTicks = pdMS_TO_TICKS(200);
constexpr uint32_t kPosePollDelayMs = 5U;

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

int16_t headingYawDeg() {
  switch (merlin_map::heading()) {
    case merlin_map::Heading::PosX:
      return 0;
    case merlin_map::Heading::PosY:
      return 90;
    case merlin_map::Heading::NegX:
      return 180;
    case merlin_map::Heading::NegY:
      return -90;
  }

  return 0;
}

field::StairPose withHeadingYaw(const field::StairPose &pose) {
  return field::StairPose{pose.x, pose.y, headingYawDeg()};
}

bool refreshCurrentMerlinCell() {
  return merlin_map::identifyCurrentCell(nav_control::current_x,
                                         nav_control::current_y);
}

uint8_t levelFromCellHeight(const merlin_map::Cell &cell) {
  if (cell.height_mm <= 0) {
    return 0U;
  }
  const uint8_t level = static_cast<uint8_t>(cell.height_mm / 200);
  return (level > field::kStairMaxLevel) ? field::kStairMaxLevel : level;
}

field::StairPose headingClosePoseForCell(const merlin_map::Cell &cell) {
  const auto heading = merlin_map::heading();
  switch (heading) {
    case merlin_map::Heading::PosX:
      return field::StairPose{
          // Once the robot is already on a Merlin cell, the next climb-up
          // close pose is measured relative to the current cell center instead
          // of reusing the ground-entry close offset.
          static_cast<int16_t>(cell.center_x + 300),
          cell.center_y,
          headingYawDeg(),
      };
    case merlin_map::Heading::PosY:
      return field::StairPose{
          cell.center_x,
          static_cast<int16_t>(cell.center_y + 275),
          headingYawDeg(),
      };
    case merlin_map::Heading::NegX:
      return field::StairPose{
          static_cast<int16_t>(cell.center_x - 300),
          cell.center_y,
          headingYawDeg(),
      };
    case merlin_map::Heading::NegY:
      return field::StairPose{
          cell.center_x,
          static_cast<int16_t>(cell.center_y - 230),
          headingYawDeg(),
      };
  }

  return field::StairPose{cell.center_x, cell.center_y, headingYawDeg()};
}

field::StairPose advancePoseByHeading(int16_t x, int16_t y, int16_t advance_mm) {
  switch (merlin_map::heading()) {
    case merlin_map::Heading::PosX:
      return field::StairPose{
          static_cast<int16_t>(x + advance_mm),
          y,
          headingYawDeg(),
      };
    case merlin_map::Heading::PosY:
      return field::StairPose{
          x,
          static_cast<int16_t>(y + advance_mm),
          headingYawDeg(),
      };
    case merlin_map::Heading::NegX:
      return field::StairPose{
          static_cast<int16_t>(x - advance_mm),
          y,
          headingYawDeg(),
      };
    case merlin_map::Heading::NegY:
      return field::StairPose{
          x,
          static_cast<int16_t>(y - advance_mm),
          headingYawDeg(),
      };
  }

  return field::StairPose{x, y, headingYawDeg()};
}

field::StairPose advancePoseNegY(int16_t x, int16_t y, int16_t advance_mm) {
  return field::StairPose{x, static_cast<int16_t>(y - advance_mm),
                          kR1ClimbYawDeg};
}

field::StairPose stairStandbyPoseForLevel(uint8_t level) {
  return withHeadingYaw(
      offsetPoseX(field::kStairFrontPose, stairLevelOffsetX(level)));
}

field::StairPose stairClosePoseForLevel(uint8_t level) {
  return withHeadingYaw(
      offsetPoseX(field::kStairClosePose, stairLevelOffsetX(level)));
}

field::StairPose stairHighDrivePoseForLevel(uint8_t level) {
  return withHeadingYaw(
      offsetPoseX(field::kStairHighDrivePose, stairLevelOffsetX(level)));
}

field::StairPose stairCenterPoseForLevel(uint8_t level) {
  if (level == 0U) {
    return withHeadingYaw(field::kStairFrontPose);
  }
  return withHeadingYaw(
      offsetPoseX(field::kStairCenterPose, stairLevelOffsetX(level - 1U)));
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

template <typename T>
bool move_to_pose_until_trigger(const field::StairPose &pose, bool allow_pass,
                                T &&trigger) {
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
    stairAssistUpdate();
    if (trigger()) {
      return true;
    }

    if (nav_control::arrived) {
      return false;
    }

    const TickType_t now = osKernelGetTickCount();
    if (allow_pass && hasFreshPoseSample(now)) {
      if (poseDistanceMm(pose) <= kPassDistanceMm) {
        if (entered_pass_tick == 0U) {
          entered_pass_tick = now;
        } else if ((now - entered_pass_tick) >= pdMS_TO_TICKS(kPassHoldMs)) {
          return false;
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
  (void)refreshCurrentMerlinCell();
  uint8_t current_level = g_stair_waypoint_level.load();
  merlin_map::Cell current_cell{};
  const bool has_current_cell = merlin_map::tryGetCurrentCell(&current_cell);
  if (has_current_cell) {
    current_level = levelFromCellHeight(current_cell);
  }
  if (current_level >= field::kStairMaxLevel) {
    g_stair_waypoint_step.store(0);
    stop_auto_nav();
    return;
  }

  const uint8_t next_level = current_level + 1U;
  field::StairPose standby_pose = stairCenterPoseForLevel(current_level);
  field::StairPose close_pose = stairClosePoseForLevel(current_level);
  field::StairPose high_drive_pose = stairHighDrivePoseForLevel(current_level);
  field::StairPose center_pose = stairCenterPoseForLevel(next_level);

  merlin_map::Cell next_cell{};
  const bool has_next_cell = merlin_map::tryGetNeighborCell(true, &next_cell);

  if (has_current_cell) {
    standby_pose =
        field::StairPose{current_cell.center_x, current_cell.center_y,
                         headingYawDeg()};
  }
  if (has_current_cell) {
    close_pose = headingClosePoseForCell(current_cell);
  }
  if (has_next_cell) {
    center_pose =
        field::StairPose{next_cell.center_x, next_cell.center_y, headingYawDeg()};
  }

  stairAssistSetMode(StairAssistMode::ClimbUp);
  stairAssistSetAutoLowerEnabled(true);
  stairAssistSetEnabled(true);

  g_stair_waypoint_step.store(1);
  move_to_pose(standby_pose, true);

  g_stair_waypoint_step.store(2);
  const bool climb_high_triggered_early =
      move_to_pose_until_trigger(close_pose, false, []() {
        return stairAssistSuggestClimbUp();
      });

  g_stair_waypoint_step.store(21);
  stop_auto_nav();
  if (!climb_high_triggered_early) {
    publishManualChassisCmd(0.18f, 0.0f, 0.0f);
    wait_until([]() {
      stairAssistUpdate();
      return stairAssistSuggestClimbUp();
    }, 10U);
    stop_manual_chassis_motion();
  }

  const int16_t trigger_x = nav_control::current_x;
  const int16_t trigger_y = nav_control::current_y;
  high_drive_pose =
      advancePoseByHeading(trigger_x, trigger_y, kClimbAdvanceToLowerMm);
  if (has_next_cell) {
    center_pose =
        field::StairPose{next_cell.center_x, next_cell.center_y, headingYawDeg()};
  } else {
    center_pose =
        advancePoseByHeading(trigger_x, trigger_y, kClimbAdvanceToCenterMm);
  }

  g_stair_waypoint_step.store(3);
  stop_auto_nav();
  liftRequestHigh();
  wait_until([]() { return nav_control::high_mode_active; }, 10U);

  g_stair_waypoint_step.store(4);
  const bool climb_lower_triggered_early =
      move_to_pose_until_trigger(high_drive_pose, true, []() {
        return stairAssistShouldLowerAfterClimbAdvance();
      });

  g_stair_waypoint_step.store(5);
  stop_auto_nav();
  if (!climb_lower_triggered_early) {
    pub_high_nav_cmd crawl_cmd{};
    crawl_cmd.active = true;
    crawl_cmd.allow_without_auto = true;
    crawl_cmd.forward_speed = kClimbLaserSeekSpeedRpm;
    crawl_cmd.omega = 0.0f;
    stair_high_nav_pub.Publish(crawl_cmd);
    wait_until([]() {
      stairAssistUpdate();
      return stairAssistShouldLowerAfterClimbAdvance();
    }, 10U);
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
  (void)refreshCurrentMerlinCell();

  if (has_next_cell) {
    g_stair_waypoint_level.store(levelFromCellHeight(next_cell));
  } else {
    g_stair_waypoint_level.store(next_level);
  }
  g_stair_waypoint_armed.store(true);
  g_stair_waypoint_step.store(0);
  stop_auto_nav();
}

void stairWaypointRunUpR1() {
  stairAssistSetMode(StairAssistMode::ClimbUp);
  stairAssistSetAutoLowerEnabled(true);
  stairAssistSetEnabled(true);

  g_stair_waypoint_step.store(31);
  stop_auto_nav();
  wait_until([]() {
    stairAssistUpdate();
    return stairAssistSuggestClimbUp();
  }, 10U);

  const int16_t trigger_x = nav_control::current_x;
  const int16_t trigger_y = nav_control::current_y;
  const field::StairPose high_drive_pose =
      advancePoseNegY(trigger_x, trigger_y, kClimbAdvanceToLowerMm);

  g_stair_waypoint_step.store(32);
  stop_auto_nav();
  liftRequestHigh();
  wait_until([]() { return nav_control::high_mode_active; }, 10U);

  g_stair_waypoint_step.store(33);
  const bool climb_lower_triggered_early =
      move_to_pose_until_trigger(high_drive_pose, true, []() {
        return stairAssistShouldLowerAfterClimbAdvance();
      });

  g_stair_waypoint_step.store(34);
  stop_auto_nav();
  if (!climb_lower_triggered_early) {
    pub_high_nav_cmd crawl_cmd{};
    crawl_cmd.active = true;
    crawl_cmd.allow_without_auto = true;
    crawl_cmd.forward_speed = kClimbLaserSeekSpeedRpm;
    crawl_cmd.omega = 0.0f;
    stair_high_nav_pub.Publish(crawl_cmd);
    wait_until([]() {
      stairAssistUpdate();
      return stairAssistShouldLowerAfterClimbAdvance();
    }, 10U);
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

  g_stair_waypoint_step.store(35);
  if (kR1PostLowAdvanceMm > 0) {
    const field::StairPose post_low_pose =
        advancePoseNegY(nav_control::current_x, nav_control::current_y,
                        kR1PostLowAdvanceMm);
    move_to_pose(post_low_pose, true);
  }

  g_stair_waypoint_step.store(0);
  stop_auto_nav();
}

void stairWaypointRunDown() {
  (void)refreshCurrentMerlinCell();
  uint8_t current_level = g_stair_waypoint_level.load();
  merlin_map::Cell current_cell{};
  const bool has_current_cell = merlin_map::tryGetCurrentCell(&current_cell);
  if (has_current_cell) {
    current_level = levelFromCellHeight(current_cell);
  }
  if (current_level == 0U) {
    g_stair_waypoint_step.store(0);
    stop_auto_nav();
    return;
  }

  const field::StairPose current_center_pose =
      has_current_cell ? field::StairPose{current_cell.center_x,
                                          current_cell.center_y,
                                          headingYawDeg()}
                       : stairCenterPoseForLevel(current_level);

  field::StairPose high_drive_pose =
      stairHighDrivePoseForLevel(current_level - 1U);
  field::StairPose close_pose =
      stairClosePoseForLevel(current_level - 1U);
  field::StairPose lower_center_pose =
      stairCenterPoseForLevel(current_level - 1U);

  merlin_map::Cell lower_cell{};
  const bool has_lower_cell = merlin_map::tryGetNeighborCell(false, &lower_cell);

  if (has_current_cell) {
    high_drive_pose = advancePoseByHeading(current_cell.center_x,
                                           current_cell.center_y,
                                           -kDescendRetreatToHighMm);
    close_pose = advancePoseByHeading(current_cell.center_x,
                                      current_cell.center_y,
                                      -kDescendRetreatToLowerMm);
  }
  if (has_lower_cell) {
    lower_center_pose =
        field::StairPose{lower_cell.center_x, lower_cell.center_y,
                         headingYawDeg()};
  }

  g_stair_waypoint_step.store(11);
  stairAssistSetMode(StairAssistMode::Descend);
  stairAssistSetAutoLowerEnabled(true);
  stairAssistSetEnabled(true);
  move_to_pose(current_center_pose, true);

  g_stair_waypoint_step.store(12);
  const bool descend_high_triggered_early =
      move_to_pose_until_trigger(high_drive_pose, true, []() {
        return stairAssistSuggestDescendHighMode();
      });

  g_stair_waypoint_step.store(13);
  stop_auto_nav();
  if (!descend_high_triggered_early) {
    publishManualChassisCmd(kDescendEdgeSeekSpeedMps, 0.0f, 0.0f);
    wait_until([]() {
      stairAssistUpdate();
      return stairAssistSuggestDescendHighMode();
    }, 10U);
    stop_manual_chassis_motion();
  }
  liftRequestHigh();
  wait_until([]() { return nav_control::high_mode_active; }, 10U);

  g_stair_waypoint_step.store(14);
  const bool descend_lower_triggered_early =
      move_to_pose_until_trigger(close_pose, false, []() {
        return stairAssistShouldLowerAfterDescendRetreat();
      });

  g_stair_waypoint_step.store(15);
  stop_auto_nav();
  if (!descend_lower_triggered_early) {
    pub_high_nav_cmd crawl_cmd{};
    crawl_cmd.active = true;
    crawl_cmd.allow_without_auto = true;
    crawl_cmd.forward_speed = kDescendLaserSeekSpeedRpm;
    crawl_cmd.omega = 0.0f;
    stair_high_nav_pub.Publish(crawl_cmd);
    wait_until([]() {
      stairAssistUpdate();
      return stairAssistShouldLowerAfterDescendRetreat();
    }, 10U);
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
  if (has_lower_cell) {
    move_to_pose(lower_center_pose, true);
    (void)refreshCurrentMerlinCell();
  } else {
    taskENTER_CRITICAL();
    nav_control::target_x = nav_control::current_x;
    nav_control::target_y = nav_control::current_y;
    nav_control::target_yaw = nav_control::current_yaw;
    nav_control::arrived = true;
    nav_control::target_active = false;
    nav_control::arrival_reported = false;
    taskEXIT_CRITICAL();
    merlin_map::invalidateCurrentCell();
  }

  if (has_lower_cell) {
    g_stair_waypoint_level.store(levelFromCellHeight(lower_cell));
  } else if (current_level > 0U) {
    g_stair_waypoint_level.store(current_level - 1U);
  } else {
    g_stair_waypoint_level.store(0U);
  }
  g_stair_waypoint_armed.store(true);
  g_stair_waypoint_step.store(0);
  stop_auto_nav();
}

uint8_t stairWaypointStep() { return g_stair_waypoint_step.load(); }

uint8_t stairWaypointLevel() { return g_stair_waypoint_level.load(); }

bool stairWaypointArmed() { return g_stair_waypoint_armed.load(); }
