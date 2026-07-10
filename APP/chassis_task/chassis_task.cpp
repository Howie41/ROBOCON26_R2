/**
 * @file chassis_task.cpp
 * @author YE
 * @brief 全向轮底盘任务实现 (含位置环锁底盘)
 * @version 0.3
 * @date 2026-06-11
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */

#include "chassis_task.h"
#include "Motor.hpp"
#include "NavProtocol.hpp"
#include "PoseEstimator.hpp"
#include "chassis_solution.hpp"
#include "merlin_map/merlin_map.h"
#include "pid_controller.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "waypoint_navigator.hpp"
#include "control_task.h"
#include <array>
#include <cmath>

osThreadId_t ChassisTaskHandle;

extern C620Motor chassis_motor1, chassis_motor2, chassis_motor3, chassis_motor4;
extern volatile float g_hwt101_yaw_deg;
extern volatile uint32_t g_hwt101_frame_count;

static TypedTopicSubscriber<pub_chassis_cmd> chassis_cmd_sub("chassis_cmd", 8);
static pub_chassis_cmd chassis_cmd{};

volatile float g_chassis_yaw_deg = 0.0f;
volatile float g_chassis_yaw_lock_deg = 0.0f;
volatile float g_chassis_yaw_hold_omega = 0.0f;
volatile float g_chassis_cmd_linear_x = 0.0f;
volatile float g_chassis_cmd_linear_y = 0.0f;
volatile float g_chassis_cmd_omega = 0.0f;
volatile float g_chassis_final_omega = 0.0f;
volatile float g_chassis_target_rad_s_fl = 0.0f;
volatile float g_chassis_target_rad_s_fr = 0.0f;
volatile float g_chassis_target_rad_s_rl = 0.0f;
volatile float g_chassis_target_rad_s_rr = 0.0f;
volatile bool g_chassis_hold_active = false;

namespace {

float normalizeDeg(float angle_deg);
void refreshYawReference();
void updateWheelOdometry();
void updateHeadingFromChassisYaw();
bool hasMotionCommand(const pub_chassis_cmd &cmd);
void requestYawRotateDegInternal(float delta_deg);
void requestYawRotateCcw90Internal();
void requestYawRotateCw90Internal();
bool yawRotateActiveInternal();
bool takeYawRotateFinishedInternal();
float yawRotateTargetDegInternal();
void waitForYawRotateFinished();
bool isNearCurrentMerlinCellCenter();

}  // namespace

namespace chassis_action {

constexpr float kYawRotate90Deg = 90.0f;
constexpr float kYawRotateToleranceDeg = 1.0f;

bool g_yaw_rotate_active = false;
bool g_yaw_rotate_finished = false;
float g_yaw_rotate_target_deg = 0.0f;
bool g_yaw_rotate_was_auto = false;
PID_t g_yaw_rotate_pid = {
    .Kp = 2.10f,
    .Ki = 0.22f,
    .Kd = 0.001f,
    .MaxOut = MAX_ROTATION_VELOCITY*0.75*180.0f/M_PI,
    .IntegralLimit = 5.0f,
    .DeadBand = 0.3f,
    .Improve = Integral_Limit,
};

}  // namespace chassis_action

namespace {

void requestYawRotateDegInternal(float delta_deg) {
  refreshYawReference();
  chassis_action::g_yaw_rotate_target_deg =
      normalizeDeg(g_chassis_yaw_deg + delta_deg);
  chassis_action::g_yaw_rotate_active = true;
  chassis_action::g_yaw_rotate_finished = false;
  g_chassis_yaw_lock_deg = chassis_action::g_yaw_rotate_target_deg;
  g_chassis_yaw_hold_omega = 0.0f;

  chassis_action::g_yaw_rotate_was_auto = nav_control::auto_enabled;
  if (chassis_action::g_yaw_rotate_was_auto) {
    nav_control::auto_enabled = false;
  }

  PID_Init(&chassis_action::g_yaw_rotate_pid);
}

void requestYawRotateCcw90Internal() {
  requestYawRotateDegInternal(chassis_action::kYawRotate90Deg);
}

void requestYawRotateCw90Internal() {
  requestYawRotateDegInternal(-chassis_action::kYawRotate90Deg);
}

bool yawRotateActiveInternal() {
  return chassis_action::g_yaw_rotate_active;
}

bool takeYawRotateFinishedInternal() {
  const bool finished = chassis_action::g_yaw_rotate_finished;
  chassis_action::g_yaw_rotate_finished = false;
  return finished;
}

float yawRotateTargetDegInternal() {
  return chassis_action::g_yaw_rotate_target_deg;
}

void waitForYawRotateFinished() {
  while (yawRotateActiveInternal()) {
    osDelay(10U);
  }
}

bool isNearCurrentMerlinCellCenter() {
  merlin_map::Cell current_cell{};
  if (!merlin_map::tryGetCurrentCell(&current_cell)) {
    return false;
  }

  const float error_x =
      static_cast<float>(nav_control::current_x - current_cell.center_x);
  const float error_y =
      static_cast<float>(nav_control::current_y - current_cell.center_y);
  const float dist_error = sqrtf(error_x * error_x + error_y * error_y);
  return dist_error < 30.0f;
}

Omni45Chassis chassis_solver(chassis_motor1, chassis_motor2, chassis_motor3,
                             chassis_motor4);

const std::array<Omni45Chassis::SpeedPidParam, Omni45Chassis::kWheelCount>
    kWheelPidParams = {
        Omni45Chassis::SpeedPidParam(3500.0f, 7600.0f, 0.0f, 16000.0f, 0.1f, IMCREATEMENT_OF_OUT),
        Omni45Chassis::SpeedPidParam(3500.0f, 7600.0f, 0.0f, 16000.0f, 0.1f, IMCREATEMENT_OF_OUT),
        Omni45Chassis::SpeedPidParam(3500.0f, 7600.0f, 0.0f, 16000.0f, 0.1f, IMCREATEMENT_OF_OUT),
        Omni45Chassis::SpeedPidParam(3500.0f, 7600.0f, 0.0f, 16000.0f, 0.1f, IMCREATEMENT_OF_OUT),
    };

PID_t yaw_hold_pid = {
    .Kp = 0.16f,
    .Ki = 0.0008f,
    .Kd = 0.001f,
    .MaxOut = 1.5f,
    .IntegralLimit = 0.35f,
    .DeadBand = 0.1f,
    .Improve = Integral_Limit,
};

PID_t hold_yaw_pos_pid = {
    .Kp = 0.16f,
    .Ki = 0.008f,
    .Kd = 0.001f,
    .MaxOut = 3.0f,
    .IntegralLimit = 0.36f,
    .DeadBand = 0.1f,
    .Improve = Integral_Limit,
};

bool yaw_zero_initialized = false;
float yaw_zero_raw_deg = 0.0f;
constexpr float kHoldCmdDeadbandXY = 0.05f;
constexpr float kHoldCmdDeadbandOmega = 0.1f;
constexpr uint8_t kHoldIdleCycles = 3U;

float normalizeDeg(float angle_deg) {
  while (angle_deg > 180.0f) {
    angle_deg -= 360.0f;
  }
  while (angle_deg < -180.0f) {
    angle_deg += 360.0f;
  }
  return angle_deg;
}

float currentRelativeYawDeg() {
  if (!yaw_zero_initialized || g_hwt101_frame_count == 0U) {
    return 0.0f;
  }
  return normalizeDeg(g_hwt101_yaw_deg - yaw_zero_raw_deg);
}

void refreshYawReference() {
  if (!yaw_zero_initialized && g_hwt101_frame_count > 0U) {
    yaw_zero_raw_deg = g_hwt101_yaw_deg;
    yaw_zero_initialized = true;
    g_chassis_yaw_lock_deg = 0.0f;
    PID_Init(&yaw_hold_pid);
  }
  g_chassis_yaw_deg = currentRelativeYawDeg();
}

void updateWheelOdometry() {
  static bool baseline_initialized = false;
  static std::array<float, Omni45Chassis::kWheelCount> previous_pos_deg{};

  const std::array<float, Omni45Chassis::kWheelCount> current_pos_deg = {
      chassis_motor1.getCurrentSumPos(),
      chassis_motor2.getCurrentSumPos(),
      chassis_motor3.getCurrentSumPos(),
      chassis_motor4.getCurrentSumPos(),
  };

  if (!baseline_initialized) {
    previous_pos_deg = current_pos_deg;
    baseline_initialized = true;
    return;
  }

  std::array<float, Omni45Chassis::kWheelCount> delta_pos_deg{};
  for (size_t i = 0; i < delta_pos_deg.size(); ++i) {
    delta_pos_deg[i] = current_pos_deg[i] - previous_pos_deg[i];
  }
  previous_pos_deg = current_pos_deg;

  const Omni45Chassis::BodyDisplacement body_delta =
      chassis_solver.solveBodyDisplacement(delta_pos_deg);
  nav_localization::integrateBodyDisplacement(
      body_delta.dx_m * 1000.0f, body_delta.dy_m * 1000.0f,
      g_chassis_yaw_deg);
}

void updateHeadingFromChassisYaw() {
  const float yaw_deg = g_chassis_yaw_deg;
  merlin_map::Heading matched_heading{};
  bool matched = true;

  if (yaw_deg >= -30.0f && yaw_deg <= 30.0f) {
    matched_heading = merlin_map::Heading::PosX;
  } else if (yaw_deg >= 60.0f && yaw_deg <= 120.0f) {
    matched_heading = merlin_map::Heading::PosY;
  } else if (yaw_deg >= 150.0f || yaw_deg <= -150.0f) {
    matched_heading = merlin_map::Heading::NegX;
  } else if (yaw_deg >= -120.0f && yaw_deg <= -60.0f) {
    matched_heading = merlin_map::Heading::NegY;
  } else {
    matched = false;
  }

  if (matched && merlin_map::heading() != matched_heading) {
    merlin_map::setHeading(matched_heading);
  }
}

bool hasMotionCommand(const pub_chassis_cmd &cmd) {
  return std::fabs(cmd.linear_x_) > kHoldCmdDeadbandXY ||
         std::fabs(cmd.linear_y_) > kHoldCmdDeadbandXY ||
         std::fabs(cmd.omega_) > kHoldCmdDeadbandOmega;
}

bool updateYawRotateControl(pub_chassis_cmd &final_cmd) {
  if (!yawRotateActiveInternal()) {
    return false;
  }

  const float yaw_error =
      normalizeDeg(yawRotateTargetDegInternal() - g_chassis_yaw_deg);
  final_cmd.linear_x_ = 0.0f;
  final_cmd.linear_y_ = 0.0f;
  final_cmd.omega_ =
      PID_Calculate(&chassis_action::g_yaw_rotate_pid, 0.0f, yaw_error) *
      static_cast<float>(M_PI) / 180.0f;

  if (std::fabs(yaw_error) <= chassis_action::kYawRotateToleranceDeg) {
    final_cmd.omega_ = 0.0f;
    chassis_action::g_yaw_rotate_active = false;
    chassis_action::g_yaw_rotate_finished = true;
  }
  return true;
}

} // namespace

static inline void chassisInit() {
  chassis_solver.configureSpeedPid(kWheelPidParams);
  PID_Init(&yaw_hold_pid);
  PID_Init(&chassis_action::g_yaw_rotate_pid);
  PID_Init(&hold_yaw_pos_pid);
}

void chassisTask(void *argument) {
  (void)argument;
  TickType_t currentTime = xTaskGetTickCount();
  static bool chassis_hold_active = false;
  static uint8_t chassis_hold_idle_count = 0U;
  static std::array<float, Omni45Chassis::kWheelCount> chassis_hold_target_pos{};

  chassisInit();

  for (;;) {
    if (chassis_cmd_sub.TryGet(&chassis_cmd)) {
      g_chassis_cmd_linear_x = chassis_cmd.linear_x_;
      g_chassis_cmd_linear_y = chassis_cmd.linear_y_;
      g_chassis_cmd_omega = chassis_cmd.omega_;
    }

    refreshYawReference();
    updateWheelOdometry();
    updateHeadingFromChassisYaw();

    pub_chassis_cmd final_cmd = chassis_cmd;

    if (updateYawRotateControl(final_cmd)) {
      g_chassis_final_omega = final_cmd.omega_;
      chassis_solver.run(final_cmd);

      const auto &target_rad_s = chassis_solver.targetRadPerSec();
      g_chassis_target_rad_s_fl = target_rad_s[0];
      g_chassis_target_rad_s_fr = target_rad_s[1];
      g_chassis_target_rad_s_rl = target_rad_s[2];
      g_chassis_target_rad_s_rr = target_rad_s[3];

      chassis_hold_active = false;
      chassis_hold_idle_count = 0U;

    vTaskDelayUntil(&currentTime, 1);
      continue;
    }

    if (takeYawRotateFinishedInternal()) {
      chassis_hold_active = false;
      chassis_hold_idle_count = 0U;
      if (chassis_action::g_yaw_rotate_was_auto) {
        nav_control::target_yaw = static_cast<int16_t>(g_chassis_yaw_deg);
        nav_control::auto_enabled = true;
        nav_control::resetAllPIDs();
      }
    }

    // 只有在手动模式（nav_mode_=false）时才执行锁头逻辑
    // 自动导航模式下，由导航任务控制omega，不启用锁头
    static bool nav_mode_was_auto = false;
    if (!chassis_cmd.nav_mode_) {
      // 自动→手动切换时，刷新锁头角度为当前朝向，避免回转到旧角度
      if (nav_mode_was_auto) {
        g_chassis_yaw_lock_deg = g_chassis_yaw_deg;
        g_chassis_yaw_hold_omega = 0.0f;
        PID_Init(&yaw_hold_pid);
      }
      const bool operator_rotating = std::fabs(chassis_cmd.omega_) > 0.05f;
      if (operator_rotating) {
        g_chassis_yaw_lock_deg = g_chassis_yaw_deg;
        g_chassis_yaw_hold_omega = 0.0f;
        PID_Reset(&yaw_hold_pid);
      } else {
        const float yaw_error =
            normalizeDeg(g_chassis_yaw_lock_deg - g_chassis_yaw_deg);
        g_chassis_yaw_hold_omega =
            PID_Calculate(&yaw_hold_pid, 0.0f, yaw_error);
        final_cmd.omega_ = g_chassis_yaw_hold_omega;
      }
    }
    nav_mode_was_auto = chassis_cmd.nav_mode_;
  

    g_chassis_final_omega = final_cmd.omega_;

    if (chassis_cmd.nav_mode_) {
      chassis_hold_active = false;
      chassis_hold_idle_count = 0U;
      g_chassis_hold_active = false;
      chassis_solver.run(final_cmd);
    } else {
      const bool motion_requested = hasMotionCommand(chassis_cmd);
      if (!motion_requested) {
        if (chassis_hold_idle_count < kHoldIdleCycles) {
          ++chassis_hold_idle_count;
        }
      } else {
        chassis_hold_idle_count = 0U;
        chassis_hold_active = false;
      }

      static std::array<float, Omni45Chassis::kWheelCount> hold_base_pos{};

      if (!chassis_hold_active && chassis_hold_idle_count >= kHoldIdleCycles) {
        hold_base_pos = {
            chassis_motor1.getCurrentSumPos(),
            chassis_motor2.getCurrentSumPos(),
            chassis_motor3.getCurrentSumPos(),
            chassis_motor4.getCurrentSumPos(),
        };
        chassis_hold_target_pos = hold_base_pos;
        chassis_hold_active = true;
      }

      g_chassis_hold_active = chassis_hold_active;
      if (chassis_hold_active) {
        const float yaw_target = g_chassis_yaw_lock_deg;
        float yaw_error = normalizeDeg(yaw_target - g_chassis_yaw_deg);
        float wheel_offset = PID_Calculate(&hold_yaw_pos_pid, 0.0f, yaw_error);
        for (size_t i = 0; i < 4; i++) {
          chassis_hold_target_pos[i] = hold_base_pos[i] - wheel_offset;
        }
        chassis_solver.runHold(chassis_hold_target_pos);
      } else {
        chassis_solver.run(final_cmd);
      }
    }
    const auto &target_rad_s = chassis_solver.targetRadPerSec();
    g_chassis_target_rad_s_fl = target_rad_s[0];
    g_chassis_target_rad_s_fr = target_rad_s[1];
    g_chassis_target_rad_s_rl = target_rad_s[2];
    g_chassis_target_rad_s_rr = target_rad_s[3];

    vTaskDelayUntil(&currentTime, 1);
  }
}

namespace chassis_action {

void turn_left_90_deg() {
  if (yawRotateActiveInternal()) {
    waitForYawRotateFinished();
  }

  requestYawRotateCcw90Internal();
  waitForYawRotateFinished();
  refreshYawReference();
  updateHeadingFromChassisYaw();
}

void turn_right_90_deg() {
  if (yawRotateActiveInternal()) {
    waitForYawRotateFinished();
  }

  requestYawRotateCw90Internal();
  waitForYawRotateFinished();
  refreshYawReference();
  updateHeadingFromChassisYaw();
}

void turn_right_180_deg() {
  if (yawRotateActiveInternal()) {
    waitForYawRotateFinished();
  }

  requestYawRotateDegInternal(-180.0f);
  waitForYawRotateFinished();
  refreshYawReference();
  updateHeadingFromChassisYaw();
}

void start_climb_upstairs() { stairWaypointRunUp(); }

void start_climb_R1() { stairWaypointRunUpR1(); }

void start_climb_downstairs() { stairWaypointRunDown(); }

void start_go_to_edge() { stairWaypointRunGoToEdge(); }

void start_return_to_center() { stairWaypointRunReturnToCenter(); }

bool is_chassis_idle() {
  return !yawRotateActiveInternal() &&
         (stairWaypointStep() == 0U) &&
         !nav_control::auto_enabled &&
         !nav_control::high_mode_active &&
         isNearCurrentMerlinCellCenter();
}

}  // namespace chassis_action
