#include "NavProtocol.hpp"

#include "UsbPort.hpp"
#include "pid_controller.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "control_task.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <math.h>

extern volatile float g_chassis_yaw_deg;

volatile float g_nav_cruise_speed_mps = 2.3f;
volatile float g_nav_brake_safety_scale = 8.00f;
volatile float g_nav_max_accel_mps2 = 2.50f;
volatile float g_nav_max_decel_mps2 = 13.00f;
volatile float g_nav_blend_dist_mm = 50000.0f;
volatile float g_nav_pid_max_xy_speed_mps = 2.00f;
volatile float g_nav_pid_max_omega_radps = 4.50f;
volatile float g_nav_max_omega_radps = 3.00f;
volatile float g_nav_min_omega_radps = 0.15f;
volatile float g_nav_omega_slowdown_deg = 10.0f;
volatile float g_nav_max_omega_accel_radps2 = 1000.00f;
volatile float g_nav_yaw_slowdown_start_deg = 12.0f;
volatile float g_nav_yaw_slowdown_min_scale = 0.40f;
volatile float g_nav_arrive_dist_mm = 30.0f;
volatile float g_nav_arrive_yaw_deg = 1.0f;
volatile uint8_t g_nav_arrive_hold_count_target = 2U;

volatile float g_ozone_nav_dist_mm = 0.0f;
volatile float g_ozone_nav_yaw_err_deg = 0.0f;
volatile float g_ozone_nav_path_err_mm = 0.0f;
volatile float g_ozone_nav_lateral_err_mm = 0.0f;
volatile float g_ozone_nav_blend = 0.0f;
volatile float g_ozone_nav_plan_speed_mps = 0.0f;
volatile float g_ozone_nav_v_ref_mps = 0.0f;
volatile float g_ozone_nav_vx_ref_mps = 0.0f;
volatile float g_ozone_nav_vy_ref_mps = 0.0f;
volatile float g_ozone_nav_omega_ref_radps = 0.0f;
volatile float g_ozone_nav_pid_vx_mps = 0.0f;
volatile float g_ozone_nav_pid_vy_mps = 0.0f;
volatile float g_ozone_nav_pid_omega_radps = 0.0f;
volatile float g_ozone_nav_path_tx = 1.0f;
volatile float g_ozone_nav_path_ty = 0.0f;
volatile float g_ozone_nav_path_nx = 0.0f;
volatile float g_ozone_nav_path_ny = 1.0f;
volatile float g_ozone_nav_vx_cmd_mps = 0.0f;
volatile float g_ozone_nav_vy_cmd_mps = 0.0f;
volatile float g_ozone_nav_omega_cmd_radps = 0.0f;
volatile float g_ozone_nav_cmd_speed_mps = 0.0f;
volatile float g_ozone_nav_brake_dist_mm = 0.0f;
volatile uint8_t g_ozone_nav_arrive_hold_count = 0U;



PID_t pid_x = {
    .Kp = 2.2f,
    .Ki = 0.22f,
    .Kd = 0.0f,
    .MaxOut = 1500.0f,
    .IntegralLimit = 100.0f,
    .DeadBand = 20.0f,
    .Improve = Integral_Limit,
};

PID_t pid_y = {
    .Kp = 2.2f,
    .Ki = 0.22f,
    .Kd = 0.0f,
    .MaxOut = 1500.0f,
    .IntegralLimit = 100.0f,
    .DeadBand = 20.0f,
    .Improve = Integral_Limit,
};

PID_t pid_yaw = {
    .Kp = 2.10f,
    .Ki = 0.22f,
    .Kd = 0.001f,
    .MaxOut = MAX_ROTATION_VELOCITY*0.75*180.0f/M_PI,
    .IntegralLimit = 5.0f,
    .DeadBand = 0.3f,
    .Improve = Integral_Limit,
};

PID_t pid_high_distance = {
    .Kp = 0.16f,
    .Ki = 0.05f,
    .Kd = 0.0f,
    .MaxOut = 400.0f,
    .IntegralLimit = 200.0f,
    .DeadBand = 5.0f,
    .Improve = Integral_Limit,
};

PID_t pid_high_yaw = {
    .Kp = 2.0f,
    .Ki = 0.02f,
    .Kd = 0.0f,
    .MaxOut = 300.0f,
    .IntegralLimit = 150.0f,
    .DeadBand = 0.3f,
    .Improve = Integral_Limit,
};

static TypedTopicPublisher<pub_chassis_cmd> chassis_cmd_pub("chassis_cmd");
static TypedTopicPublisher<pub_high_nav_cmd> high_nav_pub("high_nav_cmd");
TypedTopicPublisher<pc_nav_event_t> pc_nav_event_pub("pc_nav_event_pub");

namespace {
void resetLowNavRuntime();
void resetPathTrackingReference();
}

namespace nav_control {
int16_t current_x = 0;
int16_t current_y = 0;
int16_t current_yaw = 0;
int16_t pc_reported_yaw = 0;
int16_t target_x = 0;
int16_t target_y = 0;
int16_t target_yaw = 0;
bool auto_enabled = false;
bool arrived = false;
bool target_active = false;
bool arrival_reported = false;
bool high_mode_active = false;
TickType_t g_last_position_update_tick = 0;

void updatePositionTimestamp() {
  g_last_position_update_tick = xTaskGetTickCount();
}

void resetAllPIDs() {
  PID_Init(&pid_x);
  PID_Init(&pid_y);
  PID_Init(&pid_yaw);
  PID_Init(&pid_high_distance);
  PID_Init(&pid_high_yaw);
  resetPathTrackingReference();
  resetLowNavRuntime();
}

}  // namespace nav_control

namespace {

constexpr TickType_t kPositionTimeoutTicks = pdMS_TO_TICKS(1000);
constexpr float kHighCruiseSpeedRpm = 500.0f;
constexpr float kHighCrawlSpeedRpm = 50.0f;
constexpr float kHighSlowdownDistMm = 100.0f;
constexpr float kNavControlDtSec = 0.01f;
constexpr float kMmToM = 0.001f;

float s_nav_vx_cmd = 0.0f;
float s_nav_vy_cmd = 0.0f;
float s_nav_omega_cmd = 0.0f;
uint8_t s_nav_arrive_hold_count = 0U;
float s_nav_path_tx = 1.0f;
float s_nav_path_ty = 0.0f;
float s_nav_path_nx = 0.0f;
float s_nav_path_ny = 1.0f;

bool isPositionFresh(TickType_t now) {
  return (nav_control::g_last_position_update_tick != 0U) &&
         ((now - nav_control::g_last_position_update_tick) <=
          kPositionTimeoutTicks);
}

float normalizeDeg(float angle_deg) {
  while (angle_deg > 180.0f) {
    angle_deg -= 360.0f;
  }
  while (angle_deg < -180.0f) {
    angle_deg += 360.0f;
  }
  return angle_deg;
}

float clampf(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

void resetLowNavRuntime() {
  s_nav_vx_cmd = 0.0f;
  s_nav_vy_cmd = 0.0f;
  s_nav_omega_cmd = 0.0f;
  s_nav_arrive_hold_count = 0U;

  g_ozone_nav_dist_mm = 0.0f;
  g_ozone_nav_yaw_err_deg = 0.0f;
  g_ozone_nav_path_err_mm = 0.0f;
  g_ozone_nav_lateral_err_mm = 0.0f;
  g_ozone_nav_blend = 0.0f;
  g_ozone_nav_plan_speed_mps = 0.0f;
  g_ozone_nav_v_ref_mps = 0.0f;
  g_ozone_nav_vx_ref_mps = 0.0f;
  g_ozone_nav_vy_ref_mps = 0.0f;
  g_ozone_nav_omega_ref_radps = 0.0f;
  g_ozone_nav_pid_vx_mps = 0.0f;
  g_ozone_nav_pid_vy_mps = 0.0f;
  g_ozone_nav_pid_omega_radps = 0.0f;
  g_ozone_nav_path_tx = s_nav_path_tx;
  g_ozone_nav_path_ty = s_nav_path_ty;
  g_ozone_nav_path_nx = s_nav_path_nx;
  g_ozone_nav_path_ny = s_nav_path_ny;
  g_ozone_nav_vx_cmd_mps = 0.0f;
  g_ozone_nav_vy_cmd_mps = 0.0f;
  g_ozone_nav_omega_cmd_radps = 0.0f;
  g_ozone_nav_cmd_speed_mps = 0.0f;
  g_ozone_nav_brake_dist_mm = 0.0f;
  g_ozone_nav_arrive_hold_count = 0U;
}

void resetPathTrackingReference() {
  const float dx =
      static_cast<float>(nav_control::target_x - nav_control::current_x);
  const float dy =
      static_cast<float>(nav_control::target_y - nav_control::current_y);
  const float length = sqrtf(dx * dx + dy * dy);

  if (length > 1e-3f) {
    s_nav_path_tx = dx / length;
    s_nav_path_ty = dy / length;
    s_nav_path_nx = -s_nav_path_ty;
    s_nav_path_ny = s_nav_path_tx;
  } else {
    s_nav_path_tx = 1.0f;
    s_nav_path_ty = 0.0f;
    s_nav_path_nx = 0.0f;
    s_nav_path_ny = 1.0f;
  }

  g_ozone_nav_path_tx = s_nav_path_tx;
  g_ozone_nav_path_ty = s_nav_path_ty;
  g_ozone_nav_path_nx = s_nav_path_nx;
  g_ozone_nav_path_ny = s_nav_path_ny;
}

float calcBlendFactor(float dist_mm) {
  const float denom = g_nav_blend_dist_mm - g_nav_arrive_dist_mm;
  if (denom <= 1e-3f) {
    return (dist_mm > g_nav_arrive_dist_mm) ? 1.0f : 0.0f;
  }

  const float t =
      clampf((dist_mm - g_nav_arrive_dist_mm) / denom, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

float calcBrakeLimitedSpeed(float dist_mm) {
  const float remain_mm = dist_mm - g_nav_arrive_dist_mm;
  if (remain_mm <= 0.0f) {
    return 0.0f;
  }

  const float safety =
      (g_nav_brake_safety_scale < 1.0f) ? 1.0f : g_nav_brake_safety_scale;
  const float decel =
      (g_nav_max_decel_mps2 <= 1e-3f) ? 1e-3f : g_nav_max_decel_mps2;
  const float remain_m = remain_mm * kMmToM;
  const float effective_decel = decel / safety;
  const float v_allow = sqrtf(2.0f * effective_decel * remain_m);
  return clampf(v_allow, 0.0f, g_nav_cruise_speed_mps);
}

float calcBrakeDistanceMm(float speed_mps) {
  const float safety =
      (g_nav_brake_safety_scale < 1.0f) ? 1.0f : g_nav_brake_safety_scale;
  const float decel =
      (g_nav_max_decel_mps2 <= 1e-3f) ? 1e-3f : g_nav_max_decel_mps2;
  return 1000.0f * speed_mps * speed_mps * safety / (2.0f * decel);
}

float calcYawSlowdownScale(float yaw_error_deg) {
  const float abs_yaw = fabsf(yaw_error_deg);

  if (abs_yaw <= g_nav_yaw_slowdown_start_deg) {
    return 1.0f;
  }
  if (abs_yaw >= 90.0f) {
    return g_nav_yaw_slowdown_min_scale;
  }

  const float denom = 90.0f - g_nav_yaw_slowdown_start_deg;
  if (denom <= 1e-3f) {
    return g_nav_yaw_slowdown_min_scale;
  }

  const float t = clampf(
      (abs_yaw - g_nav_yaw_slowdown_start_deg) / denom, 0.0f, 1.0f);
  return 1.0f - t * (1.0f - g_nav_yaw_slowdown_min_scale);
}

[[maybe_unused]] float calcOmegaPlan(float yaw_error_deg) {
  const float abs_yaw = fabsf(yaw_error_deg);

  if (abs_yaw <= g_nav_arrive_yaw_deg) {
    return 0.0f;
  }

  float omega = g_nav_max_omega_radps;
  if (abs_yaw < g_nav_omega_slowdown_deg) {
    const float denom = g_nav_omega_slowdown_deg;
    const float t =
        (denom <= 1e-3f) ? 1.0f : clampf(abs_yaw / denom, 0.0f, 1.0f);
    omega = g_nav_min_omega_radps +
            t * (g_nav_max_omega_radps - g_nav_min_omega_radps);
  }

  return (yaw_error_deg >= 0.0f) ? omega : -omega;
}

float rampToward(float current, float target, float accel_step,
                 float decel_step) {
  if (target > current) {
    const float step = (target >= 0.0f) ? accel_step : decel_step;
    return ((target - current) > step) ? (current + step) : target;
  }

  if (target < current) {
    const float step = (target <= 0.0f) ? accel_step : decel_step;
    return ((current - target) > step) ? (current - step) : target;
  }

  return target;
}

void publishAutoStopCmd() {
  pub_chassis_cmd cmd{};
  cmd.nav_mode_ = true;
  chassis_cmd_pub.Publish(cmd);
  resetLowNavRuntime();
}

void reportArrivalOnce() {
  if (!nav_control::target_active || nav_control::arrival_reported) {
    return;
  }

  nav_control::arrival_reported = true;

  pc_nav_event_t evt{0x0201};
  pc_nav_event_pub.Publish(evt);
}

}  // namespace

void NavProtocol::reset() {
  index_ = 0;
  memset(line_buf_, 0, sizeof(line_buf_));
}

bool NavProtocol::processByte(uint8_t byte, NavCmd &out_cmd) {
  if (byte == '\n') {
    line_buf_[index_] = '\0';
    const char cmd_char = static_cast<char>(toupper(line_buf_[0]));
    int16_t p1 = 0;
    int16_t p2 = 0;
    int16_t p3 = 0;

    if (cmd_char == 'T') {
      const int num_read =
          sscanf(line_buf_, "%*c %hd %hd %hd", &p1, &p2, &p3);
      if (num_read >= 2) {
        out_cmd.type = CmdType::TARGET;
        out_cmd.param1 = p1;
        out_cmd.param2 = p2;
        out_cmd.param3 = (num_read >= 3) ? p3 : 0;
        reset();
        return true;
      }
    } else if (cmd_char == 'P') {
      const int num_read = sscanf(line_buf_, "%*c %hd %hd", &p1, &p2);
      if (num_read >= 2) {
        out_cmd.type = CmdType::POSITION;
        out_cmd.param1 = p1;
        out_cmd.param2 = p2;
        out_cmd.param3 = 0;
        reset();
        return true;
      }
    } else if (cmd_char == 'Q') {
      out_cmd.type = CmdType::QUERY;
      out_cmd.param1 = 0;
      out_cmd.param2 = 0;
      out_cmd.param3 = 0;
      reset();
      return true;
    } else if (cmd_char == 'S') {
      out_cmd.type = CmdType::STOP;
      out_cmd.param1 = 0;
      out_cmd.param2 = 0;
      out_cmd.param3 = 0;
      reset();
      return true;
    }

    reset();
    return false;
  }

  if (byte != '\r' && index_ < sizeof(line_buf_) - 1) {
    line_buf_[index_++] = static_cast<char>(byte);
  }
  return false;
}

void NavProtocol::buildResponse(const NavCmd &cmd, char *buf, size_t buf_size) {
  const TickType_t now = xTaskGetTickCount();

  switch (cmd.type) {
    case CmdType::TARGET:
      nav_control::target_x = cmd.param1;
      nav_control::target_y = cmd.param2;
      nav_control::target_yaw = cmd.param3;
      nav_control::auto_enabled = true;
      nav_control::arrived = false;
      nav_control::target_active = true;
      nav_control::arrival_reported = false;
      nav_control::resetAllPIDs();
      break;

    case CmdType::POSITION:
      nav_control::current_x = cmd.param1;
      nav_control::current_y = cmd.param2;
      nav_control::g_last_position_update_tick = now;
      snprintf(buf, buf_size, "POS OK X=%hd Y=%hd\n",
               nav_control::current_x, nav_control::current_y);
      break;

    case CmdType::QUERY:
      snprintf(
          buf, buf_size,
          "STATE X=%hd Y=%hd YAW=%hd AUTO=%d POS_FRESH=%d ARRIVED=%d TARGET=%d\n",
          nav_control::current_x, nav_control::current_y,
          nav_control::current_yaw, nav_control::auto_enabled ? 1 : 0,
          isPositionFresh(now) ? 1 : 0, nav_control::arrived ? 1 : 0,
          nav_control::target_active ? 1 : 0);
      break;

    case CmdType::STOP:
      nav_control::auto_enabled = false;
      nav_control::arrived = false;
      nav_control::target_active = false;
      nav_control::arrival_reported = false;
      resetLowNavRuntime();
      snprintf(buf, buf_size, "STOP OK\n");
      break;

    default:
      snprintf(buf, buf_size, "UNKNOWN\n");
      break;
  }
}

osThreadId_t NavControlTaskHandle;

void NavControlTask(void *argument) {
  (void)argument;
  TickType_t lastWakeTime = xTaskGetTickCount();

  nav_control::resetAllPIDs();

  for (;;) {
    const TickType_t now = xTaskGetTickCount();

    nav_control::current_yaw = static_cast<int16_t>(g_chassis_yaw_deg);

    if (nav_control::auto_enabled) {
      if (!isPositionFresh(now)) {
        nav_control::arrived = false;
        publishAutoStopCmd();
        vTaskDelayUntil(&lastWakeTime, 10);
        continue;
      }

      if (nav_control::high_mode_active) {
        const float error_x =
            static_cast<float>(nav_control::target_x - nav_control::current_x);
        const float error_y =
            static_cast<float>(nav_control::target_y - nav_control::current_y);

        const float yaw_rad =
            static_cast<float>(nav_control::current_yaw) * 3.14159f / 180.0f;
        const float error_x_body =
            error_x * cosf(yaw_rad) + error_y * sinf(yaw_rad);
        const float error_y_body =
            error_x * sinf(yaw_rad) - error_y * cosf(yaw_rad);

        const float raw_heading =
            atan2f(error_y_body, error_x_body) * 180.0f / 3.14159f;
        const float heading_error =
            (error_x_body >= 0) ? raw_heading
                                : normalizeDeg(raw_heading - 180.0f);

        pub_high_nav_cmd high_cmd{};
        high_cmd.active = true;

        const float abs_error_x_body = fabsf(error_x_body);
        const float base_speed =
            (abs_error_x_body <= kHighSlowdownDistMm) ? kHighCrawlSpeedRpm
                                                      : kHighCruiseSpeedRpm;
        high_cmd.forward_speed = (error_x_body >= 0) ? base_speed : -base_speed;
        high_cmd.omega = PID_Calculate(&pid_high_yaw, 0.0f, heading_error);

        if (high_cmd.forward_speed > 500.0f) high_cmd.forward_speed = 500.0f;
        if (high_cmd.forward_speed < -500.0f)
          high_cmd.forward_speed = -500.0f;
        if (high_cmd.omega > 500.0f) high_cmd.omega = 500.0f;
        if (high_cmd.omega < -500.0f) high_cmd.omega = -500.0f;

        const bool reached = (fabsf(error_x_body) < 10.0f);
        nav_control::arrived = reached;
        if (reached) {
          reportArrivalOnce();
        } else {
          nav_control::arrival_reported = false;
        }

        high_nav_pub.Publish(high_cmd);
        vTaskDelayUntil(&lastWakeTime, 10);
        continue;
      }

      const float error_x =
          static_cast<float>(nav_control::target_x - nav_control::current_x);
      const float error_y =
          static_cast<float>(nav_control::target_y - nav_control::current_y);
      const float error_yaw = normalizeDeg(static_cast<float>(
          nav_control::target_yaw - nav_control::current_yaw));

      const float yaw_rad =
          static_cast<float>(nav_control::current_yaw) * 3.14159f / 180.0f;
      const float path_error =
          error_x * s_nav_path_tx + error_y * s_nav_path_ty;
      const float lateral_error =
          error_x * s_nav_path_nx + error_y * s_nav_path_ny;
      const float dist_error =
          sqrtf(path_error * path_error + lateral_error * lateral_error);

      const float blend = calcBlendFactor(dist_error);
      const float brake_speed = calcBrakeLimitedSpeed(dist_error);
      const float yaw_scale = calcYawSlowdownScale(error_yaw);
      const float plan_speed = brake_speed * yaw_scale;
      const float path_dir = (path_error >= 0.0f) ? 1.0f : -1.0f;
      const float vx_plan_world = plan_speed * path_dir * s_nav_path_tx;
      const float vy_plan_world = plan_speed * path_dir * s_nav_path_ty;
      const float path_pid_raw =
          PID_Calculate(&pid_x, 0.0f, path_error) / 1000.0f;
      const float lateral_pid_raw =
          PID_Calculate(&pid_y, 0.0f, lateral_error) / 1000.0f;

      const float omega_pid_raw = PID_Calculate(&pid_yaw, 0.0f, error_yaw)*M_PI/180.0f;

      const float path_pid = clampf(path_pid_raw, -g_nav_pid_max_xy_speed_mps,
                                    g_nav_pid_max_xy_speed_mps);
      const float lateral_pid =
          clampf(lateral_pid_raw, -g_nav_pid_max_xy_speed_mps,
                 g_nav_pid_max_xy_speed_mps);
      const float omega_pid = clampf(omega_pid_raw, -g_nav_pid_max_omega_radps,
                                     g_nav_pid_max_omega_radps);

      const float vx_pid_world =
          path_pid * s_nav_path_tx + lateral_pid * s_nav_path_nx;
      const float vy_pid_world =
          path_pid * s_nav_path_ty + lateral_pid * s_nav_path_ny;
      const float vx_ref_world =
          blend * vx_plan_world + (1.0f - blend) * vx_pid_world;
      const float vy_ref_world =
          blend * vy_plan_world + (1.0f - blend) * vy_pid_world;
      const float vx_ref =
          vx_ref_world * cosf(yaw_rad) + vy_ref_world * sinf(yaw_rad);
      const float vy_ref =
          -vx_ref_world * sinf(yaw_rad) + vy_ref_world * cosf(yaw_rad);
      const float omega_ref = omega_pid;

      s_nav_vx_cmd = rampToward(
          s_nav_vx_cmd, vx_ref, g_nav_max_accel_mps2 * kNavControlDtSec,
          g_nav_max_decel_mps2 * kNavControlDtSec);
      s_nav_vy_cmd = rampToward(
          s_nav_vy_cmd, vy_ref, g_nav_max_accel_mps2 * kNavControlDtSec,
          g_nav_max_decel_mps2 * kNavControlDtSec);
      s_nav_omega_cmd = rampToward(
          s_nav_omega_cmd, omega_ref,
          g_nav_max_omega_accel_radps2 * kNavControlDtSec,
          g_nav_max_omega_accel_radps2 * kNavControlDtSec);

      pub_chassis_cmd cmd{};
      // cmd.linear_x_ = s_nav_vx_cmd;
      // cmd.linear_y_ = s_nav_vy_cmd;
      // cmd.omega_ = s_nav_omega_cmd;
      cmd.linear_x_ = vx_ref;
      cmd.linear_y_ = vy_ref;
      cmd.omega_ = omega_ref;
      cmd.nav_mode_ = true;
      chassis_cmd_pub.Publish(cmd);

      const bool pose_ok = dist_error < g_nav_arrive_dist_mm;
      const bool yaw_ok = fabsf(error_yaw) < g_nav_arrive_yaw_deg;
      if (pose_ok && yaw_ok) {
        if (s_nav_arrive_hold_count < g_nav_arrive_hold_count_target) {
          ++s_nav_arrive_hold_count;
        }
      } else {
        s_nav_arrive_hold_count = 0U;
      }

      const bool reached =
          (s_nav_arrive_hold_count >= g_nav_arrive_hold_count_target);

      nav_control::arrived = reached;
      if (reached) {
        reportArrivalOnce();
      } else {
        nav_control::arrival_reported = false;
      }

      g_ozone_nav_dist_mm = dist_error;
      g_ozone_nav_yaw_err_deg = error_yaw;
      g_ozone_nav_path_err_mm = path_error;
      g_ozone_nav_lateral_err_mm = lateral_error;
      g_ozone_nav_blend = blend;
      g_ozone_nav_plan_speed_mps = plan_speed;
      g_ozone_nav_v_ref_mps = sqrtf(vx_ref * vx_ref + vy_ref * vy_ref);
      g_ozone_nav_vx_ref_mps = vx_ref;
      g_ozone_nav_vy_ref_mps = vy_ref;
      g_ozone_nav_omega_ref_radps = omega_ref;
      g_ozone_nav_pid_vx_mps = path_pid;
      g_ozone_nav_pid_vy_mps = lateral_pid;
      g_ozone_nav_pid_omega_radps = omega_pid;
      g_ozone_nav_vx_cmd_mps = s_nav_vx_cmd;
      g_ozone_nav_vy_cmd_mps = s_nav_vy_cmd;
      g_ozone_nav_omega_cmd_radps = s_nav_omega_cmd;
      g_ozone_nav_cmd_speed_mps =
          sqrtf(s_nav_vx_cmd * s_nav_vx_cmd + s_nav_vy_cmd * s_nav_vy_cmd);
      g_ozone_nav_brake_dist_mm =
          calcBrakeDistanceMm(g_ozone_nav_cmd_speed_mps);
      g_ozone_nav_arrive_hold_count = s_nav_arrive_hold_count;
    }

    vTaskDelayUntil(&lastWakeTime, 10);
  }
}
