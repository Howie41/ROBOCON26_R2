#include "PoseEstimator.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include <cmath>

volatile float g_ozone_pose_fused_x_mm = 0.0f;
volatile float g_ozone_pose_fused_y_mm = 0.0f;
volatile float g_ozone_pose_body_dx_mm = 0.0f;
volatile float g_ozone_pose_body_dy_mm = 0.0f;
volatile float g_ozone_pose_world_dx_mm = 0.0f;
volatile float g_ozone_pose_world_dy_mm = 0.0f;
volatile float g_ozone_pose_radar_correction_x_mm = 0.0f;
volatile float g_ozone_pose_radar_correction_y_mm = 0.0f;
volatile uint32_t g_ozone_pose_radar_update_count = 0U;

namespace nav_localization {
namespace {

constexpr float kDegToRad = 0.01745329251994329577f;

float s_fused_x_mm = 0.0f;
float s_fused_y_mm = 0.0f;
bool s_pose_valid = false;

}  // namespace

void submitRadarPosition(float x_mm, float y_mm) {
  taskENTER_CRITICAL();

  if (s_pose_valid) {
    g_ozone_pose_radar_correction_x_mm = x_mm - s_fused_x_mm;
    g_ozone_pose_radar_correction_y_mm = y_mm - s_fused_y_mm;
  } else {
    g_ozone_pose_radar_correction_x_mm = 0.0f;
    g_ozone_pose_radar_correction_y_mm = 0.0f;
  }

  s_fused_x_mm = x_mm;
  s_fused_y_mm = y_mm;
  s_pose_valid = true;
  g_ozone_pose_fused_x_mm = s_fused_x_mm;
  g_ozone_pose_fused_y_mm = s_fused_y_mm;
  const uint32_t next_update_count = g_ozone_pose_radar_update_count + 1U;
  g_ozone_pose_radar_update_count = next_update_count;

  taskEXIT_CRITICAL();
}

void integrateBodyDisplacement(float dx_body_mm, float dy_body_mm,
                               float yaw_deg) {
  const float yaw_rad = yaw_deg * kDegToRad;
  const float cos_yaw = std::cos(yaw_rad);
  const float sin_yaw = std::sin(yaw_rad);
  const float dx_world_mm =
      dx_body_mm * cos_yaw - dy_body_mm * sin_yaw;
  const float dy_world_mm =
      dx_body_mm * sin_yaw + dy_body_mm * cos_yaw;

  taskENTER_CRITICAL();

  g_ozone_pose_body_dx_mm = dx_body_mm;
  g_ozone_pose_body_dy_mm = dy_body_mm;
  g_ozone_pose_world_dx_mm = dx_world_mm;
  g_ozone_pose_world_dy_mm = dy_world_mm;

  if (s_pose_valid) {
    s_fused_x_mm += dx_world_mm;
    s_fused_y_mm += dy_world_mm;
    g_ozone_pose_fused_x_mm = s_fused_x_mm;
    g_ozone_pose_fused_y_mm = s_fused_y_mm;
  }

  taskEXIT_CRITICAL();
}

PoseSnapshot snapshot() {
  PoseSnapshot result{};

  taskENTER_CRITICAL();
  result.x_mm = s_fused_x_mm;
  result.y_mm = s_fused_y_mm;
  result.valid = s_pose_valid;
  taskEXIT_CRITICAL();

  return result;
}

}  // namespace nav_localization
