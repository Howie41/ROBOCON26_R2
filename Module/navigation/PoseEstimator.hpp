#pragma once

#include <cstdint>

// 导航使用的世界坐标系融合位置，单位：mm。
extern volatile float g_ozone_pose_fused_x_mm;
extern volatile float g_ozone_pose_fused_y_mm;

// 最近一帧雷达上报的世界坐标系原始位置，单位：mm。
extern volatile float g_ozone_pose_radar_x_mm;
extern volatile float g_ozone_pose_radar_y_mm;

// 最近一次 1 kHz 轮式里程计更新得到的底盘坐标系位移增量，单位：mm。
extern volatile float g_ozone_pose_body_dx_mm;
extern volatile float g_ozone_pose_body_dy_mm;

// 使用 IMU 航向角旋转到世界坐标系后的同一位移增量，单位：mm。
extern volatile float g_ozone_pose_world_dx_mm;
extern volatile float g_ozone_pose_world_dy_mm;

// 雷达位置减去重新锚定前的融合位置，单位：mm。
// 数值过大或长期存在同向偏差时，应检查里程计比例、方向、车轮打滑和雷达延迟。
extern volatile float g_ozone_pose_radar_correction_x_mm;
extern volatile float g_ozone_pose_radar_correction_y_mm;

// 上电后已接收并采用的雷达位置帧数量。
extern volatile uint32_t g_ozone_pose_radar_update_count;

namespace nav_localization {

struct PoseSnapshot {
  float x_mm;  // 世界坐标系融合 X 位置，单位：mm。
  float y_mm;  // 世界坐标系融合 Y 位置，单位：mm。
  float radar_x_mm;  // 最近一帧雷达原始 X 位置，单位：mm。
  float radar_y_mm;  // 最近一帧雷达原始 Y 位置，单位：mm。
  bool valid;  // 接收到第一帧有效雷达位置后为 true。
};

void submitRadarPosition(float x_mm, float y_mm);
void integrateBodyDisplacement(float dx_body_mm, float dy_body_mm,
                               float yaw_deg);
PoseSnapshot snapshot();

}  // namespace nav_localization
