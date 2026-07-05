#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <stdint.h>

void debugTask(void *argument);

extern volatile int32_t g_ozone_laser2_mm;
extern volatile uint8_t g_ozone_laser2_fresh;
extern volatile int32_t g_ozone_laser3_mm;
extern volatile uint8_t g_ozone_laser3_fresh;
extern volatile uint8_t g_ozone_laser3_profile;
extern volatile int32_t g_ozone_laser3_near_min_used_mm;
extern volatile int32_t g_ozone_laser3_near_max_used_mm;

extern volatile uint8_t g_ozone_stair_assist_enabled;
extern volatile uint8_t g_ozone_stair_assist_mode;
extern volatile uint8_t g_ozone_stair_step;
extern volatile uint8_t g_ozone_stair_level;
extern volatile uint8_t g_ozone_stair_armed;
extern volatile uint8_t g_ozone_high_mode_active;

extern volatile int32_t g_ozone_target_x;
extern volatile int32_t g_ozone_current_x;
extern volatile int32_t g_ozone_target_y;
extern volatile int32_t g_ozone_current_y;
extern volatile int32_t g_ozone_target_yaw;
extern volatile int32_t g_ozone_current_yaw;

extern volatile int32_t g_ozone_xbox_target_x;
extern volatile int32_t g_ozone_xbox_target_y;
extern volatile int32_t g_ozone_xbox_target_yaw;

extern volatile uint8_t g_ozone_merlin_cell_valid;
extern volatile uint8_t g_ozone_merlin_row;
extern volatile uint8_t g_ozone_merlin_col;
extern volatile int32_t g_ozone_merlin_height_mm;
extern volatile uint8_t g_ozone_merlin_heading;
extern volatile int32_t g_ozone_merlin_query_x;
extern volatile int32_t g_ozone_merlin_query_y;
extern volatile int32_t g_ozone_merlin_matched_center_x;
extern volatile int32_t g_ozone_merlin_matched_center_y;
extern volatile int32_t g_ozone_merlin_nearest_dist_sq;

#ifdef __cplusplus
}
#endif
