/**
 * @file debug_task.cpp
 * @brief 调试任务，VOFA+ FireWater 输出
 */
#include "debug_task.h"

#include "task.h"
#include "cmsis_os2.h"

#include "logger.hpp"
#include "com_config.h"
#include "merlin_map/merlin_map.h"
#include "Motor.hpp"
#include "NavProtocol.hpp"
#include "lift_task.h"
#include "stair_assist.h"
#include "state_machine_task.h"
#include "waypoint_navigator.hpp"

extern C620Motor chassis_motor1, chassis_motor2, chassis_motor3, chassis_motor4;
extern C610Motor lift_2006_motor1, lift_2006_motor2;
extern Logger logger;
extern LoggerQueue logger_queue;
extern float lift_2006_motor1_pid_out;
extern float lift_2006_motor2_pid_out;

osThreadId_t Debug_TaskHandle;

volatile int32_t g_ozone_laser2_mm = 0;
volatile uint32_t g_ozone_laser2_frame_count = 0;
volatile uint8_t g_ozone_laser2_fresh = 0;
volatile int32_t g_ozone_laser3_mm = 0;
volatile uint32_t g_ozone_laser3_frame_count = 0;
volatile uint8_t g_ozone_laser3_fresh = 0;

volatile uint8_t g_ozone_stair_assist_enabled = 0;
volatile uint8_t g_ozone_stair_assist_mode = 0;
volatile uint8_t g_ozone_suggest_climb_up = 0;
volatile uint8_t g_ozone_suggest_descend_high = 0;
volatile uint8_t g_ozone_should_lower_after_climb = 0;
volatile uint8_t g_ozone_should_lower_after_descend = 0;
volatile uint8_t g_ozone_stair_step = 0;
volatile uint8_t g_ozone_stair_level = 0;
volatile uint8_t g_ozone_stair_armed = 0;
volatile uint8_t g_ozone_robot_state = 0;
volatile uint8_t g_ozone_high_mode_active = 0;

volatile int32_t g_ozone_target_x = 0;
volatile int32_t g_ozone_current_x = 0;
volatile int32_t g_ozone_target_y = 0;
volatile int32_t g_ozone_current_y = 0;

volatile uint8_t g_ozone_merlin_cell_valid = 0;
volatile uint8_t g_ozone_merlin_row = 0;
volatile uint8_t g_ozone_merlin_col = 0;
volatile int32_t g_ozone_merlin_height_mm = 0;
volatile uint8_t g_ozone_merlin_heading = 0;
volatile int32_t g_ozone_merlin_query_x = 0;
volatile int32_t g_ozone_merlin_query_y = 0;
volatile int32_t g_ozone_merlin_matched_center_x = 0;
volatile int32_t g_ozone_merlin_matched_center_y = 0;
volatile int32_t g_ozone_merlin_nearest_dist_sq = 0;

void debugTask(void *argument) {
  (void)argument;

  for (;;) {
    osDelay(50);
    // logger_queue.log("Hello world!");
    logger_queue.trySend();
    logger.log(
        "%.1f|%.0f|%.1f|%.1f/"
        "%.1f|%.0f|%.1f|%.1f/"
        "%.1f|%.0f|%.1f|%.1f/"
        "%.1f|%.0f|%.1f|%.1f/"
        "\n",
        chassis_motor1.getCurrentSpeed() / RPM_2_RAD_PER_SEC,
        chassis_motor1.getRawCurrentTorque(),
        chassis_motor1.getCurrentTemperature(),
        chassis_motor1.cmd_,

        chassis_motor2.getCurrentSpeed() / RPM_2_RAD_PER_SEC,
        chassis_motor2.getRawCurrentTorque(),
        chassis_motor2.getCurrentTemperature(),
        chassis_motor2.cmd_,

        chassis_motor3.getCurrentSpeed() / RPM_2_RAD_PER_SEC,
        chassis_motor3.getRawCurrentTorque(),
        chassis_motor3.getCurrentTemperature(),
        chassis_motor3.cmd_,

        chassis_motor4.getCurrentSpeed() / RPM_2_RAD_PER_SEC,
        chassis_motor4.getRawCurrentTorque(),
        chassis_motor4.getCurrentTemperature(),
        chassis_motor4.cmd_
    );
  }
}
