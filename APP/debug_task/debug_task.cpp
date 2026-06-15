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
extern float lift_2006_motor1_pid_out;
extern float lift_2006_motor2_pid_out;

osThreadId_t Debug_TaskHandle;

void debugTask(void *argument) {
  (void)argument;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, 20);
    const auto &map_debug = merlin_map::debug();

    logger.log(
        "%.1f,%.0f,%.1f,%.1f,"
        "%.1f,%.0f,%.1f,%.1f,"
        "%.1f,%.0f,%.1f,%.1f,"
        "%.1f,%.0f,%.1f,%.1f,"
        "%.1f,%.1f,%.1f,%.1f,"
        "%d,%d,%d,%d,%d,%d,%d,"
        "%ld,%d,%d,%d,%d,%d,%ld\n",
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
        chassis_motor4.cmd_,

        lift_2006_motor1.getRawCurrentSpeed(),
        lift_2006_motor2.getRawCurrentSpeed(),
        lift_2006_motor1_pid_out,
        lift_2006_motor2_pid_out,

        nav_control::target_x,
        nav_control::current_x,
        nav_control::target_y,
        nav_control::current_y,
        static_cast<int>(stairWaypointStep()),
        static_cast<int>(stairWaypointLevel()),
        stairWaypointArmed() ? 1 : 0,
        stairAssistDebug().laser2_mm,
        stairAssistDebug().laser2_fresh ? 1 : 0,
        stairAssistDebug().should_lower_after_descend ? 1 : 0,
        map_debug.cell_valid ? 1 : 0,
        static_cast<int>(map_debug.row),
        static_cast<int>(map_debug.col),
        static_cast<int>(map_debug.height_mm),
        static_cast<int>(map_debug.heading),
        static_cast<long>(map_debug.nearest_dist_sq));
  }
}
