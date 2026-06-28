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
#include "motor_task.hpp"

osThreadId_t Debug_TaskHandle;


extern MotorPlanningUnit arm2006_motor_planner;


uint8_t funfer;

void debugTask(void *argument) {
    funfer = 0;

    for (;;) {

        if (funfer) {
            funfer = 0;
            
            arm2006_motor_planner.plan(2.0f, 360.0f, 10.0f);
            
        }

        osDelay(1);
    }
}
