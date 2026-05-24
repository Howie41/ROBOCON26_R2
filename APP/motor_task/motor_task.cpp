/**
 * @file motor_task.cpp
 * @author FunFer
 * @brief
 * @version 0.1
 * @date 2026-04-18
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "stm32h7xx_hal.h"
#include "motor_task.hpp"
#include "com_config.h"
#include "Motor.hpp"
#include "pid_controller.h"
#include "topics.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <vector>


osThreadId_t Motor_TaskHandle;

extern MotorPlanningSystem motor_planning_system;

extern C620Motor arm3508_motor;
extern C610Motor arm2006_motor;


/** @brief 电机任务函数
 *  @param argument 任务参数
 */
void motorTask(void *argument) {
  TickType_t currentTime;
  currentTime = xTaskGetTickCount();

  for (;;) {
    motor_planning_system.update();
    vTaskDelayUntil(&currentTime, 1);

  }
}



// static PID_t arm3508_pos_pid{
//     .Kp = 45.0f,
//     .Ki = 0.0f,
//     .Kd = 2.5f,
//     .MaxOut = 60.0f,
//     .DeadBand = 0.1f
// };
// static PID_t arm3508_speed_pid{
//     .Kp = 2000.0f,
//     .Ki = 0.06f,
//     .Kd = 1.8f,
//     .MaxOut = 12000.0f,
//     .DeadBand = 0.5f
// };
