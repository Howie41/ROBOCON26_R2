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
#include "motor_task.h"
#include "com_config.h"
#include "Motor.hpp"
#include "pid_controller.h"
#include "topics.hpp"
#include "FreeRTOS.h"
#include "task.h"


osThreadId_t Motor_TaskHandle;


extern C620Motor arm3508_motor;
extern C610Motor arm2006_motor;


static PID_t arm3508_pos_pid{
    .Kp = 45.0f,
    .Ki = 0.0f,
    .Kd = 2.5f,
    .MaxOut = 60.0f,
    .DeadBand = 0.1f
};
static PID_t arm3508_speed_pid{
    .Kp = 2000.0f,
    .Ki = 0.06f,
    .Kd = 1.8f,
    .MaxOut = 12000.0f,
    .DeadBand = 0.5f
};


static PID_t arm2006_pos_pid{
    .Kp = 45.0f,
    .Ki = 0.0f,
    .Kd = 2.5f,
    .MaxOut = 60.0f,
    .DeadBand = 0.1f
};
static PID_t arm2006_speed_pid{
    .Kp = 2000.0f,
    .Ki = 0.06f,
    .Kd = 1.8f,
    .MaxOut = 12000.0f,
    .DeadBand = 0.5f
};


void motorInit() {
    PID_Init(&arm3508_pos_pid);
    PID_Init(&arm3508_speed_pid);

    PID_Init(&arm2006_pos_pid);
    PID_Init(&arm2006_speed_pid);
}

void motorTask(void *argument) {
  TickType_t currentTime;
  currentTime = xTaskGetTickCount();
  motorInit();

  for (;;) {

    arm3508_pos_pid.MaxOut = arm3508_motor.updatePosProcess();
    arm3508_motor.setMotorCmd(PID_Calculate(&arm3508_speed_pid, arm3508_motor.getCurrentSpeed(), PID_Calculate(&arm3508_pos_pid, arm3508_motor.getCurrentSumPos(), arm3508_motor.tar_sum_pos_)));

    arm2006_pos_pid.MaxOut = arm2006_motor.updatePosProcess();
    arm2006_motor.setMotorCmd(PID_Calculate(&arm2006_speed_pid, arm2006_motor.getCurrentSpeed(), PID_Calculate(&arm2006_pos_pid, arm2006_motor.getCurrentSumPos(), arm2006_motor.tar_sum_pos_)));

    vTaskDelayUntil(&currentTime, 1);

  }
}
