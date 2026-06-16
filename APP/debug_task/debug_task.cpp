/**
 * @file debug_task.cpp
 * @author 大帅将军
 * @brief 调试任务，测试用，后续可能会删除
 * @version 0.1
 * @date 2026-04-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "debug_task.h"
#include "Motor.hpp"
#include "cmsis_os2.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_tim.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "gpio.h"
#include "task.h"
#include "logger.hpp"
#include "com_config.h"
#include "pid_controller.h"
#include "motor_task.hpp"
#include "Motor.hpp"
#include "arm_task.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>


osThreadId_t Debug_TaskHandle;

uint8_t flag = 1;
float angle = 0.0f;

extern C610Motor arm2006_motor;

/** @brief 调试任务函数
 * @note 该函数用于测试电机控制和PID调节功能，周期性地更新电机命令以验证系统响应。实际使用中可以根据需要修改测试内容或删除该任务。
 *  @param argument 任务参数
 */
void debugTask(void *argument) {
  // osThreadExit();

  for (;;) {

    if (flag == 1) {
        arm2006_motor.posWithSpeedControl(angle, 12.0f, 2.0f, 2.0f, 0.0f, 0.0f);
        flag = 0;
    }

    osDelay(1);
  }
}
