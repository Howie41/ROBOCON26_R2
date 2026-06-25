/**
 * @file watchdog_task.cpp
 * @author Keten (2863861004@qq.com)
 * @brief 看门狗任务，确保这个任务不要被饿死就行了
 * @version 0.1
 * @date 2026-06-08
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "watchdog_task.h"
#include "Motor.hpp"
#include "com_config.h"
#include "pid_controller.h"
#include <sys/_types.h>

osThreadId_t Watchdog_TaskHandle;

// 看门狗manager
WatchdogManager<8> wd_mgr;

// 想要监测啥
// extern C610Motor arm2006_motor;

void clearPidOutput(void *ctx) {
  auto *pid = static_cast<PID_t *>(ctx);
  // 清空pid输出值
  pid->Iout = 0;
  pid->ITerm = 0;
  pid->Output = 0;
}

// void test_func(void *ctx) {
//     HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);
// }

void watchdogInit(void) {
  // 注册电机离线行为

    // arm2006_motor.offline_wd_.setAction({test_func, NULL});

  // 注册到集中管理器
    // wd_mgr.registerWd(&arm2006_motor.offline_wd_);
}

void watchdogTask(void *argument) {
  TickType_t currentTime = xTaskGetTickCount();
  watchdogInit();

  for (;;) {
    wd_mgr.poll(); // 统一检查所有看门狗
    vTaskDelayUntil(&currentTime, 5);
  }
}