/**
 * @file debug_task.cpp
 * @brief 调试任务 — VOFA+ Firewater 实时监视底盘四电机原始数据
 * @note  数据格式: 每个电机4个通道(转速/电流/温度/指令), 共16通道, CSV换行
 *         VOFA+ 协议: Firewater (CSV帧尾\n)
 */
#include "debug_task.h"
<<<<<<< HEAD
#include "Motor.hpp"
#include "cmsis_os2.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_tim.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "gpio.h"
#include "task.h"
=======
#include "task.h"
#include "cmsis_os2.h"
>>>>>>> c6c3c2da201613770fce097c77a66e7e2fb1dc16
#include "logger.hpp"
#include "com_config.h"
#include "Motor.hpp"
<<<<<<< HEAD
#include "arm_task.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>

=======

extern C620Motor chassis_motor1, chassis_motor2, chassis_motor3, chassis_motor4;
extern Logger logger;
extern LoggerQueue logger_queue;
>>>>>>> c6c3c2da201613770fce097c77a66e7e2fb1dc16

osThreadId_t Debug_TaskHandle;

void debugTask(void *argument) {
  (void)argument;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, 20);
    // 异步发送日志
    logger_queue.trySend();
    // VOFA+ Firewater: CSV格式, \n结尾
    // 通道: M1转速(RPM),M1电流(CAN原始值),M1温度(℃),M1指令,
    //       M2转速,M2电流,M2温度,M2指令,
    //       M3转速,M3电流,M3温度,M3指令,
    //       M4转速,M4电流,M4温度,M4指令
    logger.log(
        "%.1f,%.0f,%.1f,%.1f,"
        "%.1f,%.0f,%.1f,%.1f,"
        "%.1f,%.0f,%.1f,%.1f,"
        "%.1f,%.0f,%.1f,%.1f\n",
        // Motor 1 (左前)
        chassis_motor1.getCurrentSpeed() / RPM_2_RAD_PER_SEC,
        chassis_motor1.getRawCurrentTorque(),
        chassis_motor1.getCurrentTemperature(),
        chassis_motor1.cmd_,
        // Motor 2 (右前)
        chassis_motor2.getCurrentSpeed() / RPM_2_RAD_PER_SEC,
        chassis_motor2.getRawCurrentTorque(),
        chassis_motor2.getCurrentTemperature(),
        chassis_motor2.cmd_,
        // Motor 3 (左后)
        chassis_motor3.getCurrentSpeed() / RPM_2_RAD_PER_SEC,
        chassis_motor3.getRawCurrentTorque(),
        chassis_motor3.getCurrentTemperature(),
        chassis_motor3.cmd_,
        // Motor 4 (右后)
        chassis_motor4.getCurrentSpeed() / RPM_2_RAD_PER_SEC,
        chassis_motor4.getRawCurrentTorque(),
        chassis_motor4.getCurrentTemperature(),
        chassis_motor4.cmd_);
  }
}
