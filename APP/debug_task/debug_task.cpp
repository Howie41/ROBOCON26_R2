/**
 * @file debug_task.cpp
 * @brief 调试任务，VOFA+ FireWater 输出
 */
#include "debug_task.h"

#include "task.h"
#include "cmsis_os2.h"
#include "logger.hpp"
#include "com_config.h"


osThreadId_t Debug_TaskHandle;
extern LoggerQueue logger_queue;


void debugTask(void *argument) {

  for (;;) {
    osDelay(50);
    logger_queue.trySend();
    // logger.log(
    //     "%.1f|%.0f|%.1f|%.1f/"
    //     "%.1f|%.0f|%.1f|%.1f/"
    //     "%.1f|%.0f|%.1f|%.1f/"
    //     "%.1f|%.0f|%.1f|%.1f/"
    //     "\n",
    //     chassis_motor1.getCurrentSpeed() / RPM_2_RAD_PER_SEC,
    //     chassis_motor1.getRawCurrentTorque(),
    //     chassis_motor1.getCurrentTemperature(),
    //     chassis_motor1.cmd_,

    //     chassis_motor2.getCurrentSpeed() / RPM_2_RAD_PER_SEC,
    //     chassis_motor2.getRawCurrentTorque(),
    //     chassis_motor2.getCurrentTemperature(),
    //     chassis_motor2.cmd_,

    //     chassis_motor3.getCurrentSpeed() / RPM_2_RAD_PER_SEC,
    //     chassis_motor3.getRawCurrentTorque(),
    //     chassis_motor3.getCurrentTemperature(),
    //     chassis_motor3.cmd_,

    //     chassis_motor4.getCurrentSpeed() / RPM_2_RAD_PER_SEC,
    //     chassis_motor4.getRawCurrentTorque(),
    //     chassis_motor4.getCurrentTemperature(),
    //     chassis_motor4.cmd_
    // );
  }
}
