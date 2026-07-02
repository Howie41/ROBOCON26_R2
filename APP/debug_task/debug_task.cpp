/**
 * @file debug_task.cpp
 * @brief 调试任务，VOFA+ FireWater 输出
 */
#include "debug_task.h"

#include "task.h"
#include "cmsis_os2.h"
#include "logger.hpp"
#include "com_config.h"
#include "topics.hpp"


osThreadId_t Debug_TaskHandle;
extern LoggerQueue logger_queue;

volatile uint8_t debug_r1_cmd{0x00};
TypedTopicPublisher<pub_qr_code_parsed> qr_code_pub{"qr_code_parsed"};

void debugTask(void *argument) {

  for (;;) {
    osDelay(50);
    logger_queue.trySend();
    if (debug_r1_cmd != 0x00) {
      pub_qr_code_parsed msg{.data = debug_r1_cmd};
      qr_code_pub.Publish(msg);
      logger_queue.log("DEBUG r1_cmd %02X\n", debug_r1_cmd);
      debug_r1_cmd = 0x00;
    }
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
