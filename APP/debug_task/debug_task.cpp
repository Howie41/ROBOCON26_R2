/**
 * @file debug_task.cpp
 * @brief Export runtime debug globals for Ozone inspection.
 */
#include "debug_task.h"

#include "cmsis_os2.h"
#include "logger.hpp"
#include "com_config.h"
#include "topics.hpp"

#include "NavProtocol.hpp"
#include "merlin_map/merlin_map.h"
#include "stair_assist.h"
#include "waypoint_navigator.hpp"

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
  }
}
