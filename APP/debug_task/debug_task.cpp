/**
 * @file debug_task.cpp
 * @brief Export runtime debug globals for Ozone inspection.
 */
#include "debug_task.h"

#include "PoseEstimator.hpp"
#include "cmsis_os2.h"
#include "logger.hpp"
#include "task.h"
#include "com_config.h"
#include "topics.hpp"


osThreadId_t Debug_TaskHandle;
extern Logger logger;
extern LoggerQueue logger_queue;

volatile uint8_t debug_r1_cmd{0x00};
TypedTopicPublisher<pub_qr_code_parsed> qr_code_pub{"qr_code_parsed"};

void debugTask(void *argument) {
  (void)argument;
  TickType_t last_wake_time = xTaskGetTickCount();
  TickType_t last_position_log_tick = last_wake_time;

  for (;;) {
    const TickType_t now = xTaskGetTickCount();
    if ((now - last_position_log_tick) >= pdMS_TO_TICKS(20U)) {
      last_position_log_tick = now;
      const nav_localization::PoseSnapshot pose =
          nav_localization::snapshot();
      if (pose.valid) {
        logger.log("pos:%.0f,%.0f,%.2f,%.2f\n", pose.radar_x_mm,
                   pose.radar_y_mm, pose.x_mm, pose.y_mm);
      }
    }

    logger_queue.try_send();

    if (debug_r1_cmd != 0x00) {
      pub_qr_code_parsed msg{.data = debug_r1_cmd};
      qr_code_pub.Publish(msg);
      logger_queue.log("DEBUG\tr1_cmd %02X\n", debug_r1_cmd);
      debug_r1_cmd = 0x00;
    }

    vTaskDelayUntil(&last_wake_time, 1U);
  }
}
