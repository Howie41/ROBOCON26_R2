/**
 * @file arm_task.cpp
 * @author FunFer
 * @brief 取矿任务实现
 * @version 0.1
 * @date 2026-05-26
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "arm_task.hpp"

#include "Motor.hpp"
#include "com_config.h"
#include "pid_controller.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "logger.hpp"


osThreadId_t Arm_TaskHandle;

static TypedTopicSubscriber<pub_arm_cmd> arm_cmd_sub("arm_cmd", 8);
static pub_arm_cmd arm_cmd{};


extern Arm arm;

uint8_t flag = 1;


void fetch_step(uint8_t step) {
    if (arm.kfs_num_ == 3) return;
    switch (step) {
        case 1:
        
            break;
    }
}


void armTask(void *argument) {
    
    arm.reset();

  for (;;) {

    if (arm_cmd_sub.TryGet(&arm_cmd)) {
        if (arm_cmd.update) {
            arm.fetch_proceed(-1, flag);
            flag++;
        }
        if (arm_cmd.fetch) {
            flag = 1;
        }
    }

    osDelay(1);
  }
}
