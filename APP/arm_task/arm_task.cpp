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
#include "bsp_dwt.h"


osThreadId_t Arm_TaskHandle;

static TypedTopicSubscriber<pub_arm_cmd> arm_cmd_sub("arm_cmd", 8);
static pub_arm_cmd arm_cmd{};


extern Arm arm;

static uint8_t index = 1;
static float time_rec = 0.0f;


void fetch_step(int8_t step) {
    if (arm.kfs_num_ == 3) return;
    index = 1;
    switch (step) {
        case 1:
            arm.is_fecthing_step_M_ = true;
            break;
        case 2:
            arm.is_fecthing_step_H_ = true;
            break;
        case -1:
            arm.is_fecthing_step_L_ = true;
            break;
    }
}


// void armTask(void *argument) {

//     time_rec = DWT_GetTimeline_s();
    
//     for (;;) {
//         if (arm.is_fecthing_step_H_) {

//         } else if (arm.is_fecthing_step_M_) {

//         } else if (arm.is_fecthing_step_L_) {

//         }
//         osDelay(1);
//     }
// }

uint8_t flag = 1;
void armTask(void *argument) {
    
    arm.reset();

  for (;;) {

    if (arm_cmd_sub.TryGet(&arm_cmd)) {
        if (arm_cmd.update) {
            arm.fetch_proceed(1, flag);
            flag++;
        }
        if (arm_cmd.fetch) {
            flag = 1;
        }
    }

    osDelay(1);
  }
}

