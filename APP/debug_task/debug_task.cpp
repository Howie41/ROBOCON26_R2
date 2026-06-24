/**
 * @file debug_task.cpp
 * @brief 调试任务，VOFA+ FireWater 输出
 */

#include "debug_task.h"

osThreadId_t Debug_TaskHandle;

void debugTask(void *argument) {
    osThreadExit();

    for (;;) {
        osDelay(1);
    }
}
