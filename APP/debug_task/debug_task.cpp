/**
 * @file debug_task.cpp
 * @brief 调试任务，VOFA+ FireWater 输出
 */
#include "debug_task.h"

#include "task.h"
#include "cmsis_os2.h"


osThreadId_t Debug_TaskHandle;


void debugTask(void *argument) {

    for (;;) {


        osDelay(1);
    }
}
