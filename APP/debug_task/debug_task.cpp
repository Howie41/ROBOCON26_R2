/**
 * @file debug_task.cpp
 * @brief Export runtime debug globals for Ozone inspection.
 */
#include "debug_task.h"
#include "task.h"


osThreadId_t Debug_TaskHandle;


uint8_t flag = 1;

void debugTask(void *argument) {

    for (;;) {

        // Test...
        // HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);  // 开泵
        // HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, flag ? GPIO_PIN_SET : GPIO_PIN_RESET);  // 阀置低，放气；阀置高，吸气（低电平：23通；高电平：12通）

        osDelay(1);
    }
}
