#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <stdint.h>

void debugTask(void *argument);

#ifdef __cplusplus
}
#endif
