#pragma once

#include "PCcom.hpp"
#include "stm32h7xx_hal_pcd.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "bsp_dwt.h"
#include "cmsis_os.h"

#define LASER_MEASURE_ENABLE 1

uint8_t comServiceInit();

void can1SendTask(void *argument);
void can2SendTask(void *argument);
void can3SendTask(void *argument);
void uart2RxProcessTask(void *argument);
void uart3RxProcessTask(void *argument);
void laserMeasureTask(void *argument);
void usbCdcProcessTask(void *argument);
void PcComTask(void *argument);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "LaserMeasure/LaserMeasure.hpp"
#include "SK60PlusLaser/SK60PlusLaser.hpp"
#include "infrared_com.hpp"

extern InfraredModule infrared_module;
extern LaserMeasure laser1;
extern LaserMeasure laser2;
extern SK60PlusLaser laser3;

#endif
