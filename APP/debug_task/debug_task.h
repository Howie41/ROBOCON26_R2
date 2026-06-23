/**
 * @file debug_task.h
 * @author 大帅将军
 * @brief 调试任务 — VOFA+ Firewater 底盘电机监视
 * @version 0.2
 * @date 2026-06-11
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------include-----------------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <stdint.h>
#include "task.h"

/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void debugTask(void *argument);
/*------------------------------------test------------------------------------*/


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#endif
