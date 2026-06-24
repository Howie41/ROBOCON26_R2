/**
 * @file watchdog_task.h
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2026-06-08
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once
#include "FreeRTOS.h"
#include "Watchdog.hpp"
#include "cmsis_os.h"
#include "task.h"
#include "watchdog_task.h"

void watchdogInit(void);

void watchdogTask(void *argument);