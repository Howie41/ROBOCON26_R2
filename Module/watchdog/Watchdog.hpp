/**
 * @file Watchdog.hpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * 看门狗总头文件，直接引用这个，在这里可以添加看门狗的时钟源，暂时使用dwt
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
#include "SoftwareWatchdog.hpp"
#include "WatchdogActions.hpp"
#include "WatchdogManager.hpp"
#include "bsp_dwt.h"
#include "system_stm32h7xx.h"
#include <cstdint>

// 系统时钟
#define SYSTEMCORECLOCK 275000000

struct DWTMsSource {
  static uint32_t now() { return DWT->CYCCNT / (SYSTEMCORECLOCK / 1000U); }
};

constexpr uint32_t ms_to_cycles(uint32_t ms) {
  return ms * (SYSTEMCORECLOCK / 1000U);
}