/**
 * @file chassis_task.h
 * @author 大帅将军
 * @brief
 * @version 0.1
 * @date 2026-04-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */

#pragma once

#include "FreeRTOS.h"
#include "cmsis_os.h"

#include "task.h"
#include "topics.hpp"
#include "chassis_solution.hpp"

void chassisTask(void *argument);
namespace chassis_action {

void turn_left_90_deg();
void turn_right_90_deg();
void turn_right_180_deg();
void start_climb_upstairs();
void start_climb_R1();
void start_climb_downstairs();
void start_go_to_edge();
void start_return_to_center();
bool is_chassis_idle();

}  // namespace chassis_action
