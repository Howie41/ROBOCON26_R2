/**
 * @file state_machine_task.h
 * @author zhy (Howie41)
 * @brief 状态机任务头文件
 * @date 2026-05-24
 */

#pragma once

#include "cmsis_os2.h"

extern osThreadId_t StateMachineTaskHandle;

void stateMachineTask(void *argument);

/** ========== 比赛类型 ========== */

#define MATCH_CWTY

/** ============================= */

#ifdef __cplusplus

enum class RobotState : uint8_t {
#ifdef MATCH_CWTY
    begin = 0,
    go_to_SHR,
    go_to_stair_front,
    aim_at_weapon,
    catch_weapon,
    rotate_weapon_claw,
    wait_for_cmd,
    go_to_MF,
    test_stair_up,
    test_stair_down,
    turn_left_90,
    turn_right_90,
    go_to_R2_EXIT,
    stop

#elif MATCH_JGCB
    begin = 0,
#endif
};

void change_state_to(RobotState new_state);
RobotState get_current_state();
bool state_machine_idle();

#endif // __cplusplus

#if !defined(MATCH_CWTY) && !defined(MATCH_JGCB)
#error "未设置比赛类型"
#endif

#if defined(MATCH_CWTY) && defined(MATCH_JGCB)
#error "比赛类型配置异常"
#endif
