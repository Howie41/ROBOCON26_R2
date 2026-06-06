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
// SpearheadRack   SHR 端头架
// StaffRack       SR 长杆架

enum class RobotState: uint8_t {
#ifdef MATCH_CWTY
    /** ========== 崇武探幽 单项赛 ========== */
    // 武馆
    begin = 0,             // 启动
    go_to_SHR,             // 前往端头架
    aim_at_weapon,         // 夹爪对准对应武器头
    catch_weapon,          // 夹爪夹取武器
    rotate_weapon_claw,    // 夹爪反转
    wait_for_cmd,          // 等待R1指令 决定继续夹取or前往梅林
    // 梅林
    go_to_MF_entrance,     // 前往梅林入口
    request_for_path_step, // 请求路径规划步骤
    execute_chassis_action,// 执行底盘动作
    execute_arm_action,    // 执行取矿机构动作
    go_to_MF_exit,         // 前往梅林出口
    // TODO 进入梅林后的状态... 
    go_to_R2_EXIT,         // 前往R2出口
    stop                   // 停止

#elif MATCH_JGCB
    /** ========== 九宫藏宝 单项赛 ========== */
    
    begin = 0,            // 启动
    // TODO ...
    
#endif
};
#endif // __cplusplus

#if !defined(MATCH_CWTY) && !defined(MATCH_JGCB)
#error "未设置比赛类型"
#endif

#if defined(MATCH_CWTY) && defined(MATCH_JGCB)
#error "比赛类型配置异常"
#endif