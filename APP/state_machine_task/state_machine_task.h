/**
 * @file state_machine_task.h
 * @author zhy (Howie41)
 * @brief 状态机任务头文件
 * @date 2026-05-24
 */

#pragma once

#include "cmsis_os2.h"
#include <cstdint>
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
    begin = 0,            // 启动
    go_to_SHR,            // 前往端头架
    aim_at_weapon,        // 夹爪对准对应武器头
    catch_weapon,         // 夹爪夹取武器
    rotate_weapon_claw,   // 夹爪反转
    wait_for_cmd,         // 等待R1指令 决定继续夹取or前往梅林
    test1,
    test2,
    test3,
    // 梅林
    go_to_MF_entrance,     // 前往梅林入口
    request_for_path_cmd,  // 请求路径规划命令
    execute_chassis_action,// 执行底盘动作
    execute_arm_action,    // 执行取矿机构动作
    go_to_MF_exit,         // 前往梅林出口
    stop                   // 停止

#elif MATCH_JGCB
    /** ========== 九宫藏宝 单项赛 ========== */
    
    begin = 0,            // 启动
    // TODO ...
    
#endif
};

enum class PathCmd: uint16_t {
    unknown = 0x0000,               // 无效指令

    request = 0x0301,               // 下位机 -> 上位机: 请求下一个指令
    move_forward = 0x0311,          // 前进
    move_backward = 0x0312,         // 后退
    turn_left_90 = 0x0313,          // 左转90°
    turn_right_90 = 0x0314,         // 右转90°
    move_left = 0x0315,             // 左移
    move_right = 0x0316,            // 右移
    grab_low_r2kfs = 0x0317,        // 抓取低位R2KFS
    grab_mid_r2kfs = 0x0318,        // 抓取中位R2KFS
    grab_high_r2kfs = 0x0319,       // 抓取高位R2KFS
    drop_and_grab_new_kfs = 0x031A, // 抛弃手中R2KFS并抓新的KFS（已有3个方块时触发）
    no_more_commands = 0x031B,      // 已经无命令可获取（已经走出梅林）
};
#endif // __cplusplus

#if !defined(MATCH_CWTY) && !defined(MATCH_JGCB)
#error "未设置比赛类型"
#endif

#if defined(MATCH_CWTY) && defined(MATCH_JGCB)
#error "比赛类型配置异常"
#endif

struct Waypoint {
    int16_t x;
    int16_t y;
    int16_t yaw;
};