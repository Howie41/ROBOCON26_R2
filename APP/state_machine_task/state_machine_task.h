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

namespace path_cmd {
enum class code: uint16_t {
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
    turn_around = 0x031C,           // 直接转180°
}; // ! 记得改 is_path_cmd 的逻辑 !
inline bool is_path_cmd(uint16_t code) {
    return (code >= static_cast<uint16_t>(path_cmd::code::request)) && (code <= static_cast<uint16_t>(path_cmd::code::turn_around));
}
} // namespace path_cmd


#endif // __cplusplus

#if !defined(MATCH_CWTY) && !defined(MATCH_JGCB)
#error "未设置比赛类型"
#endif

#if defined(MATCH_CWTY) && defined(MATCH_JGCB)
#error "比赛类型配置异常"
#endif