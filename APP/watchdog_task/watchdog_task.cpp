/**
 * @file watchdog_task.cpp
 * @author Keten (2863861004@qq.com)
 * @brief 看门狗任务，确保这个任务不要被饿死就行了
 * @version 0.1
 * @date 2026-06-08
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */

#include "pid_controller.h"
#include "watchdog_task.h"
#include "com_config.h"
#include "portmacro.h"
#include "task.h"

#include "memory_map.h"

#include "Canbus.hpp"
#include "Motor.hpp"
#include "tail_claw_task.hpp"
#include "motor_task.hpp"
#include "chassis_task.h"
#include <cstddef>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <sys/types.h>
#include "SoftwareWatchdog.hpp"

osThreadId_t Watchdog_TaskHandle;

// 看门狗manager
WatchdogManager<16> wd_mgr;

// 电机管理系统
extern MotorPlanningSystem motor_planning_system;
// 其他电机
extern C620Motor chassis_motor1;
extern C620Motor chassis_motor2;
extern C620Motor chassis_motor3;
extern C620Motor chassis_motor4;
extern C620Motor tail_claw_roll_motor;
extern C610Motor lift_2006_motor1;
extern C610Motor lift_2006_motor2;
extern C620Motor lift_3508_motor1;
extern C620Motor lift_3508_motor2;
// 其他pid对象位置
extern Omni45Chassis chassis_solver;
extern PID_t lift_3508_pos_pid;
extern PID_t lift_3508_motor1_pid;
extern PID_t lift_3508_motor2_pid;
extern PID_t lift_2006_motor1_pid;
extern PID_t lift_2006_motor2_pid;
extern PID_t lift_3508_sync_pid;
extern PID_t high_yaw_lock_pid;


void clearPidOutput(void *ctx) {
    auto *pid = static_cast<PID_t *>(ctx);
    // 清空pid输出值
    pid->Iout = 0;
    pid->ITerm = 0;
    pid->Output = 0;
}

// 注册电机离线行为
void watchdogInit(void) {
    // motor规划系统中，统一注册看门狗
    for (uint8_t i = 0; i < motor_planning_system.get_motor_count(); ++i) {
        MotorPlanningUnit *mpu = motor_planning_system.get_motor_planning_unit(i);
        mpu->motor->offline_wd_.addAction(WatchdogAction{clearPidOutput, &mpu->pos_pid});
        mpu->motor->offline_wd_.addAction(WatchdogAction{clearPidOutput, &mpu->speed_pid});
        wd_mgr.registerWd(&mpu->motor->offline_wd_);
    }
    // 手动注册电机离线行为
    chassis_motor1.offline_wd_.addAction(WatchdogAction{clearPidOutput, &chassis_solver.pos_pid_[0]});
    chassis_motor1.offline_wd_.addAction(WatchdogAction{clearPidOutput, &chassis_solver.speed_pid_[0]});
    wd_mgr.registerWd(&chassis_motor1.offline_wd_);
    chassis_motor2.offline_wd_.addAction(WatchdogAction{clearPidOutput, &chassis_solver.pos_pid_[1]});
    chassis_motor2.offline_wd_.addAction(WatchdogAction{clearPidOutput, &chassis_solver.speed_pid_[1]});
    wd_mgr.registerWd(&chassis_motor2.offline_wd_);
    chassis_motor3.offline_wd_.addAction(WatchdogAction{clearPidOutput, &chassis_solver.pos_pid_[2]});
    chassis_motor3.offline_wd_.addAction(WatchdogAction{clearPidOutput, &chassis_solver.speed_pid_[2]});
    wd_mgr.registerWd(&chassis_motor3.offline_wd_);
    chassis_motor4.offline_wd_.addAction(WatchdogAction{clearPidOutput, &chassis_solver.pos_pid_[3]});
    chassis_motor4.offline_wd_.addAction(WatchdogAction{clearPidOutput, &chassis_solver.speed_pid_[3]});
    wd_mgr.registerWd(&chassis_motor4.offline_wd_);
    tail_claw_roll_motor.offline_wd_.addAction(WatchdogAction{clearPidOutput, &TailClawController::Instance().roll_pos_pid_});
    tail_claw_roll_motor.offline_wd_.addAction(WatchdogAction{clearPidOutput, &TailClawController::Instance().roll_speed_pid_});
    wd_mgr.registerWd(&tail_claw_roll_motor.offline_wd_);
    lift_2006_motor1.offline_wd_.addAction(WatchdogAction{clearPidOutput, &lift_2006_motor1_pid});
    lift_2006_motor1.offline_wd_.addAction(WatchdogAction{clearPidOutput, &high_yaw_lock_pid});
    wd_mgr.registerWd(&lift_2006_motor1.offline_wd_);
    lift_2006_motor2.offline_wd_.addAction(WatchdogAction{clearPidOutput, &lift_2006_motor2_pid});
    lift_2006_motor2.offline_wd_.addAction(WatchdogAction{clearPidOutput, &high_yaw_lock_pid});
    wd_mgr.registerWd(&lift_2006_motor2.offline_wd_);
    lift_3508_motor1.offline_wd_.addAction(WatchdogAction{clearPidOutput, &lift_3508_motor1_pid});
    lift_3508_motor1.offline_wd_.addAction(WatchdogAction{clearPidOutput, &high_yaw_lock_pid});
    lift_3508_motor1.offline_wd_.addAction(WatchdogAction{clearPidOutput, &lift_3508_pos_pid});
    lift_3508_motor1.offline_wd_.addAction(WatchdogAction{clearPidOutput, &lift_3508_sync_pid});
    wd_mgr.registerWd(&lift_3508_motor1.offline_wd_);
    lift_3508_motor2.offline_wd_.addAction(WatchdogAction{clearPidOutput, &lift_3508_motor2_pid});
    lift_3508_motor2.offline_wd_.addAction(WatchdogAction{clearPidOutput, &high_yaw_lock_pid});
    lift_3508_motor2.offline_wd_.addAction(WatchdogAction{clearPidOutput, &lift_3508_pos_pid});
    lift_3508_motor2.offline_wd_.addAction(WatchdogAction{clearPidOutput, &lift_3508_sync_pid});
    wd_mgr.registerWd(&lift_3508_motor2.offline_wd_);
}

void watchdogTask(void *argument) {
    watchdogInit();

    for (;;) {
        wd_mgr.poll(); // 统一检查所有看门狗
        osDelay(5);
    }
}
