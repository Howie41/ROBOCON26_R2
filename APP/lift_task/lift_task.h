#pragma once

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "pid_controller.h"

extern float lift_2006_speed;
extern float lift_3508_target_pos;
extern float lift_3508_pos_pid_out;
extern bool lift_3508_hold_enable;
extern bool lift_3508_manual_last;
 
extern float lift_3508_motor1_pos;
extern float lift_3508_motor2_pos;
extern float lift_3508_motor1_speed;
extern float lift_3508_motor2_speed;
 
extern float lift_3508_avg_pos;
extern float lift_3508_diff_pos;
 
extern float lift_3508_base_speed;
extern float lift_3508_sync_pid_out;
extern float lift_3508_motor1_ref_speed;
extern float lift_3508_motor2_ref_speed;
 
extern float lift_2006_motor1_pid_out;
extern float lift_2006_motor2_pid_out;
extern float lift_3508_motor1_pid_out;
extern float lift_3508_motor2_pid_out;

constexpr float MAX_LIFT_2006_SPEED = 600.0f;
constexpr float MAX_LIFT_3508_SPEED = 300.0f;
constexpr float MAX_LIFT_3508_SYNC_COMP = 30.0f;

constexpr float LIFT_RISE_SPEED    = 125.0f;   // 自动上升速度 (3508 RPM)
constexpr float LIFT_FALL_SPEED    = 100.0f;   // 自动下降速度 (可以和上升不同)
constexpr float LIFT_POS_TOLERANCE =  2.0f;   // 位置到达判定容差 (度)

constexpr float LIFT_SPEED_RAMP = 10000.0f; // 速度斜坡 (RPM/s), 出力爬升速率

constexpr float LIFT_LOW_POS = -50.0f;
constexpr float LIFT_HIGH_POS = 500.0f;

constexpr float LIFT_2006_MOTOR1_DIR = 1.0f;
constexpr float LIFT_2006_MOTOR2_DIR = -1.0f;
constexpr float LIFT_3508_MOTOR1_DIR = -1.0f;
constexpr float LIFT_3508_MOTOR2_DIR = -1.0f;

extern PID_t lift_3508_pos_pid;
extern PID_t lift_3508_motor1_pid;
extern PID_t lift_3508_motor2_pid;
extern PID_t lift_2006_motor1_pid;
extern PID_t lift_2006_motor2_pid;
extern PID_t lift_3508_sync_pid;
extern PID_t high_yaw_lock_pid;

void liftTask(void *argument);

// Phase 1: 状态机可调用的抬升接口
#ifdef __cplusplus
extern "C" {
#endif

void liftRequestHigh();     // 请求自动升到高位，非阻塞
void liftRequestLow();      // 请求自动降到低位，非阻塞
bool liftAtTarget();        // 3508是否已到达目标位置
float liftCurrentPos();     // 当前3508平均位置(度)
bool liftIsHigh();          // 当前位置是否在高位判定范围内
bool highModeActive();      // Phase 2: 2006是否着地（高位=锁角前进/后退）

#ifdef __cplusplus
}
#endif
