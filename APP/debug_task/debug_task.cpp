/**
 * @file debug_task.cpp
 * @brief 调试任务 — VOFA+ Firewater 实时监视底盘四电机原始数据
 * @note  数据格式: 每个电机4个通道(转速/电流/温度/指令), 共16通道, CSV换行
 *         VOFA+ 协议: Firewater (CSV帧尾\n)
 */
#include "debug_task.h"
#include "Motor.hpp"
#include "cmsis_os2.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_tim.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "gpio.h"
#include "task.h"
#include "cmsis_os2.h"
#include "logger.hpp"
#include "com_config.h"
#include "Motor.hpp"
#include "arm_task.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>
#include "pid_controller.h"


osThreadId_t Debug_TaskHandle;

extern C610Motor arm2006_motor;

PID_t arm2006_pid_pos = {
    .Kp = 45.0f,
    .Ki = 0.0f,
    .Kd = 2.5f,
    .MaxOut = 60.0f,
    .DeadBand = 0.01f
};

PID_t arm2006_pid_speed = {
    .Kp = 8000.0f,
    .Ki = 600.0f,
    .Kd = 5.2f,
    .MaxOut = 12000.0f,
    .DeadBand = 0.1f
};

float tar_speed = 0.0f;

void debugTask(void *argument) {
    
    for (;;) {

        float output = PID_Calculate(&arm2006_pid_speed, arm2006_motor.getCurrentSpeed(), tar_speed);
        arm2006_motor.setMotorCmd(output);
        
        osDelay(1);
    }
}
