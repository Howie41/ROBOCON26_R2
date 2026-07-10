#pragma once

#include "Motor.hpp"
#include "pid_controller.h"
#include "topic_pool.h"
#include "Hwt101.hpp"

#include <array>
#include <cstddef>
#include <cmath>

class Chassis {
public:
    Chassis(MotorBase* M1, MotorBase* M2, MotorBase* M3, MotorBase* M4) {
        imu_ = new Hwt101IMU();
        motors_ = {M1, M2, M3, M4};
    }

    void set_v(float vx, float vy, float omega) {
        float angle_temp = -imu_.get_yaw() + M_PI * 0.25f;
        float sin_temp = sin(angle_temp);
        float cos_temp = cos(angle_temp);
        float bias_temp = omega * 368.756f;
        float tar_speed1 = (bias_temp - sin_temp * vx + cos_temp * vy) * 0.0125f;
        float tar_speed2 = (bias_temp - cos_temp * vx - sin_temp * vy) * 0.0125f;
        float tar_speed3 = (bias_temp + sin_temp * vx - cos_temp * vy) * 0.0125f;
        float tar_speed4 = (bias_temp + cos_temp * vx + sin_temp * vy) * 0.0125f;
    }

    void update_imu(uint8_t data) { imu_.processByte(data); }

    float get_yaw() { return ium_.get_yaw(); }

private:
    std::array<MotorBase*, 4> motors_;  // 四个轮子（面对电机轴那一侧，正转为逆时针）
    Hwt101IMU imu_;  // 陀螺仪

    std::array<float, 2> cur_v_{0.0f, 0.0f, 0.0f};  // 当前速度与角速度

};


/**

前宽：604 mm
侧宽：439 mm
轮径：80 mm

M2   M1
|----|
|    |
|----|
M3   M4

 */
