/**
 * @file arm_task.hpp
 * @author FunFer
 * @brief 取矿机构任务
 * @version 0.1
 * @date 2026-05-26
 *
 * @copyright Copyright (c) 2026
 * @note :
 * @versioninfo :
 */

#pragma once

#include "Motor.hpp"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_gpio.h"
#include <cmath>
#include <stdint.h>
#include <stdio.h>


typedef enum {
    
} ArmPose;

class Arm {

public:
    Arm(DM43xxMotor &arm_lift, MotorBase &arm_rotate, MotorBase &arm_expand, DM43xxMotor &arm_flip) : arm_lift_(arm_lift), arm_rotate_(arm_rotate), arm_expand_(arm_expand), arm_flip_(arm_flip) {}
    ~Arm() {}

    Arm& fetch() {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);
        return *this;
    }
    Arm& release() {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);
        return *this;
    }

    Arm& setHeight(float height_rate) {
        arm_lift_.posWithSpeedControl(height_rate * 1080.0f, 1000.0f);
        return *this;
    }

    Arm& setRotate(float rotate_angle) {
        arm_rotate_.posWithSpeedControl(rotate_angle, 10.0f, 30.0f, 0.0f, 0.0f);
        return *this;
    }

    Arm& setExpand(float expand_rate) {
        arm_expand_.posWithSpeedControl(expand_rate * 1080.0f, 8.0f, 0.2f, 0.3f, 0.0f, 0.0f);
        return *this;
    }

    Arm& setFlip(float flip_angle) {
        arm_flip_.posWithSpeedControl(flip_angle, 120.0f);
        return *this;
    }

    bool getIsFinished() {
        return arm_expand_.getIsFinished() && arm_rotate_.getIsFinished();
    }
    
    Arm& reset() {
        arm_lift_.posWithSpeedControl(570.0f, 1000.0f);
        arm_flip_.posWithSpeedControl(0.0f, 120.0f);
        arm_rotate_.posWithSpeedControl(0.0f, 3.0f, 10.0f, 20.0f, 0.0f, 0.0f);
        arm_expand_.posWithSpeedControl(0.0f, 18.0f, 120.0f, 240.0f, 0.0f, 0.0f);
        return *this;
    }
    bool fetch(int8_t step, uint8_t index) {  // 此函数不会增加kfs_num_，需要在外部结束动作链后主动增加kfs_num_
        if (step == 1 || step == 2) {
            if (kfs_num_ == 0 || kfs_num_ == 1) {
                switch (index) {
                    case 1:
                        arm_lift_.posWithSpeedControl(570.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(78.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-15.0f, 2.8f, 15.0f, 30.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(1080.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 2:
                        arm_lift_.posWithSpeedControl(570.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(78.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(24.0f, 2.2f, 5.0f, 10.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(1080.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        fetch();
                        break;
                    case 3:
                        arm_lift_.posWithSpeedControl(900.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(78.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-9.0f, 2.2f, 5.0f, 10.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(480.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 4:
                        arm_lift_.posWithSpeedControl(920.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(0.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-40.0f, 2.5f, 10.0f, 15.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(200.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 5:
                        arm_lift_.posWithSpeedControl(940.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(-87.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-90.0f, 2.5f, 15.0f, 30.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(370.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 6:
                        arm_lift_.posWithSpeedControl(1080.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(-88.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-168.0f, 2.3f, 15.0f, 30.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(524.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 7:
                        release();
                        break;
                    case 8:
                        arm_lift_.posWithSpeedControl(820.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(0.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-90.0f, 2.8f, 15.0f, 30.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(200.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 9:
                        arm_lift_.posWithSpeedControl(660.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(0.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-90.0f, 2.8f, 15.0f, 30.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(1000.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 10:
                        arm_lift_.posWithSpeedControl(570.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(0.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(0.0f, 2.4f, 20.0f, 40.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(800.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 11:
                        reset();
                        break;
                    default:
                        return true;
                }
            } else if (kfs_num_ == 2) {
                switch (index) {
                    case 1:
                        arm_lift_.posWithSpeedControl(570.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(78.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-15.0f, 2.8f, 15.0f, 30.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(1080.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 2:
                        arm_lift_.posWithSpeedControl(570.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(78.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(24.0f, 2.2f, 5.0f, 10.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(1080.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        fetch();
                        break;
                    case 3:
                        arm_lift_.posWithSpeedControl(570.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(78.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-9.0f, 3.9f, 5.0f, 10.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(840.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 4:
                        arm_lift_.posWithSpeedControl(570.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(-80.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-40.0f, 3.9f, 6.0f, 15.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(760.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 5:
                        arm_lift_.posWithSpeedControl(570.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(-80.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-85.0f, 3.9f, 15.0f, 30.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(720.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 6:
                        arm_lift_.posWithSpeedControl(570.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(-80.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-85.0f, 2.5f, 15.0f, 30.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(280.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    default:
                        return true;
                }
            }
        } else if (step == -1) {
            if (kfs_num_ == 0 || kfs_num_ == 1) {
                switch (index) {
                    case 1:
                        arm_lift_.posWithSpeedControl(0.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(60.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(10.0f, 2.8f, 15.0f, 30.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(1080.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 2:
                        arm_lift_.posWithSpeedControl(0.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(60.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(40.0f, 2.2f, 5.0f, 10.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(1080.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        fetch();
                        break;
                    case 3:
                        arm_lift_.posWithSpeedControl(900.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(78.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-9.0f, 2.2f, 5.0f, 10.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(480.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 4:
                        arm_lift_.posWithSpeedControl(920.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(0.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-40.0f, 2.5f, 10.0f, 15.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(200.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 5:
                        arm_lift_.posWithSpeedControl(940.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(-87.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-90.0f, 2.5f, 15.0f, 30.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(370.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 6:
                        arm_lift_.posWithSpeedControl(1080.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(-88.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-168.0f, 2.3f, 15.0f, 30.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(524.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 7:
                        release();
                        break;
                    case 8:
                        arm_lift_.posWithSpeedControl(820.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(0.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-90.0f, 2.8f, 15.0f, 30.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(200.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 9:
                        arm_lift_.posWithSpeedControl(660.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(0.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-90.0f, 2.8f, 15.0f, 30.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(1000.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 10:
                        arm_lift_.posWithSpeedControl(570.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(0.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(0.0f, 2.4f, 20.0f, 40.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(800.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 11:
                        reset();
                        break;
                    default:
                        break;
                }
            } else if (kfs_num_ == 2) {
                switch (index) {
                    case 1:
                        arm_lift_.posWithSpeedControl(0.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(60.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(10.0f, 2.8f, 15.0f, 30.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(1080.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 2:
                        arm_lift_.posWithSpeedControl(0.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(60.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(40.0f, 2.2f, 5.0f, 10.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(1080.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        fetch();
                        break;
                    case 3:
                        arm_lift_.posWithSpeedControl(570.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(78.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-9.0f, 3.9f, 5.0f, 10.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(840.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 4:
                        arm_lift_.posWithSpeedControl(570.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(-80.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-40.0f, 3.9f, 6.0f, 15.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(760.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 5:
                        arm_lift_.posWithSpeedControl(570.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(-80.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-85.0f, 3.9f, 15.0f, 30.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(720.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    case 6:
                        arm_lift_.posWithSpeedControl(570.0f, 1000.0f);
                        arm_flip_.posWithSpeedControl(-80.0f, 120.0f);
                        arm_rotate_.posWithSpeedControl(-85.0f, 2.5f, 15.0f, 30.0f, 0.0f, 0.0f);
                        arm_expand_.posWithSpeedControl(280.0f, 18.0f, 20.0f, 240.0f, 0.0f, 0.0f);
                        break;
                    default:
                        return true;
                }
            }
        }
        return false;
    }
    bool place(uint8 index) {
        if (kfs_num_ == 1) {
            switch (index) {
            }
        } else if (kfs_num_ == 2) {
            switch (index) {
            }
        } else if (kfs_num_ == 3) {
            switch (index) {
            }
        }
        return false;
    }

    DM43xxMotor &arm_lift_;
    MotorBase &arm_rotate_;
    MotorBase &arm_expand_;
    DM43xxMotor &arm_flip_;

    uint8_t kfs_num_{2};
};

void armTask(void *argument);
