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


#define B 66.0f // 预高度补偿

void fetch_step(int8_t step);
void place_kfs(int8_t kfs_layer);
void place_release();

namespace arm_action {
    void load_kfs(int8_t step);
    void unload_kfs(int8_t level);
    void release_kfs();
}

class Arm {

public:
    // 构造函数
    Arm(DM43xxMotor &arm_lift, MotorBase &arm_rotate, MotorBase &arm_expand, DM43xxMotor &arm_flip, uint8_t kfs_num = 0) : arm_lift_(arm_lift), arm_rotate_(arm_rotate), arm_expand_(arm_expand), arm_flip_(arm_flip), kfs_num_(kfs_num) {}
    ~Arm() {}

    // 气泵控制类行为
    void fetch() {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);
    }
    void release() {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);
    }
    void destroy_vaccum_start() {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
    }
    void destroy_vaccum_stop() {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
    }

    // 气泵控制类接口
    void place_release_start() { release(); destroy_vaccum_start(); }
    void place_release_stop() { destroy_vaccum_stop(); }

    // 电机控制类行为基，为电机角度控制提供相对的基准值
    void setHeight(float pos_deg, float speed_deg) { arm_lift_.posWithSpeedControl(B + pos_deg, speed_deg); }
    void setRotate(float pos, float speed, float ini_buffer_pos, float end_buffer_pos) { arm_rotate_.posWithSpeedControl(pos, speed, ini_buffer_pos, end_buffer_pos, 0.0f, 0.0f); }
    void setExpand(float pos, float speed, float ini_buffer_pos, float end_buffer_pos) { arm_expand_.posWithSpeedControl(-pos, speed, ini_buffer_pos, end_buffer_pos, 0.0f, 0.0f); }
    void setFlip(float pos_deg, float speed_deg) { arm_flip_.posWithSpeedControl(-pos_deg, speed_deg); }
    
    // KFS数量控制类接口
    void addKFS() { kfs_num_++; }
    void rmvKFS() { kfs_num_--; }
    uint8_t get_kfs_amount() { return kfs_num_; }
    void set_kfs_amount(uint8_t num) { kfs_num_ = num; }

    // 恢复至默认姿态
    void reset() {
        setHeight(570.0f, 1000.0f);
        setFlip(0.0f, 120.0f);
        setRotate(0.0f, 3.0f, 10.0f, 20.0f);
        setExpand(0.0f, 18.0f, 120.0f, 240.0f);
    }

    // 吸取KFS入储存的具体原子动作序列（包含姿态点位，不包含时间序列）
    bool fetch_proceed(int8_t step, uint8_t index) {  // 此函数不会增加kfs_num_，需要在外部结束动作链后主动增加kfs_num_
        if (step == 1) {
            if (kfs_num_ == 0 || kfs_num_ == 1) {
                switch (index) {
                    case 1:
                        setHeight(570.0f, 1000.0f);
                        setFlip(78.0f, 120.0f);
                        setRotate(-15.0f, 2.7f, 15.0f, 30.0f);
                        setExpand(1080.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 2:
                        setHeight(570.0f, 1000.0f);
                        setFlip(78.0f, 120.0f);
                        setRotate(24.0f, 2.2f, 5.0f, 10.0f);
                        setExpand(1080.0f, 18.0f, 20.0f, 240.0f);
                        fetch();
                        break;
                    case 3:
                        setHeight(900.0f, 1000.0f);
                        setFlip(78.0f, 120.0f);
                        setRotate(-9.0f, 2.2f, 15.0f, 10.0f);
                        setExpand(480.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 4:
                        setHeight(920.0f, 1000.0f);
                        setFlip(0.0f, 120.0f);
                        setRotate(-40.0f, 2.4f, 10.0f, 15.0f);
                        setExpand(200.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 5:
                        setHeight(940.0f, 1000.0f);
                        setFlip(-87.0f, 120.0f);
                        setRotate(-90.0f, 2.4f, 15.0f, 30.0f);
                        setExpand(370.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 6:
                        setHeight(1080.0f, 1000.0f);
                        setFlip(-88.0f, 120.0f);
                        setRotate(-168.0f, 2.3f, 15.0f, 30.0f);
                        setExpand(524.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 7:
                        release();
                        destroy_vaccum_start();
                        break;
                    case 8:
                        destroy_vaccum_stop();
                        break;
                    case 9:
                        setHeight(820.0f, 1000.0f);
                        setFlip(0.0f, 120.0f);
                        setRotate(-90.0f, 2.7f, 15.0f, 30.0f);
                        setExpand(200.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 10:
                        setHeight(660.0f, 1000.0f);
                        setFlip(0.0f, 120.0f);
                        setRotate(-90.0f, 2.7f, 15.0f, 30.0f);
                        setExpand(1000.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 11:
                        setHeight(570.0f, 1000.0f);
                        setFlip(0.0f, 120.0f);
                        setRotate(0.0f, 2.4f, 20.0f, 40.0f);
                        setExpand(800.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 12:
                        reset();
                        break;
                    default:
                        return true;
                }
            } else if (kfs_num_ == 2) {
                switch (index) {
                    case 1:
                        setHeight(570.0f, 1000.0f);
                        setFlip(78.0f, 120.0f);
                        setRotate(-15.0f, 2.7f, 15.0f, 30.0f);
                        setExpand(1080.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 2:
                        setHeight(570.0f, 1000.0f);
                        setFlip(78.0f, 120.0f);
                        setRotate(24.0f, 2.2f, 5.0f, 10.0f);
                        setExpand(1080.0f, 18.0f, 20.0f, 240.0f);
                        fetch();
                        break;
                    case 3:
                        setHeight(600.0f, 1000.0f);
                        setFlip(78.0f, 120.0f);
                        setRotate(-9.0f, 2.7f, 15.0f, 10.0f);
                        setExpand(880.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 4:
                        setHeight(570.0f, 1000.0f);
                        setFlip(-80.0f, 120.0f);
                        setRotate(-40.0f, 2.6f, 16.0f, 15.0f);
                        setExpand(760.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 5:
                        setHeight(570.0f, 1000.0f);
                        setFlip(-80.0f, 120.0f);
                        setRotate(-85.0f, 2.7f, 15.0f, 30.0f);
                        setExpand(720.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 6:
                        setHeight(570.0f, 1000.0f);
                        setFlip(-80.0f, 120.0f);
                        setRotate(-85.0f, 2.5f, 15.0f, 30.0f);
                        setExpand(280.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    default:
                        return true;
                }
            }
        } else if (step == 2) {
            if (kfs_num_ == 0 || kfs_num_ == 1) {
                switch (index) {
                    case 1:
                        setHeight(630.0f, 1000.0f);
                        setFlip(78.0f, 120.0f);
                        setRotate(-15.0f, 2.7f, 15.0f, 30.0f);
                        setExpand(1080.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 2:
                        setHeight(630.0f, 1000.0f);
                        setFlip(78.0f, 120.0f);
                        setRotate(10.0f, 2.2f, 5.0f, 10.0f);
                        setExpand(1080.0f, 18.0f, 20.0f, 240.0f);
                        fetch();
                        break;
                    case 3:
                        setHeight(900.0f, 1000.0f);
                        setFlip(78.0f, 120.0f);
                        setRotate(-12.0f, 2.2f, 15.0f, 10.0f);
                        setExpand(480.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 4:
                        setHeight(920.0f, 1000.0f);
                        setFlip(0.0f, 120.0f);
                        setRotate(-40.0f, 2.4f, 10.0f, 15.0f);
                        setExpand(200.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 5:
                        setHeight(940.0f, 1000.0f);
                        setFlip(-87.0f, 120.0f);
                        setRotate(-90.0f, 2.4f, 15.0f, 30.0f);
                        setExpand(370.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 6:
                        setHeight(1080.0f, 1000.0f);
                        setFlip(-88.0f, 120.0f);
                        setRotate(-168.0f, 2.3f, 15.0f, 30.0f);
                        setExpand(524.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 7:
                        release();
                        destroy_vaccum_start();
                        break;
                    case 8:
                        destroy_vaccum_stop();
                        break;
                    case 9:
                        setHeight(820.0f, 1000.0f);
                        setFlip(0.0f, 120.0f);
                        setRotate(-90.0f, 2.7f, 15.0f, 30.0f);
                        setExpand(200.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 10:
                        setHeight(660.0f, 1000.0f);
                        setFlip(0.0f, 120.0f);
                        setRotate(-90.0f, 2.7f, 15.0f, 30.0f);
                        setExpand(1000.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 11:
                        setHeight(570.0f, 1000.0f);
                        setFlip(0.0f, 120.0f);
                        setRotate(0.0f, 2.4f, 20.0f, 40.0f);
                        setExpand(800.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 12:
                        reset();
                        break;
                    default:
                        return true;
                }
            } else if (kfs_num_ == 2) {
                switch (index) {
                    case 1:
                        setHeight(630.0f, 1000.0f);
                        setFlip(78.0f, 120.0f);
                        setRotate(-15.0f, 2.7f, 15.0f, 30.0f);
                        setExpand(1080.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 2:
                        setHeight(630.0f, 1000.0f);
                        setFlip(78.0f, 120.0f);
                        setRotate(10.0f, 2.2f, 5.0f, 10.0f);
                        setExpand(1080.0f, 18.0f, 20.0f, 240.0f);
                        fetch();
                        break;
                    case 3:
                        setHeight(630.0f, 1000.0f);
                        setFlip(78.0f, 120.0f);
                        setRotate(-12.0f, 2.7f, 15.0f, 10.0f);
                        setExpand(840.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 4:
                        setHeight(570.0f, 1000.0f);
                        setFlip(-80.0f, 120.0f);
                        setRotate(-40.0f, 2.6f, 16.0f, 15.0f);
                        setExpand(760.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 5:
                        setHeight(570.0f, 1000.0f);
                        setFlip(-80.0f, 120.0f);
                        setRotate(-85.0f, 2.7f, 15.0f, 30.0f);
                        setExpand(720.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 6:
                        setHeight(570.0f, 1000.0f);
                        setFlip(-80.0f, 120.0f);
                        setRotate(-85.0f, 2.5f, 15.0f, 30.0f);
                        setExpand(280.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    default:
                        return true;
                }
            }
        } else if (step == -1) {
            if (kfs_num_ == 0 || kfs_num_ == 1) {
                switch (index) {
                    case 1:
                        setHeight(0.0f, 1000.0f);
                        setFlip(60.0f, 120.0f);
                        setRotate(10.0f, 2.7f, 15.0f, 30.0f);
                        setExpand(1080.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 2:
                        setHeight(0.0f, 1000.0f);
                        setFlip(60.0f, 120.0f);
                        setRotate(40.0f, 2.2f, 5.0f, 10.0f);
                        setExpand(1080.0f, 18.0f, 20.0f, 240.0f);
                        fetch();
                        break;
                    case 3:
                        setHeight(900.0f, 1000.0f);
                        setFlip(78.0f, 120.0f);
                        setRotate(-9.0f, 2.2f, 15.0f, 10.0f);
                        setExpand(480.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 4:
                        setHeight(920.0f, 1000.0f);
                        setFlip(0.0f, 120.0f);
                        setRotate(-40.0f, 2.4f, 10.0f, 15.0f);
                        setExpand(200.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 5:
                        setHeight(940.0f, 1000.0f);
                        setFlip(-87.0f, 120.0f);
                        setRotate(-90.0f, 2.4f, 15.0f, 30.0f);
                        setExpand(370.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 6:
                        setHeight(1080.0f, 1000.0f);
                        setFlip(-88.0f, 120.0f);
                        setRotate(-168.0f, 2.3f, 15.0f, 30.0f);
                        setExpand(524.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 7:
                        release();
                        destroy_vaccum_start();
                        break;
                    case 8:
                        destroy_vaccum_stop();
                        break;
                    case 9:
                        setHeight(820.0f, 1000.0f);
                        setFlip(0.0f, 120.0f);
                        setRotate(-90.0f, 2.7f, 15.0f, 30.0f);
                        setExpand(200.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 10:
                        setHeight(660.0f, 1000.0f);
                        setFlip(0.0f, 120.0f);
                        setRotate(-90.0f, 2.7f, 15.0f, 30.0f);
                        setExpand(1000.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 11:
                        setHeight(570.0f, 1000.0f);
                        setFlip(0.0f, 120.0f);
                        setRotate(0.0f, 2.4f, 20.0f, 40.0f);
                        setExpand(800.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 12:
                        reset();
                        break;
                    default:
                        break;
                }
            } else if (kfs_num_ == 2) {
                switch (index) {
                    case 1:
                        setHeight(0.0f, 1000.0f);
                        setFlip(60.0f, 120.0f);
                        setRotate(10.0f, 2.7f, 15.0f, 30.0f);
                        setExpand(1080.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 2:
                        setHeight(0.0f, 1000.0f);
                        setFlip(60.0f, 120.0f);
                        setRotate(40.0f, 2.1f, 5.0f, 10.0f);
                        setExpand(1080.0f, 18.0f, 20.0f, 240.0f);
                        fetch();
                        break;
                    case 3:
                        setHeight(570.0f, 1000.0f);
                        setFlip(78.0f, 120.0f);
                        setRotate(-9.0f, 2.7f, 15.0f, 10.0f);
                        setExpand(840.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 4:
                        setHeight(570.0f, 1000.0f);
                        setFlip(-80.0f, 120.0f);
                        setRotate(-40.0f, 2.7f, 6.0f, 15.0f);
                        setExpand(760.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 5:
                        setHeight(570.0f, 1000.0f);
                        setFlip(-80.0f, 120.0f);
                        setRotate(-85.0f, 2.7f, 15.0f, 30.0f);
                        setExpand(720.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    case 6:
                        setHeight(570.0f, 1000.0f);
                        setFlip(-80.0f, 120.0f);
                        setRotate(-85.0f, 2.4f, 15.0f, 30.0f);
                        setExpand(280.0f, 18.0f, 20.0f, 240.0f);
                        break;
                    default:
                        return true;
                }
            }
        }
        return false;
    }
    bool place_proceed(uint8_t index) {
        if (kfs_num_ == 1) {
            switch (index) {
                case 1:
                    setHeight(570.0f, 1000.0f);
                    setFlip(-78.0f, 120.0f);
                    setRotate(-180.0f, 2.7f, 20.0f, 60.0f);
                    setExpand(370.0f, 18.0f, 20.0f, 240.0f);
                    break;
                case 2:
                    setHeight(0.0f, 1000.0f);
                    setFlip(-78.0f, 120.0f);
                    setRotate(-180.0f, 2.5f, 15.0f, 30.0f);
                    setExpand(370.0f, 18.0f, 20.0f, 240.0f);
                    fetch();
                    break;
                case 3:
                    setHeight(1080.0f, 1000.0f);
                    setFlip(-78.0f, 120.0f);
                    setRotate(-158.0f, 3.9f, 20.0f, 30.0f);
                    setExpand(355.0f, 18.0f, 20.0f, 240.0f);
                    break;
                case 4:
                    setHeight(1080.0f, 1000.0f);
                    setFlip(-80.0f, 150.0f);
                    setRotate(-80.0f, 3.8f, 2.0f, 30.0f);
                    setExpand(360.0f, 18.0f, 20.0f, 240.0f);
                    break;
                case 5:
                    setHeight(1080.0f, 1000.0f);
                    setFlip(-70.0f, 120.0f);
                    setRotate(-60.0f, 2.5f, 20.0f, 30.0f);
                    setExpand(660.0f, 18.0f, 20.0f, 240.0f);
                    break;
                case 6:
                    setHeight(920.0f, 1000.0f);
                    setFlip(-70.0f, 120.0f);
                    setRotate(-48.0f, 1.7f, 20.0f, 30.0f);
                    setExpand(660.0f, 18.0f, 20.0f, 240.0f);
                    break;
                case 7:
                    setHeight(920.0f, 1000.0f);
                    setFlip(12.0f, 50.0f);
                    setRotate(-25.0f, 1.5f, 20.0f, 30.0f);
                    setExpand(660.0f, 18.0f, 20.0f, 240.0f);
                    break;
                default:
                    return true;
            }
        } else if (kfs_num_ == 2) {
            switch (index) {
                case 1:
                    setHeight(860.0f, 1000.0f);
                    setFlip(-88.0f, 120.0f);
                    setRotate(0.0f, 2.5f, 15.0f, 30.0f);
                    setExpand(880.0f, 18.0f, 20.0f, 240.0f);
                    break;
                case 2:
                    setHeight(850.0f, 1000.0f);
                    setFlip(-88.0f, 120.0f);
                    setRotate(-174.0f, 2.7f, 1.0f, 30.0f);
                    setExpand(860.0f, 18.0f, 20.0f, 240.0f);
                    fetch();
                    break;
                case 3:
                    setHeight(1080.0f, 1000.0f);
                    setFlip(-90.0f, 120.0f);
                    setRotate(-168.0f, 3.9f, 2.0f, 30.0f);
                    setExpand(370.0f, 18.0f, 20.0f, 240.0f);
                    break;
                case 4:
                    setHeight(1020.0f, 1000.0f);
                    setFlip(-40.0f, 150.0f);
                    setRotate(-80.0f, 3.9f, 2.0f, 30.0f);
                    setExpand(360.0f, 18.0f, 20.0f, 240.0f);
                    break;
                case 5:
                    setHeight(1020.0f, 1000.0f);
                    setFlip(-70.0f, 120.0f);
                    setRotate(-60.0f, 2.5f, 20.0f, 30.0f);
                    setExpand(660.0f, 18.0f, 20.0f, 240.0f);
                    break;
                case 6:
                    setHeight(920.0f, 1000.0f);
                    setFlip(-70.0f, 120.0f);
                    setRotate(-48.0f, 1.7f, 20.0f, 30.0f);
                    setExpand(660.0f, 18.0f, 20.0f, 240.0f);
                    break;
                case 7:
                    setHeight(920.0f, 1000.0f);
                    setFlip(12.0f, 50.0f);
                    setRotate(-25.0f, 1.5f, 20.0f, 30.0f);
                    setExpand(660.0f, 18.0f, 20.0f, 240.0f);
                    break;
                default:
                    return true;
            }
        } else if (kfs_num_ == 3) {
            switch (index) {
                case 1:
                    setHeight(1020.0f, 1000.0f);
                    setFlip(-80.0f, 120.0f);
                    setRotate(-80.0f, 2.5f, 15.0f, 30.0f);
                    setExpand(660.0f, 18.0f, 20.0f, 240.0f);
                    break;
                case 2:
                    setHeight(1020.0f, 1000.0f);
                    setFlip(-80.0f, 120.0f);
                    setRotate(-60.0f, 2.4f, 20.0f, 30.0f);
                    setExpand(660.0f, 18.0f, 20.0f, 240.0f);
                    break;
                case 3:
                    setHeight(920.0f, 1000.0f);
                    setFlip(-80.0f, 120.0f);
                    setRotate(-48.0f, 1.7f, 20.0f, 30.0f);
                    setExpand(660.0f, 18.0f, 20.0f, 240.0f);
                    break;
                case 4:
                    setHeight(920.0f, 1000.0f);
                    setFlip(12.0f, 50.0f);
                    setRotate(-25.0f, 1.5f, 20.0f, 30.0f);
                    setExpand(660.0f, 18.0f, 20.0f, 240.0f);
                    break;
                default:
                    return true;
            }
        }
        return false;
    }

    bool place_release_proceed(uint8_t index) {
        switch (index) {
            case 1:
                place_release_start();
                break;
            case 2:
                place_release_stop();
                break;
            case 3:
                setHeight(570.0f, 1000.0f);
                setFlip(0.0f, 120.0f);
                setRotate(0.0f, 2.1f, 20.0f, 30.0f);
                setExpand(900.0f, 18.0f, 20.0f, 240.0f);
                break;
            case 4:
                reset();
                break;
            default:
                return true;
        }
        return false;
    }

    DM43xxMotor &arm_lift_;
    MotorBase &arm_rotate_;
    MotorBase &arm_expand_;
    DM43xxMotor &arm_flip_;

    uint8_t kfs_num_{0};

    bool is_fecthing_step_L_{false};
    bool is_fecthing_step_M_{false};
    bool is_fecthing_step_H_{false};

    bool is_placing_kfs_L_{false};
    bool is_placing_kfs_M_{false};
    bool is_placing_kfs_H_{false};

    bool is_place_releasing_{false};

};

void armTask(void *argument);
