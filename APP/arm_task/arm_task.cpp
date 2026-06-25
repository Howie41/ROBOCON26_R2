/**
 * @file arm_task.cpp
 * @author FunFer
 * @brief 取矿任务实现
 * @version 0.1
 * @date 2026-05-26
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "arm_task.hpp"

#include "Motor.hpp"
#include "com_config.h"
#include "pid_controller.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "logger.hpp"
#include "bsp_dwt.h"


osThreadId_t Arm_TaskHandle;


static TypedTopicSubscriber<pub_arm_cmd> arm_cmd_sub("arm_cmd", 8);
static pub_arm_cmd arm_cmd{};

extern Arm arm;  // 取矿机构实例

static float time_rec = 0.0f;  // 用于记录绝对时间

static float now_t = 0.0f;  // 当前相对时刻记录
static float last_t = 0.0f;  // 上一次相对时刻记录

static uint8_t flag = 0;  // 当前功能状态值
static uint8_t test_flag = 0;  // 上位机进行速度规划调试用时的flag


namespace arm_action {

/**
 * @brief 吸取并放入对应层高的kfs，自动处理存入kfs的数量
 * @note 伸出、吸取、抬起、伸入储存区、释放、恢复默认、kfs_amount+1
 * @param 参数接收: 1, 2, -1的输入
 */
void load_kfs(int8_t step) { fetch_step(step); }

/**
 * @brief 取出kfs，自动识别当前kfs数量，从对应的高度取出(取最外层)
 * @note 抬起、伸入储存区、吸取、抬起、伸出、kfs_amount-1
 * @param -1:自动识别高度，1,2,3:指定高度
 */
void unload_kfs(int8_t level = -1) { place_kfs(level); }

/**
 * @brief 承接unload_kfs，释放kfs并恢复默认动作
 */
void release_kfs() { place_release(); }

}


void fetch_step(int8_t step) { 
    if (arm.get_kfs_amount() == 3) return;
    time_rec = DWT_GetTimeline_s();
    now_t = 0.0f;
    last_t = 0.0f;

    switch (step) {
        case 1:
            arm.set_is_fetching_step_M(true);
            break;
        case 2:
            arm.set_is_fetching_step_H(true);
            break;
        case -1:
            arm.set_is_fetching_step_L(true);
            break;
    }
}

void place_kfs(int8_t kfs_layer = -1) {
    if (arm.get_kfs_amount() == 0) return;
    time_rec = DWT_GetTimeline_s();
    now_t = 0.0f;
    last_t = 0.0f;
    switch (kfs_layer) {
        case -1:
            place_kfs(arm.get_kfs_amount());
            break;
        case 1:
            arm.set_is_placing_kfs_L(true);
            break;
        case 2:
            arm.set_is_placing_kfs_M(true);
            break;
        case 3:
            arm.set_is_placing_kfs_H(true);
            break;
    }
}

void place_release() {
    time_rec = DWT_GetTimeline_s();
    now_t = 0.0f;
    last_t = 0.0f;

    arm.set_is_place_releasing(true);
}

void armTask(void *argument) {

    arm.reset();
    time_rec = DWT_GetTimeline_s();
    
    for (;;) {
        if (arm.get_is_fetching_step_H()) {  // 抓取高台阶
            now_t = DWT_GetTimeline_s() - time_rec;
            if (arm.get_kfs_amount() == 0 || arm.get_kfs_amount() == 1) {  // 如果 储存了 0/1 个 KFS
                if (now_t >= 0.5f && last_t < 0.5f) { arm.fetch_proceed(2, 0 ); }
                else if (now_t >= 1.5f && last_t < 1.5f) { arm.fetch_proceed(2, 1); }
                else if (now_t >= 2.5f && last_t < 2.5f) { arm.fetch_proceed(2, 2); }
                else if (now_t >= 3.5f && last_t < 3.5f) { arm.fetch_proceed(2, 3); }
                else if (now_t >= 4.5f && last_t < 4.5f) { arm.fetch_proceed(2, 4); }
                else if (now_t >= 5.5f && last_t < 5.5f) { arm.fetch_proceed(2, 5); }
                else if (now_t >= 6.5f && last_t < 6.5f) { arm.fetch_proceed(2, 6); }
                else if (now_t >= 7.5f && last_t < 7.5f) { arm.fetch_proceed(2, 7); }
                else if (now_t >= 8.5f && last_t < 8.5f) { arm.fetch_proceed(2, 8); }
                else if (now_t >= 9.5f && last_t < 9.5f) { arm.fetch_proceed(2, 9); }
                else if (now_t >= 10.5f && last_t < 10.5f) { arm.fetch_proceed(2, 10); }
                else if (now_t >= 11.5f && last_t < 11.5f) { arm.fetch_proceed(2, 11); }
                else if (now_t >= 12.5f && last_t < 12.5f) { arm.set_is_fetching_step_H(false); arm.addKFS(); }
            } else if (arm.get_kfs_amount() == 2) {  // 如果 储存了 2 个 KFS
                if (now_t >= 0.5f && last_t < 0.5f) { arm.fetch_proceed(2, 0 ); }
                else if (now_t >= 1.5f && last_t < 1.5f) { arm.fetch_proceed(2, 1); }
                else if (now_t >= 2.5f && last_t < 2.5f) { arm.fetch_proceed(2,2); }
                else if (now_t >= 3.5f && last_t < 3.5f) { arm.fetch_proceed(2, 3); }
                else if (now_t >= 4.5f && last_t < 4.5f) { arm.fetch_proceed(2, 4); }
                else if (now_t >= 5.5f && last_t < 5.5f) { arm.fetch_proceed(2, 5); }
                else if (now_t >= 6.5f && last_t < 6.5f) { arm.set_is_fetching_step_H(false); arm.addKFS(); }
            }
            last_t = now_t;
        } else if (arm.get_is_fetching_step_M()) {  // 抓取中台阶
            now_t = DWT_GetTimeline_s() - time_rec;
            if (arm.get_kfs_amount() == 0 || arm.get_kfs_amount() == 1) {  // 如果 储存了 0/1 个 KFS
                if (now_t >= 0.5f && last_t < 0.5f) { arm.fetch_proceed(1, 0 ); }
                else if (now_t >= 1.5f && last_t < 1.5f) { arm.fetch_proceed(1, 1); }
                else if (now_t >= 2.5f && last_t < 2.5f) { arm.fetch_proceed(1, 2); }
                else if (now_t >= 3.5f && last_t < 3.5f) { arm.fetch_proceed(1, 3); }
                else if (now_t >= 4.5f && last_t < 4.5f) { arm.fetch_proceed(1, 4); }
                else if (now_t >= 5.5f && last_t < 5.5f) { arm.fetch_proceed(1, 5); }
                else if (now_t >= 6.5f && last_t < 6.5f) { arm.fetch_proceed(1, 6); }
                else if (now_t >= 7.5f && last_t < 7.5f) { arm.fetch_proceed(1, 7); }
                else if (now_t >= 8.5f && last_t < 8.5f) { arm.fetch_proceed(1, 8); }
                else if (now_t >= 9.5f && last_t < 9.5f) { arm.fetch_proceed(1, 9); }
                else if (now_t >= 10.5f && last_t < 10.5f) { arm.fetch_proceed(1, 10); }
                else if (now_t >= 11.5f && last_t < 11.5f) { arm.fetch_proceed(1, 11); }
                else if (now_t >= 12.5f && last_t < 12.5f) { arm.set_is_fetching_step_M(false); arm.addKFS(); }
            } else if (arm.get_kfs_amount() == 2) {  // 如果 储存了 2 个 KFS
                if (now_t >= 0.5f && last_t < 0.5f) { arm.fetch_proceed(1, 0 ); }
                else if (now_t >= 1.5f && last_t < 1.5f) { arm.fetch_proceed(1, 1); }
                else if (now_t >= 2.5f && last_t < 2.5f) { arm.fetch_proceed(1, 2); }
                else if (now_t >= 3.5f && last_t < 3.5f) { arm.fetch_proceed(1, 3); }
                else if (now_t >= 4.5f && last_t < 4.5f) { arm.fetch_proceed(1, 4); }
                else if (now_t >= 5.5f && last_t < 5.5f) { arm.fetch_proceed(1, 5); }
                else if (now_t >= 6.5f && last_t < 6.5f) { arm.set_is_fetching_step_M(false); arm.addKFS(); }
            }
            last_t = now_t;
        } else if (arm.get_is_fetching_step_L()) {  // 抓取低台阶
            now_t = DWT_GetTimeline_s() - time_rec;
            if (arm.get_kfs_amount() == 0 || arm.get_kfs_amount() == 1) {  // 如果 储存了 0/1 个 KFS
                if (now_t >= 0.5f && last_t < 0.5f) { arm.fetch_proceed(-1, 0 ); }
                else if (now_t >= 2.5f && last_t < 2.5f) { arm.fetch_proceed(-1, 1); }
                else if (now_t >= 3.5f && last_t < 3.5f) { arm.fetch_proceed(-1, 2); }
                else if (now_t >= 4.5f && last_t < 4.5f) { arm.fetch_proceed(-1, 3); }
                else if (now_t >= 5.5f && last_t < 5.5f) { arm.fetch_proceed(-1, 4); }
                else if (now_t >= 6.5f && last_t < 6.5f) { arm.fetch_proceed(-1, 5); }
                else if (now_t >= 7.5f && last_t < 7.5f) { arm.fetch_proceed(-1, 6); }
                else if (now_t >= 8.5f && last_t < 8.5f) { arm.fetch_proceed(-1, 7); }
                else if (now_t >= 9.5f && last_t < 9.5f) { arm.fetch_proceed(-1,8); }
                else if (now_t >= 10.5f && last_t < 10.5f) { arm.fetch_proceed(-1,9); }
                else if (now_t >= 11.5f && last_t < 11.5f) { arm.fetch_proceed(-1, 10); }
                else if (now_t >= 12.5f && last_t < 12.5f) { arm.fetch_proceed(-1, 11); }
                else if (now_t >= 13.5f && last_t < 13.5f) { arm.set_is_fetching_step_L(false); arm.addKFS(); }
            } else if (arm.get_kfs_amount() == 2) {  // 如果 储存了 2 个 KFS
                if (now_t >= 0.5f && last_t < 0.5f) { arm.fetch_proceed(-1, 0 ); }
                else if (now_t >= 2.5f && last_t < 2.5f) { arm.fetch_proceed(-1, 1); }
                else if (now_t >= 3.5f && last_t < 3.5f) { arm.fetch_proceed(-1, 2); }
                else if (now_t >= 4.5f && last_t < 4.5f) { arm.fetch_proceed(-1, 3); }
                else if (now_t >= 5.5f && last_t < 5.5f) { arm.fetch_proceed(-1, 4); }
                else if (now_t >= 6.5f && last_t < 6.5f) { arm.fetch_proceed(-1, 5); }
                else if (now_t >= 7.5f && last_t < 7.5f) { arm.set_is_fetching_step_L(false); arm.addKFS(); }
            }
            last_t = now_t;
        }
        
        else if (arm.get_is_placing_kfs_L()) {  // 放置最底下的KFS进第二层
            now_t = DWT_GetTimeline_s() - time_rec;
            if (now_t >= 0.5f && last_t < 0.5f) { arm.place_proceed(0); }
            if (now_t >= 2.0f && last_t < 2.0f) { arm.place_proceed(1); }
            if (now_t >= 3.5f && last_t < 3.5f) { arm.place_proceed(2); }
            if (now_t >= 6.5f && last_t < 6.5f) { arm.place_proceed(3); }
            if (now_t >= 8.5f && last_t < 8.5f) { arm.place_proceed(4); }
            if (now_t >= 10.5f && last_t < 10.5f) { arm.place_proceed(5); }
            if (now_t >= 12.5f && last_t < 12.5f) { arm.place_proceed(6); }
            else if (now_t >= 14.5f && last_t < 14.5f) { arm.set_is_placing_kfs_L(false); arm.rmvKFS(); }
            last_t = now_t;
        } else if (arm.get_is_placing_kfs_M()) {    // 放置中间层的KFS进第二层
            now_t = DWT_GetTimeline_s() - time_rec;
            if (now_t >= 0.5f && last_t < 0.5f) { arm.place_proceed(0); }
            if (now_t >= 2.5f && last_t < 2.5f) { arm.place_proceed(1); }
            if (now_t >= 4.5f && last_t < 4.5f) { arm.place_proceed(2); }
            if (now_t >= 6.5f && last_t < 6.5f) { arm.place_proceed(3); }
            if (now_t >= 8.5f && last_t < 8.5f) { arm.place_proceed(4); }
            if (now_t >= 10.5f && last_t < 10.5f) { arm.place_proceed(5); }
            if (now_t >= 12.5f && last_t < 12.5f) { arm.place_proceed(6); }
            else if (now_t >= 16.5f && last_t < 16.5f) { arm.set_is_placing_kfs_M(false); arm.rmvKFS(); }
            last_t = now_t;
        } else if (arm.get_is_placing_kfs_H()) {  // 放置最上面的KFS进第二层
            now_t = DWT_GetTimeline_s() - time_rec;
            if (now_t >= 0.5f && last_t < 0.5f) { arm.place_proceed(0); }
            if (now_t >= 1.5f && last_t < 1.5f) { arm.place_proceed(1); }
            if (now_t >= 2.5f && last_t < 2.5f) { arm.place_proceed(2); }
            if (now_t >= 3.5f && last_t < 3.5f) { arm.place_proceed(3); }
            else if (now_t >= 4.5f && last_t < 4.5f) { arm.set_is_placing_kfs_H(false); arm.rmvKFS(); }
            last_t = now_t;
        }
        
        else if (arm.get_is_place_releasing()) {  // 释放KFS，回归默认姿态
            now_t = DWT_GetTimeline_s() - time_rec;
            if (now_t >= 0.5f && last_t < 0.5f) { arm.place_release_proceed(0); }
            if (now_t >= 1.5f && last_t < 1.5f) { arm.place_release_proceed(1); }
            if (now_t >= 2.5f && last_t < 2.5f) { arm.place_release_proceed(2); }
            if (now_t >= 3.5f && last_t < 3.5f) { arm.place_release_proceed(3); }
            else if (now_t >= 4.5f && last_t < 4.5f) { arm.set_is_place_releasing(false); }
            last_t = now_t;
        }

        if (arm_cmd_sub.TryGet(&arm_cmd) || test_flag) {
            if (arm_cmd.update) {
                flag++;
            }
            if (arm_cmd.fetch || test_flag) {
                switch (flag) {
                    case 1:
                        fetch_step(1);
                        break;
                    case 2:
                        fetch_step(2);
                        break;
                    case 3:
                        fetch_step(-1);
                        break;
                    case 4:
                        place_kfs();
                        break;
                    case 5:
                        place_release();
                        break;
                }
                flag = 0;
                test_flag = 0;
            }
        }

        osDelay(1);
    }
}

