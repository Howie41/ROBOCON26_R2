// /**
//  * @file arm_task.cpp
//  * @author FunFer
//  * @brief 取矿任务实现
//  * @version 0.1
//  * @date 2026-05-26
//  *
//  * @copyright Copyright (c) 2026
//  *
//  * @attention :
//  * @note :
//  * @versioninfo :
//  */
// #include "arm_task.hpp"

// #include "Motor.hpp"
// #include "com_config.h"
// #include "pid_controller.h"
// #include "topic_pool.h"
// #include "topics.hpp"
// #include "logger.hpp"
// #include "bsp_dwt.h"


// osThreadId_t Arm_TaskHandle;

// static TypedTopicSubscriber<pub_arm_cmd> arm_cmd_sub("arm_cmd", 8);
// static pub_arm_cmd arm_cmd{};


// extern Arm arm;

// static float time_rec = 0.0f;

// static float now_t = 0.0f;
// static float last_t = 0.0f;

// static uint8_t flag = 0;

// void fetch_step(int8_t step) {
//     if (arm.kfs_num_ == 3) return;
//     time_rec = DWT_GetTimeline_s();
//     now_t = 0.0f;
//     last_t = 0.0f;

//     switch (step) {
//         case 1:
//             arm.is_fecthing_step_M_ = true;
//             break;
//         case 2:
//             arm.is_fecthing_step_H_ = true;
//             break;
//         case -1:
//             arm.is_fecthing_step_L_ = true;
//             break;
//     }
// }

// void place_kfs() {
//     if (arm.kfs_num_ == 0) return;
//     time_rec = DWT_GetTimeline_s();
//     now_t = 0.0f;
//     last_t = 0.0f;

//     switch (arm.kfs_num_) {
//         case 1:
//             arm.is_placing_kfs_L_ = true;
//             break;
//         case 2:
//             arm.is_placing_kfs_M_ = true;
//             break;
//         case 3:
//             arm.is_placing_kfs_H_ = true;
//             break;
//     }
// }

// void place_release() {
//     time_rec = DWT_GetTimeline_s();
//     now_t = 0.0f;
//     last_t = 0.0f;

//     arm.is_place_releasing_ = true;
// }

// void armTask(void *argument) {

//     arm.reset();
//     time_rec = DWT_GetTimeline_s();
    
//     for (;;) {
//         if (arm.is_fecthing_step_H_) {
//             now_t = DWT_GetTimeline_s() - time_rec;
//             if (arm.kfs_num_ == 0 || arm.kfs_num_ == 1) {
//                 if (now_t >= 0.5f && last_t < 0.5f) { arm.fetch_proceed(2, 1 ); }
//                 else if (now_t >= 1.5f && last_t < 1.5f) { arm.fetch_proceed(2, 2); }
//                 else if (now_t >= 2.5f && last_t < 2.5f) { arm.fetch_proceed(2, 3); }
//                 else if (now_t >= 3.5f && last_t < 3.5f) { arm.fetch_proceed(2, 4); }
//                 else if (now_t >= 4.5f && last_t < 4.5f) { arm.fetch_proceed(2, 5); }
//                 else if (now_t >= 5.5f && last_t < 5.5f) { arm.fetch_proceed(2, 6); }
//                 else if (now_t >= 6.5f && last_t < 6.5f) { arm.fetch_proceed(2, 7); }
//                 else if (now_t >= 7.5f && last_t < 7.5f) { arm.fetch_proceed(2, 8); }
//                 else if (now_t >= 8.5f && last_t < 8.5f) { arm.fetch_proceed(2, 9); }
//                 else if (now_t >= 9.5f && last_t < 9.5f) { arm.fetch_proceed(2, 10); }
//                 else if (now_t >= 10.5f && last_t < 10.5f) { arm.fetch_proceed(2, 11); }
//                 else if (now_t >= 11.5f && last_t < 11.5f) { arm.fetch_proceed(2, 12); }
//                 else if (now_t >= 12.5f && last_t < 12.5f) { arm.is_fecthing_step_H_ = false; arm.addKFS(); }
//             } else if (arm.kfs_num_ == 2) {
//                 if (now_t >= 0.5f && last_t < 0.5f) { arm.fetch_proceed(2, 1 ); }
//                 else if (now_t >= 1.5f && last_t < 1.5f) { arm.fetch_proceed(2, 2); }
//                 else if (now_t >= 2.5f && last_t < 2.5f) { arm.fetch_proceed(2, 3); }
//                 else if (now_t >= 3.5f && last_t < 3.5f) { arm.fetch_proceed(2, 4); }
//                 else if (now_t >= 4.5f && last_t < 4.5f) { arm.fetch_proceed(2, 5); }
//                 else if (now_t >= 5.5f && last_t < 5.5f) { arm.fetch_proceed(2, 6); }
//                 else if (now_t >= 6.5f && last_t < 6.5f) { arm.is_fecthing_step_H_ = false; arm.addKFS(); }
//             }
//             last_t = now_t;
//         } else if (arm.is_fecthing_step_M_) {
//             now_t = DWT_GetTimeline_s() - time_rec;
//             if (arm.kfs_num_ == 0 || arm.kfs_num_ == 1) {
//                 if (now_t >= 0.5f && last_t < 0.5f) { arm.fetch_proceed(1, 1 ); }
//                 else if (now_t >= 1.5f && last_t < 1.5f) { arm.fetch_proceed(1, 2); }
//                 else if (now_t >= 2.5f && last_t < 2.5f) { arm.fetch_proceed(1, 3); }
//                 else if (now_t >= 3.5f && last_t < 3.5f) { arm.fetch_proceed(1, 4); }
//                 else if (now_t >= 4.5f && last_t < 4.5f) { arm.fetch_proceed(1, 5); }
//                 else if (now_t >= 5.5f && last_t < 5.5f) { arm.fetch_proceed(1, 6); }
//                 else if (now_t >= 6.5f && last_t < 6.5f) { arm.fetch_proceed(1, 7); }
//                 else if (now_t >= 7.5f && last_t < 7.5f) { arm.fetch_proceed(1, 8); }
//                 else if (now_t >= 8.5f && last_t < 8.5f) { arm.fetch_proceed(1, 9); }
//                 else if (now_t >= 9.5f && last_t < 9.5f) { arm.fetch_proceed(1, 10); }
//                 else if (now_t >= 10.5f && last_t < 10.5f) { arm.fetch_proceed(1, 11); }
//                 else if (now_t >= 11.5f && last_t < 11.5f) { arm.fetch_proceed(1, 12); }
//                 else if (now_t >= 12.5f && last_t < 12.5f) { arm.is_fecthing_step_M_ = false; arm.addKFS(); }
//             } else if (arm.kfs_num_ == 2) {
//                 if (now_t >= 0.5f && last_t < 0.5f) { arm.fetch_proceed(1, 1 ); }
//                 else if (now_t >= 1.5f && last_t < 1.5f) { arm.fetch_proceed(1, 2); }
//                 else if (now_t >= 2.5f && last_t < 2.5f) { arm.fetch_proceed(1, 3); }
//                 else if (now_t >= 3.5f && last_t < 3.5f) { arm.fetch_proceed(1, 4); }
//                 else if (now_t >= 4.5f && last_t < 4.5f) { arm.fetch_proceed(1, 5); }
//                 else if (now_t >= 5.5f && last_t < 5.5f) { arm.fetch_proceed(1, 6); }
//                 else if (now_t >= 6.5f && last_t < 6.5f) { arm.is_fecthing_step_M_ = false; arm.addKFS(); }
//             }
//             last_t = now_t;
//         } else if (arm.is_fecthing_step_L_) {
//             now_t = DWT_GetTimeline_s() - time_rec;
//             if (arm.kfs_num_ == 0 || arm.kfs_num_ == 1) {
//                 if (now_t >= 0.5f && last_t < 0.5f) { arm.fetch_proceed(-1, 1 ); }
//                 else if (now_t >= 2.5f && last_t < 2.5f) { arm.fetch_proceed(-1, 2); }
//                 else if (now_t >= 3.5f && last_t < 3.5f) { arm.fetch_proceed(-1, 3); }
//                 else if (now_t >= 4.5f && last_t < 4.5f) { arm.fetch_proceed(-1, 4); }
//                 else if (now_t >= 5.5f && last_t < 5.5f) { arm.fetch_proceed(-1, 5); }
//                 else if (now_t >= 6.5f && last_t < 6.5f) { arm.fetch_proceed(-1, 6); }
//                 else if (now_t >= 7.5f && last_t < 7.5f) { arm.fetch_proceed(-1, 7); }
//                 else if (now_t >= 8.5f && last_t < 8.5f) { arm.fetch_proceed(-1, 8); }
//                 else if (now_t >= 9.5f && last_t < 9.5f) { arm.fetch_proceed(-1, 9); }
//                 else if (now_t >= 10.5f && last_t < 10.5f) { arm.fetch_proceed(-1, 10); }
//                 else if (now_t >= 11.5f && last_t < 11.5f) { arm.fetch_proceed(-1, 11); }
//                 else if (now_t >= 12.5f && last_t < 12.5f) { arm.fetch_proceed(-1, 12); }
//                 else if (now_t >= 13.5f && last_t < 13.5f) { arm.is_fecthing_step_L_ = false; arm.addKFS(); }
//             } else if (arm.kfs_num_ == 2) {
//                 if (now_t >= 0.5f && last_t < 0.5f) { arm.fetch_proceed(-1, 1 ); }
//                 else if (now_t >= 2.5f && last_t < 2.5f) { arm.fetch_proceed(-1, 2); }
//                 else if (now_t >= 3.5f && last_t < 3.5f) { arm.fetch_proceed(-1, 3); }
//                 else if (now_t >= 4.5f && last_t < 4.5f) { arm.fetch_proceed(-1, 4); }
//                 else if (now_t >= 5.5f && last_t < 5.5f) { arm.fetch_proceed(-1, 5); }
//                 else if (now_t >= 6.5f && last_t < 6.5f) { arm.fetch_proceed(-1, 6); }
//                 else if (now_t >= 7.5f && last_t < 7.5f) { arm.is_fecthing_step_L_ = false; arm.addKFS(); }
//             }
//             last_t = now_t;
//         } else if (arm.is_placing_kfs_L_) {
//             now_t = DWT_GetTimeline_s() - time_rec;
//             if (now_t >= 0.5f && last_t < 0.5f) { arm.place_proceed(1); }
//             if (now_t >= 2.0f && last_t < 2.0f) { arm.place_proceed(2); }
//             if (now_t >= 3.5f && last_t < 3.5f) { arm.place_proceed(3); }
//             if (now_t >= 6.5f && last_t < 6.5f) { arm.place_proceed(4); }
//             if (now_t >= 8.5f && last_t < 8.5f) { arm.place_proceed(5); }
//             if (now_t >= 10.5f && last_t < 10.5f) { arm.place_proceed(6); }
//             if (now_t >= 12.5f && last_t < 12.5f) { arm.place_proceed(7); }
//             else if (now_t >= 14.5f && last_t < 14.5f) { arm.is_placing_kfs_L_ = false; arm.rmvKFS(); }
//             last_t = now_t;
//         } else if (arm.is_placing_kfs_M_) {
//             now_t = DWT_GetTimeline_s() - time_rec;
//             if (now_t >= 0.5f && last_t < 0.5f) { arm.place_proceed(1); }
//             if (now_t >= 2.5f && last_t < 2.5f) { arm.place_proceed(2); }
//             if (now_t >= 4.5f && last_t < 4.5f) { arm.place_proceed(3); }
//             if (now_t >= 6.5f && last_t < 6.5f) { arm.place_proceed(4); }
//             if (now_t >= 8.5f && last_t < 8.5f) { arm.place_proceed(5); }
//             if (now_t >= 10.5f && last_t < 10.5f) { arm.place_proceed(6); }
//             if (now_t >= 12.5f && last_t < 12.5f) { arm.place_proceed(7); }
//             else if (now_t >= 16.5f && last_t < 16.5f) { arm.is_placing_kfs_M_ = false; arm.rmvKFS(); }
//             last_t = now_t;
//         } else if (arm.is_placing_kfs_H_) {
//             now_t = DWT_GetTimeline_s() - time_rec;
//             if (now_t >= 0.5f && last_t < 0.5f) { arm.place_proceed(1); }
//             if (now_t >= 1.5f && last_t < 1.5f) { arm.place_proceed(2); }
//             if (now_t >= 2.5f && last_t < 2.5f) { arm.place_proceed(3); }
//             if (now_t >= 3.5f && last_t < 3.5f) { arm.place_proceed(4); }
//             else if (now_t >= 4.5f && last_t < 4.5f) { arm.is_placing_kfs_H_ = false; arm.rmvKFS(); }
//             last_t = now_t;
//         } else if (arm.is_place_releasing_) {
//             now_t = DWT_GetTimeline_s() - time_rec;
//             if (now_t >= 0.5f && last_t < 0.5f) { arm.place_release_proceed(1); }
//             if (now_t >= 1.5f && last_t < 1.5f) { arm.place_release_proceed(2); }
//             if (now_t >= 2.5f && last_t < 2.5f) { arm.place_release_proceed(3); }
//             if (now_t >= 3.5f && last_t < 3.5f) { arm.place_release_proceed(4); }
//             else if (now_t >= 4.5f && last_t < 4.5f) { arm.is_place_releasing_ = false; }
//             last_t = now_t;
//         }

//         if (arm_cmd_sub.TryGet(&arm_cmd)) {
//             if (arm_cmd.update) {
//                 flag++;
//             }
//             if (arm_cmd.fetch) {
//                 switch (flag) {
//                     case 1:
//                         fetch_step(1);
//                         break;
//                     case 2:
//                         fetch_step(2);
//                         break;
//                     case 3:
//                         fetch_step(-1);
//                         break;
//                     case 4:
//                         place_kfs();
//                         break;
//                     case 5:
//                         place_release();
//                         break;
//                 }
//                 flag = 0;
//             }
//         }

//         osDelay(1);
//     }
// }

// // uint8_t flag = 1;
// // void armTask(void *argument) {
    
// //     arm.reset();

// //   for (;;) {

// //     if (arm_cmd_sub.TryGet(&arm_cmd)) {
// //         if (arm_cmd.update) {
// //             arm.fetch_proceed(1, flag);
// //             flag++;
// //         }
// //         if (arm_cmd.fetch) {
// //             flag = 1;
// //         }
// //     }

// //     osDelay(1);
// //   }
// // }

