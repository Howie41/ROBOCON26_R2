/**
 * @file robot_task.h
 * @author 大帅将军 ，Keten (2863861004@qq.com)
 * @brief 任务管理和任务间通讯
 * @version 0.1
 * @date 2026-04-18
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include "cmsis_os2.h"

void osTaskInit(void);

/// PC 日志消息队列句柄（LoggerQueue::log → PcCom::ProcessTx → USB 上位机）
extern osMessageQueueId_t pc_log_queue_handle;
