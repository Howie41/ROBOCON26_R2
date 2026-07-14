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
/* RTOS层及mcu main接口 */
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "task.h"

/* bsp 层接口头文件 */
#include "bsp_dwt.h"
#include "chassis_task.h"
#include "com_config.h"
#include "control_task.h"
#include "debug_task.h"
#include "state_machine_task.h"
#include "NavProtocol.hpp"
#include "lift_task.h"
#include "tail_claw_task.hpp"
#include "motor_task.hpp"
#include "arm_task.hpp"
#include "watchdog_task.h"

/* module层接口头文件 */

/* Definitions for TaskHand */
extern osThreadId_t CAN1_Send_TaskHandle;
extern osThreadId_t CAN2_Send_TaskHandle;
extern osThreadId_t CAN3_Send_TaskHandle;
extern osThreadId_t uart2ProcessTaskHandle;
extern osThreadId_t uart3ProcessTaskHandle;
extern osThreadId_t laserMeasureTaskHandle;
extern osThreadId_t Debug_TaskHandle;
extern osThreadId_t Motor_TaskHandle;
extern osThreadId_t Arm_TaskHandle;
// extern osThreadId_t ChassisTaskHandle;
extern osThreadId_t ControlTaskHandle;
extern osThreadId_t usbcdcProcessTaskHandle;
extern osThreadId_t tail_claw_TaskHandle;
extern osThreadId_t NavControlTaskHandle;
extern osThreadId_t LiftTaskHandle;
extern osThreadId_t PcComTaskHandle;
extern osThreadId_t Watchdog_TaskHandle;
extern osThreadId_t InfraredProcessTaskHandle;

#include "memory_map.h"
#include "logger.hpp"

// 添加任务分为两步：
// 第一步先按照格式写 DECLARE_STATIC_TASK(任务名, 任务栈大小，任务优先级) 来声明任务的控制块和栈
// 注意任务名不用双引号
// 第二步在 osTaskInit 函数里调用 osThreadNew 来创建任务，照着其他格式写就行了


/**
 * 静态创建任务栈和控制块，分配到 RAM_D1 上
 * @param sym 任务名，直接写，不是字符串，不需要写引号
 * @param stack_bytes 任务栈大小，单位字节
 * @param prio 任务优先级（osPriority_t）
 * @note 要在osTaskInit里调用osThreadNew来创建任务
 */
#define DECLARE_STATIC_TASK(sym, stack_bytes, prio) \
RAM_D1_ATTR static StaticTask_t sym##ControlBlock; \
RAM_D1_ATTR static StackType_t sym##Stack[(stack_bytes) / sizeof(StackType_t)]; \
static const osThreadAttr_t sym##Handle_attributes = { \
    .name = #sym, \
    .cb_mem = &sym##ControlBlock, \
    .cb_size = sizeof(sym##ControlBlock), \
    .stack_mem = sym##Stack, \
    .stack_size = sizeof(sym##Stack), \
    .priority = (osPriority_t)prio, \
};

DECLARE_STATIC_TASK(CAN1_SendTask, 256 * 4, osPriorityNormal);
DECLARE_STATIC_TASK(CAN2_SendTask, 256 * 4, osPriorityNormal);
DECLARE_STATIC_TASK(CAN3_SendTask, 256 * 4, osPriorityNormal);
DECLARE_STATIC_TASK(DebugTask, 1024 * 4, osPriorityBelowNormal);
DECLARE_STATIC_TASK(motorTask, 512 * 4, osPriorityNormal);
DECLARE_STATIC_TASK(armTask, 512 * 4, osPriorityNormal);
// DECLARE_STATIC_TASK(ChassisTask, 512 * 4, osPriorityNormal);
DECLARE_STATIC_TASK(ControlTask, 512 * 4, osPriorityNormal);
DECLARE_STATIC_TASK(Uart2ProcessTask, 512 * 4, osPriorityNormal1);
DECLARE_STATIC_TASK(Uart3ProcessTask, 512 * 4, osPriorityNormal1);
#if LASER_MEASURE_ENABLE
DECLARE_STATIC_TASK(LaserMeasureTask, 256 * 4, osPriorityNormal1);
#endif
DECLARE_STATIC_TASK(tail_claw_Task, 128 * 4, osPriorityNormal1);
DECLARE_STATIC_TASK(NavControlTask, 512 * 4, osPriorityNormal);
DECLARE_STATIC_TASK(LiftTask, 256 * 4, osPriorityNormal);
DECLARE_STATIC_TASK(PcComTask, 512 * 4, osPriorityNormal);
DECLARE_STATIC_TASK(StateMachineTask, 512 * 4, osPriorityNormal);
DECLARE_STATIC_TASK(InfraredProcessTask, 256 * 4, osPriorityNormal1);

// ---- 串口消息队列 ----
static constexpr uint32_t LOG_QUEUE_LENGTH = 8;

RAM_D1_ATTR static StaticQueue_t log_queue_cb;
RAM_D1_ATTR static uint8_t log_queue_buffer[LOG_QUEUE_LENGTH * sizeof(LoggerQueue::message)];
static const osMessageQueueAttr_t log_queue_attr = {
    .name      = "log_queue",
    .attr_bits = 0U,
    .cb_mem    = &log_queue_cb,
    .cb_size   = sizeof(log_queue_cb),
    .mq_mem    = log_queue_buffer,
    .mq_size   = sizeof(log_queue_buffer),
};

// ---- PC 日志消息队列 ----
RAM_D1_ATTR static StaticQueue_t pc_log_queue_cb;
RAM_D1_ATTR static uint8_t pc_log_queue_buffer[LOG_QUEUE_LENGTH * sizeof(LoggerQueue::message)];
static const osMessageQueueAttr_t pc_log_queue_attr = {
    .name      = "pc_log_queue",
    .attr_bits = 0U,
    .cb_mem    = &pc_log_queue_cb,
    .cb_size   = sizeof(pc_log_queue_cb),
    .mq_mem    = pc_log_queue_buffer,
    .mq_size   = sizeof(pc_log_queue_buffer),
};
osMessageQueueId_t pc_log_queue_handle = nullptr;

void osTaskInit(void) {
  extern LoggerQueue logger_queue;
  osMessageQueueId_t log_queue_handle =
      osMessageQueueNew(LOG_QUEUE_LENGTH, sizeof(LoggerQueue::message), &log_queue_attr);
  pc_log_queue_handle =
      osMessageQueueNew(LOG_QUEUE_LENGTH, sizeof(LoggerQueue::message), &pc_log_queue_attr);
  logger_queue.init(log_queue_handle, pc_log_queue_handle);

  CAN1_Send_TaskHandle = osThreadNew(can1SendTask, NULL, &CAN1_SendTaskHandle_attributes);
  CAN2_Send_TaskHandle = osThreadNew(can2SendTask, NULL, &CAN2_SendTaskHandle_attributes);
  CAN3_Send_TaskHandle = osThreadNew(can3SendTask, NULL, &CAN3_SendTaskHandle_attributes);
  Debug_TaskHandle = osThreadNew(debugTask, NULL, &DebugTaskHandle_attributes);
  Motor_TaskHandle = osThreadNew(motorTask, NULL, &motorTaskHandle_attributes);
  Arm_TaskHandle = osThreadNew(armTask, NULL, &armTaskHandle_attributes);
//   ChassisTaskHandle = osThreadNew(chassisTask, NULL, &ChassisTaskHandle_attributes);
  ControlTaskHandle = osThreadNew(controlTask, NULL, &ControlTaskHandle_attributes);
  uart2ProcessTaskHandle = osThreadNew(uart2RxProcessTask, NULL, &Uart2ProcessTaskHandle_attributes);
  uart3ProcessTaskHandle = osThreadNew(uart3RxProcessTask, NULL, &Uart3ProcessTaskHandle_attributes);
#if LASER_MEASURE_ENABLE
  laserMeasureTaskHandle = osThreadNew(laserMeasureTask, NULL, &LaserMeasureTaskHandle_attributes);
#endif
  tail_claw_TaskHandle = osThreadNew(tail_claw_task, NULL, &tail_claw_TaskHandle_attributes);
  //用于定位
  NavControlTaskHandle = osThreadNew(NavControlTask, NULL, &NavControlTaskHandle_attributes);
  //用于抬升
  LiftTaskHandle = osThreadNew(liftTask, NULL, &LiftTaskHandle_attributes);
  //用于上位机和下位机通讯的协议解析和封装
  PcComTaskHandle = osThreadNew(PcComTask, NULL, &PcComTaskHandle_attributes);
  // 自动状态机
  StateMachineTaskHandle = osThreadNew(stateMachineTask, NULL, &StateMachineTaskHandle_attributes);
  InfraredProcessTaskHandle = osThreadNew(infraredProcessTask, NULL, &InfraredProcessTaskHandle_attributes);
}
