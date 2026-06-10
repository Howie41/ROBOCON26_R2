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
extern osThreadId_t ChassisTaskHandle;
extern osThreadId_t ControlTaskHandle;
extern osThreadId_t usbcdcProcessTaskHandle;
extern osThreadId_t tail_claw_TaskHandle;
extern osThreadId_t NavControlTaskHandle;
extern osThreadId_t LiftTaskHandle;
extern osThreadId_t PcComTaskHandle;

#include "memory_map.h"

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
DECLARE_STATIC_TASK(DebugTask, 512 * 4, osPriorityBelowNormal);
DECLARE_STATIC_TASK(MotorTask, 512 * 4, osPriorityNormal);
DECLARE_STATIC_TASK(ChassisTask, 512 * 4, osPriorityNormal);
DECLARE_STATIC_TASK(ControlTask, 512 * 4, osPriorityNormal);
DECLARE_STATIC_TASK(Uart2ProcessTask, 512 * 4, osPriorityNormal1);
DECLARE_STATIC_TASK(Uart3ProcessTask, 512 * 4, osPriorityNormal1);
DECLARE_STATIC_TASK(tail_claw_Task, 128 * 4, osPriorityNormal1);
DECLARE_STATIC_TASK(NavControlTask, 512 * 4, osPriorityNormal);
DECLARE_STATIC_TASK(LiftTask, 512 * 4, osPriorityNormal);
DECLARE_STATIC_TASK(PcComTask, 512 * 4, osPriorityNormal);
DECLARE_STATIC_TASK(StateMachineTask, 512 * 4, osPriorityNormal);

void osTaskInit(void) {
  CAN1_Send_TaskHandle = osThreadNew(can1SendTask, NULL, &CAN1_SendTaskHandle_attributes);
  CAN2_Send_TaskHandle = osThreadNew(can2SendTask, NULL, &CAN2_SendTaskHandle_attributes);
  CAN3_Send_TaskHandle = osThreadNew(can3SendTask, NULL, &CAN3_SendTaskHandle_attributes);
  Debug_TaskHandle = osThreadNew(debugTask, NULL, &DebugTaskHandle_attributes);
  Motor_TaskHandle = osThreadNew(motorTask, NULL, &MotorTaskHandle_attributes);
  ChassisTaskHandle = osThreadNew(chassisTask, NULL, &ChassisTaskHandle_attributes);
  ControlTaskHandle = osThreadNew(controlTask, NULL, &ControlTaskHandle_attributes);
  uart2ProcessTaskHandle = osThreadNew(uart2RxProcessTask, NULL, &Uart2ProcessTaskHandle_attributes);
  uart3ProcessTaskHandle = osThreadNew(uart3RxProcessTask, NULL, &Uart3ProcessTaskHandle_attributes);
  tail_claw_TaskHandle = osThreadNew(tail_claw_task, NULL, &tail_claw_TaskHandle_attributes);
  //用于定位
  NavControlTaskHandle = osThreadNew(NavControlTask, NULL, &NavControlTaskHandle_attributes);
  //用于抬升
  LiftTaskHandle = osThreadNew(liftTask, NULL, &LiftTaskHandle_attributes);
  //用于上位机和下位机通讯的协议解析和封装
  PcComTaskHandle = osThreadNew(PcComTask, NULL, &PcComTaskHandle_attributes);
  // 自动状态机
  StateMachineTaskHandle = osThreadNew(stateMachineTask, NULL, &StateMachineTaskHandle_attributes);
}
