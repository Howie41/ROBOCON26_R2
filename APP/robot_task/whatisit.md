# robot_task

机器人所有 FreeRTOS 任务的创建入口 (`osTaskInit()`)。

## 14 个任务

| 任务 | 入口 | 栈(bytes) | 优先级 |
|------|------|-----------|--------|
| CAN1_Send | can1SendTask | 1024 | Normal |
| CAN2_Send | can2SendTask | 1024 | Normal |
| CAN3_Send | can3SendTask | 1024 | Normal |
| Debug | debugTask | 2048 | Normal |
| Motor | motorTask | 2048 | Normal |
| Chassis | chassisTask | 2048 | Normal |
| Control | controlTask | 2048 | Normal |
| Uart2Process | uart2RxProcessTask | 2048 | Normal1 |
| Uart3Process | uart3RxProcessTask | 2048 | Normal1 |
| tail_claw | tail_claw_task | 512 | Normal1 |
| NavControl | NavControlTask | 2048 | Normal |
| Lift | liftTask | 2048 | Normal |
| PcCom | PcComTask | 2048 | Normal |
| StateMachine | stateMachineTask | 2048 | Normal |

## topic_pool.h

定义所有 topic 消息结构体:
- pub_Xbox_Data: Xbox 手柄数据
- pub_chassis_cmd: 底盘运动指令
- pub_lift_cmd: 抬升指令
- pub_infrared_msg: 红外消息
- pub_high_nav_cmd: 高位导航指令
- tail_claw_msg: 尾巴夹爪消息
- pc_nav_position_t / pc_nav_target_t: 上位机导航协议
- pc_nav_event_t: 导航事件通知
- pub_qr_code_parsed: 二维码解析结果