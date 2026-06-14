# robot_init

机器人初始化入口 (`Robot_Init()`)，在 main() 中调用，在 FreeRTOS 调度器启动前执行。

初始化顺序:
1. 关中断 (__disable_irq)
2. DWT 计时器初始化 (SystemCoreClock / 1MHz)
3. comServiceInit(): CAN/UART/USB/电机/协议解析器初始化
4. osTaskInit(): 创建全部 14 个 FreeRTOS 任务
5. 开中断 (__enable_irq) → 返回 main → FreeRTOS 调度器启动