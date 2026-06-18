# debug_task

调试任务，当前为占位空跑 (入口直接 `osThreadExit()`)。

测试新功能/添加新模块可以写在这里。调试完成后清理，避免堵塞。

可用调试资源:
- 全局 `logger` (USART10 日志输出)
- 全局电机对象和 PID 变量
- 全局导航状态变量 (nav_control namespace)