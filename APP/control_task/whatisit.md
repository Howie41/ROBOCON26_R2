# control_task

遥控控制任务 (5ms 周期)。

**输入**: 订阅 `xbox` topic (pub_Xbox_Data)
**输出**: 发布 `chassis_cmd` 和 `lift_cmd`

**按键处理**:
- Xbox键: 切换 手动/自动导航 (auto_enabled)
- LB/RB: 触发 yaw 90° 旋转 (仅手动模式)
- 左摇杆: 底盘前后/左右平移
- 右摇杆 左右: 底盘旋转
- 右摇杆 上下: 2006 高位驱动 (MAX_LIFT_VELOCITY=1.0)
- Y 短按(<300ms): 请求去高位 (request_high)
- A 短按(<300ms): 请求去低位 (request_low)
- Y/A 长按: 手动升降 3508

自动模式 (auto_enabled=true) 时跳过 Xbox_Data_Process 和 Lift_Data_Process，导航由 NavControlTask 接管。