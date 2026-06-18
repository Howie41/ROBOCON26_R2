# chassis_task

全向轮底盘解算类 `Omni45Chassis` (chassis_solution.hpp)。

用法:
```cpp
Omni45Chassis chassis_solver(chassis_motor1, chassis_motor2, chassis_motor3,
                chassis_motor4);
```
电机顺序: FL(左上) → FR(右上) → RL(左下) → RR(右下)

配置速度环PID:
```cpp
chassis_solver.configureSpeedPid(kWheelPidParams);
```

主循环调用 `chassis_solver.run(cmd)` 完成解算→PID→setMotorCmd。

包含 yaw 锁角 (手动模式松杆保持) 和 yaw 90° 旋转 (LB/RB)。
