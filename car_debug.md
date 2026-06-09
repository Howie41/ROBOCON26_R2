# ROBOCON26 小车调参手册

> 工程路径: `D:\32CUBEMXCODE\ROBOCON26_R2\ROBOCON26_R2`
> STM32H723 + FreeRTOS CMSIS v2 + C++23

---

## 目录

1. [PID 结构体字段说明](#1-pid-结构体字段说明)
2. [全向轮底盘](#2-全向轮底盘-chassis)
3. [抬升机构](#3-抬升机构-lift)
4. [导航系统](#4-导航系统-navigation)
5. [尾巴夹爪](#5-尾巴夹爪-tail-claw)
6. [楼梯状态机](#6-楼梯状态机-stair-sm)
7. [遥控与手柄](#7-遥控与手柄-control)
8. [CAN 总线与电机配置](#8-can-总线与电机配置)
9. [快速上车检查清单](#9-快速上车检查清单)

---

## 1. PID 结构体字段说明

文件: `Module/controller/pid_controller.h:82`

| 字段 | 含义 | 典型范围 |
|------|------|----------|
| `Kp` | 比例系数 | 0.01 ~ 1000 |
| `Ki` | 积分系数 | 0 ~ 100 |
| `Kd` | 微分系数 | 0 ~ 10 |
| `MaxOut` | 输出限幅 | 取决于被控对象 |
| `IntegralLimit` | 积分限幅 | 0 ~ MaxOut/10 |
| `DeadBand` | 死区 | 0.01 ~ 5.0 |
| `Improve` | 优化选项(位或) | NONE / Integral_Limit / Trapezoid_Intergral 等 |

**常用 Improve 选项**:
- `NONE` — 无优化
- `Integral_Limit` — 积分限幅(防积分饱和)

---

## 2. 全向轮底盘 (Chassis)

### 2.1 运动学参数

文件: `APP/chassis_task/chassis_solution.hpp:38`

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `wheel_diameter_m` | `0.15378` | m | 轮径 |
| `track_width_m` | `0.44` | m | 轮距(左右) |
| `wheel_base_m` | `0.38` | m | 轴距(前后) |

> 更换轮胎或调整轮距后需对应修改

### 2.2 速度限制

文件: `APP/control_task/control_task.h:21`

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `MAX_VELOCITY` | `3.0` | m/s 或 rad/s | 底盘最大线/角速度 |
| `MAX_LIFT_VELOCITY` | `1.0` | m/s | 2006 手操升降最大速度(归一化系数) |

### 2.3 四轮速度环 PID (C620)

文件: `APP/chassis_task/chassis_task.cpp:111-116`

| 轮位 | Kp | Ki | Kd | MaxOut |
|------|-----|-----|-----|--------|
| 左前 (FL) | 105.0 | 75.0 | 0.20 | 20000.0 |
| 右前 (FR) | 100.0 | 72.0 | 0.15 | 20000.0 |
| 左后 (RL) | 108.0 | 78.0 | 0.22 | 20000.0 |
| 右后 (RR) | 102.0 | 74.0 | 0.18 | 20000.0 |

> DeadBand 均为 0.3, Improve = NONE
> **调参建议**: 如果某轮抖动增大 Kd, 如果某轮跟不上增大 Kp

### 2.4 底盘 Yaw Hold PID (驻车锁头)

文件: `APP/chassis_task/chassis_task.cpp:118-126`

| Kp | Ki | Kd | MaxOut | IntegralLimit | DeadBand | Improve |
|----|----|-----|--------|---------------|----------|---------|
| 0.16 | 0.0008 | 0.001 | 1.5 | 0.35 | 0.3 | Integral_Limit |

> 松摇杆时保持当前 yaw 角。如果锁不住增大 Kp，如果抖动减小 MaxOut

### 2.5 Yaw 90°旋转 PID

文件: `APP/chassis_task/chassis_task.cpp:61-69`

| Kp | Ki | Kd | MaxOut | IntegralLimit | DeadBand | Improve |
|----|----|-----|--------|---------------|----------|---------|
| 0.36 | 0.001 | 0.001 | 4.5 | 0.35 | 0.3 | Integral_Limit |

| 参数 | 值 | 说明 |
|------|-----|------|
| `kYawRotate90Deg` | `90.0` | 旋转目标角度 |
| `kYawRotateToleranceDeg` | `1.0` | 到位容差(度) |

> LB 逆时针 90°, RB 顺时针 90°

---

## 3. 抬升机构 (Lift)

### 3.1 3508 位置 PID (外环)

文件: `APP/lift_task/lift_task.cpp:95-96`

| Kp | Ki | Kd | MaxOut | DeadBand | Improve |
|----|----|-----|--------|----------|---------|
| 9.0 | 0.1 | 0.0 | 300.0 | 0.0 | NONE |

### 3.2 3508 速度环 PID (内环, 双电机独立)

文件: `APP/lift_task/lift_task.cpp:91-94`

| 电机 | Kp | Ki | Kd | MaxOut | DeadBand | Improve |
|------|----|----|-----|--------|----------|---------|
| 3508 #1 | 100.0 | 30.0 | 0.3 | 30000 | 0.1 | NONE |
| 3508 #2 | 100.0 | 30.0 | 0.3 | 30000 | 0.1 | NONE |

### 3.3 3508 同步 PID (双电机位置差归零)

文件: `APP/lift_task/lift_task.cpp:97-98`

| Kp | Ki | Kd | MaxOut | DeadBand | Improve |
|----|----|-----|--------|----------|---------|
| 0.28 | 0.0 | 0.0 | 30.0 | 0.1 | NONE |

> 一加一减注入两个电机，补偿机械不同步。Kp 太大会导致两个 3508 来回振荡

### 3.4 2006 速度环 PID (高位驱动)

文件: `APP/lift_task/lift_task.cpp:86-89`

| 电机 | Kp | Ki | Kd | MaxOut | DeadBand | Improve |
|------|----|----|-----|--------|----------|---------|
| 2006 #1 | 100.0 | 10.0 | 0.0 | 10000 | 0.3 | NONE |
| 2006 #2 | 100.0 | 10.0 | 0.0 | 10000 | 0.3 | NONE |

### 3.5 2006 Yaw 锁角 PID (高位手动模式)

文件: `APP/lift_task/lift_task.cpp:101-104`

| Kp | Ki | Kd | MaxOut | IntegralLimit | DeadBand | Improve |
|----|----|-----|--------|---------------|----------|---------|
| 15.0 | 0.02 | 0.0 | 300.0 | 150.0 | 0.5 | Integral_Limit |

### 3.6 机械/物理参数

文件: `APP/lift_task/lift_task.cpp:65-74`

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `MAX_LIFT_2006_SPEED` | `600.0` | RPM | 2006 最大转速 |
| `MAX_LIFT_3508_SPEED` | `300.0` | RPM | 3508 手动升降最大速度 |
| `MAX_LIFT_3508_SYNC_COMP` | `30.0` | RPM | 同步补偿最大幅度 |
| `LIFT_RISE_SPEED` | `200.0` | RPM | 自动上升速度 |
| `LIFT_FALL_SPEED` | `200.0` | RPM | 自动下降速度 |
| `LIFT_POS_TOLERANCE` | `2.0` | deg | 位置到达容差 |
| `LIFT_HIGH_POS` | `520.0` | deg | 高位目标位置(6005着地) |
| `LIFT_LOW_POS` | `-100.0` | deg | 低位目标位置(全向轮着地) |

### 3.7 电机方向

文件: `APP/lift_task/lift_task.cpp:76-79`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `LIFT_2006_MOTOR1_DIR` | `1.0` | 2006#1 方向(1或-1) |
| `LIFT_2006_MOTOR2_DIR` | `-1.0` | 2006#2 方向(对置安装,反号) |
| `LIFT_3508_MOTOR1_DIR` | `1.0` | 3508#1 方向 |
| `LIFT_3508_MOTOR2_DIR` | `-1.0` | 3508#2 方向(对置安装,反号) |

---

## 4. 导航系统 (Navigation)

### 4.1 全向轮导航 PID (低位)

文件: `Module/navigation/NavProtocol.cpp:15-43`

| PID | Kp | Ki | Kd | MaxOut | IntegralLimit | DeadBand | Improve |
|-----|----|----|-----|--------|---------------|----------|---------|
| pid_x (前后) | 2.0 | 0.15 | 0.05 | 1000.0 | 100.0 | 10.0 | Integral_Limit |
| pid_y (左右) | 2.0 | 0.15 | 0.05 | 1000.0 | 100.0 | 10.0 | Integral_Limit |
| pid_yaw (角度) | 0.1 | 0.001 | 0.005 | 3.0 | 0.5 | 3.0 | Integral_Limit |

### 4.2 2006 高位导航 PID

文件: `Module/navigation/NavProtocol.cpp:46-65`

| PID | Kp | Ki | Kd | MaxOut | IntegralLimit | DeadBand | Improve |
|-----|----|----|-----|--------|---------------|----------|---------|
| pid_high_distance (距离) | 8.0 | 0.05 | 0.0 | 500.0 | 200.0 | 5.0 | Integral_Limit |
| pid_high_yaw (角度) | 15.0 | 0.02 | 0.0 | 300.0 | 150.0 | 0.5 | Integral_Limit |

### 4.3 导航阈值

文件: `Module/navigation/NavProtocol.cpp`

| 参数 | 值 | 单位 | 说明 |
|------|-----|------|------|
| `kPositionTimeoutTicks` | `pdMS_TO_TICKS(200)` | ms | 位置超时,超时即停 |
| 高位 heading 阈值 | `1.0` | deg | 超过此值先原地旋转 |
| 高位 forward 限幅 | `500.0` | RPM | 前进速度上限 |
| 高位 omega 限幅 | `500.0` | RPM | 差速旋转上限 |
| 低位到达阈值 | `10.0` | mm | 距离+角度误差内判定到达 |
| 高位到达阈值 | `10.0` | mm | 距离误差内判定到达+触发降位 |

---

## 5. 尾巴夹爪 (Tail Claw)

### 5.1 2006 水平移动 PID (位置→速度 级联)

文件: `APP/tail_claw_task/tail_claw_task.cpp:35-53`

**位置环**:

| Kp | Ki | Kd | MaxOut | IntegralLimit | DeadBand | Improve |
|----|----|-----|--------|---------------|----------|---------|
| 0.05 | 0.0 | 0.0 | 5.0 | 0.35 | 0.3 | NONE |

**速度环**:

| Kp | Ki | Kd | MaxOut | IntegralLimit | DeadBand | Improve |
|----|----|-----|--------|---------------|----------|---------|
| 700.0 | 0.03 | 0.02 | 10000.0 | 0.35 | 0.3 | NONE |

> 注意: 第96行有 `pos_pid->MaxOut = 5.0f;` 在 set_move_pos() 内被硬编码覆盖

### 5.2 3508 翻转 PID (位置→速度 级联)

文件: `APP/tail_claw_task/tail_claw_task.cpp:55-71`

**位置环**:

| Kp | Ki | Kd | MaxOut | DeadBand | Improve |
|----|----|-----|--------|----------|---------|
| 30.0 | 0.0 | 1.0 | 100.0 | 0.3 | NONE |

**速度环**:

| Kp | Ki | Kd | MaxOut | DeadBand | Improve |
|----|----|-----|--------|----------|---------|
| 120.0 | 10.0 | 1.4 | 2000.0 | 0.3 | NONE |

### 5.3 机械/控制参数

文件: `APP/tail_claw_task/tail_claw_task.cpp:11-17`

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `roll_reduction_ratio` | `2.0` | — | 3508 减速比 |
| `move_max_distance` | `6.0` | cm | 水平移动最大行程 |
| `move_degree_per_cm` | `360/(3π)` ≈ 38.2 | deg/cm | 齿条齿轮换算 |
| `move_step` | `0.01` | cm | 每次按键移动量 |
| `roll_step` | `0.3` | deg | 每次按键翻转角度 |

**调试建议**:
- 移动太快: 减小 `move_step` 或 `move_max_distance`
- 翻转太快/太慢: 调整 `roll_step`
- 位置不准: 先调位置环 Kp, 再加 Kd 消除超调
- 电机啸叫: 减小速度环 Kp 或增大 Kd

（待 D-pad else 分支清除 PC 位标志的 bug 还未修复，见行148-151）

---

## 6. 楼梯状态机 (Stair SM)

### 6.1 校准常数

文件: `APP/nav_waypoint/field_waypoints.hpp:8-11`

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `STAIR_CLIMB_DIST` | `400.0` | mm | 2006 高位驱动距离 |
| `STAIR_CENTER_OFFSET` | `200.0` | mm | 降下后去中心距离 |

> **上车必须实测校准**: 实际楼梯长度 ≠ 400 时改 `STAIR_CLIMB_DIST`，降后到中心 ≠ 200 时改 `STAIR_CENTER_OFFSET`

### 6.2 航点表

文件: `APP/nav_waypoint/field_waypoints.hpp:17-20`

```cpp
constexpr Wp LIST[128] = {
    {0, 0, 0},  // [0] 保留
    // 用户自行填充 [1] ~ [127]
};
```

> 每个航点: `{x(mm), y(mm), yaw(deg)}`, 总共 128 个

### 6.3 楼梯协议

| 方向 | 消息码 | 说明 |
|------|--------|------|
| PC→STM32 | `0x0103` | 上楼梯(高位方向) |
| PC→STM32 | `0x0104` | 下楼梯(低位方向) |
| PC→STM32 | `0x010A` | 去航点(body: 1字节索引) |
| STM32→PC | `0x0207` | 2006高位驱动开始 |
| STM32→PC | `0x0208` | 2006高位驱动到达 |
| STM32→PC | `0x0209` | 去中心驱动开始 |
| STM32→PC | `0x020A` | 去中心驱动到达 |

### 6.4 状态机阶段

```
Phase 0 (IDLE)         → 等待命令
Phase 1 (LIFT_UP)      → 等待 high_mode_active=true
Phase 2 (HIGH_DRIVE)   → 等待 arrived=true (高位导航PID)
Phase 3 (LIFT_DOWN)    → 等待 high_mode_active=false
Phase 4 (DRIVE_CENTER) → 等待 arrived=true (低位导航PID)
Phase 5 (DONE)         → 回到 IDLE
```

> SM 不可打断: `stairSMStart()` 在 active=true 时直接 return

---

## 7. 遥控与手柄 (Control)

### 7.1 按键映射

文件: `APP/control_task/control_task.cpp`

| 按键 | 功能 |
|------|------|
| Xbox 键 | 切换 手动/自动导航 |
| LB | 逆时针旋转 90° |
| RB | 顺时针旋转 90° |
| Y 短按 | 自动去高位 |
| A 短按 | 自动去低位 |
| Y 长按 | 手动上升 3508 |
| A 长按 | 手动下降 3508 |
| 左摇杆 上下 | 底盘前后 |
| 左摇杆 左右 | 底盘左右 |
| 右摇杆 左右 | 底盘旋转 |
| 右摇杆 上下 | 2006 前进/后退(仅高位) |

### 7.2 尾巴夹爪按键

文件: `APP/tail_claw_task/tail_claw_task.cpp`

| 按键 | 功能 |
|------|------|
| D-pad 上 | 3508 向上翻转 |
| D-pad 下 | 3508 向下翻转 |
| D-pad 左 | 2006 向左平移 |
| D-pad 右 | 2006 向右平移 |
| Share | 爪子 开/合 |
| Menu | 气泵 开/关 |

### 7.3 按键阈值与时间

| 参数 | 值 | 说明 |
|------|-----|------|
| 摇杆死区 | `ABS(val-32767) > 2000` | 略偏离中心即响应 |
| Y/A 短按超时 | `300ms` (kLiftTapTimeout) | 短按=去高/低位,长按=手动升降 |

---

## 8. CAN 总线与电机配置

### 8.1 CAN 总线拓扑

| CAN 总线 | 挂载设备 |
|----------|----------|
| CAN1 (`hfdcan1`) | 抬升: 2006#1(0x201), 2006#2(0x202), 3508#1(0x203), 3508#2(0x204) |
| CAN2 (`hfdcan2`) | 尾部: 2006水平(0x201), 3508翻转(0x202), arm2006伸缩(0x203), arm3508旋转(0x204), DM4310翻转(0x301), DM4340抬升(0x302) |
| CAN3 (`hfdcan3`) | 底盘: 620#1(0x201), #2(0x202), #3(0x203), #4(0x204) |

### 8.2 电机类型对照

| 电机 | 驱动器 | 协议 | 减速比 | 反馈 |
|------|--------|------|--------|------|
| M2006 | C610 | DJI CAN | 36:1 | 位置+速度 |
| M3508 | C620 | DJI CAN | 19:1 | 位置+速度 |
| DM4310 | DM43xx | DM CAN | 10:1 | MIT 位置+速度+扭矩 |
| DM4340 | DM43xx | DM CAN | 10:1 | MIT 位置+速度+扭矩 |

> DJI CAN 帧速率: 1ms/帧, 每帧携带 4 个槽位 (ID 0x200)
> DM43xx 电机独立发送 CAN 帧，由 MotorPlanningSystem 管理

### 8.3 电机命令限制

文件: `Module/motor/Motor.hpp:42-48`

```cpp
void setMotorCmd(float cmd) {
    if (cmd > max_cmd_) cmd = max_cmd_;
    if (cmd < -max_cmd_) cmd = -max_cmd_;
    cmd_ = cmd;
}
```

> 每个电机的 `max_cmd_` 由构造函数传入，详见 `com_config.cpp`

---

## 9. 快速上车检查清单

### 9.1 底盘

- [ ] `wheel_diameter_m` 是否匹配实际轮胎
- [ ] 四轮方向 `direction_sign_` 是否正确 (±1)
- [ ] `MAX_VELOCITY` 是否合适
- [ ] 四轮速度 PID 是否均衡(低速不抖/高速不飘)

### 9.2 抬升

- [ ] `LIFT_HIGH_POS` / `LIFT_LOW_POS` 是否正确(实测标记, 520°/-100°)
- [ ] `LIFT_POS_TOLERANCE` 是否够用(默认2°)
- [ ] 两个 3508 同步 PID 是否调好(不抖不同步不超限)
- [ ] 2006 电机方向是否正确(前进=向前走)

### 9.3 导航

- [ ] 低位 PID 能否稳定到达 10mm 内
- [ ] 高位 2006 PID 能否驱动 400mm 准确到达
- [ ] 位置超时 200ms 是否合适

### 9.4 楼梯

- [ ] `STAIR_CLIMB_DIST` 实测校准
- [ ] `STAIR_CENTER_OFFSET` 实测校准
- [ ] 航点表 `LIST[]` 已填写

### 9.5 尾巴夹爪

- [ ] `move_max_distance` 机械行程匹配(默认6cm)
- [ ] `roll_reduction_ratio` 减速比正确(默认2:1)
- [ ] 爪子和气泵 GPIO 正确(GPIOG Pin3/4)
- [ ] Xbox D-pad 左右和 PC 距离控制互斥逻辑验证
