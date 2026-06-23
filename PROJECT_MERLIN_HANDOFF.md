# ROBOCON26 R2 梅林阶段交接文档

## 1. 文档目的

这份文档面向后续接手该工程的人，重点回答下面几个问题：

1. 当前梅林阶段已经做到什么程度。
2. 上下台阶四个阶段分别是怎么实现的。
3. 哪些 PID、阈值和距离常量目前还比较粗糙，必须继续调。
4. 当前手动模式已经验证到什么效果。
5. 后续如果要接入全自动状态机，建议怎么做，并尽量保持现有接口不变。

---

## 2. 工程概况

### 2.1 车体结构

- 底盘为四个 45 度安装的全向轮，可进行 `x / y / yaw` 平面运动。
- 前部有两条较短前腿。
- 两个 `2006` 电机负责高位模式下的前进 / 后退推进。
- 后部为较长的被动支撑。
- 两个 `3508` 电机负责车体在低位模式 / 高位模式之间切换。

### 2.2 梅林阶段目标

当前梅林阶段的目标不是单纯平地导航，而是：

- 在 `4 x 3` 的梅林台阶区逐级上台阶、下台阶。
- 支持 `LB / RB` 旋转 90 度后，继续按照新的朝向执行上下台阶。
- 利用“全局粗定位 + 近距离激光 / 光电判定”完成动作切换。

### 2.3 当前代码入口

- 梅林地图与格子识别：
  [APP/merlin_map/merlin_map.cpp](D:/32CUBEMXCODE/ROBOCON26_R2/ROBOCON26_R2/APP/merlin_map/merlin_map.cpp)
- 上下台阶 waypoint 流程：
  [APP/nav_waypoint/waypoint_navigator.cpp](D:/32CUBEMXCODE/ROBOCON26_R2/ROBOCON26_R2/APP/nav_waypoint/waypoint_navigator.cpp)
- 台阶辅助判定：
  [APP/stair_assist/stair_assist.cpp](D:/32CUBEMXCODE/ROBOCON26_R2/ROBOCON26_R2/APP/stair_assist/stair_assist.cpp)
- 手柄控制与人工触发：
  [APP/control_task/control_task.cpp](D:/32CUBEMXCODE/ROBOCON26_R2/ROBOCON26_R2/APP/control_task/control_task.cpp)
- 状态机入口：
  [APP/state_machine_task/state_machine_task.cpp](D:/32CUBEMXCODE/ROBOCON26_R2/ROBOCON26_R2/APP/state_machine_task/state_machine_task.cpp)
- 平地低位导航与高位导航 PID：
  [Module/navigation/NavProtocol.cpp](D:/32CUBEMXCODE/ROBOCON26_R2/ROBOCON26_R2/Module/navigation/NavProtocol.cpp)
- 抬升机构控制：
  [APP/lift_task/lift_task.cpp](D:/32CUBEMXCODE/ROBOCON26_R2/ROBOCON26_R2/APP/lift_task/lift_task.cpp)

---

## 3. 梅林地图与朝向模型

### 3.1 地图尺寸

- 梅林区域为 `4 行 x 3 列`
- 每格间距 `1200 mm`
- 锚点为第 `1` 行第 `2` 列中心点：`(3100, 1470)`

### 3.2 高度表

- 第 1 行：`400, 200, 400`
- 第 2 行：`200, 400, 600`
- 第 3 行：`400, 600, 400`
- 第 4 行：`200, 400, 200`

### 3.3 heading 定义

- `0 = PosX`
- `1 = PosY`
- `2 = NegX`
- `3 = NegY`

对应含义：

- `Y` 表示沿当前车头方向“上一个台阶”
- `A` 表示沿当前车尾方向“下一个台阶”

### 3.4 yaw 目标映射

- `PosX -> 0 deg`
- `PosY -> 90 deg`
- `NegX -> 180 deg`
- `NegY -> -90 deg`

这样做的作用是：旋转 90 度以后，再执行 `Y / A` 时，目标姿态会跟着新的 heading 走，不会被强行拉回原始朝向。

---

## 4. 传感器分工

### 4.1 laser1

前向激光，主要用于判断“是否已经靠近台阶正面，可以进入上台阶高位”。

当前上台阶近距离阈值：

- `430 ~ 500 mm`

### 4.2 laser2

安装在前腿附近朝下看，主要用于判断高位阶段何时可以回低位。

当前关键阈值：

- 上台阶辅助切高位：`210 ~ 240 mm`
- 下台阶回低位：`300 ~ 400 mm`

### 4.3 laser3

第二个前向激光，作为 `laser1` 的并行冗余判定。

当前上台阶近距离阈值：

- `450 ~ 500 mm`

### 4.4 rear photogate

后部光电门，不做距离测量，主要做最终动作切换时刻判定。

当前用途：

- 上台阶时：高位推进后，决定什么时候回低位
- 下台阶时：低位后退找边后，决定什么时候进入高位

### 4.5 当前判定逻辑总览

上台阶：

- 进入高位：
  `laser1` 或 `laser3` 命中近距离区间，或 `laser2` 命中辅助区间
- 回低位：
  `rear photogate`

下台阶：

- 进入高位：
  `rear photogate`
- 回低位：
  `laser2`

---

## 5. 当前手动控制链路

### 5.1 `Xbox`

进入 `RobotState::go_to_stair_front`，最终调用：

```cpp
stairWaypointGoToFront();
```

作用是把车移动到梅林入口前的待命点。

### 5.2 `View`

执行当前位置格子识别：

```cpp
merlin_map::identifyCurrentCell(nav_control::current_x, nav_control::current_y);
```

### 5.3 `LB / RB`

先请求底盘旋转 90 度，等旋转完成后再更新 `heading`。

对应接口：

```cpp
chassis_action::requestYawRotateCcw90();
chassis_action::requestYawRotateCw90();
merlin_map::rotateCcw90();
merlin_map::rotateCw90();
```

### 5.4 `Y`

进入状态机 `RobotState::test_stair_up`，最终调用：

```cpp
stairWaypointRunUp();
```

### 5.5 `A`

进入状态机 `RobotState::test_stair_down`，最终调用：

```cpp
stairWaypointRunDown();
```

---

## 6. 上下台阶具体实现

这一节是后续写自动化和调参数时最关键的部分。

### 6.1 上台阶整体函数入口

上台阶主入口：

```cpp
stairWaypointRunUp();
```

主要流程在：
[APP/nav_waypoint/waypoint_navigator.cpp](D:/32CUBEMXCODE/ROBOCON26_R2/ROBOCON26_R2/APP/nav_waypoint/waypoint_navigator.cpp)

### 6.2 上台阶详细流程

#### 阶段 A：低位模式 -> 高位模式，重负载切高位

执行逻辑：

1. 刷新当前格子识别：
   `refreshCurrentMerlinCell()`
2. 获取当前格子 `current_cell` 和前向邻格 `next_cell`
3. 根据当前格子和 `heading` 计算：
   - 当前待命点 `standby_pose`
   - 靠近台阶的粗定位点 `close_pose`
   - 目标下一格中心 `center_pose`
4. 打开台阶辅助：
   - `stairAssistSetMode(StairAssistMode::ClimbUp)`
   - `stairAssistSetAutoLowerEnabled(true)`
   - `stairAssistSetEnabled(true)`
5. 先移动到当前待命点：
   `move_to_pose(standby_pose, true)`
6. 再移动到靠近台阶的粗定位点，同时持续检查：

```cpp
stairAssistSuggestClimbUp()
```

7. 如果在到达粗定位点前已经触发，则立即进入高位流程
8. 如果到达粗定位点还未触发，则改为低速手动前爬：

```cpp
publishManualChassisCmd(0.12f, 0.0f, 0.0f);
```

直到 `stairAssistSuggestClimbUp()` 成立
9. 记录触发时刻的当前位置：
   `trigger_x / trigger_y`
10. 请求升到高位：

```cpp
liftRequestHigh();
wait_until([]() { return nav_control::high_mode_active; }, 10U);
```

这一阶段的核心特点：

- 是“低位全向导航 + 局部激光精触发”的组合
- 真正切高位的时机不是单看 waypoint 到点，而是由 `stair_assist` 决定

#### 阶段 B：高位模式 -> 低位模式，轻负载收腿

执行逻辑：

1. 以上一步记录的 `trigger_x / trigger_y` 为参考
2. 按当前 heading 向前推 `670 mm`，得到高位推进粗目标：

```cpp
high_drive_pose = advancePoseByHeading(trigger_x, trigger_y, kClimbAdvanceToLowerMm);
```

3. 如果存在下一格，则下一格中心直接取 `next_cell.center`
4. 进入高位推进阶段，调用：

```cpp
move_to_pose_until_trigger(high_drive_pose, true, []() {
  return stairAssistShouldLowerAfterClimbAdvance();
});
```

5. 如果在到达粗目标前，`rear photogate` 已经允许回低位，则立即回低位
6. 如果到达粗目标仍未满足，则发布高位慢速 crawl 命令：

```cpp
crawl_cmd.forward_speed = kClimbLaserSeekSpeedRpm;  // 30 RPM
```

继续往前慢速找回低位触发
7. 一旦 `stairAssistShouldLowerAfterClimbAdvance()` 成立，执行：

```cpp
liftRequestLow();
wait_until([]() { return !nav_control::high_mode_active; }, 10U);
```

8. 关闭台阶辅助
9. 回到下一格中心：

```cpp
move_to_pose(center_pose, true);
```

10. 刷新当前层级和 armed 状态

这一阶段的核心特点：

- 高位推进距离是经验常量
- 真正回低位依赖 `rear photogate`
- 如果粗目标不准，代码会进入慢速 creeping 补偿

### 6.3 下台阶整体函数入口

下台阶主入口：

```cpp
stairWaypointRunDown();
```

同样位于：
[APP/nav_waypoint/waypoint_navigator.cpp](D:/32CUBEMXCODE/ROBOCON26_R2/ROBOCON26_R2/APP/nav_waypoint/waypoint_navigator.cpp)

### 6.4 下台阶详细流程

#### 阶段 C：低位模式 -> 高位模式，轻负载挂边

执行逻辑：

1. 刷新当前格子识别，读取 `current_cell`
2. 根据当前格子中心和 heading 反方向，计算高位进入粗定位点：

```cpp
high_drive_pose = advancePoseByHeading(current_cell.center_x,
                                       current_cell.center_y,
                                       -kDescendRetreatToHighMm);
```

当前经验常量：

- `kDescendRetreatToHighMm = 280`

3. 台阶辅助切到下台阶模式：

```cpp
stairAssistSetMode(StairAssistMode::Descend);
stairAssistSetAutoLowerEnabled(true);
stairAssistSetEnabled(true);
```

4. 先回到当前格中心：
   `move_to_pose(current_center_pose, true)`
5. 再向后移动到进入高位粗定位点，同时持续检测：

```cpp
stairAssistSuggestDescendHighMode()
```

6. 如果在到达粗定位点前，后部光电门已经满足进入高位条件，则立即切高位
7. 如果到达粗定位点仍未满足，则改为低位慢速后退找边：

```cpp
publishManualChassisCmd(kDescendEdgeSeekSpeedMps, 0.0f, 0.0f);  // -0.12 m/s
```

8. 一旦 `stairAssistSuggestDescendHighMode()` 成立，则执行：

```cpp
liftRequestHigh();
wait_until([]() { return nav_control::high_mode_active; }, 10U);
```

这一阶段的核心特点：

- 依赖 rear photogate 判断“已经挂到合适的下台阶边缘”
- 属于轻负载切高位阶段

#### 阶段 D：高位模式 -> 低位模式，重负载落地

执行逻辑：

1. 计算回低位粗目标点：

```cpp
close_pose = advancePoseByHeading(current_cell.center_x,
                                  current_cell.center_y,
                                  -kDescendRetreatToLowerMm);
```

当前经验常量：

- `kDescendRetreatToLowerMm = 950`

2. 进入高位后退阶段，调用：

```cpp
move_to_pose_until_trigger(close_pose, false, []() {
  return stairAssistShouldLowerAfterDescendRetreat();
});
```

3. 如果 `laser2` 已经满足回低位条件，则立即降回低位
4. 如果到达粗目标点仍不满足，则发高位慢速后退 crawl：

```cpp
crawl_cmd.forward_speed = kDescendLaserSeekSpeedRpm;  // -30 RPM
```

5. 等到 `stairAssistShouldLowerAfterDescendRetreat()` 成立后执行：

```cpp
liftRequestLow();
wait_until([]() { return !nav_control::high_mode_active; }, 10U);
```

6. 如果下一级格子存在，则移动到下一级格中心
7. 如果不存在，则保持当前位置，并清除当前格识别有效性

这一阶段的核心特点：

- 高位后退时已经是重负载回落
- 是否回低位主要依赖 `laser2`
- 下降完成后才重新接管到低位全向导航

---

## 7. 四个阶段中需要重点优化的参数

### 7.1 阶段 A：上台阶低位 -> 高位，重负载切高位

重点要调：

- `close_pose` 的 heading 偏移
- `laser1 / laser3` 近距离区间
- `laser2` 辅助切高位区间
- `lift_task.cpp` 中 `CLIMBING_UP` 阶段的 3508 位置环参数

原因：

- 这一段决定“切高位是否过早 / 过晚”
- 如果切高位时机不稳定，后面所有步骤都会被放大

### 7.2 阶段 B：上台阶高位 -> 低位，轻负载收腿

重点要调：

- `kClimbAdvanceToLowerMm = 670`
- `kClimbAdvanceToCenterMm = 950`
- `rear photogate` 安装位置与触发可靠性
- 高位慢速前爬 `kClimbLaserSeekSpeedRpm = 30`

原因：

- 这一段决定“高位推进多远收腿最稳”
- 目前主要依赖经验值和单一光电门触发

### 7.3 阶段 C：下台阶低位 -> 高位，轻负载挂边

重点要调：

- `kDescendRetreatToHighMm = 280`
- `kDescendEdgeSeekSpeedMps = -0.12`
- rear photogate 的边沿判定稳定性

原因：

- 这一段决定“何时认为已经到边，可以切高位”
- 若过早切高位，可能还没挂稳；过晚切高位，可能已经姿态不利

### 7.4 阶段 D：下台阶高位 -> 低位，重负载落地

重点要调：

- `kDescendRetreatToLowerMm = 950`
- `laser2` 的 `300 ~ 400 mm` 回低位区间
- 高位慢速后退 `kDescendLaserSeekSpeedRpm = -30`
- `lift_task.cpp` 中 `DESCENDING` 阶段 3508 位置环参数

原因：

- 这一段决定“落地回低位时是否平稳、是否滞后”
- 是重负载阶段，对姿态和速度规划很敏感

---

## 8. 当前 PID 与速度规划现状

### 8.1 当前低位全向导航 PID

位于：
[Module/navigation/NavProtocol.cpp](D:/32CUBEMXCODE/ROBOCON26_R2/ROBOCON26_R2/Module/navigation/NavProtocol.cpp)

当前参数：

- `pid_x = {Kp=2.8, Ki=0.30, Kd=0.08}`
- `pid_y = {Kp=2.8, Ki=0.30, Kd=0.08}`
- `pid_yaw = {Kp=0.16, Ki=0.001, Kd=0.001}`

当前问题：

- 仍然是“位置误差直接打速度”的思路
- 缺少完整的速度规划和到点减速曲线
- 平地导航能用，但在接台阶、离台阶这类动作前后还不够柔和

### 8.2 当前高位导航

同样位于：
[Module/navigation/NavProtocol.cpp](D:/32CUBEMXCODE/ROBOCON26_R2/ROBOCON26_R2/Module/navigation/NavProtocol.cpp)

当前高位导航特点：

- 高位推进只保留前后方向推进和 yaw 锁角
- 当前速度更接近二段速：
  - 远处 `400 RPM`
  - 近处 `100 RPM`
- 到点判定仍比较粗

当前问题：

- 不是连续速度规划
- 对不同载荷阶段没有区分独立速度曲线
- 机械状态略变时，容易暴露“推进过猛 / 收尾偏硬”的问题

### 8.3 当前抬升机构 PID

位于：
[APP/lift_task/lift_task.cpp](D:/32CUBEMXCODE/ROBOCON26_R2/ROBOCON26_R2/APP/lift_task/lift_task.cpp)

当前已经做了比普通 PID 更进一步的处理：

- 分阶段 `LiftPhase`
- 轻载阶段使用轨迹规划
- 重载阶段使用更直接的位置闭环和速度斜坡

但依然需要继续实车细调，尤其是：

- `CLIMBING_UP`
- `DESCENDING`

这两个阶段对负载变化更敏感。

---

## 9. 当前手动模式已验证效果

当前实车已验证：

- 可以从梅林入口前待命点进入梅林
- 可以通过 `View` 识别当前处在哪个格子
- 可以通过 `LB / RB` 旋转 90 度后继续上下台阶
- `Y` 能沿当前 heading 上一个台阶
- `A` 能沿当前反 heading 下一个台阶
- 上下台阶均支持：
  - 传感器提前命中则提前切动作
  - 若到粗定位点仍未命中，则自动慢速 creeping 补偿

当前仍未完成：

- 完整比赛任务级全自动状态机
- 梅林外完整出入场路径
- 自动调度多步上下台阶和转向组合动作
- 更细致的高位速度规划

---

## 10. 面向全自动的接口建议

原则：

- 尽量不改现有底层执行接口
- 尽量不破坏当前手柄 `Y / A / LB / RB / View / Xbox` 行为
- 自动化优先在“状态机层”增加调用入口

### 10.1 现有可直接复用的底层接口

```cpp
stairWaypointGoToFront();
stairWaypointRunUp();
stairWaypointRunDown();

chassis_action::requestYawRotateCcw90();
chassis_action::requestYawRotateCw90();
```

这些接口已经能完成：

- 去台阶入口待命点
- 前进上一个台阶
- 后退下一个台阶
- 左转 90 度
- 右转 90 度

### 10.2 建议新增的状态机状态

建议在 [APP/state_machine_task/state_machine_task.h](D:/32CUBEMXCODE/ROBOCON26_R2/ROBOCON26_R2/APP/state_machine_task/state_machine_task.h) 中新增：

- `rotate_ccw_90`
- `rotate_cw_90`
- `merlin_step_up`
- `merlin_step_down`
- `goto_merlin_front`

### 10.3 建议新增的消息码

建议上位机 / 自动任务下发新消息码，例如：

- `0x30`：前进上一个台阶
- `0x31`：后退下一个台阶
- `0x32`：左转 90 度
- `0x33`：右转 90 度
- `0x34`：去梅林入口待命点

### 10.4 状态机调用示例

示例 1：前进上一个台阶

```cpp
case RobotState::merlin_step_up: {
  stairWaypointRunUp();
  change_state_to(RobotState::begin);
  break;
}
```

示例 2：后退下一个台阶

```cpp
case RobotState::merlin_step_down: {
  stairWaypointRunDown();
  change_state_to(RobotState::begin);
  break;
}
```

示例 3：左转 90 度

```cpp
case RobotState::rotate_ccw_90: {
  chassis_action::requestYawRotateCcw90();
  wait_until([]() { return !chassis_action::yawRotateActive(); }, 10U);
  merlin_map::rotateCcw90();
  change_state_to(RobotState::begin);
  break;
}
```

示例 4：右转 90 度

```cpp
case RobotState::rotate_cw_90: {
  chassis_action::requestYawRotateCw90();
  wait_until([]() { return !chassis_action::yawRotateActive(); }, 10U);
  merlin_map::rotateCw90();
  change_state_to(RobotState::begin);
  break;
}
```

这样做的好处：

- 手动按键逻辑可以继续保留
- 自动控制只是在状态机层多了一种“像按键一样”的触发方式
- 下层台阶执行逻辑不需要大改

---

## 11. 速度规划优化建议

目标：

- 尽量保持现有外部接口不变
- 不影响现在已经能用的手动与半自动流程

### 11.1 低位全向导航建议

保持外部接口不变：

```cpp
nav_control::target_x
nav_control::target_y
nav_control::target_yaw
```

内部优化方向：

- 增加距离分段限速
- 增加接近目标时的连续降速
- yaw 也做角度分段限速

不建议一上来只改 PID 数值，因为核心问题不只是增益，而是缺少速度规划层。

### 11.2 高位推进建议

保持外部接口不变：

- `pub_high_nav_cmd`
- `stairWaypointRunUp()`
- `stairWaypointRunDown()`

内部优化方向：

- 把当前“远处固定高速 + 近处固定低速”的二段速，改成连续减速
- 按四个阶段分别设置推进速度上限
- 对重负载阶段单独使用更保守的速度规划

优先级建议：

1. 先加速度规划
2. 再细调 PID
3. 最后再根据实车表现微调激光阈值和距离常量

---

## 12. Ozone 推荐监视变量

### 12.1 导航与状态

- `g_ozone_target_x`
- `g_ozone_current_x`
- `g_ozone_target_y`
- `g_ozone_current_y`
- `g_ozone_stair_step`
- `g_ozone_stair_level`
- `g_ozone_stair_armed`
- `g_ozone_high_mode_active`

### 12.2 梅林格与朝向

- `g_ozone_merlin_cell_valid`
- `g_ozone_merlin_row`
- `g_ozone_merlin_col`
- `g_ozone_merlin_height_mm`
- `g_ozone_merlin_heading`

### 12.3 台阶判定是否成立

- `g_ozone_suggest_climb_up`
- `g_ozone_suggest_descend_high`
- `g_ozone_should_lower_after_climb`
- `g_ozone_should_lower_after_descend`

### 12.4 传感器数据

- `g_ozone_laser2_mm`
- `g_ozone_laser2_fresh`
- `g_ozone_laser3_mm`
- `g_ozone_laser3_fresh`
- `g_photogate_rear_blocked`
- `g_photogate_rear_unblocked`

---

## 13. 后续开发建议

建议顺序：

1. 先把当前四阶段参数调稳。
2. 再补状态机消息码和自动调用入口。
3. 再补梅林外部完整任务链。
4. 最后优化更细的速度规划和自动调度。

开发原则：

- 优先保现有可运行流程
- 少改 `control_task` 主循环结构
- 地图规则优先改 `merlin_map`
- 台阶判定优先改 `stair_assist`
- 动作流程优先改 `waypoint_navigator`

