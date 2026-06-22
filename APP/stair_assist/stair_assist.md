# stair_assist 模块说明

## 1. 模块定位

`APP/stair_assist/stair_assist.h/.cpp` 是当前工程的“上下台阶判据层”。

它不直接驱动电机，也不直接改状态机步骤，而是专门负责：

1. 读取 `laser1 / laser2 / laser3 / rear photogate`
2. 把原始测距值或遮挡状态转换成可用判据
3. 做新鲜度判断
4. 做稳定帧统计
5. 向上层返回“是否该切高位 / 切低位”

当前它被两类上层同时使用：

- 手动辅助模式
- `nav_waypoint` 自动上下台阶流程

## 2. 当前四个传感器各自负责什么

### 2.1 laser1

作用：

- 前向判断是否已经贴近楼梯，可进入高位上台阶

当前近距离区间：

- `430 ~ 500 mm`

此外它还保留了两个辅助区间：

- `950 ~ 1120 mm`
- `1650 ~ 1700 mm`

这两个区间主要是给后续扩展预留的，不是当前主流程的核心判据。

### 2.2 laser2

作用：

- 安装在前腿，朝下
- 主要负责下台阶时“高位什么时候回低位”
- 也保留了一个上高位辅助区间

当前关键区间：

- 上高位辅助区间：`210 ~ 240 mm`
- 下台阶回低位区间：`300 ~ 400 mm`

同时内部还会把它分成三类：

- `StepContact`
- `GroundNormal`
- `HighSuspended`

当前分类区间：

- `0 ~ 449 mm -> StepContact`
- `450 ~ 800 mm -> GroundNormal`
- `801 ~ 1400 mm -> HighSuspended`

### 2.3 laser3

作用：

- 第二个前向激光
- 和 `laser1` 在上层逻辑上等价
- 当前只用于辅助“上台阶进入高位”

当前近距离区间：

- `450 ~ 500 mm`

### 2.4 rear photogate

作用：

- 后部光电门
- 不测距离，只看遮挡与边沿

当前主流程中作用为：

- 上台阶时：
- 决定高位推进后什么时候回低位

- 下台阶时：
- 决定低位靠近楼梯边缘后什么时候进入高位

## 3. 当前状态枚举

### 3.1 laser1 / laser3 使用的状态

```cpp
enum class StairAssistLaser1State : uint8_t {
  Invalid = 0,
  Far,
  NearStair,
  EdgeOpen,
};
```

含义：

- `Invalid`
- 当前数据无效或不新鲜

- `Far`
- 还没贴近楼梯

- `NearStair`
- 已进入贴近楼梯区间

- `EdgeOpen`
- 前向距离变大，可用于“前方开口 / 边缘”类辅助判断

### 3.2 laser2 使用的状态

```cpp
enum class StairAssistLaser2State : uint8_t {
  Invalid = 0,
  GroundNormal,
  HighSuspended,
  StepContact,
};
```

含义：

- `Invalid`
- 数据无效

- `GroundNormal`
- 低位正常看地面

- `HighSuspended`
- 进入高位后，脚下距离变大

- `StepContact`
- 已经扫到更近的结构

### 3.3 模式枚举

```cpp
enum class StairAssistMode : uint8_t {
  ClimbUp = 0,
  Descend,
};
```

含义：

- `ClimbUp`
- 当前把判据层当成“上台阶辅助”

- `Descend`
- 当前把判据层当成“下台阶辅助”

## 4. 当前最重要的 5 个接口

### 4.1 初始化

```cpp
void stairAssistInit();
```

作用：

- 清空内部状态
- 清空计数
- 默认回到关闭、上台阶模式

### 4.2 开关

```cpp
void stairAssistSetEnabled(bool enabled);
bool stairAssistEnabled();
```

作用：

- 打开或关闭整个判据层

当前使用场景：

- 手动模式下，通过 `RS` 开关辅助
- 自动 `Y / A` 流程中，在进入对应流程时打开，在流程结束时关闭

### 4.3 模式切换

```cpp
void stairAssistSetMode(StairAssistMode mode);
StairAssistMode stairAssistMode();
```

作用：

- 告诉判据层当前是在做“上台阶”还是“下台阶”

当前使用场景：

- 手动模式下 `LS` 切换模式
- 自动上台阶前强制切到 `ClimbUp`
- 自动下台阶前强制切到 `Descend`

### 4.4 自动回低位开关

```cpp
void stairAssistSetAutoLowerEnabled(bool enabled);
bool stairAssistAutoLowerEnabled();
```

作用：

- 控制判据层是否允许输出“回低位”相关结果

这样做的原因：

- 有时只想开“上高位判据”
- 不想让系统过早自动回低位

### 4.5 周期更新

```cpp
void stairAssistUpdate();
```

作用：

- 每次调用时刷新全部传感器状态
- 更新计数与判据

这是本模块最核心的周期函数。

## 5. 当前给上层提供的判定函数

### 5.1 上台阶进入高位

```cpp
bool stairAssistSuggestClimbUp();
```

返回 `true` 代表：

- 现在可以请求进入高位

当前成立条件：

- `laser1` 命中近距离区间
- 或 `laser3` 命中近距离区间
- 或 `laser2` 命中 `210 ~ 240 mm`

### 5.2 下台阶进入高位

```cpp
bool stairAssistSuggestDescendHighMode();
```

返回 `true` 代表：

- 现在可以请求进入高位

当前成立条件：

- 后部光电门出现 `unblocked edge`
- 或当前保持 `rear_unblocked`

### 5.3 下台阶边缘辅助接口

```cpp
bool stairAssistSuggestDescendEdgeReady();
```

返回 `true` 代表：

- `laser1` 认为前向出现开口

注意：

- 这个接口当前不是主流程核心
- 只是辅助保留

### 5.4 上台阶回低位

```cpp
bool stairAssistShouldLowerAfterClimbAdvance();
```

返回 `true` 代表：

- 当前上台阶高位推进后，可以收腿回低位

当前成立条件：

- 自动回低位开关已打开
- 当前模式是 `ClimbUp`
- 后部光电门出现 `blocked edge`
- 或当前保持 `rear_blocked`

### 5.5 下台阶回低位

```cpp
bool stairAssistShouldLowerAfterDescendRetreat();
```

返回 `true` 代表：

- 当前下台阶高位后退后，可以回低位

当前成立条件：

- 自动回低位开关已打开
- 当前模式是 `Descend`
- `laser2` 落在 `300 ~ 400 mm`

## 6. 当前稳定判据

当前内部设置：

```cpp
constexpr uint8_t kStableFrames = 1;
```

也就是说：

- 现在只要满足 `1 帧` 就成立

为什么现在是 1：

- 用户已经在实车上验证过，当前响应速度优先
- 如果后续出现误触发，再把它提高到 `2` 或 `3`

## 7. 当前新鲜度判断

当前超时时间：

```cpp
constexpr uint32_t kLaserDataTimeoutMs = 500;
```

含义：

- 超过 `500ms` 没更新的新数据，会被判成不新鲜
- 不新鲜的数据不会参与有效判断

## 8. 当前手动模式下怎么用

手动辅助接在：

- `APP/control_task/control_task.cpp`

当前手动逻辑：

1. `RS` 打开 / 关闭辅助
2. `LS` 切换上台阶辅助 / 下台阶辅助
3. 当未处于高位时：
- 上台阶模式看 `stairAssistSuggestClimbUp()`
- 下台阶模式看 `stairAssistSuggestDescendHighMode()`
4. 当已经处于高位时：
- 上台阶模式看 `stairAssistShouldLowerAfterClimbAdvance()`
- 下台阶模式看 `stairAssistShouldLowerAfterDescendRetreat()`

## 9. 当前自动模式下怎么用

自动模式主要接在：

- `APP/nav_waypoint/waypoint_navigator.cpp`

当前用法是：

- `Y` 上台阶流程开始时：
- `stairAssistSetMode(ClimbUp)`
- `stairAssistSetAutoLowerEnabled(true)`
- `stairAssistSetEnabled(true)`

- `A` 下台阶流程开始时：
- `stairAssistSetMode(Descend)`
- `stairAssistSetAutoLowerEnabled(true)`
- `stairAssistSetEnabled(true)`

流程结束后再关闭。

## 10. 当前推荐 Ozone 监视项

### 10.1 最常用的一组

- `g_ozone_stair_assist_enabled`
- `g_ozone_stair_assist_mode`
- `g_ozone_suggest_climb_up`
- `g_ozone_suggest_descend_high`
- `g_ozone_should_lower_after_climb`
- `g_ozone_should_lower_after_descend`
- `g_ozone_laser2_mm`
- `g_ozone_laser2_fresh`
- `g_ozone_laser3_mm`
- `g_ozone_laser3_fresh`
- `g_photogate_rear_blocked`
- `g_photogate_rear_unblocked`

### 10.2 如果要更细看内部状态

可以继续在 `stairAssistDebug()` 里看这些成员：

- `laser1_mm`
- `laser2_mm`
- `laser3_mm`
- `laser1_state`
- `laser2_state`
- `laser3_state`
- `laser1_near_count`
- `laser3_near_count`
- `laser2_climb_high_count`
- `laser2_descend_lower_count`
- `rear_photogate_blocked`
- `rear_photogate_unblocked`
- `rear_photogate_blocked_edge`
- `rear_photogate_unblocked_edge`

## 11. 当前实际作用总结

一句话总结现在的 `stair_assist`：

- 它已经不是单纯的“激光读数层”
- 而是当前梅林上下台阶动作切换的统一判据层

当前已经被实车验证可用的主分工是：

- `laser1 / laser3`：上台阶进入高位
- `rear photogate`：上台阶收腿回低位
- `rear photogate`：下台阶进入高位
- `laser2`：下台阶回低位

## 12. 后续如果继续扩展，建议怎么做

如果后面继续加新传感器或新判据，建议遵循下面规则：

1. 底层驱动只负责拿原始数据
2. 是否触发动作，一律收口到 `stair_assist`
3. `control_task` 只负责手动触发
4. `waypoint_navigator` 只负责动作流程与阶段切换

这样结构最清楚，也最不容易打架。
