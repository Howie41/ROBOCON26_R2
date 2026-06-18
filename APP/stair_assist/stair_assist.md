# stair_assist 说明

## 1. 模块作用

`APP/stair_assist/stair_assist.h/.cpp` 是一层“楼梯激光辅助判定模块”。

它的职责不是直接驱动电机，也不是直接切换整车状态机，而是：

1. 读取 `laser1` 和 `laser2` 的最新测距结果
2. 把原始毫米值转换成有物理意义的状态
3. 做连续多帧稳定判定，避免单帧抖动误触发
4. 向外部提供“建议上楼 / 建议到下楼边缘 / 建议切回低位”这类可直接调用的接口

这层模块后面可以同时被下面几种逻辑复用：

- 手动模式下的半自动楼梯辅助
- 自动状态机
- 楼梯导航流程
- 调试与上位机观察

---

## 2. 两个激光的职责分工

### 2.1 激光1

`laser1` 主要负责判断“什么时候可以开始动作”。

当前预期用途：

- 上楼时，判断底盘是否已经贴近楼梯
- 下楼时，在“前方还有下一阶楼梯可以扫到”的场景下，辅助判断是否已经到达边缘

注意：

- `laser1` 不是“什么时候切回低位”的主判据
- 尤其是从最高一阶往下时，前方可能没有楼梯面可扫，这时 `laser1` 不可靠

### 2.2 激光2

`laser2` 主要负责判断“进入高位之后，什么时候该切回低位”。

当前预期用途：

- 上楼时：
  `GroundNormal -> HighSuspended -> StepContact`
- 下楼时：
  先有近距离区，再进入更远的悬空区

因此：

- 上楼切低位主要看 `laser2`
- 下楼切低位主要也看 `laser2`

---

## 3. 当前定义的状态

### 3.1 激光1状态

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
  - 数据无效，不能参与判定
- `Far`
  - 离楼梯还远
- `NearStair`
  - 已经贴近楼梯，可用于开始上楼流程
- `EdgeOpen`
  - 前方距离明显变大，可辅助表示“可能到达下楼边缘”

### 3.2 激光2状态

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
  - 进入高位后离地更远
- `StepContact`
  - 高位前推后扫到更近的台阶/平台结构

---

## 4. 当前预留的接口

### 4.1 初始化与开关

```cpp
void stairAssistInit();
void stairAssistSetEnabled(bool enabled);
bool stairAssistEnabled();
```

作用：

- 初始化整个楼梯辅助模块
- 启用或关闭楼梯辅助功能

典型用途：

- 手动模式下，给一个按键切换“激光楼梯辅助开/关”

### 4.2 周期刷新与过程复位

```cpp
void stairAssistUpdate();
void stairAssistResetProgress();
```

作用：

- `stairAssistUpdate()`
  - 周期调用一次
  - 内部读取 `laser1`、`laser2`
  - 更新状态分类、连续帧计数、过程记忆、最终判据
- `stairAssistResetProgress()`
  - 清除“已经见过高位区/近距离区”等过程记忆
  - 适合在动作开始前重新置零

### 4.3 当前状态查询

```cpp
StairAssistLaser1State stairAssistLaser1State();
StairAssistLaser2State stairAssistLaser2State();
```

作用：

- 读取当前分类后的状态
- 调试时很有用

### 4.4 最终判据接口

```cpp
bool stairAssistSuggestClimbUp();
bool stairAssistSuggestDescendEdgeReady();
bool stairAssistShouldLowerAfterClimbAdvance();
bool stairAssistShouldLowerAfterDescendRetreat();
```

含义：

- `stairAssistSuggestClimbUp()`
  - 建议开始上楼
  - 当前主要由 `laser1 == NearStair` 连续稳定若干帧得到

- `stairAssistSuggestDescendEdgeReady()`
  - 建议认为已经到达下楼边缘
  - 当前主要由 `laser1 == EdgeOpen` 连续稳定若干帧得到
  - 这是辅助接口，不是所有场景都可靠

- `stairAssistShouldLowerAfterClimbAdvance()`
  - 上楼时进入高位并向前推进后，是否该切回低位
  - 当前主要由 `laser2` 先见过 `HighSuspended`，再稳定进入 `StepContact` 得到

- `stairAssistShouldLowerAfterDescendRetreat()`
  - 下楼时进入高位并向后退后，是否该切回低位
  - 当前主要由 `laser2` 先见过近距离区，再稳定进入 `HighSuspended` 得到

### 4.5 调试接口

```cpp
const StairAssistDebug &stairAssistDebug();
```

作用：

- 返回整个调试结构
- 适合在 Ozone 中直接观察

可观察内容包括：

- 当前是否启用
- 两个激光的原始 mm 值
- 两个激光的状态分类
- 连续稳定帧计数
- 是否已经见过某个关键阶段
- 当前建议上楼/下楼/切低位的布尔结果

---

## 5. 当前内部判定方式

### 5.1 稳定判定

当前使用：

- 连续 `3` 帧落在目标区间才认为成立

原因：

- 当前测距频率约 `10Hz`
- `3` 帧大约就是 `300ms`
- 可以明显减少单帧抖动引起的误触发

### 5.2 数据新鲜度判断

模块不会只看 `distance_mm`，还会检查：

- 当前帧是否有效
- 是否是错误帧
- 是否收到过新帧
- 最近一段时间内是否更新过

这样可以避免旧数据或错误数据误触发动作。

---

## 6. 未来想实现的功能，当前接口是否支持

### 6.1 手动模式下，移动到楼梯前自动上楼

目标流程：

1. 手动驾驶到底盘贴紧楼梯
2. 激光辅助判断已经贴紧
3. 自动请求高位
4. 高位过程中继续根据激光2判断什么时候切回低位

结论：

- 这套接口可以支撑
- 核心会用到：
  - `stairAssistSetEnabled(true)`
  - `stairAssistUpdate()`
  - `stairAssistSuggestClimbUp()`
  - `stairAssistShouldLowerAfterClimbAdvance()`

但要真正落地，还需要在调用层补下面这些逻辑：

1. 在 `control_task` 或别的上层任务里周期调用 `stairAssistUpdate()`
2. 加一个“楼梯辅助开关”按键
3. 当 `stairAssistSuggestClimbUp()` 成立时，自动触发 `liftRequestHigh()`
4. 进入高位并开始 2006 前推后，等待 `stairAssistShouldLowerAfterClimbAdvance()` 成立，再触发 `liftRequestLow()`

### 6.2 手动模式下，靠 2006 后退自动下楼

目标流程：

1. 手动把车退到底盘接近楼梯边缘
2. 进入高位
3. 2006 向后退
4. 激光2判断是否已经跨出边缘，满足后自动切低位

结论：

- 这套接口也可以支撑
- 核心会用到：
  - `stairAssistSetEnabled(true)`
  - `stairAssistUpdate()`
  - `stairAssistShouldLowerAfterDescendRetreat()`

辅助上可以用：

- `stairAssistSuggestDescendEdgeReady()`

但这个辅助接口有场景限制：

- 如果前面还有一阶楼梯面可扫，它可能有帮助
- 如果从最高一阶直接往下，前方没有楼梯面，这个接口不能强依赖

因此下楼真正“什么时候切低位”，还是应以 `laser2` 的接口为主。

---

## 7. 当前不能保证自动完成的部分

虽然接口层已经具备，但下面这些内容还没有接入：

1. 还没有接手柄按键开关
2. 还没有接入 `control_task` 的自动触发逻辑
3. 还没有接入 `state_machine_task` 或 `waypoint_navigator`
4. 激光1、激光2的阈值目前还是第一版估值，需要实测调参

所以现在的状态是：

- “判据层”已经有了
- “调用层”和“动作层”还没接

---

## 8. 推荐下一步

推荐按下面顺序继续：

1. 在 `control_task` 中接入 `stairAssistUpdate()`
2. 增加手动模式下的楼梯辅助启停按键
3. 先实现“贴紧楼梯自动升高位”
4. 再实现“高位过程中自动切回低位”
5. 最后根据实测数据微调各阈值

这样风险最小，也最容易调试。

---

## 9. 后续重点调试项

后续调试重点主要分成 4 类：

### 9.1 距离阈值

这是最核心的调试项。

需要实测并修改：

- `laser1` 贴近楼梯时的距离范围
- `laser1` 到下楼边缘时的距离范围
- `laser2` 低位正常地面范围
- `laser2` 高位悬空范围
- `laser2` 前推接阶范围

当前这些阈值都在：

- `APP/stair_assist/stair_assist.cpp`

第一版只是估值，后面必须根据实机数据调整。

### 9.2 稳定帧数

当前使用：

- 连续 `3` 帧成立才算有效

后面如果发现：

- 太敏感，容易误触发
  - 可以改成 `4` 帧或 `5` 帧
- 太迟钝，动作反应慢
  - 可以改成 `2` 帧

### 9.3 自动动作触发时机

后面接入 `control_task` 或状态机后，要重点看：

- 贴近楼梯后，是不是触发高位过早或过晚
- 进入高位后，2006 前推/后退是否时机合适
- `laser2` 满足条件后，是否应该立刻降低位
- 降低位前是否需要延时或增加保护条件

这部分不只是“阈值调参”，还是“动作节奏调参”。

### 9.4 场景边界

后面要特别注意这些特殊情况：

- 从最高一阶下楼时，`laser1` 可能没有前方楼梯面可扫
- 不同楼梯材质可能让反射稳定性变差
- 底盘抖动时，`laser2` 可能在两个区间反复跳变
- 高位过程中，传感器姿态变化可能导致距离值整体偏移

因此：

- `laser1` 的下楼边缘判定只能作为辅助
- 下楼真正“什么时候切低位”还是要以 `laser2` 为主

---

## 10. Ozone 推荐观察变量

后面调试时，建议优先观察 `stairAssistDebug()` 返回结构中的这些成员：

- `enabled`
- `laser1_fresh`
- `laser2_fresh`
- `laser1_mm`
- `laser2_mm`
- `laser1_state`
- `laser2_state`
- `laser1_near_count`
- `laser1_edge_count`
- `laser2_ground_count`
- `laser2_high_count`
- `laser2_step_count`
- `saw_laser2_high_for_climb`
- `saw_laser2_close_for_descend`
- `suggest_climb_up`
- `suggest_descend_edge_ready`
- `should_lower_after_climb`
- `should_lower_after_descend`

如果要抓最关键的一批，可以先重点看：

- `laser1_mm`
- `laser2_mm`
- `laser1_state`
- `laser2_state`
- `suggest_climb_up`
- `should_lower_after_climb`
- `should_lower_after_descend`

---

## 11. 各阶段预期数值变化

下面这些不是最终定值，而是后面调试时的“预期趋势”。

### 11.1 上楼时

#### 阶段1：低位平地接近楼梯

预期：

- `laser1` 从 `Far` 逐渐进入 `NearStair`
- `suggest_climb_up` 最终变为 `true`

重点观察：

- `laser1_mm`
- `laser1_state`
- `laser1_near_count`

#### 阶段2：进入高位

预期：

- `laser2` 从 `GroundNormal` 进入 `HighSuspended`
- `laser2_high_count` 开始累积
- `saw_laser2_high_for_climb` 变为 `true`

重点观察：

- `laser2_mm`
- `laser2_state`
- `laser2_high_count`

#### 阶段3：2006 向前推进

预期：

- `laser2` 从 `HighSuspended` 进入 `StepContact`
- `laser2_step_count` 开始累积
- `should_lower_after_climb` 最终变为 `true`

重点观察：

- `laser2_mm`
- `laser2_state`
- `laser2_step_count`
- `should_lower_after_climb`

#### 阶段4：回到低位

预期：

- `laser2` 最终重新回到某个稳定地面区
- 后续如需要，可再次重新开始下一轮判定

### 11.2 下楼时

#### 阶段1：接近下楼边缘

预期：

- 如果前方还有楼梯面可扫，`laser1` 可能进入 `EdgeOpen`
- `suggest_descend_edge_ready` 可能变为 `true`

注意：

- 这个阶段不是所有楼梯场景都可靠
- 尤其是最高一阶向下时，可能没有这个现象

重点观察：

- `laser1_mm`
- `laser1_state`
- `laser1_edge_count`
- `suggest_descend_edge_ready`

#### 阶段2：进入高位并开始后退

预期：

- `laser2` 先处于近距离区或地面区
- `saw_laser2_close_for_descend` 变为 `true`

重点观察：

- `laser2_mm`
- `laser2_state`
- `saw_laser2_close_for_descend`

#### 阶段3：2006 向后退，下方变空

预期：

- `laser2` 进入 `HighSuspended`
- `laser2_high_count` 累积
- `should_lower_after_descend` 最终变为 `true`

重点观察：

- `laser2_mm`
- `laser2_state`
- `laser2_high_count`
- `should_lower_after_descend`

#### 阶段4：回到低位

预期：

- 低位落下后，`laser2` 再次回到新的稳定地面区

---

## 12. 调试建议顺序

推荐按下面顺序调试：

1. 先只看 `laser1_mm` 和 `laser2_mm` 原始值
2. 确认每个机械阶段的实际距离范围
3. 再看 `laser1_state` 和 `laser2_state` 是否分类正确
4. 再看连续帧计数是否稳定增长
5. 最后再看 `suggest_*` 和 `should_lower_*` 是否在正确时机翻转

这样调试最稳，不容易一开始就把“阈值问题”和“动作流程问题”混在一起。
