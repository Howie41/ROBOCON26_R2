# Merlin Update 2026-06-25

## 本次完成

本次已经完成并编译通过的新增内容：

1. `laser2` 的低台阶去边缘判据改为 `300 ~ 500 mm`
2. 新增 `chassis_action::turn_right_180_deg()`
3. 新增 `chassis_action::start_go_to_edge()`
4. 新增 `chassis_action::start_return_to_center()`
5. `heading` 改为根据真实底盘角度自动吸附更新

## 新接口

文件：[APP/chassis_task/chassis_task.h](D:/32CUBEMXCODE/ROBOCON26_R2/ROBOCON26_R2/APP/chassis_task/chassis_task.h)

当前新增接口如下：

```cpp
namespace chassis_action {

void turn_right_180_deg();
void start_go_to_edge();
void start_return_to_center();

}
```

说明：

- `turn_right_180_deg()`
  - 单次顺时针旋转 180 度
  - 内部直接请求 `-180.0f`
  - 不是两个 90 度拼接

- `start_go_to_edge()`
  - 梅林格子中，从当前格子中心朝当前车头方向去边缘
  - 走到满足传感器条件后停住
  - 这个动作本身不上台阶，只是“去到边缘等待后续夹取”

- `start_return_to_center()`
  - 从边缘回当前格子中心
  - 当前只靠雷达中心点，不依赖激光判据

## heading 规则

文件：[APP/chassis_task/chassis_task.cpp](D:/32CUBEMXCODE/ROBOCON26_R2/ROBOCON26_R2/APP/chassis_task/chassis_task.cpp)

`heading` 现在不再只靠 `turn_left_90_deg()` / `turn_right_90_deg()` 手工更新。
现在改为每轮底盘任务根据 `g_chassis_yaw_deg` 自动吸附：

- `-30° ~ 30°` -> `PosX`
- `60° ~ 120°` -> `PosY`
- `150° ~ 180°` 或 `-180° ~ -150°` -> `NegX`
- `-120° ~ -60°` -> `NegY`

其他角度区间保持上一次 `heading` 不变。

额外补充：

- 90 度和 180 度旋转接口在返回前会主动刷新一次 `heading`
- 这样刚旋转完立刻接下一条动作，也能拿到更新后的朝向

## 去边缘逻辑

文件：[APP/nav_waypoint/waypoint_navigator.cpp](D:/32CUBEMXCODE/ROBOCON26_R2/ROBOCON26_R2/APP/nav_waypoint/waypoint_navigator.cpp)

入口函数：

```cpp
void stairWaypointRunGoToEdge();
```

当前逻辑分两种情况：

1. 前方格子比当前格子高
   - 使用 `laser3`
   - 粗定位先走到 `headingClosePoseForCell(current_cell)`
   - 如果粗定位途中 `laser3` 已满足判据，就提前停
   - 如果到粗定位点还没满足，就以小速度继续向前找，直到满足

2. 前方格子比当前格子低
   - 使用 `laser2`
   - 当前判据区间是 `300 ~ 500 mm`
   - 粗定位先保守前进 `200 mm`
   - 如果到粗定位点还没触发，就继续以小速度向前找
   - 触发后当前默认不额外延时，`post trigger delay = 0 ms`

当前保留参数：

```cpp
constexpr float kGoToEdgeSeekSpeedMps = 0.08f;
constexpr float kGoToEdgeLowPostTriggerSpeedMps = 0.05f;
constexpr int16_t kGoToEdgeLowCoarseAdvanceMm = 200;
constexpr uint32_t kGoToEdgeLowPostTriggerDelayMs = 0U;
```

你后面如果觉得“低台阶触发后还想再往前蹭一点”，就调 `kGoToEdgeLowPostTriggerDelayMs`。

## 回中心逻辑

文件：[APP/nav_waypoint/waypoint_navigator.cpp](D:/32CUBEMXCODE/ROBOCON26_R2/ROBOCON26_R2/APP/nav_waypoint/waypoint_navigator.cpp)

入口函数：

```cpp
void stairWaypointRunReturnToCenter();
```

当前逻辑：

- `start_go_to_edge()` 开始时会先把“当前中心点”保存下来
- 回中心时优先回这个保存的中心点
- 如果保存无效，则退化为“重新识别当前格子，再回当前格子中心”

## 判据层变更

文件：[APP/stair_assist/stair_assist.cpp](D:/32CUBEMXCODE/ROBOCON26_R2/ROBOCON26_R2/APP/stair_assist/stair_assist.cpp)

新增两个接口：

```cpp
bool stairAssistSuggestGoToEdgeHigh();
bool stairAssistSuggestGoToEdgeLow();
```

当前判据：

- `stairAssistSuggestGoToEdgeHigh()`
  - 使用 `laser3`
  - 逻辑是 `laser3_near_count >= kStableFrames`

- `stairAssistSuggestGoToEdgeLow()`
  - 使用 `laser2`
  - 逻辑是 `laser2_fresh && laser2_mm in [300, 500]`

## Ozone 建议监视

这轮新增相关建议优先看：

- `g_chassis_yaw_deg`
- `g_ozone_merlin_heading`
- `nav_control::current_x`
- `nav_control::current_y`
- `nav_control::target_x`
- `nav_control::target_y`
- `g_ozone_stair_step`
- `g_ozone_laser2_mm`
- `g_ozone_laser2_fresh`
- `g_ozone_laser3_mm`
- `g_ozone_laser3_fresh`

如果你要验证 `go_to_edge()`：

- 高台阶场景重点看 `g_ozone_laser3_mm`
- 低台阶场景重点看 `g_ozone_laser2_mm`
- 同时看 `g_ozone_stair_step` 判断卡在哪一步

## 当前风险说明

这次改动已经编译通过，但还没做你实车验收。

当前最需要你现场确认的点：

1. `turn_right_180_deg()` 旋转完成后，`g_ozone_merlin_heading` 是否立刻变到正确朝向
2. `start_go_to_edge()` 在“前方更高”和“前方更低”两种场景下，是否都能按预期停住
3. 低台阶场景里，`laser2` 的 `300 ~ 500 mm` 是否过宽或过窄
4. `start_return_to_center()` 是否能稳定回到原中心点
