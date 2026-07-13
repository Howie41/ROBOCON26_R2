/**
 * @file lift_task.cpp
 * @author YE
 * @brief 抬升任务实现 (优化版: 梯形轨迹规划 + 重力补偿 + 分段参数)
 * @version 0.2
 * @date 2026-06-09
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "lift_task.h"

#include "Motor.hpp"
#include "NavProtocol.hpp"
#include "com_config.h"
#include "photogate.hpp"
#include "pid_controller.h"
#include "topic_pool.h"
#include "topics.hpp"

extern C610Motor lift_2006_motor1;
extern C610Motor lift_2006_motor2;
extern C620Motor lift_3508_motor1;
extern C620Motor lift_3508_motor2;

float lift_2006_speed = 0.0f;
float lift_3508_target_pos = 0.0f;
float lift_3508_pos_pid_out = 0.0f;
bool lift_3508_hold_enable = false;
bool lift_3508_manual_last = false;

float lift_3508_motor1_pos = 0.0f;
float lift_3508_motor2_pos = 0.0f;
float lift_3508_motor1_speed = 0.0f;
float lift_3508_motor2_speed = 0.0f;

float lift_3508_avg_pos = 0.0f;
float lift_3508_diff_pos = 0.0f;

float lift_3508_base_speed = 0.0f;
float lift_3508_sync_pid_out = 0.0f;
float lift_3508_motor1_ref_speed = 0.0f;
float lift_3508_motor2_ref_speed = 0.0f;

float lift_2006_motor1_pid_out = 0.0f;
float lift_2006_motor2_pid_out = 0.0f;
float lift_3508_motor1_pid_out = 0.0f;
float lift_3508_motor2_pid_out = 0.0f;

PID_t lift_3508_pos_pid = {
    .Kp = 9.0f, .Ki = 0.1f, .Kd = 0.0f,
    .MaxOut = MAX_LIFT_3508_SPEED, .IntegralLimit = 5000.0f, .DeadBand = 0.0f,
    .Improve = Integral_Limit,
};

// 速度PID — 电机电流控制, 保持原始配置确保出力够
PID_t lift_3508_motor1_pid = {
    .Kp = 100.0f, .Ki = 30.0f, .Kd = 0.3f, .MaxOut = 30000, .DeadBand = 0.1f,
    .Improve = NONE,
};
PID_t lift_3508_motor2_pid = {
    .Kp = 100.0f, .Ki = 30.0f, .Kd = 0.3f, .MaxOut = 30000, .DeadBand = 0.1f,
    .Improve = NONE,
};

PID_t lift_2006_motor1_pid = {
    .Kp = 100.0f, .Ki = 30.0f, .Kd = 0.0f,
    .MaxOut = 10000, .DeadBand = 0.3f,
    .Improve = NONE,
};
PID_t lift_2006_motor2_pid = {
    .Kp = 100.0f, .Ki = 30.0f, .Kd = 0.0f,
    .MaxOut = 10000, .DeadBand = 0.3f,
    .Improve = NONE,
};

PID_t lift_3508_sync_pid = {
    .Kp = 0.88f, .Ki = 0.1f, .Kd = 0.0f,
    .MaxOut = MAX_LIFT_3508_SYNC_COMP, .DeadBand = 0.1f,
    .Improve = NONE,
};

// Phase 2: 高位模式手动yaw锁角PID
PID_t high_yaw_lock_pid = {
    .Kp = 15.0f, .Ki = 0.05f, .Kd = 0.0f, .MaxOut = 400.0f,
    .IntegralLimit = 150.0f, .DeadBand = 0.3f, .Improve = Integral_Limit,
};
// 引用 NavProtocol.cpp 中定义的全局导航事件发布者
extern TypedTopicPublisher<pc_nav_event_t> pc_nav_event_pub;

osThreadId_t LiftTaskHandle;

static TypedTopicSubscriber<pub_lift_cmd> lift_cmd_sub("lift_cmd", 8);
static pub_lift_cmd lift_cmd{};

static TypedTopicSubscriber<pub_high_nav_cmd> high_nav_sub("high_nav_cmd", 4);
static pub_high_nav_cmd high_nav_cmd{};

extern volatile float g_chassis_yaw_deg;

// ============================================================================
//  常量定义
// ============================================================================

// ============================================================================
//  梯形轨迹规划器 (Trapezoidal Velocity Profile)
//  加速度恒定, 速度梯形, 位置平滑连续
// ============================================================================
class LiftTrajectory {
    bool active_;
    float q_start_, q_end_;
    float v_max_, a_max_;
    float t_acc_, t_const_, t_total_;
    float t_elapsed_;
    float sign_;
    float dist_accum_;

public:
    LiftTrajectory() : active_(false) {}

    void plan(float from, float to, float vmax, float amax) {
        q_start_ = from;
        q_end_ = to;
        v_max_ = vmax;
        a_max_ = amax;
        t_elapsed_ = 0;
        dist_accum_ = 0;
        sign_ = (to > from) ? 1.0f : -1.0f;

        float dist = fabsf(to - from);
        if (dist < 0.01f) {
            active_ = false;
            return;
        }

        // 计算能否达到最大速度
        float t_to_vmax = v_max_ / a_max_;
        float dist_accel = a_max_ * t_to_vmax * t_to_vmax; // 加减速段所需距离

        if (dist_accel <= dist) {
            // 完整梯形: 加速 + 匀速 + 减速
            t_acc_ = t_to_vmax;
            t_const_ = (dist - dist_accel) / v_max_;
            t_total_ = 2.0f * t_acc_ + t_const_;
        } else {
            // 三角形: 加速不足, 未达最大速度即开始减速
            t_acc_ = sqrtf(dist / a_max_);
            t_const_ = 0;
            t_total_ = 2.0f * t_acc_;
        }
        active_ = true;
    }

    // 每步调用, 返回参考位置 (度)
    float update(float dt) {
        if (!active_) return q_end_;

        t_elapsed_ += dt;

        if (t_elapsed_ >= t_total_) {
            dist_accum_ = fabsf(q_end_ - q_start_);
            active_ = false;
            return q_end_;
        }

        float t = t_elapsed_;
        float v;
        if (t < t_acc_) {
            v = a_max_ * t;
        } else if (t < t_acc_ + t_const_) {
            v = v_max_;
        } else {
            v = a_max_ * (t_total_ - t);
        }

        dist_accum_ += v * dt;
        return q_start_ + sign_ * dist_accum_;
    }

    // 参考速度 (deg/s), 用于速度前馈
    float refVelocity() const {
        if (!active_) return 0;
        float t = t_elapsed_;
        if (t >= t_total_) return 0;
        if (t < t_acc_) return sign_ * a_max_ * t;
        if (t < t_acc_ + t_const_) return sign_ * v_max_;
        return sign_ * a_max_ * (t_total_ - t);
    }

    bool done() const { return !active_; }
    bool active() const { return active_; }
};

// ============================================================================
//  运动阶段枚举: 不同阶段使用不同的轨迹参数和PID增益
// ============================================================================
enum class LiftPhase {
    HOLDING,       // 保持位置
    GENTLE_UP,     // -100→0 无负载: 前腿贴紧楼梯 / 下楼梯前腿搭边
    GENTLE_DOWN,   // 0→-100 无负载: 后轮轻放着陆 / 防止后腿拖地
    CLIMBING_UP,   // 0→510 有负载: 抬车身(450°后全车重)
    DESCENDING,    // 510→0 有负载: 降车身
};

// 各阶段轨迹参数 { v_max(°/s), a_max(°/s²) }
struct PhaseTrajectoryParams {
    float v_max;
    float a_max;
};

// 轨迹参数(仅 GENTLE_UP/DOWN 使用): v_max(°/s), a_max(°/s²)
// CLIMBING_UP/DESCENDING 不用轨迹, 直接怼目标+LIFT_SPEED_RAMP控加速
constexpr PhaseTrajectoryParams TRAJ_PARAMS[] = {
    {   0,    0 },  // HOLDING
    {  80,  200 },  // GENTLE_UP: -100→0 无负载, 速度前馈+位置PID修正
    {  70,  180 },  // GENTLE_DOWN: 0→-100 无负载, 轻柔着陆
    {   0,    0 },  // CLIMBING_UP: 不用轨迹
    {   0,    0 },  // DESCENDING: 不用轨迹
};

// 各阶段位置PID { Kp, Ki, Kd, MaxOut(RPM) }
// 重载阶段的力来自 Kp×满误差 瞬间饱和到 MaxOut, 积分补稳态
struct PhasePIDParams {
    float Kp, Ki, Kd, MaxOut;
};

constexpr PhasePIDParams POS_PID_PARAMS[] = {
    { 9.0f, 0.10f, 0.0f, 300.0f },  // HOLDING
    { 3.5f, 0.03f, 0.0f,  60.0f },  // GENTLE_UP: 无负载, 小Kp+轨迹前馈
    { 3.0f, 0.02f, 0.0f,  50.0f },  // GENTLE_DOWN: 无负载, 防触地冲击
    {12.0f, 0.25f, 0.0f, 250.0f },  // CLIMBING_UP: 重载, 大Kp×满误差→瞬间饱和
    { 7.0f, 0.06f, 0.0f, 260.0f },  // DESCENDING: 快速收腿, MaxOut↑ Ki↑
};

// ============================================================================
//  3508 控制模式
// ============================================================================
enum class Lift3508Mode {
    MANUAL,
    TARGETING,
};
static Lift3508Mode lift_3508_mode = Lift3508Mode::MANUAL;

// ============================================================================
//  状态变量
// ============================================================================
static LiftTrajectory lift_traj;
static LiftPhase lift_phase = LiftPhase::HOLDING;
static float prev_base_speed = 0.0f;

// 位置PID — IntegralLimit=50 RPM 防止积分饱和, 但允许足够的稳态出力

static float high_yaw_lock_ref = 0.0f;
static bool high_was_active = false;

// --------------------------------------------------------------------------
//  辅助函数
// --------------------------------------------------------------------------
static inline float normalizeDeg(float angle_deg) {
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

static inline bool liftIsLow();

// 根据当前位置和目标位置判断运动阶段
static LiftPhase determinePhase(float target, float current) {
    if (fabsf(current - target) <= LIFT_POS_TOLERANCE) {
        return LiftPhase::HOLDING;
    }

    if (target >= LIFT_HIGH_POS - 5.0f) {
        return LiftPhase::CLIMBING_UP;
    }

    if (target <= LIFT_LOW_POS + 5.0f) {
        if (current > 0.0f) {
            return LiftPhase::DESCENDING;
        } else if (current > LIFT_LOW_POS + 2.0f) {
            return LiftPhase::GENTLE_DOWN;
        } else {
            return LiftPhase::HOLDING;
        }
    }

    // 目标在0°附近 (轻着陆区)
    if (target < 5.0f && target > LIFT_LOW_POS + 5.0f) {
        return (current < target) ? LiftPhase::GENTLE_UP : LiftPhase::GENTLE_DOWN;
    }

    // 默认: 根据方向判断
    return (target > current) ? LiftPhase::CLIMBING_UP : LiftPhase::DESCENDING;
}

// 应用阶段参数到位置PID和轨迹规划器
static void applyPhaseParams(LiftPhase phase, float current, float target) {
    const auto &pidp = POS_PID_PARAMS[static_cast<int>(phase)];
    lift_3508_pos_pid.Kp = pidp.Kp;
    lift_3508_pos_pid.Ki = pidp.Ki;
    lift_3508_pos_pid.Kd = pidp.Kd;
    lift_3508_pos_pid.MaxOut = pidp.MaxOut;

    // 只有轻柔阶段用轨迹做位置参考, 爬楼/下楼直接怼目标
    if (phase == LiftPhase::GENTLE_UP || phase == LiftPhase::GENTLE_DOWN) {
        const auto &tp = TRAJ_PARAMS[static_cast<int>(phase)];
        lift_traj.plan(current, target, tp.v_max, tp.a_max);
    }
}

// --------------------------------------------------------------------------
//  初始化
// --------------------------------------------------------------------------
static inline void liftInit(void) {
    PID_Init(&lift_2006_motor1_pid);
    PID_Init(&lift_2006_motor2_pid);
    PID_Init(&lift_3508_motor1_pid);
    PID_Init(&lift_3508_motor2_pid);
    PID_Init(&lift_3508_pos_pid);
    PID_Init(&lift_3508_sync_pid);
    PID_Init(&high_yaw_lock_pid);
    photogate::init();

    lift_3508_motor1_pos = -lift_3508_motor1.getCurrentSumPos();
    lift_3508_motor2_pos = -lift_3508_motor2.getCurrentSumPos();
    lift_3508_avg_pos = (lift_3508_motor1_pos + lift_3508_motor2_pos) / 2.0f;
    lift_3508_diff_pos = lift_3508_motor1_pos - lift_3508_motor2_pos;

    lift_3508_target_pos =LIFT_LOW_POS; 
    lift_3508_hold_enable = true;
    lift_3508_manual_last = false;
    lift_phase = LiftPhase::HOLDING;

    if (!lift_cmd_sub.IsValid()) {
        return;
    }
}

// --------------------------------------------------------------------------
//  数据处理
// --------------------------------------------------------------------------
static inline void Lift_Data_Process(void) {
    // --- 解析指令 ---
    if (lift_cmd.request_high) {
        lift_3508_target_pos = LIFT_HIGH_POS;
        lift_3508_mode = Lift3508Mode::TARGETING;
        lift_3508_hold_enable = false;
        PID_Init(&lift_3508_pos_pid);
        // 根据当前位置和目标判断阶段并启动轨迹
        lift_phase = determinePhase(lift_3508_target_pos, lift_3508_avg_pos);
        applyPhaseParams(lift_phase, lift_3508_avg_pos, lift_3508_target_pos);
    } else if (lift_cmd.request_low) {
        lift_3508_target_pos = LIFT_LOW_POS;
        lift_3508_mode = Lift3508Mode::TARGETING;
        lift_3508_hold_enable = false;
        PID_Init(&lift_3508_pos_pid);
        lift_phase = determinePhase(lift_3508_target_pos, lift_3508_avg_pos);
        applyPhaseParams(lift_phase, lift_3508_avg_pos, lift_3508_target_pos);
    }
    lift_2006_speed = lift_cmd.lift_2006_input * MAX_LIFT_2006_SPEED;

    // --- 读取电机状态 ---
    lift_3508_motor1_pos = -lift_3508_motor1.getCurrentSumPos();
    lift_3508_motor2_pos = -lift_3508_motor2.getCurrentSumPos();
    lift_3508_motor1_speed = lift_3508_motor1.getRawCurrentSpeed();
    lift_3508_motor2_speed = lift_3508_motor2.getRawCurrentSpeed();

    lift_3508_avg_pos = (lift_3508_motor1_pos + lift_3508_motor2_pos) / 2.0f;
    lift_3508_diff_pos = lift_3508_motor1_pos - lift_3508_motor2_pos;

    const bool lift_3508_manual_active =
        (lift_cmd.lift_up && !lift_cmd.lift_down) ||
        (lift_cmd.lift_down && !lift_cmd.lift_up);

    // ===== 模式切换: 手动 / 自动定位 / 保持 =====
    if (lift_3508_mode == Lift3508Mode::TARGETING) {
        // --- 自动定位模式 ---
        if (lift_phase == LiftPhase::GENTLE_UP || lift_phase == LiftPhase::GENTLE_DOWN) {
            // 轻柔阶段: 轨迹位置参考 + 轨迹速度前馈(°/s→RPM)
            if (lift_traj.active()) {
                float traj_pos = lift_traj.update(0.001f);
                float traj_vel_rpm = lift_traj.refVelocity() / RPM_2_ANGLE_PER_SEC;
                float pos_pid_out = PID_Calculate(&lift_3508_pos_pid,
                                                  lift_3508_avg_pos, traj_pos);
                lift_3508_base_speed = pos_pid_out + traj_vel_rpm;
                lift_3508_hold_enable = false;
            } else {
                if (fabsf(lift_3508_avg_pos - lift_3508_target_pos) <= LIFT_POS_TOLERANCE) {
                    lift_3508_target_pos = lift_3508_avg_pos;
                    lift_3508_mode = Lift3508Mode::MANUAL;
                    lift_3508_hold_enable = true;
                    lift_phase = LiftPhase::HOLDING;
                    PID_Init(&lift_3508_pos_pid);
                } else {
                    float target_speed = PID_Calculate(&lift_3508_pos_pid,
                                                       lift_3508_avg_pos,
                                                       lift_3508_target_pos);
                    if (target_speed > 30.0f)  target_speed = 30.0f;
                    if (target_speed < -30.0f) target_speed = -30.0f;
                    lift_3508_base_speed = target_speed;
                }
            }
        } else {
            // 爬楼/下楼: PID直接怼目标(满误差→瞬间饱和出力), 接近后减速平滑刹停
            float target_speed = PID_Calculate(&lift_3508_pos_pid,
                                               lift_3508_avg_pos,
                                               lift_3508_target_pos);
            float err = fabsf(lift_3508_avg_pos - lift_3508_target_pos);
            float max_rpm = lift_3508_pos_pid.MaxOut;  // 阶段限速

            if (err < 50.0f) {
                // 接近目标: 速度按剩余距离线性递减, 确保平滑到位
                float limit = err * (max_rpm / 50.0f);
                if (limit < 20.0f) limit = 20.0f;  // 最低20 RPM, 保证能走完
                if (target_speed >  limit) target_speed =  limit;
                if (target_speed < -limit) target_speed = -limit;
            }
            // 远离目标: 不限速, PID可到MaxOut(300RPM), 跟原代码一样满力

            lift_3508_base_speed = target_speed;
            lift_3508_hold_enable = false;

            // 到达判定
            if (err <= LIFT_POS_TOLERANCE) {
                lift_3508_target_pos = lift_3508_avg_pos;
                lift_3508_mode = Lift3508Mode::MANUAL;
                lift_3508_hold_enable = true;
                lift_phase = LiftPhase::HOLDING;
                PID_Init(&lift_3508_pos_pid);
            }
        }

    } else if (lift_3508_manual_active) {
        // --- 手控模式 ---
        lift_phase = LiftPhase::HOLDING;
        if (lift_cmd.lift_up && !lift_cmd.lift_down) {
            lift_3508_base_speed = MAX_LIFT_3508_SPEED;
            lift_3508_hold_enable = false;
        } else if (lift_cmd.lift_down && !lift_cmd.lift_up) {
            lift_3508_base_speed = -MAX_LIFT_3508_SPEED;
            lift_3508_hold_enable = false;
        }

    } else {
        // --- 松手保持位置 ---
        if (lift_3508_manual_last) {
            lift_3508_target_pos = lift_3508_avg_pos;
            if (lift_3508_target_pos < LIFT_LOW_POS) {
                lift_3508_target_pos = LIFT_LOW_POS;
            }
            if (lift_3508_target_pos > LIFT_HIGH_POS) {
                lift_3508_target_pos = LIFT_HIGH_POS;
            }
            PID_Init(&lift_3508_pos_pid);
            PID_Init(&lift_3508_sync_pid);
            lift_3508_hold_enable = true;
            lift_phase = LiftPhase::HOLDING;
        }
        if (lift_3508_hold_enable) {
            // 保持模式也用位置PID, 但更新重力补偿
            lift_3508_pos_pid_out = PID_Calculate(&lift_3508_pos_pid,
                                                  lift_3508_avg_pos,
                                                  lift_3508_target_pos);
            lift_3508_base_speed = lift_3508_pos_pid_out;
        } else {
            lift_3508_base_speed = 0.0f;
        }
    }

    // --- 速度斜坡: 限制加速度防突变 ---
    {
        float max_delta = LIFT_SPEED_RAMP * 0.001f;
        float delta = lift_3508_base_speed - prev_base_speed;
        if (delta > max_delta)       lift_3508_base_speed = prev_base_speed + max_delta;
        else if (delta < -max_delta) lift_3508_base_speed = prev_base_speed - max_delta;
        prev_base_speed = lift_3508_base_speed;
    }

    // --- 左右同步 ---
    lift_3508_sync_pid_out =
        PID_Calculate(&lift_3508_sync_pid, lift_3508_diff_pos, 0.0f);

    lift_3508_motor1_ref_speed = lift_3508_base_speed + lift_3508_sync_pid_out;
    lift_3508_motor2_ref_speed = lift_3508_base_speed - lift_3508_sync_pid_out;

    lift_3508_manual_last = lift_3508_manual_active;
}

// --------------------------------------------------------------------------
//  主任务
// --------------------------------------------------------------------------
void liftTask(void *argument) {
    (void)argument;
    TickType_t currentTime;
    currentTime = xTaskGetTickCount();

    liftInit();

    for (;;) {
        photogate::update();
        if (lift_cmd_sub.TryGet(&lift_cmd)) {
        }
        if (high_nav_sub.TryGet(&high_nav_cmd)) {
        }
        Lift_Data_Process();

        // ===== Phase 2: 2006 速度控制 =====
        float high_forward = 0.0f;
        float high_omega = 0.0f;

        const bool high_nav_active =
            high_nav_cmd.active &&
            (nav_control::auto_enabled || high_nav_cmd.allow_without_auto);

        if (high_nav_active) {
            high_forward = -high_nav_cmd.forward_speed;
            const float yaw_error =
                normalizeDeg(static_cast<float>(nav_control::target_yaw) -
                             g_chassis_yaw_deg);
            high_omega = PID_Calculate(&high_yaw_lock_pid, 0.0f, yaw_error);
        } else if (nav_control::high_mode_active) {
            high_forward = lift_cmd.lift_2006_input * 500.0f;
            const float yaw_error =
                normalizeDeg(high_yaw_lock_ref - g_chassis_yaw_deg);
            high_omega = PID_Calculate(&high_yaw_lock_pid, 0.0f, yaw_error);
        }

        const float motor1_ref = high_forward * LIFT_2006_MOTOR1_DIR - high_omega;
        const float motor2_ref = high_forward * LIFT_2006_MOTOR2_DIR - high_omega;

        lift_2006_motor1_pid_out =
            PID_Calculate(&lift_2006_motor1_pid,
                          lift_2006_motor1.getRawCurrentSpeed(),
                          motor1_ref);
        lift_2006_motor2_pid_out =
            PID_Calculate(&lift_2006_motor2_pid,
                          lift_2006_motor2.getRawCurrentSpeed(),
                          motor2_ref);

        // ===== high_mode_active 自动管理 =====
        const bool is_high = liftIsHigh();
        const bool is_low = liftIsLow();
        if (is_high && !high_was_active) {
            nav_control::high_mode_active = true;
            high_yaw_lock_ref = g_chassis_yaw_deg;
            PID_Init(&high_yaw_lock_pid);
            pc_nav_event_t evt{static_cast<uint16_t>(0x0202)};
            pc_nav_event_pub.Publish(evt);
            high_was_active = true;
        } else if (is_low && high_was_active) {
            nav_control::high_mode_active = false;
            pc_nav_event_t evt{static_cast<uint16_t>(0x0203)};
            pc_nav_event_pub.Publish(evt);
            high_was_active = false;
        }

        // ===== 响应自动导航到达后降位请求 =====
        if (high_nav_cmd.request_lower && is_high) {
            high_nav_cmd.request_lower = false;
            liftRequestLow();
        }

        // ===== 3508 速度PID + 重力补偿 =====
        lift_3508_motor1_pid_out =
            PID_Calculate(&lift_3508_motor1_pid,
                          lift_3508_motor1_speed,
                          lift_3508_motor1_ref_speed * LIFT_3508_MOTOR1_DIR);
        lift_3508_motor2_pid_out =
            PID_Calculate(&lift_3508_motor2_pid,
                          lift_3508_motor2_speed,
                          lift_3508_motor2_ref_speed * LIFT_3508_MOTOR2_DIR);

        // --- 设置电机指令 ---
        lift_2006_motor1.setMotorCmd(lift_2006_motor1_pid_out);
        lift_2006_motor2.setMotorCmd(lift_2006_motor2_pid_out);
        lift_3508_motor1.setMotorCmd(lift_3508_motor1_pid_out);
        lift_3508_motor2.setMotorCmd(lift_3508_motor2_pid_out);

        vTaskDelayUntil(&currentTime, 1);
    }
}

// --------------------------------------------------------------------------
//  对外接口
// --------------------------------------------------------------------------
void liftRequestHigh() {
    lift_3508_target_pos = LIFT_HIGH_POS;
    lift_3508_mode = Lift3508Mode::TARGETING;
    lift_3508_hold_enable = false;
    lift_phase = determinePhase(lift_3508_target_pos, lift_3508_avg_pos);
    applyPhaseParams(lift_phase, lift_3508_avg_pos, lift_3508_target_pos);
    PID_Init(&lift_3508_pos_pid);
    PID_Init(&lift_3508_motor1_pid);
    PID_Init(&lift_3508_motor2_pid);
    PID_Init(&lift_3508_sync_pid);
}

void liftRequestLow() {
    lift_3508_target_pos = LIFT_LOW_POS;
    lift_3508_mode = Lift3508Mode::TARGETING;
    lift_3508_hold_enable = false;
    lift_phase = determinePhase(lift_3508_target_pos, lift_3508_avg_pos);
    applyPhaseParams(lift_phase, lift_3508_avg_pos, lift_3508_target_pos);
    PID_Init(&lift_3508_pos_pid);
    PID_Init(&lift_3508_motor1_pid);
    PID_Init(&lift_3508_motor2_pid);
    PID_Init(&lift_3508_sync_pid);
}

bool liftAtTarget() {
    return (lift_3508_mode == Lift3508Mode::MANUAL);
}

float liftCurrentPos() {
    return lift_3508_avg_pos;
}

static inline bool liftIsLow() {
    return (fabsf(lift_3508_avg_pos - LIFT_LOW_POS) <= 10.0f);
}

bool liftIsHigh() {
    return (fabsf(lift_3508_avg_pos - LIFT_HIGH_POS) <= 30.0f);
}

bool highModeActive() {
    return nav_control::high_mode_active;
}
