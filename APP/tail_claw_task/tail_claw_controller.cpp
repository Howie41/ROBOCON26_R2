#include "pid_controller.h"
#ifndef TAIL_CLAW_CLASS_VERSION_ENABLE
#define TAIL_CLAW_CLASS_VERSION_ENABLE 1
#endif

#if TAIL_CLAW_CLASS_VERSION_ENABLE

#include "tail_claw_controller.hpp"

#include "stm32h7xx_hal_gpio.h"

#include <cmath>

namespace {

constexpr float roll_reduction_ratio = 2.5f;
constexpr float move_max_distance = 16.0f;
constexpr float move_degree_per_cm = 360.0f / (3.0f * 3.1415926f);

constexpr float move_step = 0.005f;
constexpr float roll_step = 0.3f;

constexpr int16_t match_enter_threshold = 7;
constexpr int16_t match_exit_threshold = 9;
constexpr uint8_t match_ok_count_limit = 0;
constexpr uint8_t match_lost_count_limit = 3;

constexpr TickType_t distance_timeout_ticks = pdMS_TO_TICKS(200);

constexpr float move_pos_tolerance = 2.0f;
constexpr float move_speed_tolerance = 5.0f;
constexpr float roll_pos_tolerance = 2.0f;
constexpr float roll_speed_tolerance = 5.0f;

//按键上升沿
static bool button_rising_edge(bool current_state, bool* last_state)
{
    const bool rising_edge = current_state && !(*last_state);
    *last_state = current_state;
    return rising_edge;
}

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

}  // namespace

TailClawController& TailClawController::Instance()
{
    static TailClawController instance;
    return instance;
}

void TailClawController::Init(C610Motor& move_motor, C620Motor& roll_motor)
{
    move_motor_ = &move_motor;
    roll_motor_ = &roll_motor;
    init_pid();
    ResetMatchState(); 
}

void TailClawController::init_pid()
{
    move_pos_pid_ = {};
    move_pos_pid_.Kp = 0.05f;
    move_pos_pid_.Ki = 0.03f;
    move_pos_pid_.Kd = 0.5f;
    move_pos_pid_.MaxOut = 5.0f;
    move_pos_pid_.IntegralLimit = 0.35f;
    move_pos_pid_.DeadBand = 0.3f;
    move_pos_pid_.Improve = NONE;

    move_speed_pid_ = {};
    move_speed_pid_.Kp = 400.0f;
    move_speed_pid_.Ki = 0.03f;
    move_speed_pid_.Kd = 0.02f;
    move_speed_pid_.MaxOut = 4000.0f;
    move_speed_pid_.IntegralLimit = 0.35f;
    move_speed_pid_.DeadBand = 0.3f;
    move_speed_pid_.Improve = NONE;

    roll_pos_pid_ = {};
    roll_pos_pid_.Kp = 40.0f;
    roll_pos_pid_.Ki = 0.0f;
    roll_pos_pid_.Kd = 8.0f;
    roll_pos_pid_.MaxOut = 100.0f;
    roll_pos_pid_.DeadBand = 0.3f;
    roll_pos_pid_.Improve = NONE;

    roll_speed_pid_ = {};
    roll_speed_pid_.Kp = 70.0f;
    roll_speed_pid_.Ki = 0.4;
    roll_speed_pid_.Kd = 1.0f;
    roll_speed_pid_.MaxOut = 3000.0f;
    roll_speed_pid_.DeadBand = 0.3f;
    roll_speed_pid_.Improve = NONE;

    roll_heigh_pos_pid_ = {};
    roll_heigh_pos_pid_.Kp = 70.0f;
    roll_heigh_pos_pid_.Ki = 0.0f;
    roll_heigh_pos_pid_.Kd = 1.0f;
    roll_heigh_pos_pid_.MaxOut = 100.0f;
    roll_heigh_pos_pid_.DeadBand = 0.3f;
    roll_heigh_pos_pid_.Improve = NONE;

    roll_heigh_speed_pid_ = {};
    roll_heigh_speed_pid_.Kp = 110.0f;
    roll_heigh_speed_pid_.Ki = 0.4;
    roll_heigh_speed_pid_.Kd = 0.6f;
    roll_heigh_speed_pid_.IntegralLimit=1000.0f;
    roll_heigh_speed_pid_.MaxOut = 5000.0f;
    roll_heigh_speed_pid_.DeadBand = 0.3f;
    roll_heigh_speed_pid_.Improve = Integral_Limit;

    PID_Init(&move_pos_pid_);
    PID_Init(&move_speed_pid_);
    PID_Init(&roll_pos_pid_);
    PID_Init(&roll_speed_pid_);
    PID_Init(&roll_heigh_speed_pid_);
    PID_Init(&roll_heigh_pos_pid_);
}
void TailClawController::Tick1ms()
{
    consume_commands();
    consume_pc_distance();
    consume_xbox_data();
    update_button_toggle();

    if (mode_ == TailClawMode::Disabled) {
        motion_bits_ = none;
        if (move_motor_ != nullptr) {
            move_motor_->setMotorCmd(0.0f);
        }
        if (roll_motor_ != nullptr) {
            roll_motor_->setMotorCmd(0.0f);
        }
        publish_status();
        return;
    }

    update_control_bits();
    update_target_by_control_bits();
    apply_gpio();
    apply_motor_output();
    publish_status();
}

void TailClawController::HandleCommand(const pub_tail_claw_cmd& cmd)
{
    if (cmd.set_mode) {
        SetMode(cmd.mode);
    }
    if (cmd.set_move_target) {
        SetMoveTarget(cmd.move_target_cm);
    }
    if (cmd.set_roll_target) {
        SetRollTarget(cmd.roll_target_deg);
    }
    if (cmd.set_weapon_claw) {
        SetWeaponClaw(cmd.weapon_claw_close);
    }
    if (cmd.set_air_pump) {
        SetAirPump(cmd.air_pump_on);
    }
    if (cmd.reset_match) {
        ResetMatchState();
    }
}

void TailClawController::OnPcDistance(int16_t distance)
{
    last_distance_ = distance;
    has_last_distance_ = true;
    last_distance_tick_ = xTaskGetTickCount();
}

void TailClawController::SetMode(TailClawMode mode)
{
    if (mode_ == mode) {
        return;
    }
    if (mode == TailClawMode::Hold) {
        lock_current_position();
    }
    if (mode == TailClawMode::AutoAlign) {
        ResetMatchState();
    }

    mode_ = mode;
}

void TailClawController::SetMoveTarget(float cm)
{
    move_target_cm_ = clamp_float(cm, 0.0f, move_max_distance);
}

void TailClawController::SetRollTarget(float deg)
{
    roll_target_deg_ = deg;
}

void TailClawController::SetWeaponClaw(bool close)
{
    weapon_claw_open_ = close;
}


void TailClawController::SetAirPump(bool on)
{
    air_pump_on_ = on;
}

void TailClawController::ResetMatchState()
{
    match_ok_count_ = 0;
    match_lost_count_ = 0;
    weapon_matched_stable_ = false;
    motion_bits_ &= static_cast<uint8_t>(~ismatch);
}

pub_tail_claw_status TailClawController::GetStatus() const
{
    pub_tail_claw_status status{};
    status.mode = mode_;
    status.motion_bits = motion_bits_;
    status.weapon_matched = weapon_matched_stable_;
    status.weapon_claw_closed = weapon_claw_open_;
    status.air_pump_on = air_pump_on_;
    status.move_target_cm = move_target_cm_;
    status.roll_target_deg = roll_target_deg_;
    update_arrived_status(&status);
    return status;
}

void TailClawController::consume_commands()
{
    pub_tail_claw_cmd cmd{};
    while (tail_claw_cmd_sub_.TryGet(&cmd)) {
        HandleCommand(cmd);
    }
}

void TailClawController::consume_pc_distance()
{
    tail_claw_msg msg{};
    while (pc_tail_claw_sub_.TryGet(&msg)) {
        OnPcDistance(msg.distance);
    }
}

void TailClawController::consume_xbox_data()
{
    pub_Xbox_Data msg{};
    while (xbox_sub_.TryGet(&msg)) {
        xbox_cmd_ = msg;
    }
}

void TailClawController::update_control_bits()
{
    motion_bits_ &= ismatch;

    if (mode_ == TailClawMode::AutoAlign) {
        tail_claw_msg msg{last_distance_};
        update_manual_control();
        update_auto_align(is_distance_fresh() ? &msg : nullptr);
        return;
    }

    if (mode_ == TailClawMode::Manual) {
        update_manual_control();
    }
}

void TailClawController::update_auto_align(const tail_claw_msg* msg)
{
    if (msg == nullptr || xbox_cmd_.btnDirLeft || xbox_cmd_.btnDirRight) {
        if (!is_distance_fresh()) {
            motion_bits_ &= static_cast<uint8_t>(~ismatch);
        }
        return;
    }

    if (msg->distance < -match_enter_threshold) {
        motion_bits_ = (motion_bits_ & ~motor_move_right) | motor_move_left;

        if (match_lost_count_ < match_lost_count_limit) {
            match_lost_count_++;
        }

        if (match_lost_count_ >= match_lost_count_limit) {
            weapon_matched_stable_ = false;
            if(match_ok_count_ > 0)match_ok_count_ --;
            motion_bits_ &= static_cast<uint8_t>(~ismatch);
        }
        return;
    }

    if (msg->distance > match_enter_threshold) {
        motion_bits_ = (motion_bits_ & ~motor_move_left) | motor_move_right;

        if (msg->distance > match_exit_threshold &&
            match_lost_count_ < match_lost_count_limit) {
            match_lost_count_++;
        }

        if (match_lost_count_ >= match_lost_count_limit) {
            weapon_matched_stable_ = false;
             if(match_ok_count_ > 0)match_ok_count_ --;
            motion_bits_ &= static_cast<uint8_t>(~ismatch);
        }
        return;
    }

    motion_bits_ &= static_cast<uint8_t>(~(motor_move_left | motor_move_right));
    if (match_ok_count_ < match_ok_count_limit) {
        match_ok_count_++;
    }
    match_lost_count_ = 0;

    if (match_ok_count_ >= match_ok_count_limit) {
        weapon_matched_stable_ = true;
        motion_bits_ |= ismatch;
    }
}

void TailClawController::update_manual_control()
{
    if (xbox_cmd_.btnDirLeft) {
        motion_bits_ = (motion_bits_ & ~motor_move_right) | motor_move_left;
    } else if (xbox_cmd_.btnDirRight) {
        motion_bits_ = (motion_bits_ & ~motor_move_left) | motor_move_right;
    } else {
        motion_bits_ &= static_cast<uint8_t>(~(motor_move_left | motor_move_right));
    }

    if (xbox_cmd_.btnDirUp) {
        motion_bits_ = (motion_bits_ & ~motor_roll_down) | motor_roll_up;
    } else if (xbox_cmd_.btnDirDown) {
        motion_bits_ = (motion_bits_ & ~motor_roll_up) | motor_roll_down;
    } else {
        motion_bits_ &= static_cast<uint8_t>(~(motor_roll_up | motor_roll_down));
    }
}

void TailClawController::update_button_toggle()
{
    if (button_rising_edge(xbox_cmd_.btnShare, &btn_share_last_)) {
        weapon_claw_open_ = !weapon_claw_open_;
    }

    if (button_rising_edge(xbox_cmd_.btnMenu, &btn_menu_last_)) {
        air_pump_on_ = !air_pump_on_;
    }
}

void TailClawController::update_target_by_control_bits()
{
    if (motion_bits_ & motor_move_left) {
        SetMoveTarget(move_target_cm_ - move_step);
    } else if (motion_bits_ & motor_move_right) {
        SetMoveTarget(move_target_cm_ + move_step);
    }

    if (motion_bits_ & motor_roll_down) {
        roll_target_deg_ -= roll_step;
    } else if (motion_bits_ & motor_roll_up) {
        roll_target_deg_ += roll_step;
    }
}

void TailClawController::apply_gpio()
{
    HAL_GPIO_WritePin(GPIOG,
                      GPIO_PIN_4,
                      weapon_claw_open_ ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void TailClawController::apply_motor_output()
{
    if (move_motor_ != nullptr) {
        move_motor_->setMotorCmd(calcMoveCmd(move_target_cm_));
    }

    if (roll_motor_ != nullptr) {
        roll_motor_->setMotorCmd(calcRollCmd(roll_target_deg_));
    }
}

void TailClawController::publish_status()
{
    status_publish_count_++;
    if (status_publish_count_ < 10U) {
        return;
    }
    status_publish_count_ = 0U;

    const pub_tail_claw_status status = GetStatus();
    tail_claw_status_pub_.Publish(status);
}

bool TailClawController::is_distance_fresh() const
{
    if (!has_last_distance_) {
        return false;
    }
    return (xTaskGetTickCount() - last_distance_tick_) < distance_timeout_ticks;
}

void TailClawController::lock_current_position()
{
    if (move_motor_ != nullptr) {
        move_target_cm_ = clamp_float(move_motor_->getCurrentSumPos() / move_degree_per_cm,
                                      0.0f,
                                      move_max_distance);
    }

    if (roll_motor_ != nullptr) {
        roll_target_deg_ = roll_motor_->getCurrentSumPos() / roll_reduction_ratio;
    }
}

void TailClawController::update_arrived_status(pub_tail_claw_status* status) const
{
    if (status == nullptr) {
        return;
    }

    if (move_motor_ != nullptr) {
        const float target_pos = status->move_target_cm * move_degree_per_cm;
        status->move_arrived =
            (std::fabs(move_motor_->getCurrentSumPos() - target_pos) < move_pos_tolerance) &&
            (std::fabs(move_motor_->getCurrentSpeed()) < move_speed_tolerance);
    }

    if (roll_motor_ != nullptr) {
        const float target_pos = status->roll_target_deg * roll_reduction_ratio;
        status->roll_arrived =
            (std::fabs(roll_motor_->getCurrentSumPos() - target_pos) < roll_pos_tolerance) &&
            (std::fabs(roll_motor_->getCurrentSpeed()) < roll_speed_tolerance);
    }
}

float TailClawController::calcMoveCmd(float target_cm)
{
    if (move_motor_ == nullptr) {
        return 0.0f;
    }

    const float clamped_target = clamp_float(target_cm, 0.0f, move_max_distance);
    const float target_degree = clamped_target * move_degree_per_cm;
    const float speed_cmd = PID_Calculate(&move_pos_pid_,
                                          move_motor_->getCurrentSumPos(),
                                          target_degree);
    return PID_Calculate(&move_speed_pid_,
                         move_motor_->getCurrentSpeed(),
                         speed_cmd);
}

float TailClawController::calcRollCmd(float target_deg)
{
    if (roll_motor_ == nullptr) {
        return 0.0f;
    }

    const float target_pos = target_deg * roll_reduction_ratio;
    if (target_deg - roll_motor_->getCurrentSumPos() >= 0.0f) {
        // 正转方向：使用 roll_pos_pid_ / roll_speed_pid_
        if (roll_last_direction_ != 1) {
            // 刚从反转/初始切换到正转，清零正转 PID 的冻结积分，防止突变
            PID_Reset(&roll_pos_pid_);
            PID_Reset(&roll_speed_pid_);
            roll_last_direction_ = 1;
        }
        const float speed_cmd = PID_Calculate(&roll_pos_pid_,
                                          roll_motor_->getCurrentSumPos(),
                                          target_pos);
        return PID_Calculate(&roll_speed_pid_,
                         roll_motor_->getCurrentSpeed(),
                         speed_cmd);
    } else {
        // 反转方向：使用 roll_heigh_pos_pid_ / roll_heigh_speed_pid_
        if (roll_last_direction_ != -1) {
            // 刚从正转/初始切换到反转，清零反转 PID 的冻结积分，防止突变
            PID_Reset(&roll_heigh_pos_pid_);
            PID_Reset(&roll_heigh_speed_pid_);
            roll_last_direction_ = -1;
        }
        const float speed_cmd = PID_Calculate(&roll_heigh_pos_pid_,
                                          roll_motor_->getCurrentSumPos(),
                                          target_pos);
        return PID_Calculate(&roll_heigh_speed_pid_,
                         roll_motor_->getCurrentSpeed(),
                         speed_cmd);
    }
}

#endif
