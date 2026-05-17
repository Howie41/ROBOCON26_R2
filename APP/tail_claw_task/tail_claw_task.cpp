#include "tail_claw_task.hpp"
#include "Motor.hpp"
#include "pid_controller.h"
#include <cstdint>

#include "control_task.h"
#include"topic_pool.h"
osThreadId_t tail_claw_TaskHandle;;

extern C610Motor tail_claw_move_motor;
extern C620Motor tail_claw_roll_motor;

extern pub_Xbox_Data control_xbox_cmd;
bool weapon_claw_open = false;//武器气泵的夹紧，ture 为吸，false为放
bool KFS_claw_open = false;//KFS的夹紧，ture 为吸，false为放
uint8_t weapon_match_state_ = 0x00;//武器在配对过程中上位机发的信号

float tail_claw_move_target_pos= 0.0f;
float tail_claw_roll_target_pos= 0.0f;

PID_t tail_claw_move_pos_pid={
    .Kp = 0.16f,
    .Ki = 0.0008f,
    .Kd = 0.001f,
    .MaxOut = 1.5f,
    .IntegralLimit = 0.35f,
    .DeadBand = 0.3f,
    .Improve = NONE,
};

PID_t tail_claw_move_speed_pid={
    .Kp = 0.16f,
    .Ki = 0.0008f,
    .Kd = 0.001f,
    .MaxOut = 1.5f,
    .IntegralLimit = 0.35f,
    .DeadBand = 0.3f,
    .Improve = NONE,
};

PID_t tail_claw_roll_pos_pid={
    .Kp = 0.16f,
    .Ki = 0.0008f,
    .Kd = 0.001f,
    .MaxOut = 1.5f,
    .IntegralLimit = 0.35f,
    .DeadBand = 0.3f,
    .Improve = NONE,
};

PID_t tail_claw_roll_speed_pid={
    .Kp = 0.16f,
    .Ki = 0.0008f,
    .Kd = 0.001f,
    .MaxOut = 1.5f,
    .IntegralLimit = 0.35f,
    .DeadBand = 0.3f,
    .Improve = NONE,
};

void tail_claw_init()
{ 
    PID_Init(&tail_claw_move_pos_pid);
    PID_Init(&tail_claw_move_speed_pid);
    PID_Init(&tail_claw_roll_speed_pid);
    PID_Init(&tail_claw_roll_pos_pid);
}
void weapon_open(bool open){
    if(open) weapon_claw_open=true;
    else weapon_claw_open=false;
}

void KFS_open(bool open){
    if(open) KFS_claw_open=true;
    else KFS_claw_open=false;
}

float set_move_pos(float pos,PID_t *pos_pid,PID_t *speed_pid)
{
    float speed_cmd=PID_Calculate(pos_pid,pos,tail_claw_move_motor.getCurrentSinglePos());
    return PID_Calculate(speed_pid,speed_cmd,tail_claw_move_motor.getCurrentSpeed());
}

float set_roll_pos(float pos,PID_t *pos_pid,PID_t *speed_pid)
{
    float speed_cmd=PID_Calculate(pos_pid,tail_claw_roll_motor.getCurrentSinglePos(),pos);
    return PID_Calculate(speed_pid,speed_cmd,tail_claw_roll_motor.getCurrentSpeed());
}

//由于没有上位机，此处先以xbox来代替 
void get_weapon_match_state()
{   
    
    //左移和右移
    if(control_xbox_cmd.btnDirLeft)
    {
        weapon_match_state_ = (weapon_match_state_ & ~motor_move_right) | motor_move_left;
    }
    else if(control_xbox_cmd.btnDirDown)
    {
        weapon_match_state_ = (weapon_match_state_ & ~motor_move_left) | motor_move_right;
    }else
    {
        weapon_match_state_ = weapon_match_state_ & ~(motor_move_left | motor_move_right);
    }

    //上下翻滚
    if(control_xbox_cmd.btnDirUp)
    {
        weapon_match_state_ = (weapon_match_state_ & ~motor_roll_down) | motor_roll_up;
    }
    else if(control_xbox_cmd.btnDirDown)
    {
        weapon_match_state_ = (weapon_match_state_ & ~motor_roll_up) | motor_roll_down;
    }else
    {
        weapon_match_state_ = weapon_match_state_ & ~(motor_roll_down | motor_roll_up);
    }

    //底盘顺逆时针转动
     if(control_xbox_cmd.btnLB)
    {
        weapon_match_state_ = (weapon_match_state_ & ~chassis_contrarotate) | chassis_clockwise_rotation;
    }
    else if(control_xbox_cmd.btnRB)
    {
        weapon_match_state_ = (weapon_match_state_ & ~chassis_clockwise_rotation) | chassis_contrarotate;
    }else
    {
        weapon_match_state_ = weapon_match_state_ & ~(chassis_clockwise_rotation |chassis_contrarotate);
    }
}

void tail_claw_move_close()
{ 
    const float move_step = 0.2f;
    const float roll_step = 0.2f;

    if(weapon_match_state_&motor_move_left)
    {
        tail_claw_move_target_pos -= move_step;
    }else if(weapon_match_state_&motor_move_right) {
        tail_claw_move_target_pos += move_step;
    }

    
    if(weapon_match_state_&motor_roll_down)
    {
        tail_claw_roll_target_pos -= move_step;
    }else if(weapon_match_state_&motor_roll_up) {
        tail_claw_roll_target_pos += move_step;
    }

    float move_cmd = set_move_pos(tail_claw_move_target_pos,
                                &tail_claw_move_pos_pid,
                                &tail_claw_move_speed_pid);
    
    float roll_cmd = set_roll_pos(tail_claw_roll_target_pos,
                                &tail_claw_roll_pos_pid,
                                &tail_claw_roll_speed_pid);
    
    tail_claw_move_motor.setMotorCmd(move_cmd);
    tail_claw_roll_motor.setMotorCmd(roll_cmd);
}  
void tail_claw_task(void *argument) {
    TickType_t currentTime = xTaskGetTickCount();
    tail_claw_init();
    for(;;)
    {
        get_weapon_match_state();
        tail_claw_move_close();
        
        vTaskDelayUntil(&currentTime, 1);
    }
}


