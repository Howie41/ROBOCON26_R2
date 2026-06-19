#ifndef TAIL_CLAW_CLASS_VERSION_ENABLE
#define TAIL_CLAW_CLASS_VERSION_ENABLE 1
#endif

#if TAIL_CLAW_CLASS_VERSION_ENABLE

#include "tail_claw_task.hpp"

#include "Motor.hpp"

osThreadId_t tail_claw_TaskHandle;

extern C610Motor tail_claw_move_motor;
extern C620Motor tail_claw_roll_motor;

namespace {

TypedTopicPublisher<pub_tail_claw_cmd> tail_claw_cmd_pub("tail_claw_cmd");

}  // namespace

void tail_claw_publish_cmd(const pub_tail_claw_cmd& cmd)
{
    tail_claw_cmd_pub.Publish(cmd);
}

void tail_claw_set_mode(TailClawMode mode)
{
    pub_tail_claw_cmd cmd{};
    cmd.set_mode = true;
    cmd.mode = mode;
    tail_claw_publish_cmd(cmd);
}

void tail_claw_set_move_target(float cm)
{
    pub_tail_claw_cmd cmd{};
    cmd.set_move_target = true;
    cmd.move_target_cm = cm;
    tail_claw_publish_cmd(cmd);
}

void tail_claw_set_roll_target(float deg)
{
    pub_tail_claw_cmd cmd{};
    cmd.set_roll_target = true;
    cmd.roll_target_deg = deg;
    tail_claw_publish_cmd(cmd);
}

void tail_claw_set_weapon_claw(bool close)
{
    pub_tail_claw_cmd cmd{};
    cmd.set_weapon_claw = true;
    cmd.weapon_claw_close = close;
    tail_claw_publish_cmd(cmd);
}


void tail_claw_set_air_pump(bool on)
{
    pub_tail_claw_cmd cmd{};
    cmd.set_air_pump = true;
    cmd.air_pump_on = on;
    tail_claw_publish_cmd(cmd);
}

void tail_claw_reset_match(void)
{
    pub_tail_claw_cmd cmd{};
    cmd.reset_match = true;
    tail_claw_publish_cmd(cmd);
}

void tail_claw_task(void *argument)
{
    (void)argument;

    TickType_t currentTime = xTaskGetTickCount();
    TailClawController::Instance().Init(tail_claw_move_motor,
                                        tail_claw_roll_motor);

    for (;;) {
        TailClawController::Instance().Tick1ms();
        vTaskDelayUntil(&currentTime, 1);
    }
}

#endif
