#ifndef TAIL_CLAW_CLASS_VERSION_TASK_HPP
#define TAIL_CLAW_CLASS_VERSION_TASK_HPP

#include "tail_claw_controller.hpp"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

void tail_claw_task(void *argument);

void tail_claw_publish_cmd(const pub_tail_claw_cmd& cmd);
void tail_claw_set_mode(TailClawMode mode);
void tail_claw_set_move_target(float cm);
void tail_claw_set_roll_target(float deg);
void tail_claw_set_weapon_claw(bool close);
void tail_claw_set_air_pump(bool on);
void tail_claw_reset_match(void);

#endif  
