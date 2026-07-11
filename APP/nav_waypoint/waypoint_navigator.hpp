#pragma once

#include <cstdint>

void stairWaypointGoToFront();
void stairWaypointRunUp();
void stairWaypointRunUpR1();
void stairWaypointRunDown();
void stairWaypointRunGoToEdge();
void stairWaypointRunReturnToCenter();
bool stairWaypointCanUsePoseAnchoredTurn();
bool stairWaypointRotateByYawDelta(int16_t delta_yaw_deg);

extern volatile int32_t g_ozone_xbox_target_x;
extern volatile int32_t g_ozone_xbox_target_y;
extern volatile int32_t g_ozone_xbox_target_yaw;

uint8_t stairWaypointStep();
uint8_t stairWaypointLevel();
bool stairWaypointArmed();
