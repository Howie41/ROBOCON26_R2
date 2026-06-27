#pragma once

#include <cstdint>

void stairWaypointGoToFront();
void stairWaypointRunUp();
void stairWaypointRunUpR1();
void stairWaypointRunDown();
void stairWaypointRunGoToEdge();
void stairWaypointRunReturnToCenter();

uint8_t stairWaypointStep();
uint8_t stairWaypointLevel();
bool stairWaypointArmed();
