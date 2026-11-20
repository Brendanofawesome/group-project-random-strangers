
#pragma once

#include <Arduino.h>

//Initialize homing switch and state
void initializeHoming();

//Check for home switch activation
void updateHoming();

//Check whether home has been established
bool isHomed();

//Get the recorded home offset
float getHomeOffset();

//Convert measured motor angle to homed angle
float getHomedAngle(float motorAngle);

//Reset homing calibration
void resetHoming();
