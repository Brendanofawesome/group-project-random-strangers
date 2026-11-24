
#pragma once

#include <Arduino.h>
#include <simpleFOC.h>

//Initialize homing switch and state
void initializeHomingSensor();

//Home the motor and return the absolute angle offset
//  Assumes the motor is already initialized and it has a
//  configured sensor.
bool home(BLDCMotor* motor);

//Check whether home has been established
bool isHomed();

//Get the recorded home offset
float getHomeOffset();

//Reset homing calibration
void resetHoming();
