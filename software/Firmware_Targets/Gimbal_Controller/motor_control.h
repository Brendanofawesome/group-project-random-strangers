//Exposes the SimpleFOC sensor monitoring functions
//to gimbal.ino
#include "config.h"
#include <SimpleFOC.h>

#pragma once

// Configure the SimpleFOC sensor interface.
bool initializeMotorControl(Sensor* sensor);

// Update the SimpleFOC motor control system. Should be called continuously in the main loop.
void updateMotorControl();

// Get the current angle from the motor control system.
float getMotorCurrentAngle();

// Set the target angle for the motor control system.
void setMotorTargetAngle(const float& angle);
