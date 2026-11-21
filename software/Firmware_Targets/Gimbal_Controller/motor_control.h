//Exposes the SimpleFOC sensor monitoring functions
//to gimbal.ino
#include "config.h"
#include <SimpleFOC.h>

#pragma once

// Configure the SimpleFOC sensor interface.
// If commander interface is used, pass it into this function to register the motor
// Expects caller to manage and update commander.
bool initializeMotorControl(Sensor* sensor, Commander* commander = nullptr);

// Update the SimpleFOC motor control system. Should be called continuously in the main loop.
void updateMotorControl();

// Get the current angle from the motor control system.
float getMotorCurrentAngle();

// Set the target angle for the motor control system.
void setMotorTargetAngle(const float& angle);

// Get the target angle for the motor control system
float getMotorTargetAngle();

// Get a reference to the motor (for debugging)
BLDCMotor* getMotor();