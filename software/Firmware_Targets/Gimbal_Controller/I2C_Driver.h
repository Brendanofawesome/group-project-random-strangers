
#pragma once

#include <stdint.h>
// Function declarations for the I2C driver

// Initialize ESP32 as an I2C target
// Returns true if initialization is successful, false otherwise
bool initialize_i2c();

// Update the current angle from the sensor
// angle: the current angle measured by the sensor, in radians
void setI2CCurrentAngle(const float& angle);

// Get the current target angle set by the Jetson
// Returns the target angle in radians
float getI2CTargetAngle();

// Update the initialization-ready bit
void setReady(bool ready);

