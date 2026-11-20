#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "I2C_Driver.h"
#include "hall_sensor.h"
#include "motor_control.h"
#include "homing.h"

// Gimbal Controller main program
// Two main functions: setup() and loop()
// setup() is called once when ESP32 starts
// loop() is called continuously to read sensor angle,
// update registers, and check status

// Status values for the ESP32
constexpr uint8_t STATUS_NOT_READY = 0;
constexpr uint8_t STATUS_READY = 1;
constexpr uint8_t STATUS_SENSOR_ERROR = 2;

// Called once when the ESP32 starts up
void setup() {

    // Start serial communication for debugging
    Serial.begin(115200);

    Serial.println("Starting Gimbal ESP32...");

    // Initialize I2C communication
    if (!initialize_i2c()) {

        Serial.println("I2C initialization failed!");

        // Stop here because communication failed
        while (true) {
            delay(1000);
        }
    }

    Serial.println("I2C initialized successfully!");

    // Set status to NOT_READY until homing finishes
    setStatus(STATUS_NOT_READY);

    // Initialize analog Hall sensors
    if (!initializeHallSensor()) {

        Serial.println("Hall sensor initialization failed!");

        setStatus(STATUS_SENSOR_ERROR);

        while (true) {
            delay(1000);
        }
    }

    Serial.println("Hall sensor initialized successfully!");

    // Initialize SimpleFOC sensor monitoring
    if (!initializeMotorControl()) {

        Serial.println("Motor monitoring initialization failed!");

        setStatus(STATUS_SENSOR_ERROR);

        while (true) {
            delay(1000);
        }
    }

    Serial.println("Motor monitoring initialized successfully!");

    // Initialize homing sequence
    initializeHoming();

    Serial.println("Waiting for homing switch...");
}

void loop() {

    // Update sensor before homing
    updateMotorControl();

    // Check whether sensor measurement is valid
    float motorAngle = getMotorAngle();

    if (!isfinite(motorAngle)) {

        // Report sensor error
        setStatus(STATUS_SENSOR_ERROR);

        // Do not report an invalid angle
        delay(10);
        return;
    }

    // Update homing sequence
    updateHoming();

    // After homing completes retrieve calibrated angle
    if (isHomed()) {

        float currentAngle = getHomedAngle(motorAngle);

        if (isfinite(currentAngle)) {

            // Update CURRENT_ANGLE register
            setCurrentAngle(currentAngle);

            // Tell Jetson the controller is ready
            setStatus(STATUS_READY);
        }
        else {

            setStatus(STATUS_SENSOR_ERROR);
        }
    }
    else {

        // Controller is not ready until homing succeeds
        setStatus(STATUS_NOT_READY);
    }

    // Small delay for initial communication testing
    delay(10);
}