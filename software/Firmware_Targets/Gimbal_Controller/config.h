
#pragma once

#include <Arduino.h>

constexpr bool DEBUG = false;

//Compilation Target
enum class Axis { Pitch, Yaw };

constexpr Axis TARGET = Axis::Yaw;
constexpr bool IS_PITCH = TARGET == Axis::Pitch;

// I2C communication settings
#define YAW_I2C_ADDRESS 0x54
#define PITCH_I2C_ADDRESS 0x55

// I2C pins 
#define SDA_PIN 6
#define SCL_PIN 5

// I2C communication frequency
#define I2C_FREQUENCY 400000

// SimpleFOCmini Pins
#define PWM_A_PIN 20
#define PWM_B_PIN 21
#define PWM_C_PIN 10
#define ENABLE_PIN 7

// SPI Hall Sensor Configuration (PITCH)
#define HALL_SENSOR_CS_PIN 5
#define HALL_SENSOR_CLK_PIN 20
#define HALL_SENSOR_MISO_PIN 22
#define HALL_SENSOR_MOSI_PIN 23

//Analog Hall Sensor Configuration (YAW)
#define SENSOR_A_PIN 4
#define SENSOR_B_PIN 3
#define SENSOR_C_PIN 1

// I2C register addresses
enum I2C_ADDRESS_MAP {

    // Read-only: 1 when initialization is complete
    // Data type: uint8_t (1 byte)
    READY = 0x00,

    // Read/write: target angle in radians
    // Data type: float (4 bytes)
    TARGET_ANGLE = 0x01,

    // Read-only: current measured angle in radians
    // Data type: float (4 bytes)
    CURRENT_ANGLE = 0x05
};
