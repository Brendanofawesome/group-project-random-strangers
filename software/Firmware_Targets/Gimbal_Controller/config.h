
#pragma once

//Debug Flags
constexpr bool DEBUG = true;
//#define doAutoTune //at startup, begin PID tuning loop

//Compilation Target
enum class Axis { Pitch, Yaw };

constexpr Axis TARGET = Axis::Pitch;
constexpr bool IS_PITCH = TARGET == Axis::Pitch;
constexpr bool IS_YAW = TARGET == Axis::Yaw;

//Static Zero Angle Offsets
#define YAW_STATIC_OFFSET -0.4308f
static constexpr int YAW_HOME_SWITCH_PIN = 0;
#define YAW_SWITCH_ACTIVE 0.59633493f

#define PITCH_STATIC_OFFSET 0.0f 

// Motor Control Settings
//#define useTrapezoidalPlanner
constexpr float yaw_motion_acceleration = 10.0f; // rad/s^2
constexpr float yaw_motion_deceleration = 10.0f; // rad/s^2

constexpr float pitch_motion_acceleration = 10.0f; // rad/s^2
constexpr float pitch_motion_deceleration = 10.0f; // rad/s^2


// I2C communication settings
#define YAW_I2C_ADDRESS 0x54
#define PITCH_I2C_ADDRESS 0x55

// I2C pins 
#define SDA_PIN 6
#define SCL_PIN 5

// I2C communication frequency
#define I2C_FREQUENCY 400000

// SimpleFOCmini Pins
#define PWM_A_PIN (IS_PITCH ? 21 : 20)
#define PWM_B_PIN (IS_PITCH ? 20 : 21)
#define PWM_C_PIN 10
#define ENABLE_PIN 7

// SPI Hall Sensor Configuration (PITCH)
#define HALL_SENSOR_CS_PIN 4
#define HALL_SENSOR_CLK_PIN 0
#define HALL_SENSOR_MISO_PIN 1
#define HALL_SENSOR_MOSI_PIN 3

//Analog Hall Sensor Configuration (YAW)
#define SENSOR_A_PIN 3
#define SENSOR_B_PIN 4
#define SENSOR_C_PIN 1


// Tuning parameters for the Yaw motor
constexpr float yaw_velocity_limit = 40.0f;
constexpr float yaw_P_angle_P = 4.0f;
constexpr float yaw_P_angle_I = 0.0f;
constexpr float yaw_P_angle_D = 0.2f;
constexpr float yaw_PID_velocity_P = 16.0f;
constexpr float yaw_PID_velocity_I = 220.0f;
constexpr float yaw_PID_velocity_D = 0.0f;
constexpr float yaw_LPF_angle = 0.015f; //yaw motor has a noisy sensor
constexpr float yaw_LPF_velocity = 0.02f;
constexpr float yaw_P_angle_output_ramp = 20.0f;

// Tuning parameters for the Pitch motor
constexpr float pitch_velocity_limit = 40.0f;
constexpr float pitch_P_angle_P = 2.0f;
constexpr float pitch_P_angle_I = 0.0f;
constexpr float pitch_P_angle_D = 0.0f;
constexpr float pitch_PID_velocity_P = 2.0f;
constexpr float pitch_PID_velocity_I = 20.0f;
constexpr float pitch_PID_velocity_D = 0.0f;
constexpr float pitch_LPF_angle = 0.00f; 
constexpr float pitch_LPF_velocity = 0.01f;
constexpr float pitch_P_angle_output_ramp = 0.0f;



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
