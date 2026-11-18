//Gives motor parameters and creates the SimpleFOC objects
//Also configures the intended angle-control mode and SVPWM setting
//Settings are stored but not executed by this file


#include "motor_control.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>


// ======================================
// ORIGINAL MOTOR CONFIGURATION
// ======================================


static BLDCMotor motor = IS_PITCH ? BLDCMotor(12, 8.3f, 30.0f) : BLDCMotor(15, 14.6f, 15.0f);

// SimpleFOC Mini:
static BLDCDriver3PWM driver(PWM_A_PIN, PWM_B_PIN, PWM_C_PIN, ENABLE_PIN);

// ======================================
// INITIALIZE simpleFOC motor control system
// ======================================

bool initializeMotorControl(Sensor* sensor) {
    if (sensor == nullptr) return false;
    driver.voltage_power_supply = 24;

    // Set the intended SimpleFOC control modes
    motor.controller = MotionControlType::angle;
    motor.torque_controller = TorqueControlType::voltage;
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // Original configuration parameters
    motor.voltage_limit = 24.0f;
    motor.velocity_limit = 300.0f;

    motor.P_angle.P = 10.0f;

    motor.PID_velocity.P = 0.2f;
    motor.PID_velocity.I = 20.0f;
    motor.PID_velocity.D = 0.0f;

    motor.LPF_velocity.Tf = 0.01f;
    motor.P_angle.output_ramp = 100.0f;

    int error = 0;

    //connect sensor object to motor object
    motor.linkSensor(sensor);

    //connect driver object to motor object
    motor.linkDriver(&driver);

    //initialize the driver hardware
    if (!driver.init()) {
        return false;
    }

    if (!motor.init()) {
        driver.disable();
        return false;
    }

    if (!motor.initFOC()) {
        motor.disable();
        return false;
    }

    return true;
}

// ======================================
// UPDATE simpleFOC motor control system
// ======================================
void updateMotorControl() {
    // Update the motor control system
    motor.loopFOC();
    motor.move();
}

// ======================================
// GET CURRENT ANGLE
// ======================================
float getMotorCurrentAngle() {
    return motor.shaft_angle;
}

// ======================================
// SET TARGET ANGLE
// ======================================
void setMotorTargetAngle(const float& angle) {
    if(isfinite(angle)) {
        motor.target = angle;
    } else {
        if(DEBUG) Serial.println("Received non-finite target angle, ignoring.");
        return;
    }
}
