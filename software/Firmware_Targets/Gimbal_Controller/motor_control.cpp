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

//commander interface
Commander* commanderPTR;
void doMotor(char* cmd) { 
    if(commanderPTR != nullptr){
        commanderPTR->motor(&motor, cmd); 
    }
}

// ======================================
// INITIALIZE simpleFOC motor control system
// ======================================

bool initializeMotorControl(Sensor* sensor, Commander* commander) {
    if (sensor == nullptr) return false;
    driver.voltage_power_supply = 24;
    driver.pwm_frequency = 21000; //20kHz

    // Set the intended SimpleFOC control modes
    motor.controller = MotionControlType::angle;
    motor.torque_controller = TorqueControlType::voltage;
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // Original configuration parameters
    motor.voltage_limit = 24.0f;
    motor.current_limit = 1.25f;

    motor.velocity_limit = 30.0f;


    motor.P_angle.P = 9.788766f;

    motor.PID_velocity.P = 11.279879f;
    motor.PID_velocity.I = 36.017700f;
    motor.PID_velocity.D = 0.000021f;

    motor.LPF_velocity.Tf = 0.01f;
    motor.P_angle.output_ramp = 100.0f;

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

    commanderPTR = commander;
    if(commander != nullptr){
        commander->add('M', doMotor, "Yaw Motor");
    }

    return true;
}

// ======================================
// UPDATE simpleFOC motor control system
// ======================================
void updateMotorControl() {
    static uint8_t skip_move = 0;
    // Update the motor control system
    motor.loopFOC();

    if(skip_move++ == 5){
        motor.move();
        skip_move = 0;
    }
}

// ======================================
// GET CURRENT ANGLE
// ======================================
float getMotorCurrentAngle() {
    return _normalizeAngle(motor.shaft_angle);
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

// ======================================
// GET TARGET ANGLE
// ======================================
float getMotorTargetAngle(){
    return motor.target;
}

// ======================================
// GET MOTOR REFERENCE
// ======================================
BLDCMotor* getMotor(){
    return &motor;
}