
//Handles homing switch detection and zero calibration
//Records the motor angle when the home switch activates

#include "homing.h"
#include "motor_control.h"

#include <Arduino.h>
#include <simpleFOC.h>
#include <math.h>


static bool homed = false;
static float homeOffset = NAN;
static const float home_speed = 0.1f;
static const float find_speed = 0.25f;

// Number of samples to average for debouncing
static constexpr uint8_t sampleCount = 5; 

//Initialize homing
void initializeHomingSensor() {

    //switch
    pinMode(YAW_HOME_SWITCH_PIN, YAW_SWITCH_ACTIVE ? INPUT_PULLDOWN : INPUT_PULLUP);

    homed = false;

    homeOffset = NAN;
}

void goToAngle(BLDCMotor* motor, const float& target, const float& tolerance){
    if(motor == nullptr) return;

    motor->controller = MotionControlType::angle;
    motor->target = target; //Move away from the switch

    do {
        motor->loopFOC();
        motor->move();
    } while(fabs(motor->shaft_angle - target) > fabs(tolerance));
}

//Home the motor and return the absolute angle offset
//  Assumes the motor is already initialized and it has a
//  configured sensor.
bool home(BLDCMotor* motor) {
    if (!motor) {
        return false;
    }

    homed = false;
    homeOffset = NAN;

    //Save current motor motion control type and target
    MotionControlType oldMotion = motor->controller;
    float oldTarget = motor->target;

    //Set motor to velocity control to move slowly
    motor->target = 0.0f;
    motor->controller = MotionControlType::velocity;

    if(digitalRead(YAW_HOME_SWITCH_PIN) == YAW_SWITCH_ACTIVE){
        //Switch is already pressed, move away from it
        motor->target = find_speed; //Move in positive direction
        const uint32_t full_rotation_time = millis() + (_2PI / abs(motor->target)) * 1500; //limit to 1.5 full rotation
        while(digitalRead(YAW_HOME_SWITCH_PIN) == YAW_SWITCH_ACTIVE){
            if(millis() >= full_rotation_time){ //rotated 360 deg and switch stayed pressed
                motor->target = oldTarget;
                motor->controller = oldMotion;
                motor->disable();
                return false;
            }

            motor->loopFOC();
            motor->move();
        }
    }

    //move towards the switch until it is pressed
    float touchAngle = NAN;
    motor->target = -find_speed; //Move in negative direction
    const uint32_t full_rotation_time = millis() + (_2PI / abs(motor->target)) * 1500; //limit to 1.5 full rotation
    while(digitalRead(YAW_HOME_SWITCH_PIN) == !YAW_SWITCH_ACTIVE){ 
        if(millis() >= full_rotation_time){ //rotated 360 deg and switch never pressed
            motor->target = oldTarget;
            motor->controller = oldMotion;
            motor->disable();
            return false;
        }

        motor->loopFOC();
        motor->move();
    }
    motor->target = 0.0f; //Stop the motor
    motor->move();
    motor->loopFOC();
    touchAngle = motor->shaft_angle;

    //homing loop
    float angleAccumulation = 0.0f;
    for(int i = 0; i < sampleCount; i++){
        goToAngle(motor, touchAngle+0.2, 0.05);
        if (digitalRead(YAW_HOME_SWITCH_PIN) == YAW_SWITCH_ACTIVE) { //switch still pressed
            motor->target = oldTarget;
            motor->controller = oldMotion;
            motor->disable();
            return false;
        }

        motor->controller = MotionControlType::velocity;
        motor->target = -home_speed; //Move towards the switch
        const uint32_t full_rotation_time = millis() + (_2PI / abs(motor->target)) * 1500; //limit to 1.5 full rotation
        while(digitalRead(YAW_HOME_SWITCH_PIN) == !YAW_SWITCH_ACTIVE){ 
            if(millis() >= full_rotation_time){ //rotated 360 deg and switch never pressed
                motor->target = oldTarget;
                motor->controller = oldMotion;
                motor->disable();
                return false;
            }

            motor->loopFOC();
            motor->move();
        }
        angleAccumulation += motor->shaft_angle;
    }

    motor->target = oldTarget;
    motor->controller = oldMotion;

    motor->loopFOC();
    motor->move();

    homeOffset = angleAccumulation / sampleCount; //Record the Average home offset
    homed = true;

    return true;
}


//Check homing state
bool isHomed() {
    return homed;
}

//Get home offset
float getHomeOffset() {
    return homeOffset;
}

//Reset homing
void resetHoming() {
    homed = false;
    homeOffset = NAN;
}
