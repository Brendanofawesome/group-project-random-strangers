
//Handles homing switch detection and zero calibration
//Records the motor angle when the home switch activates

#include "homing.h"
#include "motor_control.h"

#include <Arduino.h>
#include <simpleFOC.h>
#include <math.h>


static bool homed = false;
static float homeOffset = NAN;

//GPIO connected to the homing switch
static constexpr int HOME_SWITCH_PIN = 27;

// Number of samples to average for debouncing
static constexpr uint8_t sampleCount = 5; 

//Initialize homing
void initializeHomingSensor() {

    //Normally open switch connected to GND
    pinMode(HOME_SWITCH_PIN, INPUT_PULLUP);

    homed = false;

    homeOffset = NAN;

    Serial.println("Homing initialized");
    Serial.println("Waiting for switch release...");
}

//Home the motor and return the absolute angle offset
//  Assumes the motor is already initialized and it has a
//  configured sensor.
bool home(BLDCMotor* motor) {
    if (!motor) {
        return NAN;
    }

    //Save current motor motion control type and target
    MotionControlType currentMotion = motor->motion;
    float currentTarget = motor->target;

    //Set motor to open loop velocity control to move slowly
    motor->target = 0f;
    motor->controller = MotionControlType::velocity_openloop;

    if(digitalRead(HOME_SWITCH_PIN) == LOW){
        //Switch is already pressed, move away from it
        motor->target = .25f; //Move in positive direction
        const uint32_t full_rotation_time = millis() + (_2PI / abs(motor->target)) * 1000; //limit to one full rotation
        while(digitalRead(HOME_SWITCH_PIN) == LOW && millis() < full_rotation_time){
            motor->loopFOC();
            motor->move();
        }
    }

    //move towards the switch until it is pressed
    float touchAngle = NAN;
    motor->target = -.25f; //Move in negative direction
    const uint32_t full_rotation_time = millis() + (_2PI / abs(motor->target)) * 1000; //limit to one full rotation
    while(digitalRead(HOME_SWITCH_PIN) == HIGH && millis() < full_rotation_time){
        motor->loopFOC();
        motor->move();
    }
    motor->target = 0.0f; //Stop the motor
    motor->move();
    motor->loopFOC();
    touchAngle = motor->shaft_angle;

    //homing loop
    for(int i = 0; i < sampleCount; i++){
        motor->controller = MotionControlType::angle;
        motor->target = touchAngle+.1f; //Move away from the switch
        delay(1000);

        motor->controller = MotionControlType::velocity_openloop;
        motor->target = touchAngle-.1f; //Move towards the switch
        const uint32_t full_rotation_time = millis() + (_2PI / abs(motor->target)) * 1000; //limit to one full rotation
        while(digitalRead(HOME_SWITCH_PIN) == HIGH && millis() < full_rotation_time){
            motor->loopFOC();
            motor->move();
        }
        touchAngle += motor->shaft_angle;
    }

    touchAngle /= sampleCount + 1; //Average the samples
    homeOffset = touchAngle; //Record the home offset
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
