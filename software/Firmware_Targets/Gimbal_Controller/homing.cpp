
//Handles homing switch detection and zero calibration
//Records the motor angle when the home switch activates

#include "homing.h"
#include "motor_control.h"

#include <Arduino.h>
#include <simpleFOC.h>
#include <math.h>

//Homing configuration

//GPIO connected to the homing switch
static constexpr int HOME_SWITCH_PIN = 27;

//Switch debounce time
static constexpr unsigned long DEBOUNCE_MS = 30;

//Homing state

static bool homed = false;
static bool switchState = false;
static bool lastRawState = false;

static float homeOffset = 0.0f;

static unsigned long lastSwitchChange = 0;

//Require switch to be released before accepting home
static bool switchReleased = false;

//Initialize homing
void initializeHoming() {

    //Normally open switch connected to GND
    pinMode(HOME_SWITCH_PIN, INPUT_PULLUP);

    homed = false;
    switchState = false;
    switchReleased = false;

    lastRawState = digitalRead(HOME_SWITCH_PIN) == LOW;

    lastSwitchChange = millis();
    homeOffset = 0.0f;

    Serial.println("Homing initialized");
    Serial.println("Waiting for switch release...");
}

//Update homing switch
void updateHoming(Sensor* sensor) {

    if (homed) {
        return;
    }

    bool rawState = digitalRead(HOME_SWITCH_PIN) == LOW;

    //Detect switch state changes
    if (rawState != lastRawState) {

        lastSwitchChange = millis();
        lastRawState = rawState;
    }

    //Wait for stable switch state
    if (millis() - lastSwitchChange < DEBOUNCE_MS) {
        return;
    }

    //Update debounced switch state
    switchState = rawState;

    //Wait until switch has been released
    if (!switchState) {

        if (!switchReleased) {
            switchReleased = true;
            Serial.println("Switch released");
            Serial.println("Waiting for home switch...");
        }

        return;
    }

    //Do not establish home until switch was released
    if (!switchReleased) {
        return;
    }

    //Record home position
    float angle = sensor->getMechanicalAngle();

    if (!isfinite(angle)) {

        Serial.println("Homing failed: invalid angle");
        return;
    }

    homeOffset = angle;
    homed = true;

    Serial.println("Homing complete!");
    Serial.print("Home offset: ");
    Serial.println(homeOffset, 6);
}

//Check homing state
bool isHomed() {
    return homed;
}

//Get home offset
float getHomeOffset() {
    return homeOffset;
}

//Get homed angle
float getHomedAngle(float motorAngle) {

    if (!homed || !isfinite(motorAngle)) {
        return NAN;
    }

    //Subtract the recorded home position
    return motorAngle - homeOffset;
}

//Reset homing
void resetHoming() {

    homed = false;
    switchState = false;
    switchReleased = false;

    homeOffset = 0.0f;

    lastRawState =
        digitalRead(HOME_SWITCH_PIN) == LOW;

    lastSwitchChange = millis();

    Serial.println("Homing reset");
}
