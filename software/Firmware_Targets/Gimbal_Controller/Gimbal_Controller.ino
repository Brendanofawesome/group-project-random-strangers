#include <Arduino.h>
#include <SimpleFOC.h>
#include <math.h>
#include <string.h>

#include "config.h"
#include "I2C_Driver.h"
#include "motor_control.h"
#include "homing.h"

#ifdef doAutoTune
    #include "autotune.h"
#endif

#include "analog_hall_sensor.h"
#include "MT6816.h"
// Gimbal Controller main program
// Two main functions: setup() and loop()
// setup() is called once when ESP32 starts 
// loop() is called continuoualy to read sensor angle, update registers, and chack status

static float zero_offset = IS_PITCH ? PITCH_STATIC_OFFSET : YAW_STATIC_OFFSET;

// Status values for the ESP32
constexpr uint8_t STATUS_NOT_READY = 0;
constexpr uint8_t STATUS_READY = 1;
constexpr uint8_t STATUS_SENSOR_ERROR = 2;

static analog_hall_sensor analog_sensor;
static MT6816_custom magnetic_encoder(HALL_SENSOR_CS_PIN);
static Sensor* selected_sensor = IS_PITCH ? static_cast<Sensor*>(&magnetic_encoder) : static_cast<Sensor*>(&analog_sensor);

Commander commander('\n', false);

static float last_target_angle = NAN;

//all normal position commands should use the same fixed relative range [0, 2PI].
static void setRelativeTargetAngle(float angle) {
    if (!isfinite(angle)) return;

    angle = fminf(fmaxf(angle, 0.0f), _2PI);
    const float absoluteTarget = angle + zero_offset;
    if (!isfinite(absoluteTarget)) return;

    setMotorTargetAngle(absoluteTarget);
    setI2CTargetAngle(angle);
    last_target_angle = angle;
}

//Called once when the ESP32 starts up
void setup() {

    if(DEBUG){
        // Start serial communication for debugging
        Serial.begin(115200);
        if (TARGET == Axis::Pitch) {
            Serial.println("Starting Gimbal ESP32 (PITCH)...");
        } else if (TARGET == Axis::Yaw) {
            Serial.println("Starting Gimbal ESP32 (YAW)...");
        } else {
            Serial.println("Starting Gimbal ESP32 (UNKNOWN TARGET)...");
        }

        SimpleFOCDebug::enable(&Serial);

        delay(500);
    }

    //initialize the selected sensor
    if (IS_YAW) { //yaw controller
        analog_sensor.init(); //yaw uses analog
        //analog_sensor.debugCharacterize();
    } else if (IS_PITCH) {
        SPI.begin(HALL_SENSOR_CLK_PIN, HALL_SENSOR_MISO_PIN, HALL_SENSOR_MOSI_PIN, HALL_SENSOR_CS_PIN);
        magnetic_encoder.init(&SPI);
    } else {
        if(DEBUG) Serial.println("Axis not selected!");

        //Stop here because control method is not defined
        while (true) {
            delay(1000);
        }
    }

    // Initialize motor control
    if(!initializeMotorControl(selected_sensor, DEBUG ? &commander : nullptr)) {
        if(DEBUG) Serial.println("Motor control initialization failed!");
        // Stop here because motor control initialization failed
        while (true) {
            delay(1000);
        }
    }

    
    if(IS_YAW){ // Home YAW Motor
        initializeHomingSensor();
        if(!home(getMotor()) || !isHomed()){
            if(DEBUG) Serial.println("Motor homing failed!");
            getMotor()->disable();

            while(true) {
                delay(1000);
            }
        }

        zero_offset += getHomeOffset();
        if(DEBUG) Serial.printf("Home Offset: %.4f, Total Offset: %.4f\n", getHomeOffset(), zero_offset);
    }


    // Initialize I2C communication
    if (!initialize_i2c()) {
        if(DEBUG) Serial.println("I2C initialization failed!");
        getMotor()->disable();

        // Stop here because communication failed
        while (true) {
            delay(1000);
        }
    }

    if(DEBUG) Serial.println("I2C initialized successfully!");

    #ifdef doAutoTune
        if(!DEBUG){
            Serial.begin(115200); //serial is required for autotuner 
        }
        Serial.println("Starting autotune");
        runTune(getMotor());
    #endif

    //get current number of recorded rotations to cancel out
    getMotor()->loopFOC();
    const float current_angle = getMotor()->shaftAngle();
    if (!isfinite(current_angle) || !isfinite(zero_offset)) {
        if (DEBUG) Serial.println("Invalid shaft angle or zero offset!");
        getMotor()->disable();

        while (true) {
            delay(1000);
        }
    }

    const float whole_turns = floorf((current_angle - zero_offset) / _2PI);
    zero_offset += whole_turns * _2PI;
    setMotorTargetAngle(current_angle);
    setI2CTargetAngle(fminf(fmaxf(current_angle - zero_offset, 0.0f), _2PI));

    if (DEBUG) {
        Serial.printf("Offset shifted by %.0f turns to %.4f rad\n", whole_turns, zero_offset);
        Serial.printf("Absolute target range: %.4f to %.4f rad\n", zero_offset, zero_offset + _2PI);
        Serial.printf("Current Position: %.4f, Target: %.4f\n", getMotor()->shaftAngle(), getMotor()->target);
    }

    // Set status to ready
    setReady(true);
}


void loop() {
    //do commander if debug
    if(DEBUG){
        const float previousTarget = getMotorTargetAngle();
        commander.run(Serial);
        if(previousTarget != getMotorTargetAngle() && getMotor()->controller == MotionControlType::angle || getMotor()->controller == MotionControlType::angle_nocascade){
            setRelativeTargetAngle(getMotorTargetAngle()); //make commander angle relative
            if(DEBUG) Serial.printf("New target angle: %.4f radians\n", getMotorTargetAngle());
        }
    }

    // Update the target angle
    float new_target_angle = getI2CTargetAngle();
    if(std::isfinite(new_target_angle) && new_target_angle != last_target_angle && getMotor()->controller == MotionControlType::angle || getMotor()->controller == MotionControlType::angle_nocascade) {
        setRelativeTargetAngle(new_target_angle);
        if(DEBUG) Serial.printf("New target angle: %.4f radians, offset to %.4f radians\n", new_target_angle, getMotorTargetAngle());
    }

    // Update the motor control system
    updateMotorControl();

    // Update CURRENT_ANGLE register
    setI2CCurrentAngle(getMotorCurrentAngle() - zero_offset);
}
