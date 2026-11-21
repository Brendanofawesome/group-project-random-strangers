#include <Arduino.h>
#include <SimpleFOC.h>
#include <math.h>
#include <string.h>

#include "config.h"
#include "I2C_Driver.h"
#include "motor_control.h"

#ifdef doAutoTune
    #include "autotune.h"
#endif

#include "analog_hall_sensor.h"
#include "MT6816.h"
// Gimbal Controller main program
// Two main functions: setup() and loop()
// setup() is called once when ESP32 starts 
// loop() is called continuoualy to read sensor angle, update registers, and chack status

// Status values for the ESP32
constexpr uint8_t STATUS_NOT_READY = 0;
constexpr uint8_t STATUS_READY = 1;
constexpr uint8_t STATUS_SENSOR_ERROR = 2;

static analog_hall_sensor analog_sensor;
static MT6816 magnetic_encoder(HALL_SENSOR_CS_PIN);
static Sensor* selected_sensor = IS_PITCH ? static_cast<Sensor*>(&magnetic_encoder) : static_cast<Sensor*>(&analog_sensor);

Commander commander('\n', false);

static float last_target_angle = 0.0f;

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
    if (!IS_PITCH) { //yaw controller
        analog_sensor.init(); //yaw uses analog
        //analog_sensor.debugCharacterize();
    } else {
        SPI.begin(HALL_SENSOR_CLK_PIN, HALL_SENSOR_MISO_PIN, HALL_SENSOR_MOSI_PIN, HALL_SENSOR_CS_PIN);
        magnetic_encoder.init(&SPI);
    }

    // Initialize motor control
    if(!initializeMotorControl(selected_sensor, DEBUG ? &commander : nullptr)) {
        if(DEBUG) Serial.println("Motor control initialization failed!");
        // Stop here because motor control initialization failed
        while (true) {
            delay(1000);
        }
    }

    // Initialize I2C communication
    if (!initialize_i2c()) {
        if(DEBUG) Serial.println("I2C initialization failed!");
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

    // Set status to ready
    setReady(true);
}


void loop() {
    //do commander if debug
    if(DEBUG){
        commander.run(Serial);
        setI2CTargetAngle(getMotorTargetAngle()); //update I2C register
    }

    // Update the motor control system
    updateMotorControl();

    // Update the target angle
    float new_target_angle = getI2CTargetAngle();
    if(std::isfinite(new_target_angle) && new_target_angle != last_target_angle) {
        if(DEBUG) Serial.printf("New target angle: %.4f radians\n", new_target_angle);
        last_target_angle = new_target_angle;
        setMotorTargetAngle(last_target_angle);
    }

    // Update CURRENT_ANGLE register
    setI2CCurrentAngle(getMotorCurrentAngle());
}
