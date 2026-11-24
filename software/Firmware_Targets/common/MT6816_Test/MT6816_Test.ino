#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include "..\..\Gimbal_Controller\MT6816.h"
#include "..\..\Gimbal_Controller\MT6816.cpp"
#include "..\..\Gimbal_Controller\config.h"

MT6816_custom sensor(HALL_SENSOR_CS_PIN, 600000);  //1 MHz SPI clock.

uint32_t failedReads = 0;
uint32_t lastPrint = 0;

void setup() {
    Serial.begin(115200);
    delay(1500);

    SPI.begin(HALL_SENSOR_CLK_PIN, HALL_SENSOR_MISO_PIN, HALL_SENSOR_MOSI_PIN, HALL_SENSOR_CS_PIN);
    sensor.init(&SPI);

    if (sensor.hasError()) {
        Serial.println("INIT FAILED: check SPI wiring, CS pin, and magnet.");
        Serial.println("Correct the problem, then reset the board.");
        //while (true) delay(1000);
    }

    Serial.println("Rotate the encoder magnet by hand.");
    Serial.println("Angle is cumulative: one full turn changes it by 360 degrees.");
    Serial.println("angle_deg\tfailed_reads");
}

void loop() {
    sensor.update();  //update between Serial prints.
    const bool readFailed = sensor.hasError();
    if (readFailed) ++failedReads;

    const uint32_t now = millis();
    if (now - lastPrint >= 100) {
        lastPrint = now;
        if (readFailed) {
            Serial.print("READ ERROR");
        } else {
            Serial.printf("%.8f rad -> %.4f°", sensor.getAngle(), sensor.getAngle() * RAD_TO_DEG);
        }
        Serial.print('\t');
        Serial.println(failedReads);
    }
}
