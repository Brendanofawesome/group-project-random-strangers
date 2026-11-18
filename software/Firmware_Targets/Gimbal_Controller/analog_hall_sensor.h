#pragma once

#include "config.h"
#include <SimpleFOC.h>

// Three-phase analog Hall sensor presented through SimpleFOC's Sensor API.
class analog_hall_sensor : public Sensor {
public:
    // Configure the ADC inputs and initialize Sensor's angle tracking.
    void init();

    // Read a fresh shaft angle without relying on Sensor::update().
    // Returns NAN when the analog signals are invalid.
    float readAngle();

protected:
    // SimpleFOC calls this from Sensor::update() and Sensor::init().
    // Negative values tell Sensor::update() to ignore invalid samples.
    float getSensorAngle() override;

private:
    float lastElectricalAngle = 0.0f;
    float unwrappedElectricalAngle = 0.0f;
};

