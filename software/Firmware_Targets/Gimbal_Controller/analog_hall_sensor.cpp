#include "analog_hall_sensor.h"

#include <Arduino.h>
#include <math.h>

namespace {
constexpr float A_OFFSET_MV = 1700.0f;
constexpr float B_OFFSET_MV = 1700.0f;
constexpr float C_OFFSET_MV = 1700.0f;

constexpr float A_AMPLITUDE_MV = 400.0f;
constexpr float B_AMPLITUDE_MV = 400.0f;
constexpr float C_AMPLITUDE_MV = 400.0f;

const int POLE_PAIRS = IS_PITCH ? 12 : 15;
constexpr float SENSOR_DIRECTION = 1.0f;
constexpr float ELECTRICAL_OFFSET = 0.0f;
constexpr int ADC_SAMPLES = 4;

float angleDifference(float current, float previous) {
    // normalize the difference into [-pi, pi) with simpleFOC's built-in angle normalizer.
    return _normalizeAngle(current - previous + _PI) - _PI;
}

float readAnalogMillivoltsAveraged(int pin) {
    uint32_t total = 0;
    for (int i = 0; i < ADC_SAMPLES; ++i) {
        total += analogReadMilliVolts(pin);
    }
    return total / static_cast<float>(ADC_SAMPLES);
}
} // namespace

void analog_hall_sensor::init() {
    lastElectricalAngle = 0.0f;
    unwrappedElectricalAngle = 0.0f;

    pinMode(SENSOR_A_PIN, INPUT);
    pinMode(SENSOR_B_PIN, INPUT);
    pinMode(SENSOR_C_PIN, INPUT);

    analogSetPinAttenuation(SENSOR_A_PIN, ADC_11db);
    analogSetPinAttenuation(SENSOR_B_PIN, ADC_11db);
    analogSetPinAttenuation(SENSOR_C_PIN, ADC_11db);

    // Initialize SimpleFOC's cached angle and velocity tracking from live samples.
    Sensor::init();
}

float analog_hall_sensor::readAngle() {
    const float sensorA = readAnalogMillivoltsAveraged(SENSOR_A_PIN);
    const float sensorB = readAnalogMillivoltsAveraged(SENSOR_B_PIN);
    const float sensorC = readAnalogMillivoltsAveraged(SENSOR_C_PIN);

    const float a = (sensorA - A_OFFSET_MV) / A_AMPLITUDE_MV;
    const float b = (sensorB - B_OFFSET_MV) / B_AMPLITUDE_MV;
    const float c = (sensorC - C_OFFSET_MV) / C_AMPLITUDE_MV;

    // Clarke-style components for the three analog Hall channels.
    const float x = 2.0f * a - b - c;
    const float y = _SQRT3 * (b - c);
    const float magnitudeSquared = x * x + y * y;
    if (!isfinite(magnitudeSquared) || magnitudeSquared < 0.09f) return NAN;

    float electricalAngle = _normalizeAngle(_atan2(y, x) + ELECTRICAL_OFFSET);
    if (!isfinite(electricalAngle)) return NAN;

    // The electrical angle repeats once per pole pair. Unwrap it here so the
    // derived mechanical angle can span the full shaft revolution for Sensor.
    const float delta = angleDifference(electricalAngle, lastElectricalAngle);
    unwrappedElectricalAngle += SENSOR_DIRECTION * delta;
    lastElectricalAngle = electricalAngle;

    return _normalizeAngle(unwrappedElectricalAngle / static_cast<float>(POLE_PAIRS));
}

float analog_hall_sensor::getSensorAngle() {
    const float angle = readAngle();
    // Sensor::update() treats negative values as invalid-sample signals.
    return isfinite(angle) ? angle : -1.0f;
}
