#include "analog_hall_sensor.h"

#include <Arduino.h>
#include <math.h>

namespace {
constexpr float A_OFFSET_MV = 1665.2f;
constexpr float B_OFFSET_MV = 1663.8f;
constexpr float C_OFFSET_MV = 1659.4f;

constexpr float A_AMPLITUDE_MV = 398.8f;
constexpr float B_AMPLITUDE_MV = 413.0f;
constexpr float C_AMPLITUDE_MV = 405.9f;

const int POLE_PAIRS = IS_PITCH ? 12 : 15;
constexpr float SENSOR_DIRECTION = 1.0f;
constexpr float ELECTRICAL_OFFSET = 0.0f;
constexpr int ADC_SAMPLES = 4;

float angleDifference(float current, float previous) {
    // normalize the difference into [-pi, pi) with simpleFOC's built-in angle normalizer.
    return _normalizeAngle(current - previous + _PI) - _PI;
}

float round_fractional_bits(const float& val, const int& frac_bits) {
    float scaled = std::ldexp(val, frac_bits); // val * 2^frac_bits
    return std::ldexp(std::round(scaled), -frac_bits); // round and divide by 2^frac_bits
}

float readAnalogMillivoltsAveraged(int pin) {
    uint32_t total = 0;
    for (int i = 0; i < ADC_SAMPLES; ++i) {
        total += analogReadMilliVolts(pin);
    }
    return round_fractional_bits(static_cast<float>(total) / static_cast<float>(ADC_SAMPLES), 8); //analog hall is not that precise
}
} // namespace

void analog_hall_sensor::init() {
    lastElectricalAngle = 0.0f;
    unwrappedElectricalAngle = 0.0f;

    pinMode(SENSOR_A_PIN, INPUT);
    pinMode(SENSOR_B_PIN, INPUT);
    pinMode(SENSOR_C_PIN, INPUT);

    analogReadResolution(12);
    analogSetPinAttenuation(SENSOR_A_PIN, ADC_11db);
    analogSetPinAttenuation(SENSOR_B_PIN, ADC_11db);
    analogSetPinAttenuation(SENSOR_C_PIN, ADC_11db);

    electricalAngleInitialized = false;
    lastElectricalAngle = 0.0f;
    unwrappedElectricalAngle = 0.0f;

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

    // Establish the starting reference from the first valid reading.
    if (!electricalAngleInitialized) {
        lastElectricalAngle = electricalAngle;
        unwrappedElectricalAngle = 0.0f;
        electricalAngleInitialized = true;
        return 0.0f;
    }

    const float delta = angleDifference(electricalAngle, lastElectricalAngle);

    lastElectricalAngle = electricalAngle;
    unwrappedElectricalAngle += SENSOR_DIRECTION * delta;

return _normalizeAngle(
    unwrappedElectricalAngle / static_cast<float>(POLE_PAIRS)
);

    return _normalizeAngle(unwrappedElectricalAngle / static_cast<float>(POLE_PAIRS));
}

float analog_hall_sensor::getSensorAngle() {
    const float angle = readAngle();
    // Sensor::update() treats negative values as invalid-sample signals.
    return isfinite(angle) ? angle : -1.0f;
}

// AI Generated :)
void analog_hall_sensor::debugCharacterize(unsigned trials) {
    if (trials == 0) return;

    const int pins[3] = {
        SENSOR_A_PIN, SENSOR_B_PIN, SENSOR_C_PIN
    };

    float minimum[3] = {INFINITY, INFINITY, INFINITY};
    float maximum[3] = {-INFINITY, -INFINITY, -INFINITY};
    float offset[3];
    float amplitude[3];
    float hysteresis[3];

    for (int channel = 0; channel < 3; ++channel) {
        pinMode(pins[channel], INPUT);
        analogSetPinAttenuation(pins[channel], ADC_11db);
    }

    Serial.println("\n--- Analog Hall sensor characterization ---");
    Serial.println("Keep rotating slowly in ONE direction.");
    Serial.println("Pass 1: measuring voltage ranges...");

    // Measure each channel's minimum and maximum.
    for (unsigned trial = 0; trial < trials; ++trial) {
        for (int channel = 0; channel < 3; ++channel) {
            const float mv =
                readAnalogMillivoltsAveraged(pins[channel]);

            minimum[channel] = fminf(minimum[channel], mv);
            maximum[channel] = fmaxf(maximum[channel], mv);
        }

        delay(10);
    }

    bool usable = true;

    for (int channel = 0; channel < 3; ++channel) {
        offset[channel] =
            (maximum[channel] + minimum[channel]) * 0.5f;

        amplitude[channel] =
            (maximum[channel] - minimum[channel]) * 0.5f;

        // Use 10% of measured amplitude, with a 10 mV noise margin.
        hysteresis[channel] =
            fmaxf(10.0f, amplitude[channel] * 0.1f);

        Serial.printf(
            "\n// %c: min=%.1f mV, max=%.1f mV\n",
            'A' + channel, minimum[channel], maximum[channel]
        );

        Serial.printf(
            "constexpr float %c_OFFSET_MV = %.1ff;\n",
            'A' + channel, offset[channel]
        );

        Serial.printf(
            "constexpr float %c_AMPLITUDE_MV = %.1ff;\n",
            'A' + channel, amplitude[channel]
        );

        if (amplitude[channel] <= hysteresis[channel]) {
            Serial.printf(
                "Channel %c: insufficient swing for phase-order test.\n",
                'A' + channel
            );
            usable = false;
        }
    }

    if (!usable) {
        Serial.println("Check wiring and shaft movement, then repeat.");
        return;
    }

    Serial.println("\nPass 2: keep rotating in the SAME direction.");
    Serial.println("Rising midpoint crossing order:");

    bool armed[3] = {false, false, false};
    unsigned crossings[3] = {0, 0, 0};
    unsigned abc = 0;
    unsigned acb = 0;
    int previous = -1;

    for (unsigned trial = 0; trial < trials; ++trial) {
        for (int channel = 0; channel < 3; ++channel) {
            const float centered =
                readAnalogMillivoltsAveraged(pins[channel])
                - offset[channel];

            if (centered < -hysteresis[channel]) {
                armed[channel] = true;
            }

            if (armed[channel] && centered > hysteresis[channel]) {
                armed[channel] = false;
                ++crossings[channel];

                Serial.printf("%c ", 'A' + channel);

                if (previous >= 0 && previous != channel) {
                    const int step = (channel - previous + 3) % 3;
                    if (step == 1) ++abc;
                    else           ++acb;
                }

                previous = channel;
            }
        }

        delay(10);
    }

    Serial.printf(
        "\nCrossings: A=%u B=%u C=%u\n",
        crossings[0], crossings[1], crossings[2]
    );
    Serial.printf("ABC transitions=%u, ACB transitions=%u\n", abc, acb);

    if (crossings[0] < 2 || crossings[1] < 2 || crossings[2] < 2) {
        Serial.println("Insufficient crossings on one or more channels.");
    } else if (acb == 0 && abc >= 6) {
        Serial.println("Consistent cyclic order: A -> B -> C -> A");
    } else if (abc == 0 && acb >= 6) {
        Serial.println("Consistent cyclic order: A -> C -> B -> A");
    } else {
        Serial.println(
            "Inconclusive: check noise, rotation direction, "
            "and whether rotation was too fast."
        );
    }

    Serial.println("--- Characterization complete ---\n");
}
