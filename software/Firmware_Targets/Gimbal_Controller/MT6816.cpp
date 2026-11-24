#include "MT6816.h"

namespace {
constexpr float kRadiansPerCount = 6.2831853071795864769f / 16384.0f;
constexpr uint8_t kAngleHighRegister = 0x03;
constexpr uint8_t kAngleLowRegister = 0x04;
constexpr uint8_t kReadCommand = 0x80;
constexpr uint16_t kNoMagnetBit = 0x0002;
constexpr uint32_t kMaxClockSpeed = 15625000; // The datasheet specifies a minimum SCK period of 64 ns.
}

MT6816_custom::MT6816_custom(int chip_select_pin, uint32_t clock_speed)
    : chip_select_pin_(chip_select_pin), clock_speed_(clock_speed), settings_(clock_speed, MSBFIRST, SPI_MODE3) {}

void MT6816_custom::init() {
    init(&SPI);
}

void MT6816_custom::init(SPIClass* spi) {
    error_ = true;
    spi_ = spi;
    if (spi_ == nullptr || chip_select_pin_ < 0 || clock_speed_ < 500000 || clock_speed_ > kMaxClockSpeed) {
        return;
    }

    //set the output latch before enabling the output to avoid a low glitch.
    digitalWrite(chip_select_pin_, HIGH);
    pinMode(chip_select_pin_, OUTPUT);
    spi_->begin();

    readAngle(); //read once to clear the parity bit and check for a magnet.
    if (error_) return;

    Sensor::init();
}

float MT6816_custom::getSensorAngle() {
    const float angle = readAngle();
    return angle;
}

float MT6816_custom::readAngle() {
    //keep the pair of register reads together on a shared SPI bus.
    spi_->beginTransaction(settings_);
    const uint16_t high = readRegister(kAngleHighRegister);
    const uint16_t low = readRegister(kAngleLowRegister);
    spi_->endTransaction();
    const uint16_t frame = static_cast<uint16_t>((high << 8) | low);

    //bit 0 makes the complete 16-bit frame even parity.
    uint16_t parity = frame;
    parity ^= parity >> 8;
    parity ^= parity >> 4;
    parity ^= parity >> 2;
    parity ^= parity >> 1;
    error_ = ((parity & 1u) != 0u) || ((frame & kNoMagnetBit) != 0u);
    if (error_) return -1.0f;

    return static_cast<float>(frame >> 2) * kRadiansPerCount;
}

uint8_t MT6816_custom::readRegister(uint8_t address) {
    digitalWrite(chip_select_pin_, LOW);
    delayMicroseconds(1); //at least 100 ns from CS falling to first clock.
    spi_->transfer(static_cast<uint8_t>(address | kReadCommand));
    const uint8_t value = spi_->transfer(0x00); // Clock out the data byte.
    delayMicroseconds(1); //at least half an SCK period before CS rises.
    digitalWrite(chip_select_pin_, HIGH);
    delayMicroseconds(1); //separate consecutive register accesses.
    return value;
}
