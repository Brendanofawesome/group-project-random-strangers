#ifndef MT6816_H_
#define MT6816_H_

#include <Arduino.h>
#include <SPI.h>
#include <common/base_classes/Sensor.h>

//MT6816 encoder using the four-wire SPI interface.
class MT6816_custom : public Sensor {
public:
    //CS is required. SPI uses mode 3, MSB first; default clock is 8 MHz.
    explicit MT6816_custom(int chip_select_pin, uint32_t clock_speed = 8000000);

    void init() override;
    // For custom pins, configure the bus with spi->begin(...) before init().
    void init(SPIClass* spi);

    //true before successful init() or if the latest hardware read failed.
    //errors clear automatically on the next valid update().
    bool hasError() const { return error_; }

protected:
    //fresh mechanical angle in [0, 2PI), or -1 on error.
    float getSensorAngle() override;

private:
    float readAngle();
    uint8_t readRegister(uint8_t address);

    const int chip_select_pin_;
    const uint32_t clock_speed_;
    const SPISettings settings_;
    SPIClass* spi_ = nullptr;
    bool error_ = true;
};

#endif
