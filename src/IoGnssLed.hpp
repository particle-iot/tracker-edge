#pragma once

#include "Particle.h"
#include "IGnssLed.hpp"

namespace particle
{
    class IoGnssLed : public IGnssLed
    {
    public:
        IoGnssLed(uint32_t pin) : _pin(pin)
        {
        }

        void init()
        {
            pinMode(_pin, OUTPUT);
            off();
        }

        void on()
        {
            digitalWrite(_pin, LOW);
        }

        void off()
        {
            digitalWrite(_pin, HIGH);
        }

    private:
        uint32_t _pin;
    };
}