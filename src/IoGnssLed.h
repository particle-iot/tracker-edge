#pragma once

#include "Particle.h"
#include "IGnssLed.h"

namespace particle
{
    class IoGnssLed : public IGnssLed
    {
    public:
        IoGnssLed(uint32_t pin) : _pin(pin)
        {
        }

        /**
         * @brief Initialize LED pin
         */
        void init()
        {
            pinMode(_pin, OUTPUT);
            off();
        }

        /**
         * @brief set LED on
         */
        void on()
        {
            digitalWrite(_pin, LOW);
        }

        /**
         * @brief set LED off
         */
        void off()
        {
            digitalWrite(_pin, HIGH);
        }
    private:
        uint32_t _pin;
    };
}