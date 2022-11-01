#pragma once

#include "Particle.h"
#include "adp8866_rgb.h"
#include "IGnssLed.hpp"

namespace particle
{
    class Adp8866GnssLed : public IGnssLed
    {
    public:
        Adp8866GnssLed(ADP8866_RGB &instance) : _instace(instance)
        {
        }

        void init()
        {
            _instace.setPattern(LED_PATTERN_SOLID);
            _instace.brightness(80);
            _instace.color(0, 128, 0);
            off();
        }

        void on()
        {
            _instace.on();
        }

        void off()
        {
            _instace.off();
        }

    private:
        ADP8866_RGB &_instace;
    };
}