#pragma once

#include "Particle.h"

namespace particle
{
    class IGnssLed
    {
    public:
        IGnssLed(){};

        /**
         * @brief Initialize some hardware resource
         */
        virtual void init() = 0;

        /**
         * @brief set LED on
         */
        virtual void on() = 0;

        /**
         * @brief set LED off
         */
        virtual void off() = 0;
    };
}
