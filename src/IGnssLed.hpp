#pragma once

#include "Particle.h"

namespace particle
{
    class IGnssLed
    {
    public:    
        IGnssLed(){};
        virtual void init() = 0;    
        virtual void on() = 0;
        virtual void off() = 0;        
    };
}