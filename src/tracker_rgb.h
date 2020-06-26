/*
 * Copyright (c) 2020 Particle Industries, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "config_service.h"

enum class RGBControlType {
    APP_PARTICLE, // standard particle device-os control
    APP_OFF, // force LED off
    APP_TRACKER, // display cell/cloud connection and signal strength customized for tracker
    APP_GRADIENT, // with added gradient
    APP_DIRECT, // direct control of LED (rgb and brightness)
};

class TrackerRGB
{
    public:
        TrackerRGB(ConfigService &config_service) :
            config_service(config_service)
        {}

        void init();
        static int setType(RGBControlType type);
        static RGBControlType getType();
    private:
        ConfigService &config_service;
};
