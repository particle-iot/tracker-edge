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
#include "adp8866_rgb.h"

class TrackerUserRGB
{
    public:
        static TrackerUserRGB &instance()
        {
            if(!_instance)
            {
                _instance = new TrackerUserRGB();
            }
            return *_instance;
        }
        int init();
        ADP8866_RGB &get_rgb1_instance();
        ADP8866_RGB &get_rgb2_instance();
    private:
        TrackerUserRGB(){}
        static TrackerUserRGB *_instance;
        ADP8866_RGB *pRGB1      = nullptr;
        ADP8866_RGB *pRGB2      = nullptr;
        ADP8866 *pAdp8866Drv    = nullptr;
};
