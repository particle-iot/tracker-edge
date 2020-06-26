/*
 * Copyright (c) 2020 Particle Industries, Inc.  All rights reserved.
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

#include "dct.h"
#include "deviceid_hal.h"

#include "tracker_config.h"

#include "cloud_service.h"
#include "config_service.h"
#include "location_service.h"
#include "motion_service.h"
#include "AM1805.h"

#include "tracker_location.h"
#include "tracker_shipping.h"
#include "tracker_rgb.h"
#include "gnss_led.h"
#include "temperature.h"

// this class encapsulates the underlying modules and builds on top of them to
// provide a cohesive asset tracking application
class Tracker
{
    public:
        Tracker();

        void init();
        void loop();
        int stop();

        uint32_t getModel() {return _model;}
        uint32_t getVariant() {return _variant;}

        // underlying services exposed to allow sharing with rest of the system
        CloudService cloudService;
        ConfigService configService;
        LocationService locationService;
        MotionService motionService;

        AM1805 rtc;

        TrackerLocation location;
        TrackerShipping shipping;
        TrackerRGB rgb;
    private:
        BackgroundPublish background_publish;

        uint32_t _model;
        uint32_t _variant;

        uint32_t last_loop_sec;

        static void loc_gen_cb(JSONWriter& writer, LocationPoint &loc, const void *context);
};
