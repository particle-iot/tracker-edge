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
#include "tracker_motion.h"
#include "tracker_shipping.h"
#include "tracker_rgb.h"
#include "gnss_led.h"
#include "temperature.h"

struct TrackerConfig
{
    bool UsbCommandEnable;
};

enum class TrackerChargeState
{
    CHARGE_INIT,
    CHARGE_DONT_CARE,
    CHARGE_CARE,
};

struct TrackerChargeStatus {
    unsigned int uptime;
    TrackerChargeState state;
};

// this class encapsulates the underlying modules and builds on top of them to
// provide a cohesive asset tracking application
class Tracker
{
    public:
        static Tracker &instance()
        {
            if(!_instance)
            {
                _instance = new Tracker();
            }
            return *_instance;
        }

        void enterLowBatteryShippingMode();
        void init();
        void loop();
        int stop();

        uint32_t getModel() {return _model;}
        uint32_t getVariant() {return _variant;}

        int enableCharging();
        int disableCharging();

        bool isUsbCommandEnabled() { return _config.UsbCommandEnable; }

        // underlying services exposed to allow sharing with rest of the system
        CloudService &cloudService;
        ConfigService &configService;
        LocationService &locationService;
        MotionService &motionService;

        AM1805 rtc;

        TrackerLocation &location;
        TrackerMotion &motion;
        TrackerShipping &shipping;
        TrackerRGB &rgb;
    private:
        Tracker();

        static Tracker* _instance;
        TrackerConfig _config;

        uint32_t _model;
        uint32_t _variant;

        uint32_t last_loop_sec;
        bool _pastWarnLimit;
        unsigned int _evalTick;
        bool _lastBatteryCharging;
        bool _delayedBatteryCheck;
        unsigned int _delayedBatteryCheckTick;
        TrackerChargeStatus _pendingChargeStatus;
        std::mutex _pendingLock;
        TrackerChargeState _chargeStatus;
        unsigned int _lowBatteryEvent;

        int registerConfig();
        static void loc_gen_cb(JSONWriter& writer, LocationPoint &loc, const void *context);
        TrackerChargeState batteryDecode(battery_state_t state);
        void setPendingChargeStatus(unsigned int uptime, TrackerChargeState state);
        TrackerChargeStatus getPendingChargeStatus();
        static void lowBatteryHandler(system_event_t event, int data);
        static void batteryStateHandler(system_event_t event, int data);
        void initBatteryMonitor();
        void evaluateBatteryCharge();
};
