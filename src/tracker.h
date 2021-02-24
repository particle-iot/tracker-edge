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

#include "tracker_sleep.h"
#include "tracker_location.h"
#include "tracker_motion.h"
#include "tracker_shipping.h"
#include "tracker_rgb.h"
#include "gnss_led.h"
#include "temperature.h"
#include "mcp_can.h"

struct TrackerCloudConfig {
    bool UsbCommandEnable;
};

enum class TrackerChargeState {
    CHARGE_INIT,
    CHARGE_DONT_CARE,
    CHARGE_CARE,
};

struct TrackerChargeStatus {
    unsigned int uptime;
    TrackerChargeState state;
};

/**
 * @brief TrackerConfiguration class to configure the tracker device in application
 *
 */
class TrackerConfiguration {
public:
    /**
     * @brief Construct a new Tracker Configuration object
     *
     */
    TrackerConfiguration() :
        _enableIo(true),
        _enableIoSleep(false) {

    }

    /**
     * @brief Construct a new Tracker Configuration object
     *
     */
    TrackerConfiguration(TrackerConfiguration&&) = default;

    /**
     * @brief Enable or disable IO/CAN power at initialization
     *
     * @param enable Turn IO/power on CAN power at initialization
     * @return TrackerConfiguration&
     */
    TrackerConfiguration& enableIoCanPower(bool enable) {
        _enableIo = enable;
        return *this;
    }

    /**
     * @brief Indicate if IO/CAN power is powered on at initialization
     *
     * @return true Powered at initialization
     * @return false Not powered at initialization
     */
    bool enableIoCanPower() const {
        return _enableIo;
    }

    /**
     * @brief Enable or disable IO/CAN power shutdown prior to sleep
     *
     * @param enable Power down IO/CAN power before entering sleep; otherwise, leave alone for user
     * @return TrackerConfiguration&
     */
    TrackerConfiguration& enableIoCanPowerSleep(bool enable) {
        _enableIoSleep = enable;
        return *this;
    }

    /**
     * @brief Indicate if IO/CAN power will be powered down prior to sleep
     *
     * @return true Powered off at sleep
     * @return false IO/CAN power under user control
     */
    bool enableIoCanPowerSleep() const {
        return _enableIoSleep;
    }

    TrackerConfiguration& operator=(const TrackerConfiguration& rhs) {
        if (this == &rhs) {
            return *this;
        }
        this->_enableIo = rhs._enableIo;
        this->_enableIoSleep = rhs._enableIoSleep;

        return *this;
    }
private:
    bool _enableIo;
    bool _enableIoSleep;
};

// this class encapsulates the underlying modules and builds on top of them to
// provide a cohesive asset tracking application
class Tracker {
    public:
        static Tracker &instance() {
            if(!_instance) {
                _instance = new Tracker();
            }
            return *_instance;
        }

        /**
         * @brief Startup for early device intitialization
         *
         */
        static void startup();

        /**
         * @brief Initialize device for application setup()
         *
         * @retval SYSTEM_ERROR_NONE
         */
        int init();

        /**
         * @brief Initializate device with given configuration for application setup()
         *
         * @param config Configuration for general tracker operation
         * @retval SYSTEM_ERROR_NONE
         */
        int init(const TrackerConfiguration& config) {
            _deviceConfig = config;
            return init();
        }

        /**
         * @brief Perform device functionality for application loop()
         *
         */
        void loop();

        /**
         * @brief Stop services on device
         *
         * @retval SYSTEM_ERROR_NONE
         */
        int stop();

        /**
         * @brief Prepare tracker IO and peripherals for shutdown
         *
         * @retval SYSTEM_ERROR_NONE
         */
        int end();

        /**
         * @brief Get the tracker hardware model number
         *
         * @return uint32_t Model number
         */
        uint32_t getModel() const {
            return _model;
        }

        /**
         * @brief Get the tracker hardware variant number
         *
         * @return uint32_t Variant number
         */
        uint32_t getVariant() const {
            return _variant;
        }

        /**
         * @brief Enable battery charging
         *
         * @retval SYSTEM_ERROR_NONE
         */
        int enableCharging();

        /**
         * @brief Disable battery charging
         *
         * @retval SYSTEM_ERROR_NONE
         */
        int disableCharging();

        /**
         * @brief Force battery charge current
         *
         * @param current Current in milliampheres
         * @retval SYSTEM_ERROR_NONE
         */
        int setChargeCurrent(uint16_t current);

        /**
         * @brief Enable or disable IO/CAN power
         *
         * @param enable Enable IO/CAN power when true
         */
        void enableIoCanPower(bool enable);

        /**
         * @brief Indicates whether device can accept commands through USB interface
         *
         * @return true Commands are accepted via USB
         * @return false Commands are not accepted via USB
         */
        bool isUsbCommandEnabled() const {
            return _cloudConfig.UsbCommandEnable;
        }

        /**
         * @brief Enable or disable application watchdog
         *
         * @param enable
         */
        void enableWatchdog(bool enable);

        /**
         * @brief Invoke shipping mode
         *
         */
        void startShippingMode();

        /**
         * @brief Start preparing for sleep
         *
         */
        int prepareSleep();

        /**
         * @brief Exit sleep
         *
         */
        int prepareWake();

        // underlying services exposed to allow sharing with rest of the system
        CloudService &cloudService;
        ConfigService &configService;
        TrackerSleep &sleep;
        LocationService &locationService;
        MotionService &motionService;

        AM1805 rtc;

        TrackerLocation &location;
        TrackerMotion &motion;
        TrackerShipping &shipping;
        TrackerRGB &rgb;

    private:
        Tracker();

        int chargeCallback(TemperatureChargeEvent event);

        static Tracker* _instance;
        TrackerCloudConfig _cloudConfig;
        TrackerConfiguration _deviceConfig;

        uint32_t _model;
        uint32_t _variant;

        uint32_t _lastLoopSec;
        bool _canPowerEnabled;
        bool _pastWarnLimit;
        unsigned int _evalTick;
        bool _lastBatteryCharging;
        bool _delayedBatteryCheck;
        unsigned int _delayedBatteryCheckTick;
        TrackerChargeStatus _pendingChargeStatus;
        Mutex _pendingLock;
        TrackerChargeState _chargeStatus;
        unsigned int _lowBatteryEvent;
        unsigned int _evalChargingTick;
        bool _batteryChargeEnabled;

        // Startup and initialization related
        static int getPowerManagementConfig(hal_power_config& conf);
        static int setPowerManagementConfig(const hal_power_config& conf);
        static int enablePowerManagement();
        static void completeSetupDone();
        int initEsp32();
        int initCan();
        int initIo();

        // Sleep related
        void onSleepPrepare(TrackerSleepContext context);
        void onSleep(TrackerSleepContext context);
        void onWake(TrackerSleepContext context);
        void onSleepStateChange(TrackerSleepContext context);

        // Shutdown related
        void startLowBatteryShippingMode();

        // Various methods
        int registerConfig();
        static void loc_gen_cb(JSONWriter& writer, LocationPoint &loc, const void *context);
        TrackerChargeState batteryDecode(battery_state_t state);
        void setPendingChargeStatus(unsigned int uptime, TrackerChargeState state);
        TrackerChargeStatus getPendingChargeStatus();
        static void lowBatteryHandler(system_event_t event, int data);
        static void batteryStateHandler(system_event_t event, int data);
        void initBatteryMonitor();
        bool getChargeEnabled();
        void evaluateBatteryCharge();
};
