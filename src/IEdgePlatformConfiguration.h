/*
 * Copyright (c) 2022 Particle Industries, Inc.
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

#include "Particle.h"
#include "IGnssLed.h"

typedef struct
{
    int canSleepRetries;
    int lowBatteryCutoff;
    int lowBatteryCutoffCorrection;
    int lowBatteryWarning;
    int lowBatteryWarningHyst;
    unsigned int lowBatteryAwakeEvalInterval;
    unsigned int lowBatterySleepEvalInterval;
    unsigned int lowBatterySleepWakeInterval;
    system_tick_t postChargeSettleTime;
    unsigned int lowBatteryStartTime;
    unsigned int lowBatteryDebounceTime;
    unsigned int chargingAwakeEvalTime;
    unsigned int chargingSleepEvalTime;
    uint16_t chargeCurrentHigh;
    uint16_t chargeCurrentLow;
    uint16_t inputCurrent;
    unsigned int failedOtaKeepAwake;
    system_tick_t watchdogExpireTime;
    float memfaultBatteryScaling;
    float memfaultTemperatureScaling;
    float memfaultTemperatureInvalid;
    IGnssLed *pGnssLed;
}EdgePlatformCommonConfiguration;


class IEdgePlatformConfiguration
{
public:
    IEdgePlatformConfiguration()
    {
        commonCfg.lowBatteryCutoff = 2; // percent of battery charge
        commonCfg.lowBatteryCutoffCorrection = 1; // percent of battery charge
        commonCfg.lowBatteryWarning = 8; // percent of battery charge
        commonCfg.lowBatteryWarningHyst = 1; // percent of battery charge
        commonCfg.lowBatteryAwakeEvalInterval = 2 * 60; // seconds to sample for low battery condition
        commonCfg.lowBatterySleepEvalInterval = 1; // seconds to sample for low battery condition
        commonCfg.lowBatterySleepWakeInterval = 15 * 60; // seconds to sample for low battery condition
        commonCfg.postChargeSettleTime = 500; // milliseconds
        commonCfg.lowBatteryStartTime = 20; // seconds to debounce low battery condition
        commonCfg.lowBatteryDebounceTime = 5; // seconds to debounce low battery condition
        commonCfg.chargingAwakeEvalTime = 10; // seconds to sample the PMIC charging state
        commonCfg.chargingSleepEvalTime = 1; // seconds to sample the PMIC charging state
        commonCfg.chargeCurrentHigh = 1536; // milliamps
        commonCfg.chargeCurrentLow = 512; // milliamps
        commonCfg.inputCurrent = 2048; // milliamps
        commonCfg.failedOtaKeepAwake = 60; // seconds to stay awake after failed OTA
        commonCfg.watchdogExpireTime = 60 * 1000; // milliseconds to expire the WDT
        commonCfg.memfaultBatteryScaling = 10.0f; // scaling for battery SOC reporting
        commonCfg.memfaultTemperatureScaling = 10.0f; // scaling for temperature reporting
        commonCfg.memfaultTemperatureInvalid = -300.0f; // invalid temperature
        commonCfg.pGnssLed = nullptr;
    }

    EdgePlatformCommonConfiguration get_common_config_data()
    {
        return commonCfg;
    }

    virtual void load_specific_platform_config() = 0;
protected:
    EdgePlatformCommonConfiguration commonCfg;
};
