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

#include "IEdgePlatformConfiguration.hpp"


enum class TrackerPmicChargeTimer {
    CHARGE_00_05_HOURS = 0,         // 00 – 5  hrs
    CHARGE_01_08_HOURS,             // 01 – 8  hrs
    CHARGE_10_12_HOURS,             // 10 – 12 hrs
    CHARGE_11_20_HOURS,             // 11 – 20 hrs
};


class MonitorOneConfiguration: public IEdgePlatformConfiguration
{
public:
    MonitorOneConfiguration()
    {
        commonCfg.canSleepRetries = 10; // Based on a series of 10ms delays
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
        Log.info("### %s ###",__FUNCTION__);
    }

    void load_specific_platform_config()
    {
        Log.info("### %s ###",__FUNCTION__);        
        updatePmicChargeTimer(TrackerPmicChargeTimer::CHARGE_11_20_HOURS);
    }

private:
    // Update PMIC Charge Timer
    void updatePmicChargeTimer(TrackerPmicChargeTimer timer)
    {
        uint8_t reg = CHARGE_TIMER_CONTROL_REGISTER,data = 0x00;
        //,readbackData = 0x00;
        PMIC pmic(true);
        data = pmic.readChargeTermRegister();
        TrackerPmicChargeTimer defaultTimer = static_cast<TrackerPmicChargeTimer>((data >> 1) & 0x03);
        //Log.info("%s: Read REG05 == 0x%02X, ChargeTimerType == %ld",__FUNCTION__,data,defaultTimer);
        if (defaultTimer != timer)
        {
            data &= 0xF9;
            data |= (static_cast<uint8_t>(timer) << 1) & 0x06;
            WITH_LOCK(Wire1)
            {
                WireTransmission config(PMIC_ADDRESS);
                config.timeout(10);
                Wire1.beginTransmission(config);
                Wire1.write(reg);
                Wire1.write(data);
                Wire1.endTransmission();                        
            }
            //readbackData = pmic.readChargeTermRegister();
        }
        else
        {
            return;
        }        
    }
};
