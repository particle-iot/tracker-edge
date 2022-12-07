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

#include "IEdgePlatformConfiguration.h"
#include "tracker_config.h"
#include "tracker_user_rgb.h"
#include "Adp8866GnssLed.h"

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
        commonCfg.chargeCurrentHigh = 1536; // milliamps
        commonCfg.inputCurrent = 2048; // milliamps

        if(TrackerUserRGB::instance().init() == SYSTEM_ERROR_NONE)
        {
            TrackerUserRGB::instance().get_rgb2_instance().brightness(80);
            TrackerUserRGB::instance().get_rgb2_instance().setPattern(LED_PATTERN_FADE);
            TrackerUserRGB::instance().get_rgb2_instance().color(0,128,0);
            TrackerUserRGB::instance().get_rgb2_instance().on();
        }
        commonCfg.pGnssLed = new Adp8866GnssLed(TrackerUserRGB::instance().get_rgb1_instance());
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
#if (PLATFORM_ID != PLATFORM_TRACKERM)            
            WITH_LOCK(Wire1)
            {
                WireTransmission config(PMIC_ADDRESS);
                config.timeout(10);
                Wire1.beginTransmission(config);
                Wire1.write(reg);
                Wire1.write(data);
                Wire1.endTransmission();                        
            }
#endif            
            //readbackData = pmic.readChargeTermRegister();
        }
        else
        {
            return;
        }        
    }
};
