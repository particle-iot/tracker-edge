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
#include "IoGnssLed.h"

class TrackerOneConfiguration: public IEdgePlatformConfiguration
{
public:
    TrackerOneConfiguration()
    {
        commonCfg.chargeCurrentHigh = 1024; // milliamps
        commonCfg.inputCurrent = 1500; // milliamps
        commonCfg.pGnssLed = new IoGnssLed(TRACKER_GNSS_LOCK_LED);
    }

    void load_specific_platform_config()
    {
    }
protected:

};
