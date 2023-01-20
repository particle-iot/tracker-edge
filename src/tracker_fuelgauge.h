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

class TrackerFuelGauge
{
    public:
        static TrackerFuelGauge &instance()
        {
            if(!_instance)
            {
                _instance = new TrackerFuelGauge();
            }
            return *_instance;
        }
        void init();
        void loop();

        /**
         * @brief get soc percentage
         * @return soc percentage value
         */
        float getSoC();

        /**
         * @brief get battery voltage
         * @return voltage value
         */
        float getVolt();
#ifdef FUEL_GAUGE_TEST
        void enable_publish_pmic_regs(bool enable);
#endif
    private:
        TrackerFuelGauge() {}
        void verify_model();
#ifdef FUEL_GAUGE_TEST
        void test();
        bool publishPMICRegs = false;
#endif
        static TrackerFuelGauge *_instance;
        uint32_t last_1h = 0;
        uint32_t verifyCount {};
        uint32_t verifyFail {};
};
