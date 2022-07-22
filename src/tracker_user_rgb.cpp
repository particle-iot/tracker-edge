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


#include "tracker_user_rgb.h"
#include "Particle.h"
using namespace particle;

constexpr int resetPin                      = PIN_INVALID;          // ADP8866 Reset PIN

constexpr IscLed rgb1_r                     = IscLed::LED2;         // RGB1 Red channel
constexpr IscLed rgb1_g                     = IscLed::LED1;         // RGB1 green channel
constexpr IscLed rgb1_b                     = IscLed::LED3;         // RGB1 blue channel

constexpr IscLed rgb2_r                     = IscLed::LED8;         // RGB2 Red channel
constexpr IscLed rgb2_g                     = IscLed::LED7;         // RGB2 green channel
constexpr IscLed rgb2_b                     = IscLed::LED9;         // RGB2 blue channel

TrackerUserRGB *TrackerUserRGB::_instance   = nullptr;

int TrackerUserRGB::init()
{
    pAdp8866Drv = new ADP8866(Wire,resetPin);
    CHECK_TRUE(pAdp8866Drv,SYSTEM_ERROR_INVALID_STATE);
    pRGB1 = new ADP8866_RGB(rgb1_r,rgb1_g,rgb1_b,pAdp8866Drv);
    CHECK_TRUE(pRGB1,SYSTEM_ERROR_INVALID_STATE);
    pRGB2 = new ADP8866_RGB(rgb2_r,rgb2_g,rgb2_b,pAdp8866Drv);
    CHECK_TRUE(pRGB2,SYSTEM_ERROR_INVALID_STATE);
    pRGB1->off();
    pRGB2->off();
    return SYSTEM_ERROR_NONE;
}

ADP8866_RGB &TrackerUserRGB::get_rgb1_instance()
{
    return *pRGB1;
}

ADP8866_RGB &TrackerUserRGB::get_rgb2_instance()
{
    return *pRGB2;
}