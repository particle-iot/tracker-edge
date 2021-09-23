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

#include "location_service.h"

constexpr system_tick_t GNSS_LED_CONTROL_TIMER_PERIOD_MS = 50;
constexpr system_tick_t GNSS_LED_CONTROL_BLINK_PERIOD_MS = 250;

int GnssLedInit();
void GnssLedEnable(bool enable);
void GnssLedError();
