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

#include "Particle.h"
#include "gnss_led.h"
#include "tracker_config.h"
#include "tracker_user_rgb.h"


#define TRACKER_GNSS_LOCK_LED_INSTANCE                 TrackerUserRGB::instance().get_rgb1_instance() 

static void GnssLedTimer();

static Timer* timer_ = nullptr;
static bool enabled = true;
static LocationStatus lastStatus_ = { .powered = -1, .locked = -1, .error = -1 };
static int blinkCount_ = GNSS_LED_CONTROL_BLINK_PERIOD_MS / GNSS_LED_CONTROL_TIMER_PERIOD_MS;
static bool blinkState_ = false;

static void GnssLedTimer() {

    if (!enabled) {
        TRACKER_GNSS_LOCK_LED_INSTANCE.off();
        lastStatus_ = { .powered = -1, .locked = -1 };
        return;
    }

    LocationStatus status = {0};
    (void)LocationService::instance().getStatus(status);

    SCOPE_GUARD({
        lastStatus_ = status;
    });

    if (lastStatus_.powered == -1) {
        return;
    }

    if (status.error) {
        if(blinkState_)
        {
            TRACKER_GNSS_LOCK_LED_INSTANCE.on();
        }
        else
        {
            TRACKER_GNSS_LOCK_LED_INSTANCE.off();
        }
        blinkState_ = !blinkState_;
        return;
    }

    if ((lastStatus_.powered != status.powered) ||
        (lastStatus_.locked != status.locked)) {
        blinkCount_ = GNSS_LED_CONTROL_BLINK_PERIOD_MS / GNSS_LED_CONTROL_TIMER_PERIOD_MS;
        blinkState_ = false;
    }

    if (status.powered == 0) {
        TRACKER_GNSS_LOCK_LED_INSTANCE.off();
    }
    else if (status.locked) {
        TRACKER_GNSS_LOCK_LED_INSTANCE.on();
    }
    else {
        if (blinkCount_ == 0) {
            blinkCount_ = GNSS_LED_CONTROL_BLINK_PERIOD_MS / GNSS_LED_CONTROL_TIMER_PERIOD_MS;
            if(blinkState_)
            {
                TRACKER_GNSS_LOCK_LED_INSTANCE.on();
            }
            else
            {
                TRACKER_GNSS_LOCK_LED_INSTANCE.off();
            }
            blinkState_ = !blinkState_;
        }
        else {
            blinkCount_--;
        }
    }
}


int GnssLedInit() {
    TRACKER_GNSS_LOCK_LED_INSTANCE.setPattern(LED_PATTERN_SOLID);
    TRACKER_GNSS_LOCK_LED_INSTANCE.brightness(80);
    TRACKER_GNSS_LOCK_LED_INSTANCE.color(0,128,0);    
    TRACKER_GNSS_LOCK_LED_INSTANCE.off();
    enabled = false;

    timer_ = new Timer(GNSS_LED_CONTROL_TIMER_PERIOD_MS, GnssLedTimer);
    if (timer_ == nullptr) {
        return SYSTEM_ERROR_NO_MEMORY;
    }

    timer_->start();

    return SYSTEM_ERROR_NONE;
}

void GnssLedEnable(bool enable) {
    enabled = enable;
    if (!enabled) {
        TRACKER_GNSS_LOCK_LED_INSTANCE.off();
    }
}

void GnssLedError() {
    enabled = false;
}
