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

#include "tracker_motion.h"
#include "tracker_location.h"
#include "tracker_sleep.h"

#include "config_service.h"
#include "motion_service.h"

TrackerMotion *TrackerMotion::_instance = nullptr;

static int get_motion_enabled_cb(int32_t &value, const void *context)
{
    value = (int32_t) static_cast<MotionService *>((void *)context)->getMotionDetection();
    return 0;
}

static int set_motion_enabled_cb(int32_t value, const void *context)
{
    MotionService *motion_service = static_cast<MotionService *>((void *)context);

    motion_service->enableMotionDetection((MotionDetectionMode) value);
    if ((MotionDetectionMode)value == MotionDetectionMode::NONE) {
        if (!motion_service->isAnyAwake()) {
            TrackerSleep::instance().ignore((pin_t)BMI160_INT_PIN);
        }
    }
    else {
        TrackerSleep::instance().wakeFor((pin_t)BMI160_INT_PIN, BMI160_INT_MODE);
    }
    return 0;
}

static int get_high_g_enabled_cb(int32_t &value, const void *context)
{
    value = (int32_t) static_cast<MotionService *>((void *)context)->getHighGDetection();
    return 0;
}

static int set_high_g_enabled_cb(int32_t value, const void *context)
{
    MotionService *motion_service = static_cast<MotionService *>((void *)context);

    if(value == (int32_t) HighGDetectionMode::DISABLE)
    {
        motion_service->disableHighGDetection();
        if (!motion_service->isAnyAwake()) {
            TrackerSleep::instance().ignore((pin_t)BMI160_INT_PIN);
        }
    }
    else if(value == (int32_t) HighGDetectionMode::ENABLE)
    {
        motion_service->enableHighGDetection();
        TrackerSleep::instance().wakeFor((pin_t)BMI160_INT_PIN, BMI160_INT_MODE);
    }
    else
    {
        return -EINVAL;
    }

    return 0;
}

void TrackerMotion::init()
{
    static ConfigObject imu_desc
    (
        "imu_trig",
        {
            ConfigStringEnum(
                "motion",
                {
                    {"disable", (int32_t) MotionDetectionMode::NONE},
                    {"low", (int32_t) MotionDetectionMode::LOW_SENSITIVITY},
                    {"medium", (int32_t) MotionDetectionMode::MEDIUM_SENSITIVITY},
                    {"high", (int32_t) MotionDetectionMode::HIGH_SENSITIVITY},
                },
                get_motion_enabled_cb,
                set_motion_enabled_cb,
                &MotionService::instance()
            ),
            ConfigStringEnum(
                "high_g",
                {
                    {"disable", (int32_t) HighGDetectionMode::DISABLE},
                    {"enable", (int32_t) HighGDetectionMode::ENABLE},
                },
                get_high_g_enabled_cb,
                set_high_g_enabled_cb,
                &MotionService::instance()
            ),
        }
    );

    ConfigService::instance().registerModule(imu_desc);
}

void TrackerMotion::loop()
{
    MotionEvent motion_event;
    size_t depth = MotionService::instance().getQueueDepth();

    do {
        MotionService::instance().waitOnEvent(motion_event, 0);
        switch (motion_event.source)
        {
            case MotionSource::MOTION_HIGH_G:
                TrackerLocation::instance().triggerLocPub(Trigger::NORMAL, "imu_g");
                break;
            case MotionSource::MOTION_MOVEMENT:
                TrackerLocation::instance().triggerLocPub(Trigger::NORMAL,"imu_m");
                break;
        }
    } while (--depth && (motion_event.source != MotionSource::MOTION_NONE));
}
