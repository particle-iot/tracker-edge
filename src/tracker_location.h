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
#include "cloud_service.h"
#include "location_service.h"
#include "motion_service.h"

#define TRACKER_LOCATION_INTERVAL_MIN_DEFAULT_SEC (30)
#define TRACKER_LOCATION_INTERVAL_MAX_DEFAULT_SEC (300)
#define TRACKER_LOCATION_MIN_PUBLISH_DEFAULT (false)

struct tracker_location_config_t {
    int32_t interval_min_seconds; // 0 = no min
    int32_t interval_max_seconds; // 0 = no max
    bool min_publish;
};

enum PendingEvent : uint32_t {
    EVENT_TIME                  = (1UL << 0),
    EVENT_RADIUS                = (1UL << 1),
    EVENT_IMU_M                 = (1UL << 2),
    EVENT_IMU_G                 = (1UL << 3),
    EVENT_TEMP_H                = (1UL << 4),
    EVENT_TEMP_L                = (1UL << 5),
    EVENT_LOCK                  = (1UL << 6),
    EVENT_USER                  = (1UL << 7),
    // special flag to indicate events should disregard standard interval
    // filtering
    EVENT_IMMEDIATE             = (1UL << 31),
};

class TrackerLocation
{
    public:
        TrackerLocation(ConfigService &config, CloudService &cloud, LocationService &location, MotionService &motion) :
            config_service(config),
            cloud_service(cloud),
            location_service(location),
            motion_service(motion),
            pending_events(0),
            location_publish_retry_str(nullptr)
        {
            config_state = {
                .interval_min_seconds = TRACKER_LOCATION_INTERVAL_MIN_DEFAULT_SEC,
                .interval_max_seconds = TRACKER_LOCATION_INTERVAL_MAX_DEFAULT_SEC,
                .min_publish = TRACKER_LOCATION_MIN_PUBLISH_DEFAULT
            };
        }

        void init();
        void loop();

        // register for callback during generation of location publish allowing
        // for insertion of custom fields into the output
        // these callbacks are persistent and not removed on generation
        int regLocGenCallback(
            std::function<void(JSONWriter&, LocationPoint &, const void *)>,
            const void *context=nullptr);

        template <typename T>
        int regLocGenCallback(
            void (T::*cb)(JSONWriter&, LocationPoint &, const void *),
            T *instance,
            const void *context=nullptr);

        // register for callback on location publish success/fail
        // these callbacks are NOT persistent and are used for the next publish
        int regLocPubCallback(
            cloud_service_send_cb_t cb, 
            const void *context=nullptr);

        template <typename T>
        int regLocPubCallback(
            int (T::*cb)(CloudServiceStatus status, JSONValue *, const char *, const void *context),
            T *instance,
            const void *context=nullptr);

        int triggerLocPub(bool immediate = false);

    private:
        ConfigService &config_service;
        CloudService &cloud_service;
        LocationService &location_service;
        MotionService &motion_service;

        std::atomic<uint32_t> pending_events;

        char *location_publish_retry_str;

        int enter_location_config_cb(bool write, const void *context);
        int exit_location_config_cb(bool write, int status, const void *context);

        int location_publish_cb(CloudServiceStatus status, JSONValue *, const char *req_event, const void *context);

        void issue_location_publish_callbacks(CloudServiceStatus status, JSONValue *, const char *req_event);

        void location_publish();

        uint32_t last_location_publish_sec;

        tracker_location_config_t config_state, config_state_shadow;

        Vector<std::function<void(JSONWriter&, LocationPoint&)>> locGenCallbacks;
        // publish callback for the next publish (not in flight)
        Vector<std::function<void(CloudServiceStatus status, JSONValue *, const char *)>> locPubCallbacks;
        // publish callbacks for the current/pending publish (in flight)
        Vector<std::function<void(CloudServiceStatus status, JSONValue *, const char *)>> pendingLocPubCallbacks;
};

template <typename T>
int TrackerLocation::regLocGenCallback(
    void (T::*cb)(JSONWriter&, LocationPoint &, const void *),
    T *instance,
    const void *context)
{
    return regLocGenCallback(std::bind(cb, instance, _1, _2), context);
}

template <typename T>
int TrackerLocation::regLocPubCallback(
    int (T::*cb)(CloudServiceStatus status, JSONValue *, const char *, const void *context),
    T *instance,
    const void *context)
{
    return regLocPubCallback(std::bind(cb, instance, _1, _2, _3), context);
}