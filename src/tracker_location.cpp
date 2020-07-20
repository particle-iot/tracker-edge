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

#include <stdint.h>

#include "Particle.h"
#include "tracker_config.h"
#include "tracker_location.h"

#include "config_service.h"
#include "location_service.h"

TrackerLocation *TrackerLocation::_instance = nullptr;

static constexpr system_tick_t sample_rate = 1000; // milliseconds

static int set_radius_cb(double value, const void *context)
{
    static_cast<LocationService *>((void *)context)->setRadiusThreshold(value);

    return 0;
}

static int get_radius_cb(double &value, const void *context)
{
    float temp;

    static_cast<LocationService *>((void *)context)->getRadiusThreshold(temp);

    value = temp;

    return 0;
}

// when entering the location config object copy from the actual config to the
// shadow for access if writing
int TrackerLocation::enter_location_config_cb(bool write, const void *context)
{
    if(write)
    {
        memcpy(&config_state_shadow, &config_state, sizeof(config_state_shadow));
    }
    return 0;
}

// when exiting the location config object copy from the shadow config to the
// actual if writing (and no error)
int TrackerLocation::exit_location_config_cb(bool write, int status, const void *context)
{
    if(write && !status)
    {
        if(config_state_shadow.interval_min_seconds > config_state_shadow.interval_max_seconds)
        {
            return -EINVAL;
        }
        memcpy(&config_state, &config_state_shadow, sizeof(config_state));
    }
    return status;
}

int TrackerLocation::get_loc_cb(CloudServiceStatus status,
    JSONValue *root,
    const void *context)
{
   triggerLocPub(Trigger::IMMEDIATE);
   return 0;
}

void TrackerLocation::init()
{
    static ConfigObject location_desc
    (
        "location",
        {
            ConfigFloat("radius", get_radius_cb, set_radius_cb, &LocationService::instance()).min(0.0).max(1000000.0),
            ConfigInt("interval_min", config_get_int32_cb, config_set_int32_cb,
                &config_state.interval_min_seconds, &config_state_shadow.interval_min_seconds,
                0, 86400l),
            ConfigInt("interval_max", config_get_int32_cb, config_set_int32_cb,
                &config_state.interval_max_seconds, &config_state_shadow.interval_max_seconds,
                0, 86400l),
            ConfigBool("min_publish",
                config_get_bool_cb, config_set_bool_cb,
                &config_state.min_publish, &config_state_shadow.min_publish)
        },
        std::bind(&TrackerLocation::enter_location_config_cb, this, _1, _2),
        std::bind(&TrackerLocation::exit_location_config_cb, this, _1, _2, _3)
    );

    ConfigService::instance().registerModule(location_desc);

    CloudService::instance().regCommandCallback("get_loc", &TrackerLocation::get_loc_cb, this);

    last_location_publish_sec = System.uptime() - config_state.interval_min_seconds;
}

int TrackerLocation::regLocGenCallback(
    std::function<void(JSONWriter&, LocationPoint &, const void *)> cb,
    const void *context)
{
    locGenCallbacks.append(std::bind(cb, _1, _2, context));
    return 0;
}

// register for callback on location publish success/fail
// these callbacks are NOT persistent and are used for the next publish
int TrackerLocation::regLocPubCallback(
    cloud_service_send_cb_t cb, 
    const void *context)
{
    locPubCallbacks.append(std::bind(cb, _1, _2, _3, context));
    return 0;
}

int TrackerLocation::triggerLocPub(Trigger type, const char *s)
{
    std::lock_guard<std::recursive_mutex> lg(mutex);
    bool matched = false;

    for(auto trigger : pending_triggers)
    {
        if(!strcmp(trigger, s))
        {
            matched = true;
            break;
        }
    }
    
    if(!matched)
    {
        pending_triggers.append(s);
    }

    if(type == Trigger::IMMEDIATE)
    {
        pending_immediate = true;
    }

    return 0;
}

void TrackerLocation::issue_location_publish_callbacks(CloudServiceStatus status, JSONValue *rsp_root, const char *req_event)
{
    for(auto cb : pendingLocPubCallbacks)
    {
        cb(status, rsp_root, req_event);
    }
    pendingLocPubCallbacks.clear();
}

int TrackerLocation::location_publish_cb(CloudServiceStatus status, JSONValue *rsp_root, const char *req_event, const void *context)
{
    bool issue_callbacks = true;

    if(status == CloudServiceStatus::SUCCESS)
    {
        // this could either be on the Particle Cloud ack (default) OR the
        // end-to-end ACK
        Log.info("location cb publish %lu success!", *(uint32_t *) context);
    }
    else if(status == CloudServiceStatus::FAILURE)
    {
        // right now FAILURE only comes out of a Particle Cloud issue
        // once Particle Cloud passes if waiting on end-to-end it will
        // only ever timeout
        
        // save on failure for retry
        if(req_event && !location_publish_retry_str)
        {
            size_t len = strlen(req_event) + 1;
            location_publish_retry_str = (char *) malloc(len);
            if(location_publish_retry_str)
            {
                memcpy(location_publish_retry_str, req_event, len);
                // we've saved for retry, defer callbacks until retry completes
                issue_callbacks = false;
            }
        }
        Log.info("location cb publish %lu failure", *(uint32_t *) context);
    }
    else if(status == CloudServiceStatus::TIMEOUT)
    {
        Log.info("location cb publish %lu timeout", *(uint32_t *) context);
    }
    else
    {
        Log.info("location cb publish %lu unexpected status: %d", *(uint32_t *) context, status);
    }

    if(issue_callbacks)
    {
        issue_location_publish_callbacks(status, rsp_root, req_event);
    }

    return 0;
}

void TrackerLocation::location_publish()
{
    int rval;
    CloudService &cloud_service = CloudService::instance();

    // maintain cloud service lock across the send to allow us to save off
    // the finalized loc publish to retry on failure
    cloud_service.lock();

    if(location_publish_retry_str)
    {
        // publish a retry loc
        rval = cloud_service.send(location_publish_retry_str,
            WITH_ACK,
            CloudServicePublishFlags::FULL_ACK,
            &TrackerLocation::location_publish_cb, this,
            CLOUD_DEFAULT_TIMEOUT_MS, &last_location_publish_sec);
    }
    else
    {
        // publish a new loc (contained in cloud_service buffer)
        rval = cloud_service.send(WITH_ACK,
            CloudServicePublishFlags::FULL_ACK,
            &TrackerLocation::location_publish_cb, this,
            CLOUD_DEFAULT_TIMEOUT_MS, &last_location_publish_sec);
    }

    if(rval == -EBUSY)
    {
        // this implies a transient failure that should recover very
        // quickly (normally another publish in progress blocking lower
        // in the system)
        // save off the generated publish to retry as it has already
        // consumed pending events if applicable
        if(!location_publish_retry_str)
        {
            size_t len = strlen(cloud_service.writer().buffer()) + 1;
            location_publish_retry_str = (char *) malloc(len);
            if(location_publish_retry_str)
            {
                memcpy(location_publish_retry_str, cloud_service.writer().buffer(), len);
            }
            else
            {
                // generated successfuly but unable to save off a copy to retry
                issue_location_publish_callbacks(CloudServiceStatus::FAILURE, NULL, cloud_service.writer().buffer());
            }
        }
    }
    else
    {
        if(rval)
        {
            issue_location_publish_callbacks(CloudServiceStatus::FAILURE, NULL, location_publish_retry_str);
        }
        if(location_publish_retry_str)
        {
            // on success or fatal failure free it
            free(location_publish_retry_str);
            location_publish_retry_str = nullptr;
        }
    }
    cloud_service.unlock();
}

void TrackerLocation::loop()
{
    static system_tick_t last_sample = 0;
    static int last_loc_locked_pub = 0;
    LocationPoint cur_loc;

    //
    // Detect events and add to pending list to publish
    //

    // This loop should only sample as fast as necessary
    if (millis() - last_sample < sample_rate)
    {
        return;
    }
    last_sample = millis();

    if(LocationService::instance().getLocation(cur_loc) != SYSTEM_ERROR_NONE)
    {
        return;
    }

    float radius;
    bool outside;

    if(cur_loc.locked &&
        LocationService::instance().getRadiusThreshold(radius) == SYSTEM_ERROR_NONE &&
        radius &&
        LocationService::instance().isOutsideRadius(outside, cur_loc) == SYSTEM_ERROR_NONE &&
        outside)
    {
        triggerLocPub(Trigger::NORMAL,"radius");
    }

    if(cur_loc.locked != last_loc_locked_pub)
    {
        triggerLocPub(Trigger::NORMAL,"lock");
    }

    //
    // Perform interval evaluation
    //
    bool pub_requested = false;
    if(pending_immediate)
    {
        // request for immediate publish overrides the default min/max interval checking
        pub_requested = true;
        pending_immediate = false;
    }
    else if(config_state.interval_max_seconds && System.uptime() - last_location_publish_sec >= (uint32_t) config_state.interval_max_seconds)
    {
        // max interval and past the max interval so have to publish
        triggerLocPub(Trigger::NORMAL,"time");
        pub_requested = true;
    }
    else if(!config_state.interval_min_seconds || System.uptime() - last_location_publish_sec >= (uint32_t) config_state.interval_min_seconds)
    {
        // no min interval or past the min interval so can publish
        // check radius and imu triggers
        if (pending_triggers.size())
        {
            pub_requested = true;
        }
    }

    //
    // Perform publish of location data if requested
    //

    // first any retry attempts of last loc
    if(location_publish_retry_str && Particle.connected())
    {
        location_publish();
    }

    // then of any new publish
    if(pub_requested)
    {
        if(location_publish_retry_str)
        {
            // retried attempt not completed in time for new publish
            // drop and issue callbacks
            issue_location_publish_callbacks(CloudServiceStatus::TIMEOUT, NULL, location_publish_retry_str);
            free(location_publish_retry_str);
            location_publish_retry_str = nullptr;
        }

        last_location_publish_sec = System.uptime();
        last_loc_locked_pub = cur_loc.locked;
        if(cur_loc.locked)
        {
            LocationService::instance().setWayPoint(cur_loc.latitude, cur_loc.longitude);
        }

        CloudService &cloud_service = CloudService::instance();
        cloud_service.beginCommand("loc");
        cloud_service.writer().name("loc").beginObject();
        if (cur_loc.locked)
        {
            cloud_service.writer().name("lck").value(1);
            cloud_service.writer().name("time").value((unsigned int) cur_loc.epochTime);
            cloud_service.writer().name("lat").value(cur_loc.latitude, 8);
            cloud_service.writer().name("lon").value(cur_loc.longitude, 8);
            if(!config_state.min_publish)
            {
                cloud_service.writer().name("alt").value(cur_loc.altitude, 3);
                cloud_service.writer().name("hd").value(cur_loc.heading, 2);
                cloud_service.writer().name("h_acc").value(cur_loc.horizontalAccuracy, 3);
                cloud_service.writer().name("v_acc").value(cur_loc.verticalAccuracy, 3);
            }
        }
        else
        {
            cloud_service.writer().name("lck").value(0);
        }
        for(auto cb : locGenCallbacks)
        {
            cb(cloud_service.writer(), cur_loc);
        }
        cloud_service.writer().endObject();
        if(!pending_triggers.isEmpty())
        {
            std::lock_guard<std::recursive_mutex> lg(mutex);
            cloud_service.writer().name("trig").beginArray();
            for (auto trigger : pending_triggers) {
                cloud_service.writer().value(trigger);
            }
            pending_triggers.clear();
            cloud_service.writer().endArray();
        }
        Log.info("%.*s", cloud_service.writer().dataSize(), cloud_service.writer().buffer());

        pendingLocPubCallbacks = locPubCallbacks;
        locPubCallbacks.clear();
        location_publish();
    }
}
