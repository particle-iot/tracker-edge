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
#include <fcntl.h>
#include <sys/stat.h>

TrackerLocation *TrackerLocation::_instance = nullptr;

static constexpr system_tick_t LoopSampleRate = 1000; // milliseconds
static constexpr uint32_t EarlySleepSec = 2; // seconds
static constexpr uint32_t MiscSleepWakeSec = 3; // seconds - miscellaneous time spent by system entering and exiting sleep
static constexpr uint32_t LockTimeoutSec = 10; // seconds - time to wait for GNSS lock (sleep disabled)

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
                &config_state.min_publish, &config_state_shadow.min_publish),
            ConfigBool("lock_trigger",
                config_get_bool_cb, config_set_bool_cb,
                &config_state.lock_trigger, &config_state_shadow.lock_trigger),
            ConfigBool("loc_ack",
                config_get_bool_cb, config_set_bool_cb,
                &config_state.process_ack, &config_state_shadow.process_ack),
            ConfigBool("offline_storage",
                config_get_bool_cb, config_set_bool_cb,
                &config_state.offline_storage, &config_state_shadow.offline_storage),
            ConfigInt("offline_kbytes",
                config_get_int32_cb, config_set_int32_cb,
                &config_state.kbytes_storage, &config_state_shadow.kbytes_storage,
                1, 4096l)
        },
        std::bind(&TrackerLocation::enter_location_config_cb, this, _1, _2),
        std::bind(&TrackerLocation::exit_location_config_cb, this, _1, _2, _3)
    );

    ConfigService::instance().registerModule(location_desc);

    CloudService::instance().regCommandCallback("get_loc", &TrackerLocation::get_loc_cb, this);

    _last_location_publish_sec = System.uptime() - config_state.interval_min_seconds;

    _sleep.registerSleepPrepare([this](TrackerSleepContext context){ this->onSleepPrepare(context); });
    _sleep.registerSleep([this](TrackerSleepContext context){ this->onSleep(context); });
    _sleep.registerSleepCancel([this](TrackerSleepContext context){ this->onSleepCancel(context); });
    _sleep.registerWake([this](TrackerSleepContext context){ this->onWake(context); });
    _sleep.registerStateChange([this](TrackerSleepContext context){ this->onSleepState(context); });
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
    std::lock_guard<RecursiveMutex> lg(mutex);
    bool matched = false;

    for(auto trigger : _pending_triggers)
    {
        if(!strcmp(trigger, s))
        {
            matched = true;
            break;
        }
    }

    if(!matched)
    {
        _pending_triggers.append(s);
    }

    if(type == Trigger::IMMEDIATE)
    {
        _pending_immediate = true;
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
        _first_publish = false;
        _pending_first_publish = false;
    }
    else if(status == CloudServiceStatus::FAILURE)
    {
        // right now FAILURE only comes out of a Particle Cloud issue
        // once Particle Cloud passes if waiting on end-to-end it will
        // only ever timeout

        // on failure, save to file if enabled. Save to RAM for retry if
        // offline storage is not enabled, or it fails to store.
        if(req_event && config_state.offline_storage && (storeLocationToFile(req_event) == (int)(strlen(req_event) + 3)) )
        {
            issue_callbacks = false;
        }
        else if(req_event && !location_publish_retry_str)
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

    CloudServicePublishFlags cloud_flags =
        (config_state.process_ack) ? CloudServicePublishFlags::FULL_ACK : CloudServicePublishFlags::NONE;

    if(location_publish_retry_str)
    {
        // publish a retry loc
        rval = cloud_service.send(location_publish_retry_str,
            WITH_ACK,
            cloud_flags,
            &TrackerLocation::location_publish_cb, this,
            CLOUD_DEFAULT_TIMEOUT_MS, &_last_location_publish_sec);
    }
    else
    {
        // publish a new loc (contained in cloud_service buffer)
        rval = cloud_service.send(WITH_ACK,
            cloud_flags,
            &TrackerLocation::location_publish_cb, this,
            CLOUD_DEFAULT_TIMEOUT_MS, &_last_location_publish_sec);
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

void TrackerLocation::enableNetworkGnss() {
    LocationService::instance().start();
    _sleep.forceFullWakeCycle();
    _gnssStartedSec = System.uptime();
}

void TrackerLocation::disableGnss() {
    LocationService::instance().stop();
}

bool TrackerLocation::isSleepEnabled() {
    return !_sleep.isSleepDisabled();
}

int TrackerLocation::storeLocationToFile(const char* req_event) {
    int fd = open(TRACKER_LOCATION_OFFLINE_STORAGE_FILE, O_WRONLY | O_CREAT | O_APPEND);
    if(fd == -1) {
        Log.error("File not open for writing");
        return SYSTEM_ERROR_FILE;
    }
    struct stat buf;
    int retval = fstat(fd, &buf);
    if (retval == 0) {
        if(buf.st_size < config_state.kbytes_storage*1024) {
            uint16_t size = strlen(req_event)+1;
            retval = write(fd, req_event, size);
            Log.trace("Previous file size: %ld. Wrote %d bytes to file", buf.st_size, retval);
            if (write(fd, (void *)&size, 2) == 2) {
                retval+=2;
            } else {
                retval = SYSTEM_ERROR_FILE;
            }
        } else {
            retval = SYSTEM_ERROR_TOO_LARGE;
        }
    } else {
        retval = SYSTEM_ERROR_FILE;
    }
    close(fd);
    return retval;
}

int TrackerLocation::getLocationFromFile() {
    Log.info("Opening file: %s", TRACKER_LOCATION_OFFLINE_STORAGE_FILE);
    int fd = open(TRACKER_LOCATION_OFFLINE_STORAGE_FILE, O_RDWR);
    if(fd == -1) {
        Log.error("File not found for reading");
        return SYSTEM_ERROR_FILE;
    }
    if (lseek(fd, -2, SEEK_END) < 0) goto read_error;
    uint16_t len;
    if (read(fd, (void *)&len, 2) == -1) goto read_error;
    Log.trace("Length of publish: %d", len);
    if(!location_publish_retry_str) {
        location_publish_retry_str = (char *) malloc(len);
        if (location_publish_retry_str) {
            if(lseek(fd, -2-len, SEEK_END) < 0) goto read_error;
            if(read(fd, location_publish_retry_str, len) < len) goto read_error;
            struct stat stat_buf;
            if(fstat(fd, &stat_buf) == -1) goto read_error;
            if(ftruncate(fd, stat_buf.st_size - len - 2) == -1) goto read_error;
        } else {
            Log.error("Failed to allocate memory for retry");
            close(fd);
            return SYSTEM_ERROR_NO_MEMORY;
        }
    }
    close(fd);
    return SYSTEM_ERROR_NONE;
read_error:
    Log.error("Error accessing file");
    close(fd);
    return SYSTEM_ERROR_FILE;
}

EvaluationResults TrackerLocation::evaluatePublish() {
    auto now = System.uptime();

    if (_pending_immediate) {
        // request for immediate publish overrides the default min/max interval checking
        Log.trace("%s pending_immediate", __FUNCTION__);
        return EvaluationResults {PublishReason::IMMEDIATE, true, false};
    }

    // This will allow a trigger publish on boot.
    // This may be pre-emptively published due to connect and execute times if sleep is enabled.
    // If sleep is disabled then timeout after some time.
    if (_first_publish && !_pending_first_publish) {
        Log.trace("%s first", __FUNCTION__);
        return EvaluationResults {PublishReason::TRIGGERS, true, (now - _gnssStartedSec) < (uint32_t)_sleep.getConfigConnectingTime()};
    }

    uint32_t interval = now - _last_location_publish_sec;
    uint32_t maxInterval = now - _monotonic_publish_sec;

    bool networkNeeded = false;
    uint32_t max = (uint32_t)config_state.interval_max_seconds;
    auto maxNetwork = max;
    if  (maxNetwork > (uint32_t)_nextEarlyWake) {
        maxNetwork -= (uint32_t)_nextEarlyWake;
    }

    if (config_state.interval_max_seconds) {
        if (maxInterval >= maxNetwork) {
            // max interval adjusted for early wake
            Log.trace("%s maxNetwork", __FUNCTION__);
            networkNeeded = true;
        }

        if (maxInterval >= max) {
            // max interval and past the max interval so have to publish
            Log.trace("%s max", __FUNCTION__);
            // timeout may be pre-empted when sleep enabled
            return EvaluationResults {PublishReason::TIME, true, (maxInterval - max) < LockTimeoutSec};
        }
    }

    uint32_t min = (uint32_t)config_state.interval_min_seconds;
    auto minNetwork = min;
    if  (minNetwork > (uint32_t)_nextEarlyWake) {
        minNetwork -= (uint32_t)_nextEarlyWake;
    }

    if (_pending_triggers.size()) {
        if (!config_state.interval_min_seconds ||
            (interval >= minNetwork)) {
            // min interval adjusted for early wake
            Log.trace("%s minNetwork", __FUNCTION__);
            networkNeeded = true;
        }

        if (!config_state.interval_min_seconds ||
            (interval >= min)) {
            // no min interval or past the min interval so can publish
            Log.trace("%s min", __FUNCTION__);
            // timeout may be pre-empted when sleep enabled
            return EvaluationResults {PublishReason::TRIGGERS, true, (interval - min) < LockTimeoutSec};
        }
    }

    if (!networkNeeded) {
        // If we have a file with saved loc points, and there's no min interval or
        // we're past the min interval (either total or adjusted for early wake),
        // then ask for network to turn on to publish the stored points.
        struct stat buf;
        if( (stat(TRACKER_LOCATION_OFFLINE_STORAGE_FILE, &buf) == 0 && buf.st_size > 2) &&
            (!config_state.interval_min_seconds || (interval >= minNetwork || interval >= min)) ) 
        {
            Log.trace("%s cached loc", __FUNCTION__);
            networkNeeded = true;
        }

    }

    return EvaluationResults {PublishReason::NONE, networkNeeded, false};
}

// The purpose of thhe sleep prepare callback is to allow each task to calculate
// the next time it needs to wake and process inputs, publish, and what not.
void TrackerLocation::onSleepPrepare(TrackerSleepContext context) {
    // The first thing to figure out is the needed interval, min or max
    unsigned int wake = _last_location_publish_sec;
    int32_t interval = 0;
    if (_pending_triggers.size()) {
        interval = config_state.interval_min_seconds;
        wake += interval;
    }
    else {
        interval = config_state.interval_max_seconds;
        wake += interval;
    }

    // Next calculate the early wake offset so that we can wake in the minimum amount of time before
    // the next publish in order to minimize time spent in fully powered operation
    auto t_conn = (uint32_t)_sleep.getConfigConnectingTime();
    if (_sleep.isFullWakeCycle()) {
        uint32_t newEarlyWakeSec = 0;
        uint32_t lastWakeSec = (uint32_t)(context.lastWakeMs + 500) / 1000; // Round ms to s
        if (lastWakeSec >= MiscSleepWakeSec) {
            lastWakeSec -= MiscSleepWakeSec;
        }
        uint32_t wakeToLockDurationSec = 0;
        int32_t publishVariance = 0;

        if (_firstLockSec == 0) {
            wakeToLockDurationSec = t_conn;
        }
        else {
            wakeToLockDurationSec = _firstLockSec - lastWakeSec;
        }

        publishVariance = (int32_t)_last_location_publish_sec - (int32_t)_monotonic_publish_sec;

        newEarlyWakeSec = wakeToLockDurationSec + publishVariance + 1;
        if (newEarlyWakeSec > t_conn) {
            newEarlyWakeSec = t_conn;
        }
        _nextEarlyWake = _earlyWake = newEarlyWakeSec;
    }
    else {  // Not in full wake (modem on)
        _nextEarlyWake = (_earlyWake == 0) ? t_conn : _earlyWake;
    }

    // If the interval and early adjustments puts the wake time in the past then
    // spoil the next sleep attempt and stay awake.
    if (wake > _nextEarlyWake)
        wake -= _nextEarlyWake;

    TrackerSleepError wakeRet = _sleep.wakeAtSeconds(wake);

    if (wakeRet == TrackerSleepError::TIME_IN_PAST) {
        wake = 0; // Force cancelled sleep
        _sleep.wakeAtSeconds(wake);
    }

    Log.trace("TrackerLocation: last=%lu, interval=%ld, wake=%u", _last_location_publish_sec, interval, wake);
}

// The purpose of this callback is to alert us that sleep has been cancelled by another task or improper wake settings.
void TrackerLocation::onSleepCancel(TrackerSleepContext context) {

}

// This callback will alert us that the system is just about to go to sleep.  This is past of the point
// of no return to cancel the pending sleep cycle.
void TrackerLocation::onSleep(TrackerSleepContext context) {
    disableGnss();
}

// This callback will be called immediately after wake from sleep and allows us to figure out if the network interface
// is needed and enable it if so.
void TrackerLocation::onWake(TrackerSleepContext context) {
    // Allow capturing of the first lock instance
    _firstLockSec = 0;

    auto result = evaluatePublish();

    if (result.networkNeeded) {
        enableNetworkGnss();
        Log.trace("%s needs to start the network", __FUNCTION__);
    }
    else {
        // Put in our vote to shutdown early
        _sleep.extendExecutionFromNow(EarlySleepSec, true);
    }

    // Ensure the loop runs immediately
    _loopSampleTick = 0;
}

void TrackerLocation::onSleepState(TrackerSleepContext context) {
    switch (context.reason) {
        case TrackerSleepReason::STATE_TO_CONNECTING: {
            Log.trace("%s starting GNSS", __FUNCTION__);
            LocationService::instance().start();
            break;
        }

        case TrackerSleepReason::STATE_TO_SHUTDOWN: {
            Log.trace("%s stopping GNSS for shutdown", __FUNCTION__);
            disableGnss();
            break;
        }
    }
}

GnssState TrackerLocation::loopLocation(LocationPoint& cur_loc) {
    static GnssState lastGnssState = GnssState::OFF;
    GnssState currentGnssState = GnssState::ON_LOCKED_STABLE;

    LocationStatus locStatus;
    LocationService::instance().getStatus(locStatus);

    do {
        if (!locStatus.powered) {
            currentGnssState = GnssState::OFF;
            break;
        }

        if (LocationService::instance().getLocation(cur_loc) != SYSTEM_ERROR_NONE)
        {
            currentGnssState = GnssState::ERROR;
            break;
        }

        if (!cur_loc.locked) {
            currentGnssState = GnssState::ON_UNLOCKED;
            break;
        }

        if (!cur_loc.stable) {
            currentGnssState = GnssState::ON_LOCKED_UNSTABLE;
            break;
        }

        float radius;
        LocationService::instance().getRadiusThreshold(radius);
        if (radius) {
            bool outside;
            LocationService::instance().isOutsideRadius(outside, cur_loc);
            if (outside) {
                triggerLocPub(Trigger::NORMAL,"radius");
            }
        }
    } while (false);

    // Detect GNSS locked changes
    if ((currentGnssState == GnssState::ON_LOCKED_STABLE) &&
        (currentGnssState != lastGnssState)) {

        // Capture the time that the first lock out of sleep happened
        if (_firstLockSec == 0) {
            _firstLockSec = System.uptime();
        }

        // Only publish with "lock" trigger when not sleeping and when enabled to do so
        if (_sleep.isSleepDisabled() && config_state.lock_trigger) {
            triggerLocPub(Trigger::NORMAL,"lock");
        }
    }

    lastGnssState = currentGnssState;

    return currentGnssState;
}

void TrackerLocation::buildPublish(LocationPoint& cur_loc) {
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
            cloud_service.writer().name("spd").value(cur_loc.speed, 2);
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
    if(!_pending_triggers.isEmpty())
    {
        std::lock_guard<RecursiveMutex> lg(mutex);
        cloud_service.writer().name("trig").beginArray();
        for (auto trigger : _pending_triggers) {
            cloud_service.writer().value(trigger);
        }
        _pending_triggers.clear();
        cloud_service.writer().endArray();
    }
    Log.info("%.*s", cloud_service.writer().dataSize(), cloud_service.writer().buffer());
}

void TrackerLocation::loop() {
    // The rest of this loop should only sample as fast as necessary
    if (millis() - _loopSampleTick < LoopSampleRate)
    {
        return;
    }
    _loopSampleTick = millis();

    // First take care of any retry attempts of last loc
    if (location_publish_retry_str && Particle.connected())
    {
        Log.info("retry failed publish");
        location_publish();
    }

    // Gather current location information and status
    LocationPoint cur_loc;
    auto locationStatus = loopLocation(cur_loc);

    // Perform interval evaluation
    auto publishReason = evaluatePublish();

    // This evaluation may have performed earlier and determined that no network was needed.  Check again
    // because this loop may overlap with required network operations.
    if (!_sleep.isFullWakeCycle() && publishReason.networkNeeded) {
        enableNetworkGnss();
    }

    bool publishNow = false;

    //                                   : NONE      TIME        TRIG        IMM
    //                                    ----------------------------------------
    // GnssState::OFF                       NA       PUB         PUB         PUB
    // GnssState::ON_UNLOCKED               NA       WAIT        WAIT        PUB
    // GnssState::ON_LOCKED_UNSTABLE        NA       WAIT        WAIT        PUB
    // GnssState::ON_LOCKED_STABLE          NA       PUB         PUB         PUB

    switch (publishReason.reason) {
        case PublishReason::NONE: {
            // If there is nothing to do then break, to possibly go into publishing
            // cached publishes if they exist.
            break;
        }

        case PublishReason::TIME: {
            switch (locationStatus) {
                case GnssState::ON_LOCKED_STABLE: {
                    Log.trace("publishing from max interval");
                    triggerLocPub(Trigger::NORMAL,"time");
                    publishNow = true;
                    break;
                }

                case GnssState::OFF:
                // fall through
                case GnssState::ON_UNLOCKED:
                // fall through
                case GnssState::ON_LOCKED_UNSTABLE: {
                    if (!publishReason.lockWait) {
                        Log.trace("publishing from max interval after waiting");
                        triggerLocPub(Trigger::NORMAL,"time");
                        publishNow = true;
                        break;
                    }
                    Log.trace("waiting for stable GNSS lock for max interval");
                    break;
                }
            }
            break;
        }

        case PublishReason::TRIGGERS: {
            switch (locationStatus) {
                case GnssState::ON_LOCKED_STABLE: {
                    Log.trace("publishing from triggers");
                    publishNow = true;
                    _newMonotonic = true;
                    break;
                }

                case GnssState::OFF:
                // fall through
                case GnssState::ON_UNLOCKED:
                // fall through
                case GnssState::ON_LOCKED_UNSTABLE: {
                    if (!publishReason.lockWait) {
                        Log.trace("publishing from triggers after waiting");
                        publishNow = true;
                        _newMonotonic = true;
                        break;
                    }
                    Log.trace("waiting for stable GNSS lock for triggers");
                    break;
                }
            }
            break;
        }

        case PublishReason::IMMEDIATE: {
            Log.trace("publishing from immediate");
            _pending_immediate = false;
            publishNow = true;
            _newMonotonic = true;
            break;
        }
    }

    //
    // Perform publish of location data if requested
    //

    // then of any new publish
    if(publishNow && ( config_state.offline_storage || Particle.connected() ) )
    {
        if(location_publish_retry_str)
        {
            Log.info("freeing unsuccessful retry");
            // retried attempt not completed in time for new publish
            // drop and issue callbacks
            issue_location_publish_callbacks(CloudServiceStatus::TIMEOUT, NULL, location_publish_retry_str);
            free(location_publish_retry_str);
            location_publish_retry_str = nullptr;
        }
        Log.info("publishing now...");
        buildPublish(cur_loc);
        pendingLocPubCallbacks = locPubCallbacks;
        locPubCallbacks.clear();
        _last_location_publish_sec = System.uptime();
        if ((_first_publish && !_pending_first_publish) || _newMonotonic)
        {
            _monotonic_publish_sec = _last_location_publish_sec;
            _newMonotonic = false;
        }
        else
        {
            _monotonic_publish_sec += (uint32_t)config_state.interval_max_seconds;
        }

        // Prevent flooding of first publishes when there are no acknowledges.
        if (!config_state.process_ack && _first_publish) {
            _first_publish = false;
        }

        location_publish();

        // There may be a delay between the first event being published and an acknowledgement
        // from the cloud.  This leads to multiple event publishes meant to be the first publish.
        if (_first_publish && !_pending_first_publish) {
            _pending_first_publish = true;
        }
    }

    // If there are no publishes, check for cached points to send to the cloud
    if(!publishNow && Particle.connected()) {
        struct stat buf;
        if(stat(TRACKER_LOCATION_OFFLINE_STORAGE_FILE, &buf) == 0 && buf.st_size > 2) {
            Log.trace("Publishing a cached loc. File size is %ld bytes", buf.st_size);
            if (getLocationFromFile() == SYSTEM_ERROR_NONE) location_publish();
        }
    }
}
