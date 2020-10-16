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

#include <mutex>

using namespace std::placeholders;

#include <spark_wiring_thread.h>
#include <protocol_defs.h>

typedef enum {
    BACKGROUND_PUBLISH_IDLE = 0,
    BACKGROUND_PUBLISH_REQUESTED,
    BACKGROUND_PUBLISH_STOP,
} publish_thread_state_t;

typedef enum {
    BACKGROUND_PUBLISH_STATUS_SUCCESS = 0,
    BACKGROUND_PUBLISH_STATUS_FAILURE,
} publish_status_t;

typedef std::function<void(publish_status_t status,
    const char *event_name,
    const char *event_data,
    const void *event_context)> publish_completed_cb_t;

class BackgroundPublish
{
    public:
        ~BackgroundPublish();

        void start();

        void stop();

        bool publish(const char *name,
            const char *data = NULL,
            PublishFlags flags = PRIVATE,
            publish_completed_cb_t cb = NULL,
            const void *context = NULL);

        template <typename T>
        bool publish(const char *name,
            const char *data = NULL,
            PublishFlags flags = PRIVATE,
            void (T::*cb)(publish_status_t status, const char *, const char *, const void *) = NULL,
            T *instance = NULL,
            const void *context = NULL);

        bool try_lock() {return mutex.try_lock();};

        void lock() {mutex.lock();}

        void unlock() {mutex.unlock();}

        bool idle() { return state == BACKGROUND_PUBLISH_IDLE; }

    private:
        Thread *thread = NULL;
        void thread_f();
        RecursiveMutex mutex;
        volatile publish_thread_state_t state = BACKGROUND_PUBLISH_IDLE;

        // arguments for Particle.publish
        char event_name[particle::protocol::MAX_EVENT_NAME_LENGTH+1];
        char event_data[particle::protocol::MAX_EVENT_DATA_LENGTH+1];
        PublishFlags event_flags;
        // callback when publish completes
        publish_completed_cb_t completed_cb = NULL;
        const void *event_context = NULL;
};

template <typename T>
bool BackgroundPublish::publish(const char *name,
            const char *data,
            PublishFlags flags,
            void (T::*cb)(publish_status_t status, const char *, const char *, const void *),
            T *instance,
            const void *context)
{
    return publish(name, data, flags, std::bind(cb, instance, _1, _2, _3, _4), context);
}
