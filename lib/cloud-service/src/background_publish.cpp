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
#include <spark_wiring_thread.h>

#include "background_publish.h"

BackgroundPublish::~BackgroundPublish()
{
    stop();
}

void BackgroundPublish::start()
{
    if(!thread)
    {
        // use OS_THREAD_PRIORITY_DEFAULT so that application, system, and
        // background publish thread will all run at the same priority and
        // be able to preempt each other
        thread = new Thread("background_publish",
            [this]() { thread_f(); },
            OS_THREAD_PRIORITY_DEFAULT);
    }
}

void BackgroundPublish::stop()
{
    if(thread)
    {
        state = BACKGROUND_PUBLISH_STOP;
        thread->dispose();
        delete thread;
        thread = NULL;
    }
}

void BackgroundPublish::thread_f()
{
    while(true)
    {
        while(state == BACKGROUND_PUBLISH_IDLE)
        {
            // yield to rest of system while we wait
            // a condition variable would be ideal but doesn't look like
            // std::condition_variable is supported
            delay(1);
        }

        if(state == BACKGROUND_PUBLISH_STOP)
        {
            break;
        }

        // temporarily acquire the lock
        // this allows a calling thread to block the publish thread if it needs
        // additional synchronization around a publish request and acts as a
        // memory barrier around publish arguments to ensure all updates
        // are complete
        lock();
        unlock();

        // kick off the publish
        // WITH_ACK does not work as expected from a background thread
        // use the Future<bool> object directly as its default wait
        // (used by WITH_ACK) short-circuits when not called from the
        // main application thread
        auto ok = Particle.publish(event_name, event_data, event_flags);

        // then wait for publish to complete
        while(!ok.isDone() && state != BACKGROUND_PUBLISH_STOP)
        {
            // yield to rest of system while we wait
            delay(1);
        }
        publish_status_t status = ok.isSucceeded() ? BACKGROUND_PUBLISH_STATUS_SUCCESS : BACKGROUND_PUBLISH_STATUS_FAILURE;

        if(completed_cb)
        {
            completed_cb(status,
                event_name,
                event_data,
                event_context);
        }
        
        WITH_LOCK(*this)
        {
            if(state == BACKGROUND_PUBLISH_STOP)
            {
                break;
            }
            event_context = NULL;
            completed_cb = NULL;
            state = BACKGROUND_PUBLISH_IDLE;
        }
    }
}

bool BackgroundPublish::publish(const char *name, const char *data, PublishFlags flags, publish_completed_cb_t cb, const void *context)
{
    // protect against separate threads trying to publish at the same time
    std::lock_guard<RecursiveMutex> lg(mutex);

    // check currently in idle state and ready to accept publish request
    if(!thread || state != BACKGROUND_PUBLISH_IDLE)
    {
        return false;
    }

    // event name is required to publish
    // all other arguments may be be left out or defaulted
    if(!name)
    {
        return false;
    }

    // have the lock and thread is currently idle
    // safe to prepare publish request
    strncpy(event_name, name, sizeof(event_name));
    event_name[sizeof(event_name)-1] = '\0'; // ensure null termination

    if(data)
    {
        strncpy(event_data, data, sizeof(event_data));
        event_data[sizeof(event_data)-1] = '\0'; // ensure null termination
    }
    else
    {
        event_data[0] = '\0'; // null terminate at start for no event data
    }

    completed_cb = cb;
    event_context = context;
    event_flags = flags;
    state = BACKGROUND_PUBLISH_REQUESTED;

    return true;
}
