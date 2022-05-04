/*
 * Copyright (c) 2022 Particle Industries, Inc.
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

#include "LocationPublish.h"
#include "tracker_location.h"
#include "tracker.h"

constexpr int HIGH_PRIORITY = 0;
constexpr int LOW_PRIORITY = 1;
const int DEFAULT_DISK_LIMIT = 64;//In KB
const size_t KILOBYTE_CONSTANT = 1024;
const char STORE_QUEUE_FILE_PATH[] = "/usr/store_queue";

uint8_t store_msg_buffer[particle::protocol::MAX_EVENT_DATA_LENGTH + 1] = {0};

void locationGenerationCallback(JSONWriter &writer,
    LocationPoint &point, const void *context);

void LocationPublish::init() {
    static ConfigObject store_forward("store", {
        ConfigBool("enable", &store_config.enable),
        ConfigInt("quota", &store_config.quota),
        ConfigStringEnum("policy", {
                {"drop_old", (int32_t) DiskQueuePolicy::FifoDeleteOld},
                {"drop_new", (int32_t) DiskQueuePolicy::FifoDeleteNew}
            }, &store_config.policy)
    });

    ConfigService::instance().registerModule(store_forward);

    if(store_config.enable) {
        start();
    }

    Tracker::instance().location.regLocGenCallback(locationGenerationCallback);
}

void LocationPublish::start() {
    if(store_msg_queue.start(STORE_QUEUE_FILE_PATH,
                        store_config.quota*KILOBYTE_CONSTANT,
                        store_config.policy) != SYSTEM_ERROR_NONE) {
        Log.error("Failed to start location publish disk queue");
    }
}

void LocationPublish::tick() {
    static StoreConfig current_config = store_config;

    //check if settings changed, if still enabled re-run start,
    //if disabled stop the disk queue
    if(current_config != store_config) {
        if(store_config.enable) {
            start();
        }
        else {
            factoryReset();
        }
        current_config = store_config;
    }

    //check if DiskQueue has messages to retry
    if(!store_msg_queue.isEmpty() && isStoreEnabled() && Particle.connected()) {
        CloudServicePublishFlags cloud_flags =
            (TrackerLocation::instance().isProcessAckEnabled()) ?
                CloudServicePublishFlags::FULL_ACK : CloudServicePublishFlags::NONE;

        auto size = store_msg_queue.peekFrontSize();
        store_msg_queue.peekFront(store_msg_buffer, size);
        store_msg_queue.popFront(); //ok to pop, disk_queue_cb will throw it back
        //on if it fails again and there's space in the disk queue

        //Priority level set to normal. don't want these to be high priority
        regPendingLocPubCallback(); //use the pending callback for this since
        //the exchange between pending vs the current hasn't occured yet
        auto send_rval =
            CloudService::instance().send((const char*)store_msg_buffer,
                WITH_ACK,
                cloud_flags,
                &LocationPublish::loc_store_cb_wrapper,
                this,
                CLOUD_DEFAULT_TIMEOUT_MS, nullptr, "loc", 0, 1);
        if(send_rval) {
            Tracker::instance().location.
                issue_location_publish_callbacks(CloudServiceStatus::FAILURE,
                                                nullptr,
                                                (const char*)store_msg_buffer);
        }
    }
}

bool LocationPublish::publish(const char *name,
                            const char *data,
                            PublishFlags flags,
                            int level,
                            const void *context) {

    //copy name and data to heap location
    char* event_name = new char[strlen(name)+1]; //+1 for null terminator
    strcpy((char*)event_name, name);

    char* event_data = new char[strlen(data)+1]; //+1 for null terminator
    strcpy((char*)event_data, data);

    return BackgroundPublish::instance().publish(event_name,
                                                event_data,
                                                flags,
                                                level,
                                                &LocationPublish::publish_cb,
                                                this,
                                                context);
}

void LocationPublish::publish_cb(publishStatus status,
                        const char *event_name,
                        const char *event_data,
                        const void *event_context) {
    if(!event_context) {
        return;
    }

    //delete event_name, not used anywhere else after this
    //delete event_data, not used anywhere else since event_context is allocated
    //from the heap and contains a String copy of event_data. The event_context
    //is deleted later in the CloudService::send_cb_wrapper() function
    delete[] event_name;
    delete[] event_data;

    auto send_handler {static_cast<const cloud_service_send_handler_t *>(event_context)};
    auto &base_handler {send_handler->base_handler};

    if(status == publishStatus::PUBLISH_COMPLETE) {
        if(base_handler.cloud_flags & CloudServicePublishFlags::FULL_ACK) {
            // expecting full end-to-end acknowledgement so set up handler waiting for the ACK
            CloudService::instance().regCommandCallback(base_handler.cmd,
                                                        base_handler.cb,
                                                        base_handler.req_id,
                                                        base_handler.timeout_ms,
                                                        base_handler.context);
        }
        else {
            cloud_service_handler_t handler {base_handler};
            handler.status = CloudServiceStatus::SUCCESS;
            CloudService::instance().regCommandDeferredCallback(handler);
        }
    }
    else if (status == publishStatus::PUBLISH_BUSY) {
        cloud_service_handler_t handler {base_handler};
        handler.status = CloudServiceStatus::FAILURE;
        CloudService::instance().regCommandDeferredCallback(handler);
    }
}

int LocationPublish::loc_store_cb_wrapper(CloudServiceStatus status,
                                        JSONValue *rsp_root,
                                        const char *req_event,
                                        const void *context) {
    return TrackerLocation::instance().location_publish_cb(status,
                                                    rsp_root,
                                                    req_event,
                                                    context);
}

void LocationPublish::regLocPubCallback() {
    Tracker::instance().location.regLocPubCallback(&LocationPublish::disk_queue_cb,
                                                this);
}

void LocationPublish::regPendingLocPubCallback() {
    Tracker::instance().location.regPendLocPubCallback(&LocationPublish::disk_queue_cb,
                                                this);
}

int LocationPublish::disk_queue_cb(CloudServiceStatus status,
                                    JSONValue * rsp_root,
                                    const char * req_event,
                                    const void *context) {
    if(req_event && (status != SUCCESS) && store_config.enable) {
        if(!store_msg_queue.pushBack((const uint8_t*)req_event, strlen(req_event)+1)) {
            Log.warn("Unable to write location message to DiskQueue, discarding");
        }
    }
    return 0;
}

void locationGenerationCallback(JSONWriter &writer, LocationPoint &point, const void *context)
{
    LocationPublish::instance().regLocPubCallback();
}
