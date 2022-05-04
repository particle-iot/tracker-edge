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

#pragma once

#include "BackgroundPublish.h"
#include "DiskQueue.h"
#include "cloud_service.h"

extern const int DEFAULT_DISK_LIMIT; //in KB
extern const size_t KILOBYTE_CONSTANT;

struct StoreConfig {
    int quota{DEFAULT_DISK_LIMIT};
    DiskQueuePolicy policy {DiskQueuePolicy::FifoDeleteOld};
    bool enable{false};

    bool operator!=(const StoreConfig& other) const {
        if((quota != other.quota) || (policy != other.policy) ||
            (enable != other.enable)) {return true;}
        else {return false;}
    }
};
class LocationPublish {
public:
    static LocationPublish& instance() {
        static LocationPublish instance;
        return instance;
    }

    /**
     * @brief Initialize the LocationPublish object
     *
     * @details creates the ConfigObject for the store forward feature, and
     * registers the object. Will also call start() if the store forward feature
     * is enabled
     */
    void init();

    /**
     * @brief Start the DiskQueue
     *
     * @details Calls DiskQueue::start() for store_msg_queue. This will get
     * the DiskQueue started with the quota and policy desired. Can be called
     * again if there is a change to the policy or size.
     */
    void start();

    /**
     * @brief Called once a second to consume the store_msg_queue if there
     * is data to send
     *
     * @details Call this function once a second to consume messages saved in the
     * store_msg_queue. This setsup the the entire callback system to run again
     */
    void tick();

    /**
     * @brief Request to publish a message
     *
     * @details Calls the BackgroundPublish::publish() function to request
     * a publish in the BackgroundPublish::thread_f thread.
     *
     * @param[in] name name of event to be sent
     * @param[in] data pointer to string of data to send out
     * @param[in] flags PublishFlags type for the request
     * @param[in] level priority of message. Lowest number is highest priority.
     * number of priorities is determined in the BackgroundPublish class.
     * @param[in] context context for the publish that is passed to the callback
     *
     * @return TRUE if request accepted, FALSE if not
     */
    bool publish(const char *name,
            const char *data,
            PublishFlags flags = PRIVATE,
            int level = 0,
            const void *context = nullptr);

    /**
     * @brief Register the callback to be called from every generated location
     * publish
     *
     * @details This function is a wraper that is called on every generated
     * location publish to add the disk_queue_cb to be issued on location publish
     */
    void regLocPubCallback();

    /**
     * @brief Register the pending callbacks
     *
     * @details This is necessary since the copying of the location publish
     * callbacks hasn't occured, this method puts the disk_queue_cb as the next
     * pending callback that can be issued if the message fails or succeeds
     */
    void regPendingLocPubCallback();

    /**
     * @brief Makes the decision to store and forward a message after any
     * location publish
     *
     * @details Checks to see if the status of the message publish is not
     * SUCCESS. If the store forward feature is enabled, and there is data,
     * push the message on to the store_msg_queue
     *
     * @param[in] status of the message that was published
     * @param[in] rsp_root JSON root of the message
     * @param[in] req_event data containing the message in JSON format
     * @param[in] context of the callback
     *
     * @return 0 for success
     */
    int disk_queue_cb(CloudServiceStatus status,
                    JSONValue * rsp_root,
                    const char * req_event,
                    const void *context);

    /**
     * @brief Return the state of the store_config.enable variable
     *
     * @details Is the store and forward feature enabled. This returns
     * its state
     *
     * @return TRUE if enabled, FALSE if not
     */
    bool isStoreEnabled() const {
        return store_config.enable;
    }

    /**
     * @brief Called to cleanup the store_msg_queue, and the background publish
     * queues. Is called if you disable the store forward feature, or reset the
     * device to factory
     *
     * @details Flushes, Closes out, and deletes the store_msg_queue files, and
     * cleans up the BackgroundPublish queue
     */
    void factoryReset() {
        BackgroundPublish::instance().cleanup();
        store_msg_queue.unlinkFiles(); //unlink the files first
        store_msg_queue.stop(); //then clear the files from _fileList
    }

    //remove copy and assignment operators
    LocationPublish(LocationPublish const&) = delete;
    void operator=(LocationPublish const&)  = delete;

private:
    LocationPublish() : store_msg_queue () {}

    DiskQueue store_msg_queue;
    StoreConfig store_config;

    /**
     * @brief Callback to be called on every publish
     *
     * @details This sets up the three layers of callbacks for a publish. Please
     * see the CloudService class for more explanation these 3 layers of
     * callbacks
     *
     * @param[in] status of the publish
     * @param[in] event_name name of the event
     * @param[in] event_data pointer to string of message published
     * @param[in] event_context context of the message to pass to the other
     * callbacks
     */
    void publish_cb(publishStatus status,
                        const char *event_name,
                        const char *event_data,
                        const void *event_context);

    /**
     * @brief Wrapper that is used to call the
     * TrackerLocation::location_publish_cb, on every publish
     *
     * @details This is a callback that is used when we resend a stored message.
     * This is just a convenience wrapper to be able to call the
     * TrackerLocation::location_publish_cb outside of it's class
     *
     * @param[in] status of the message that was published
     * @param[in] rsp_root JSON root of the message
     * @param[in] req_event data containing the message in JSON format
     * @param[in] context of the callback
     *
     * @return 0 for success
     */
    int loc_store_cb_wrapper(CloudServiceStatus status,
                                        JSONValue *rsp_root,
                                        const char *req_event,
                                        const void *context);
};
