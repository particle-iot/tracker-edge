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

#include "Particle.h"

#define CLOUD_KEY_CMD "cmd"
#define CLOUD_KEY_TIME "time"
#define CLOUD_KEY_REQ_ID "req_id"
#define CLOUD_KEY_SRC_CMD "src_cmd"

#define CLOUD_CMD_SYNC "sync"
#define CLOUD_CMD_ACK "ack"
#define CLOUD_CMD_CFG "cfg"

#define CLOUD_MAX_CMD_LEN (32)
#define CLOUD_PUB_PREFIX ""

#define CLOUD_DEFAULT_TIMEOUT_MS (10000)

#include <vector>
#include <list>

using namespace std::placeholders;

#include "background_publish.h"

enum CloudServiceStatus {
    SUCCESS = 0,
    FAILURE, // publish to Particle cloud failed, etc
    TIMEOUT, // waiting for application response, etc
};

enum CloudServicePublishFlags {
    NONE = 0x00, // no special flags
    FULL_ACK = 0x01 // full end-to-end acknowledgement
};

typedef std::function<int(CloudServiceStatus status, JSONValue *, const void *context)> cloud_service_cb_t;

typedef std::function<int(CloudServiceStatus status, JSONValue *, const char *, const void *context)> cloud_service_send_cb_t;

class cloud_service_handler_t
{
public:
    CloudServicePublishFlags cloud_flags;
    cloud_service_cb_t cb;
    // match on this cmd (or blank for don't care)
    char cmd[CLOUD_MAX_CMD_LEN + 1];
    // match on this req_id (or 0 don't care)
    uint32_t req_id;
    // fail on match after a timeout, or 0 for never timeout
    uint32_t timeout_ms;
    const void *context;
    uint32_t t0;
    CloudServiceStatus status;
};

class cloud_service_send_handler_t
{
public:
    cloud_service_handler_t base_handler;
    cloud_service_send_cb_t cb;
    const void *context;
    String req_data;
};

class CloudService
{
    public:
        /**
         * @brief Return instance of the cloud service
         *
         * @retval CloudService&
         */
        static CloudService &instance()
        {
            if(!_instance)
            {
                _instance = new CloudService();
            }
            return *_instance;
        }

        void init();

        // process quick actions
        void tick();

        // starts a new command/ack
        int beginCommand(const char *cmd);
        int beginResponse(const char *cmd, JSONValue &root);

        int send(PublishFlags publish_flags = PRIVATE,
            CloudServicePublishFlags cloud_flags = CloudServicePublishFlags::NONE,
            cloud_service_send_cb_t cb=nullptr,
            unsigned int timeout_ms=0,
            const void *context=nullptr);

        template <typename T>
        int send(PublishFlags publish_flags = PRIVATE,
            CloudServicePublishFlags cloud_flags = CloudServicePublishFlags::NONE,
            int (T::*cb)(CloudServiceStatus status, JSONValue *, const char *, const void *context)=nullptr,
            T *instance=nullptr,
            uint32_t timeout_ms=0,
            const void *context=nullptr);

        int send(const char *event,
            PublishFlags publish_flags = PRIVATE,
            CloudServicePublishFlags cloud_flags = CloudServicePublishFlags::NONE,
            cloud_service_send_cb_t cb=nullptr,
            unsigned int timeout_ms=0,
            const void *context=nullptr,
            const char *event_name=nullptr,
            uint32_t req_id=0);

        template <typename T>
        int send(const char *event,
            PublishFlags publish_flags = PRIVATE,
            CloudServicePublishFlags cloud_flags = CloudServicePublishFlags::NONE,
            int (T::*cb)(CloudServiceStatus status, JSONValue *, const char *, const void *context)=nullptr,
            T *instance=nullptr,
            uint32_t timeout_ms=0,
            const void *context=nullptr,
            const char *event_name=nullptr,
            uint32_t req_id=0);

        int sendAck(JSONValue &root, int status);

        JSONBufferWriter &writer() { return _writer; };

        void lock() {mutex.lock();}
        void unlock() {mutex.unlock();}

        // process and dispatch incoming commands to registered callbacks
        int dispatchCommand(String cmd);

        int regCommandCallback(const char *name, cloud_service_cb_t cb, uint32_t req_id=0, uint32_t timeout_ms=0, const void *context=nullptr);

        template <typename T>
        int regCommandCallback(const char *name,
            int (T::*cb)(CloudServiceStatus status, JSONValue *, const void *context),
            T *instance,
            uint32_t req_id=0,
            uint32_t timeout_ms=0,
            const void *context=nullptr);
    private:
        CloudService();
        static CloudService *_instance;

        BackgroundPublish background_publish;

        // internal callback for non-blocking publish on the send path
        void publish_cb(
            publish_status_t status,
            const char *event_name,
            const char *event_data,
            const void *event_context);

        // internal callback wrapper on the send path
        static int send_cb_wrapper(CloudServiceStatus status,
            JSONValue *rsp_root,
            const void *context);

        // process infrequent actions
        void tick_sec();

        uint32_t get_next_req_id();

        char json_buf[particle::protocol::MAX_EVENT_DATA_LENGTH + 1];
        JSONBufferWriter _writer;
        char _writer_event_name[sizeof(CLOUD_PUB_PREFIX) + CLOUD_MAX_CMD_LEN];

        // iterate req_id on each send
        uint32_t _req_id;

        uint32_t last_tick_sec;

        std::list<cloud_service_handler_t> handlers;
        std::list<cloud_service_handler_t> deferred_handlers;

        RecursiveMutex mutex;
};

template <typename T>
int CloudService::regCommandCallback(const char *name,
    int (T::*cb)(CloudServiceStatus status, JSONValue *, const void *context),
    T *instance,
    uint32_t req_id,
    uint32_t timeout_ms,
    const void *context)
{
    return regCommandCallback(name, std::bind(cb, instance, _1, _2, _3), req_id, timeout_ms, context);
}

template <typename T>
int CloudService::send(PublishFlags publish_flags,
    CloudServicePublishFlags cloud_flags,
    int (T::*cb)(CloudServiceStatus status, JSONValue *, const char *, const void *context),
    T *instance,
    uint32_t timeout_ms,
    const void *context)
{
    return send(publish_flags, cloud_flags, std::bind(cb, instance, _1, _2, _3, _4), timeout_ms, context);
}


template <typename T>
int CloudService::send(const char *event,
    PublishFlags publish_flags,
    CloudServicePublishFlags cloud_flags,
    int (T::*cb)(CloudServiceStatus status, JSONValue *, const char *, const void *context),
    T *instance,
    uint32_t timeout_ms,
    const void *context,
    const char *event_name,
    uint32_t req_id)
{
    return send(event, publish_flags, cloud_flags, std::bind(cb, instance, _1, _2, _3, _4), timeout_ms, context, event_name, req_id);
}

void log_json(const char *json, size_t size);
