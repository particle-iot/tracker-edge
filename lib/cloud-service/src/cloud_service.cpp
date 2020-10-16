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

#include "cloud_service.h"

#include "background_publish.h"

#include <string.h>

CloudService *CloudService::_instance = nullptr;

CloudService::CloudService() :
    _writer(json_buf, sizeof(json_buf)), _req_id(1)
{
}

void CloudService::init()
{
    Particle.function("cmd", &CloudService::dispatchCommand, this);
    background_publish.start();
}

void CloudService::tick()
{
    auto sec = System.uptime();
    std::lock_guard<RecursiveMutex> lg(mutex);

    if(sec != last_tick_sec)
    {
        last_tick_sec = sec;
        tick_sec();
    }

    for(auto handler : deferred_handlers)
    {
        handler.cb(handler.status, nullptr, handler.context);
    }
    deferred_handlers.clear();
}

void CloudService::tick_sec()
{
    uint32_t ms_now = millis();

    // timeout handlers
    auto it = handlers.begin();
    while(it != handlers.end())
    {
        bool erased = false;
        if(it->timeout_ms && (ms_now - it->t0) > it->timeout_ms)
        {
            if(it->cb)
            {
                it->cb(CloudServiceStatus::TIMEOUT, nullptr, it->context);
            }
            it = handlers.erase(it);
            erased = true;
        }
        if(!erased)
        {
            it++;
        }
    }
}

 uint32_t CloudService::get_next_req_id()
 {
    auto req_id = _req_id;
    if(!++_req_id)
    {
        _req_id = 1;
    }
    return req_id;
}

int CloudService::regCommandCallback(const char *cmd, cloud_service_cb_t cb, uint32_t req_id, uint32_t timeout_ms, const void *context)
{
    std::lock_guard<RecursiveMutex> lg(mutex);
    cloud_service_handler_t handler = {CloudServicePublishFlags::NONE, cb, "", req_id, timeout_ms, context, millis()};

    if(!cb)
    {
        return -EINVAL;
    }

    if(cmd)
    {
        if(strnlen(cmd, sizeof(handler.cmd)) >= CLOUD_MAX_CMD_LEN)
        {
            return -EINVAL;
        }

        // TODO: range-limit the strlen?
        strlcpy(handler.cmd, cmd, sizeof(handler.cmd));
    }

    handlers.push_front(handler);

    return 0;
}

static int _get_common_fields(JSONValue &root, const char **cmd, const char **src_cmd, uint32_t *req_id, uint32_t *timestamp)
{
    int rval = -EINVAL;

    // iterate and peel out necessary fields for command dispatching
    JSONObjectIterator it(root);
    while(it.next())
    {
        if (!it.value().isValid())
        {
            rval = -EINVAL;
            break;
        }

        const char *it_name = (const char *) it.name();
        if(!strcmp(CLOUD_KEY_CMD, (const char *) it_name))
        {
            if(!it.value().isString())
            {
                rval = -EINVAL;
                break;
            }
            if(cmd)
            {
                *cmd = (const char *) it.value().toString();
            }
        }
        else if(!strcmp(CLOUD_KEY_SRC_CMD, (const char *) it_name))
        {
            if(!it.value().isString())
            {
                rval = -EINVAL;
                break;
            }
            if(src_cmd)
            {
                *src_cmd = (const char *) it.value().toString();
            }
        }
        else if(!strcmp(CLOUD_KEY_REQ_ID, (const char *) it_name))
        {
            if(!it.value().isNumber())
            {
                rval = -EINVAL;
                break;
            }
            if(req_id)
            {
                *req_id = it.value().toInt();
            }
        }
        else if(!strcmp(CLOUD_KEY_TIME, (const char *) it_name))
        {
            if(!it.value().isNumber())
            {
                rval = -EINVAL;
                break;
            }
            if(timestamp)
            {
                *timestamp = it.value().toInt();
            }
        }
    }

    if(!*cmd)
    {
        return -EINVAL;
    }

    return rval;
}

int CloudService::dispatchCommand(String data)
{
    Log.info("cloud received: %s", data.c_str());
    JSONValue root = JSONValue::parseCopy(data, data.length());
    int rval = -ENOENT;

    const char *cmd = nullptr;
    uint32_t req_id=0, timestamp=0;

    // for now we are expecting a full json object
    // in future we may accept non-json objects and process separately
    if(!root.isObject() || !root.isValid())
    {
        return -EINVAL;
    }

    _get_common_fields(root, &cmd, nullptr, &req_id, &timestamp);

    if(!cmd)
    {
        return -EINVAL;
    }
 
    std::lock_guard<RecursiveMutex> lg(mutex);
    auto it = handlers.begin();
    while(it != handlers.end())
    {
        bool erased = false;

        if((!it->cmd[0] || !strcmp(it->cmd, cmd)) &&
            (!it->req_id || it->req_id == req_id) &&
            it->cb)
        {
            rval = it->cb(CloudServiceStatus::SUCCESS, &root, it->context);
            if(it->req_id || it->timeout_ms)
            {
                // anything looking for a specific req_id or with a
                // timeout implied to be one-shot and removed here
                it = handlers.erase(it);
                erased = true;
            }
        }
        // on handler erasure the iterator is already incremented
        if(!erased)
        {
            it++;
        }
    }

    return rval;
}

int CloudService::beginCommand(const char *cmd)
{
    // hold lock for duration between begin_command/send as the json buffer is
    // a singular shared resource
    // calling processes should not unnecessarily delay when formatting the
    // output command to not unnecessarily block other processes and should not
    // access other external resources that may result in a deadlock
    // (for example, don't begin_command and THEN read off a register from an
    // I2C device in order to format into the output command)
    mutex.lock();

    _writer = JSONBufferWriter(json_buf, sizeof(json_buf)); // reset the output

    writer().beginObject();
    writer().name(CLOUD_KEY_CMD).value(cmd);
    snprintf(_writer_event_name, sizeof(_writer_event_name), CLOUD_PUB_PREFIX "%s", cmd);
    writer().name(CLOUD_KEY_TIME).value((unsigned int) Time.now());

    return 0;
}

int CloudService::beginResponse(const char *cmd, JSONValue &root)
{
    const char *src_cmd = nullptr;
    uint32_t req_id = 0;

    if(!root.isObject())
    {
        return -EINVAL;
    }

    _get_common_fields(root, &src_cmd, nullptr, &req_id, nullptr);

    if(!src_cmd || !req_id)
    {
        return -EINVAL;
    }

    beginCommand(cmd);

    writer().name(CLOUD_KEY_REQ_ID).value((unsigned int) req_id);
    writer().name(CLOUD_KEY_SRC_CMD).value(src_cmd);

    return 0;
}

int CloudService::send_cb_wrapper(CloudServiceStatus status, JSONValue *rsp_root, const void *context)
{
    cloud_service_send_handler_t *send_handler = (cloud_service_send_handler_t *) context;

    int rval = send_handler->cb(status, rsp_root, send_handler->req_data, send_handler->context);

    delete send_handler;

    return rval;
}

void CloudService::publish_cb(
    publish_status_t status,
    const char *event_name,
    const char *event_data,
    const void *event_context)
{
    std::lock_guard<RecursiveMutex> lg(mutex);

    if(!event_context)
    {
        return;
    }

    cloud_service_handler_t &handler = ((cloud_service_send_handler_t *) event_context)->base_handler;

    if(status == BACKGROUND_PUBLISH_STATUS_SUCCESS)
    {
        if(handler.cloud_flags & CloudServicePublishFlags::FULL_ACK)
        {
            // expecting full end-to-end acknowledgement so set up handler waiting for the ACK
            regCommandCallback(handler.cmd, handler.cb, handler.req_id, handler.timeout_ms, handler.context);
        }
        else
        {
            handler.status = CloudServiceStatus::SUCCESS;
            deferred_handlers.push_front(handler);
        }
    }
    else
    {
        handler.status = CloudServiceStatus::FAILURE;
        deferred_handlers.push_front(handler);
    }
}

int CloudService::send(const char *event,
    PublishFlags publish_flags,
    CloudServicePublishFlags cloud_flags,
    cloud_service_send_cb_t cb,
    unsigned int timeout_ms,
    const void *context,
    const char *event_name,
    uint32_t req_id)
{
    int rval = 0;
    size_t event_len = strlen(event);
    std::lock_guard<RecursiveMutex> lg(mutex);

    if(!event_name ||
        (!req_id && cb && (cloud_flags & CloudServicePublishFlags::FULL_ACK)))
    {
        // should have request id or event name but it wasn't passed in
        // extract from event
        JSONValue root = JSONValue::parseCopy(event, event_len);
        _get_common_fields(root, &event_name, nullptr, &req_id, nullptr);

        if(!event_name)
        {
            return -EINVAL;
        }
    }    

    strlcpy(_writer_event_name, event_name, sizeof(_writer_event_name));

    // much simpler if there is no callback and can just publish into the void
    if(!cb)
    {
        if(!background_publish.publish(_writer_event_name, event))
        {
            rval = -EBUSY;
        }
        return rval;
    }

    // otherwise we need three (?!) layers of callbacks here to associate the
    // caller callback/context to ultimate success/failure of the transaction

    // 1. Callback into the non-blocking background publish lib
    // 2. Callback waiting on the success/failure of the app ack
    // 3. Callback as passed in by the user

    // Callback (1) is a wrapper around Callback (2) which wraps around
    // Callback (3). Both of the wrappers are provided by CloudService.

    // Callback (1) waits on success/fail of the iniital publish and 
    // sets up Callback (2).

    // Callback (2) is either triggered immediately or waits on the
    // application acknowledgement. It manages additional meta-data not
    // included in the basic handler registration, most importantly info
    // about the originating event that the user Callback (3) might want
    // to know.
    
    // allocate space for handler info and copy of requesting event
    cloud_service_send_handler_t *send_handler  = new cloud_service_send_handler_t;
    
    if(!send_handler)
    {
        rval = -ENOMEM;
    }
    else
    {
        send_handler->base_handler.cloud_flags = cloud_flags;
        send_handler->base_handler.cb = send_cb_wrapper;
        send_handler->base_handler.cmd[0] = '\0';
        send_handler->base_handler.req_id = req_id;
        send_handler->base_handler.timeout_ms = timeout_ms;
        send_handler->base_handler.context = send_handler;

        send_handler->cb = cb;
        send_handler->context = context;
        send_handler->req_data = event;
        if(!background_publish.publish(_writer_event_name, event, publish_flags | PRIVATE, &CloudService::publish_cb, this, send_handler))
        {
            delete send_handler;
            rval = -EBUSY;
        }
    }

    if(!rval)
    {
        Log.info("cloud sent: %s", event);
    }

    return rval;
}

int CloudService::send(PublishFlags publish_flags, CloudServicePublishFlags cloud_flags, cloud_service_send_cb_t cb, unsigned int timeout_ms, const void *context)
{
    int rval = 0;
    uint32_t req_id = (cb && (cloud_flags & CloudServicePublishFlags::FULL_ACK)) ? get_next_req_id() : 0;

    if(req_id)
    {
        writer().name(CLOUD_KEY_REQ_ID).value((unsigned int) req_id);
    }
    writer().endObject();

    // output json overflowed the buffer
    // dataSize does not include the null terminator
    if(writer().dataSize() >= writer().bufferSize())
    {
        unlock();
        return -ENOSPC;
    }

    // ensure null termination of the output json
    writer().buffer()[writer().dataSize()] = '\0';

    rval = send(writer().buffer(), publish_flags, cloud_flags, cb, timeout_ms, context, _writer_event_name, req_id);

    unlock();
    return rval;
}

int CloudService::sendAck(JSONValue &root, int status)
{
    int rval = beginResponse(CLOUD_CMD_ACK, root);
    if(!rval)
    {
        writer().name("status").value(status);
        rval = send();
    }

    return rval;
}

void print_tab(int count)
{
    for(int i=0; i < count; i++)
    {
        Log.printf("\t");
    }
}

// logs a parsed json object to output
void _log_json(JSONValue &root, int level)
{
    switch(root.type())
    {
        case JSON_TYPE_INVALID:
            break;
        case JSON_TYPE_NULL:
            Log.printf("null\n");
            break;
        case JSON_TYPE_BOOL:
            Log.printf("%s\n", (const char *) root.toString());
            break;
        case JSON_TYPE_NUMBER:
            Log.printf("%lf\n", root.toDouble());
            break;
        case JSON_TYPE_STRING:
            Log.printf("\"%s\"\n", (const char *) root.toString());
            break;
        case JSON_TYPE_ARRAY:
        {
            JSONArrayIterator it(root);
            Log.printf("array (length %d)\n", it.count());
            while(it.next())
            {
                JSONValue val = it.value();
                print_tab(level+1);
                _log_json(val, level + 1); // dirty dirty recursion!
            }
            break;
        }
        case JSON_TYPE_OBJECT:
        {
            JSONObjectIterator it(root);
            Log.printf("object (length %d)\n", it.count());
            while(it.next())
            {
                JSONValue val = it.value();
                print_tab(level+1);
                Log.printf("%s: ", (const char *) it.name());
                _log_json(val, level + 1); // dirty dirty recursion!
            }
            break;
        }
    }
}

// parse and log json object to output
void log_json(const char *json, size_t size)
{
    JSONValue root = JSONValue::parseCopy(json, size);

    _log_json(root, 0);
}
