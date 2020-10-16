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

#include <dirent.h>
#include <fcntl.h>

#include "config_service.h"
#include "cloud_service.h"

#include "murmur3.h"

Logger config_service_log("Config Service");

int _config_process_json(JSONValue &json_root, const char *json_root_name, ConfigNode *config_root);
int config_process_json(const char *json, size_t size, ConfigNode *config_root);
int config_write_json(ConfigNode *root, JSONWriter &writer);
void config_hash(ConfigNode *root, murmur3_hash_t &hash);

static String _format_hash_str(murmur3_hash_t &hash)
{
    return String::format("%08lX%08lX%08lX%08lX",
        (uint32_t) hash.h[0],
        (uint32_t) hash.h[1],
        (uint32_t) hash.h[2],
        (uint32_t) hash.h[3]);
}

static int _json_parse_hash_str(JSONValue value, murmur3_hash_t &hash)
{
    if(!value.isString())
    {
        return -EINVAL;
    }

    if(sscanf(value.toString().data(),
        "%08lx%08lx%08lx%08lx%*c",
        &hash.h[0],
        &hash.h[1],
        &hash.h[2],
        &hash.h[3]) != 4)
    {
        return -EINVAL;
    }

    return 0;
}

ConfigNode *ConfigObject::child(const char *name)
{
    if(!name)
    {
        return nullptr;
    }

    for(int i=0; i < child_count(); i++)
    {
        auto _child = child(i);
        if(!_child->name() || !strcmp(name, _child->name()))
        {
            return _child;
        }
    }
    return nullptr;
}

int ConfigObject::enter(bool write)
{
    if(enter_cb)
    {
        return enter_cb(write, (!write || !wcontext) ? context : wcontext);
    }
    return 0;
}

int ConfigObject::exit(bool write, int status)
{
    if(exit_cb)
    {
        return exit_cb(write, status, (!write || !wcontext) ? context : wcontext);
    }
    return status;
}

bool ConfigFloat::check(double value)
{
    return (
        (isnan(range_min) || value >= range_min) &&
        (isnan(range_max) || value <= range_max)
    );
};

bool ConfigString::check(const char *value)
{
    if(!_size || _size == SIZE_MAX)
    {
        return true;
    }

    // need to ensure room for null terminator as well
    return (strnlen(value, _size) < _size);
}

int ConfigStringEnum::set(const char *value)
{
    if(!this->set_cb)
    {
        return -EPERM;
    }

    for(auto pair : enums)
    {
        if(!strcmp(pair.first, value))
        {
            return set_cb(pair.second, this->wcontext ? this->wcontext : this->context);
        }
    }

    return -EINVAL;
}

int ConfigStringEnum::get(const char * &value)
{
    if(!this->get_cb)
    {
        return -EPERM;
    }

    int32_t enum_val;
    CHECK(get_cb(enum_val, this->context));

    for(auto pair : enums)
    {
        if(pair.second == enum_val)
        {
            value = pair.first;
            return 0;
        }
    }
    return -EINVAL;
}

ConfigService *ConfigService::_instance = nullptr;
ConfigService::ConfigService() :
    fs_ok(false),
    sync_pending(false),
    sync_ok(false),
    config_sync_pending_object(nullptr)
{
}

void ConfigService::init()
{
    CloudService::instance().regCommandCallback("set_cfg", &ConfigService::set_cfg_cb, this);
    CloudService::instance().regCommandCallback("get_cfg", &ConfigService::get_cfg_cb, this);
    CloudService::instance().regCommandCallback("reset_to_factory", &ConfigService::reset_to_factory_cb, this);

    struct stat st;

    if (!stat(CONFIG_SERVICE_FS_PATH, &st))
    {
        if (S_ISDIR(st.st_mode))
        {
            fs_ok = true;
        }
    }
    else
    {
        if(!mkdir(CONFIG_SERVICE_FS_PATH, 0775))
        {
            fs_ok = true;
        }
    }
}

void ConfigService::tick()
{
    auto sec = System.uptime();

    if(sec != last_tick_sec)
    {
        last_tick_sec = sec;
        tick_sec();
    }
}

void ConfigService::flush()
{
    if(fs_ok)
    {
        for(auto &it : configs)
        {
            config_hash(it.root, it.hash);
        }
        save_all();
    }
}

void ConfigService::tick_sec()
{
    // iterate through all configs once a second to publish config updates
    // on crc mismatch
    // make sure we are likely to succeed in publishing first...

    // update hash across all configs once a second for change detection
    for(auto &it : configs)
    {
        config_hash(it.root, it.hash);
    }

    if(Particle.connected())
    {
        CloudService &cloud_service = CloudService::instance();
        if(!sync_ok)
        {
            if(!sync_pending)
            {
                murmur3_hash_t hash_accum;

                murmur3_hash_start(hash_accum, 0);
                for(auto it : configs)
                {
                    murmur3_hash_update(hash_accum, it.hash.h, sizeof(it.hash.h));
                }
                cloud_service.beginCommand(CLOUD_CMD_SYNC);
                cloud_service.writer().name("hash").value(_format_hash_str(hash_accum).c_str());
                // TODO: Cloud is not sending app ack yet
                // if(!cloud_service.send(WITH_ACK, CloudServicePublishFlags::FULL_ACK, &ConfigService::sync_ack_cb, this, CLOUD_DEFAULT_TIMEOUT_MS, nullptr))
                if(!cloud_service.send(WITH_ACK, CloudServicePublishFlags::NONE, &ConfigService::sync_ack_cb, this, CLOUD_DEFAULT_TIMEOUT_MS, nullptr))
                {
                    sync_pending = true;
                }
            }
        }
        else
        {
            if(!config_sync_pending_object)
            {
                for(auto &it : configs)
                {
                    if(it.hash != it.sync_hash)
                    {
                        cloud_service.beginCommand(CLOUD_CMD_CFG);
                        cloud_service.writer().name("cfg").beginObject();
                        config_write_json(it.root, cloud_service.writer());
                        cloud_service.writer().endObject();
                        // TODO: Cloud is not sending app ack yet
                        // if(!cloud_service.send(WITH_ACK, CloudServicePublishFlags::FULL_ACK, &ConfigService::config_sync_ack_cb, this, CLOUD_DEFAULT_TIMEOUT_MS, nullptr))
                        if(!cloud_service.send(WITH_ACK, CloudServicePublishFlags::NONE, &ConfigService::config_sync_ack_cb, this, CLOUD_DEFAULT_TIMEOUT_MS, nullptr))
                        {
                            config_sync_pending_object = &it;
                            // possible config could be updated again before ack
                            // received so save off the hash we sent rather than
                            // simply using the current hash in the callback
                            config_sync_pending_hash = it.hash;
                            break;
                        }
                    }
                }
            }
        }
    }

    // once a second check all configs and save to filesystem as necessary
    if(fs_ok)
    {
        save_all();
    }
}

void ConfigService::resetToFactory()
{
    // reset to factory by clearing out all config files and performing a
    // device reset
    auto dir = opendir(CONFIG_SERVICE_FS_PATH);

    struct dirent* ent = nullptr;
    while ((ent = readdir(dir)) != nullptr)
    {
        if(ent->d_type == DT_DIR && strcmp(ent->d_name, ".") && strcmp(ent->d_name, ".."))
        {
            continue;
        }
        remove(CONFIG_SERVICE_FS_PATH + String::format("/%s", ent->d_name));
    }
    closedir(dir);
    rmdir(CONFIG_SERVICE_FS_PATH);

    System.reset();
}

String ConfigService::_get_filename(const char *name)
{
    return String(CONFIG_SERVICE_FS_PATH) + '/' + name + ".cfg";
}

void ConfigService::save_all(bool force)
{
    for(auto &it : configs)
    {
        _save(it, force);
    }
}

int ConfigService::save(const char *name, bool force)
{
    for(auto &it : configs)
    {
        if(!strcmp(it.root->name(), name))
        {
            _save(it, force);
        }
    }

    return 0;
}

// A bug in Device-OS 1.5.3 caused newlib to call into the unsupported _link()
// function rather than the supported _rename() function. As workaround extern
// the _rename() function (also exported via dynalb) and call directly.
extern "C" int _rename(const char* oldpath, const char* newpath);

int ConfigService::_save(config_service_desc_t &config_desc, bool force)
{
    if(!force && config_desc.hash == config_desc.file_hash && config_desc.sync_hash == config_desc.file_sync_hash)
    {
        return 0;
    }

    // TODO: use a JSONStreamWriter so we don't have a limit on the output config size
    // and don't need to consume as much memory. Will require overloading a stream to point
    // at an underlying FS file, etc.
    char buf[1024];
    JSONBufferWriter writer(buf, sizeof(buf));

    writer.beginObject();
    writer.name(CONFIG_SERVICE_FS_VERSION_KEY).value(CONFIG_SERVICE_FS_VERSION);
    writer.name(CONFIG_SERVICE_FS_SYNC_HASH_KEY).value(_format_hash_str(config_desc.sync_hash).c_str());
    config_write_json(config_desc.root, writer);
    writer.endObject();
    // output json overflowed the buffer
    if(writer.dataSize() >= writer.bufferSize())
    {
        return -ENOSPC;
    }

    Log.info("saving config %s: %.*s", config_desc.root->name(), (int) writer.dataSize(), writer.buffer());

    // to ensure we always have a valid config write new config to a temp file
    // and use rename() to move over the original file.
    // rename() is an atomic operation on the filesystem so will either succeed
    // or leave the existing config alone to recover in case of interrupution
    String filename = _get_filename(config_desc.root->name());
    String temp_filename = filename + ".tmp";

    // new config to the temp file
    int fd = open(temp_filename, O_CREAT | O_WRONLY | O_TRUNC, 0664);
    if(fd <= 0)
    {
        return -errno;
    }

    int error = 0;
    int written = write(fd, writer.buffer(), writer.dataSize());
    if(written < 0)
    {
        error = -errno;
    }
    else if(written != (int) writer.dataSize())
    {
        error = -EIO;
    }

    close(fd);

    // then rename over the existing config
    if(!error)
    {
        if(_rename(temp_filename, filename))
        {
           error = -errno;
        }
        else
        {
            config_desc.file_hash = config_desc.hash;
            config_desc.file_sync_hash = config_desc.sync_hash;
        }
    }

    return error;
}

// processes the json file format and applies into the config object
// looks for a matching file format version, valid hash, and correct
// naming of the enclosed config blob before applying
int _process_load(config_service_desc_t &config_desc, char *json, size_t size)
{
    // TODO: All of the parse() calls require the entire JSON string in memory
    // It would be useful to define a Stream variant and then we could wrap the
    // file handle in a new Stream type and use that instead to cut down on
    // dynamic memory requirements for potentially large config blobs.
    JSONValue json_root = JSONValue::parse(json, size);
    int error = 0;

    if(json_root.type() == JSON_TYPE_OBJECT)
    {
        JSONObjectIterator it(json_root);
        int field_count = 0;

        while(!error && it.next())
        {
            JSONValue json_child = it.value();
            switch(field_count++)
            {
                case 0:
                    if(strcmp(it.name().data(), CONFIG_SERVICE_FS_VERSION_KEY) ||
                        !json_child.isNumber() ||
                        json_child.toInt() != CONFIG_SERVICE_FS_VERSION)
                    {
                        error = -EINVAL;
                    }
                    break;
                case 1:
                    if(strcmp(it.name().data(), CONFIG_SERVICE_FS_SYNC_HASH_KEY) ||
                        _json_parse_hash_str(json_child, config_desc.file_sync_hash))
                    {
                        error = -EINVAL;
                    }
                    break;
                case 2:
                    if(strcmp(it.name().data(), config_desc.root->name()) ||
                        !json_child.isObject())
                    {
                        error = -EINVAL;
                    }
                    else
                    {
                        error = _config_process_json(json_child, (const char *) it.name(), config_desc.root);
                        if(!error)
                        {
                            config_hash(config_desc.root, config_desc.file_hash);
                            config_desc.hash = config_desc.file_hash;
                            config_desc.sync_hash = config_desc.file_sync_hash;
                        }
                    }
                    break;
                default:
                    error = -EINVAL;
                    break;
            }
        }
    }
    return error;
}

int ConfigService::_load(config_service_desc_t &config_desc)
{
    int error = 0;
    struct stat st;
    String filename = _get_filename(config_desc.root->name());

    if(stat(filename, &st))
    {
        return errno;
    }

    if(!S_ISREG(st.st_mode))
    {
        return -ENOENT;
    }

    int fd = open(filename, O_RDONLY);

    if(fd < 0)
    {
        return errno;
    }

    char *json = (char *) malloc(st.st_size);

    if(!json)
    {
        close(fd);
        return -ENOMEM;
    }

    int rval = read(fd, json, st.st_size);
    if(rval < 0)
    {
        error = errno;
    }
    else if(rval != st.st_size)
    {
        error = -EINVAL;
    }
    else
    {
        Log.info("loading config %s: %.*s", config_desc.root->name(), (int) st.st_size, json);
        error = _process_load(config_desc, json, st.st_size);
    }

    free(json);
    close(fd);

    return error;
}

std::list<config_service_desc_t>::iterator ConfigService::get_module(const char *name)
{
    for(auto it = configs.begin(); it != configs.end(); it++)
    {
        if(!strcmp(it->root->name(), name))
        {
            return it;
        }
    }

    return configs.end();
}

int ConfigService::registerModule(ConfigNode &root)
{
    if(get_module(root.name()) != configs.end())
    {
        return -EEXIST;
    }

    config_service_desc_t desc = {&root};

    if(fs_ok)
    {
        _load(desc);
    }

    configs.push_front(desc);

    return 0;
}

// callback for cloud "get_cfg" command
// marks requested module(s) as dirty by invalidating the last synced crc
int ConfigService::get_cfg_cb(CloudServiceStatus status, JSONValue *root, const void *context)
{
    JSONValue *config = nullptr;
    JSONValue child;
    int rval = -EINVAL;

    JSONObjectIterator obj_it(*root);
    while(obj_it.next())
    {
        if(!strcmp("cfg", (const char *) obj_it.name()))
        {
            child = obj_it.value();
            config = &child;
        }
    }

    if(!config)
    {
        rval = 0;
        for(auto it = configs.begin(); it != configs.end(); it++)
        {
            it->sync_hash.h[0]--;
        }
    }
    else if(config->isObject())
    {
        rval = 0;
        obj_it = JSONObjectIterator(*config);

        while(obj_it.next())
        {
            for(auto it = configs.begin(); it != configs.end(); it++)
            {
                if(!strcmp(it->root->name(), (const char *) obj_it.name()))
                {
                    it->sync_hash.h[0]--;
                }
            }
        }
    }

    CloudService::instance().sendAck(*root, rval);

    return rval;
}

// callback for cloud "set_cfg" command
// iterates and applies json config to the specified config descriptors
int ConfigService::set_cfg_cb(CloudServiceStatus status, JSONValue *root, const void *context)
{
    JSONValue *config = nullptr;
    JSONValue child;
    int rval = 0;

    JSONObjectIterator obj_it(*root);
    while(obj_it.next())
    {
        if(!strcmp("cfg", (const char *) obj_it.name()))
        {
            if(!obj_it.value().isObject())
            {
                break;
            }
            child = obj_it.value();
            config = &child;
            break;
        }
    }

    if(config)
    {
        obj_it = JSONObjectIterator(*config);

        while(obj_it.next())
        {
            auto it = configs.begin();
            for(; it != configs.end(); it++)
            {
                if(!strcmp(it->root->name(), obj_it.name().data()))
                {
                    auto _config = obj_it.value();
                    auto _rval = _config_process_json(_config, it->root->name(), it->root);

                    // want to continue processing all modules and not abort if
                    // one module fails but still report overall failure if
                    // an earlier module fails so only update the overall rval
                    // on an actual failure code
                    if(_rval)
                    {
                        rval = _rval;
                    }
                    break;
                }
            }

            if(it == configs.end())
            {
                Log.error("%s: unexpected module: %s", __func__, obj_it.name().data());
                rval = -ENODEV;
            }
        }
    }
    else
    {
        rval = -EINVAL;
    }

    CloudService::instance().sendAck(*root, rval);

    return rval;
}

int ConfigService::reset_to_factory_cb(CloudServiceStatus status, JSONValue *root, const void *context)
{
    resetToFactory();

    CloudService::instance().sendAck(*root, 0);

    return 0;
}

// callback for ack to overall config sync with cloud (on boot)
int ConfigService::sync_ack_cb(CloudServiceStatus status, JSONValue *root, const char *req_event, const void *context)
{
    if(status == CloudServiceStatus::SUCCESS && sync_pending)
    {
        sync_ok = true;
    }
    sync_pending = false;

    return 0;
}

// callback for ack to individual config sync with cloud
int ConfigService::config_sync_ack_cb(CloudServiceStatus status, JSONValue *root, const char *req_event, const void *context)
{
    if(status == CloudServiceStatus::SUCCESS)
    {
        if(config_sync_pending_object)
        {
            config_sync_pending_object->sync_hash = config_sync_pending_hash;
        }
    }
    config_sync_pending_object = nullptr;

    return 0;
}

// pair traversal of json object and config object and apply updates to config
// object from the json object
int _config_process_json(JSONValue &json_root, const char *json_root_name, ConfigNode *config_root)
{
    int error = -EINVAL;

    switch(json_root.type())
    {
        case JSON_TYPE_INVALID:
            break;
        case JSON_TYPE_NULL:
            break;
        case JSON_TYPE_BOOL:
            if(config_root->type() == CONFIG_NODE_TYPE_BOOL)
            {
                error = reinterpret_cast<ConfigBool *>(config_root)->set(json_root.toBool());
            }
            break;
        case JSON_TYPE_NUMBER:
            if(config_root->type() == CONFIG_NODE_TYPE_INT)
            {
                error = reinterpret_cast<ConfigInt *>(config_root)->set(json_root.toInt());
            }
            else if(config_root->type() == CONFIG_NODE_TYPE_FLOAT)
            {
                error = reinterpret_cast<ConfigFloat *>(config_root)->set(json_root.toDouble());
            }
            break;
        case JSON_TYPE_STRING:
            if(config_root->type() == CONFIG_NODE_TYPE_STRING)
            {
                error = reinterpret_cast<ConfigString *>(config_root)->set((const char *) json_root.toString());
            }
            else if(config_root->type() == CONFIG_NODE_TYPE_STRING_ENUM)
            {
                error = reinterpret_cast<ConfigStringEnum *>(config_root)->set((const char *) json_root.toString());
            }
            break;
        case JSON_TYPE_ARRAY:
        {
            error = -EINVAL;
            break;
        }
        case JSON_TYPE_OBJECT:
        {
            if(config_root->type() == CONFIG_NODE_TYPE_OBJECT)
            {
                JSONObjectIterator it(json_root);
                ConfigObject *config_object = reinterpret_cast<ConfigObject *>(config_root);
                error = config_object->enter(true);
                while(!error && it.next())
                {
                    JSONValue json_child = it.value();
                    ConfigNode *config_child = config_object->child((const char *) it.name());
                    // missing node is non-fatal, pass if child lookup fails
                    if(config_child)
                    {
                        error = _config_process_json(json_child, (const char *) it.name(), config_child);
                    }
                }
                error = config_object->exit(true, error);
            }
            break;
        }
    }

    return error;
}

// pair traversal of json object and config object and apply updates to config
// object from the json object
int config_process_json(const char *json, size_t size, ConfigNode *config_root)
{
    JSONValue json_root = JSONValue::parseCopy(json, size);

    return _config_process_json(json_root, "", config_root);
}

// writes config as a json object to the output writer
int config_write_json(ConfigNode *root, JSONWriter &writer)
{
    int error = -EINVAL;
    switch(root->type())
    {
        case CONFIG_NODE_TYPE_INT:
        {
            int32_t value;
            error = reinterpret_cast<ConfigInt *>(root)->get(value);
            if(!error)
            {
                writer.name(root->name()).value((int) value);
            }
            break;
        }
        case CONFIG_NODE_TYPE_BOOL:
        {
            bool value;
            error = reinterpret_cast<ConfigBool *>(root)->get(value);
            if(!error)
            {
                writer.name(root->name()).value(value);
            }
            break;
        }
        case CONFIG_NODE_TYPE_FLOAT:
        {
            double value;
            error = reinterpret_cast<ConfigFloat *>(root)->get(value);
            if(!error)
            {
                writer.name(root->name()).value(value);
            }
            break;
        }
        case CONFIG_NODE_TYPE_STRING:
        {
            const char *value;
            error = reinterpret_cast<ConfigString *>(root)->get(value);
            if(!error)
            {
                writer.name(root->name()).value(value);
            }
            break;
        }
        case CONFIG_NODE_TYPE_STRING_ENUM:
        {
            const char *value;
            error = reinterpret_cast<ConfigStringEnum *>(root)->get(value);
            if(!error)
            {
                writer.name(root->name()).value(value);
            }
        }
        case CONFIG_NODE_TYPE_ARRAY:
            break;
        case CONFIG_NODE_TYPE_UNKNOWN:
            break;
        case CONFIG_NODE_TYPE_OBJECT:
        {
            auto object_node = reinterpret_cast<ConfigObject *>(root);

            error = object_node->enter(false);
            if(!error)
            {
                if(root->name())
                {
                    writer.name(root->name()).beginObject();
                }
                else
                {
                    writer.beginObject();
                }

                for(int i=0; i < object_node->child_count(); i++)
                {
                    auto child = object_node->child(i);
                    if(child->name())
                    {
                        error = config_write_json(child, writer);
                    }
                }
                writer.endObject();
                error = object_node->exit(false, error);
            }
            break;
        }
    }

    return error;
}

// hash the config structure into the output hash object
void _config_hash(ConfigNode *root, murmur3_hash_t &hash)
{
    int error;

    switch(root->type())
    {
        case CONFIG_NODE_TYPE_INT:
        {
            int32_t value;
            error = reinterpret_cast<ConfigInt *>(root)->get(value);
            if(!error)
            {
                murmur3_hash_update(hash, &value, sizeof(value));
            }
            break;
        }
        case CONFIG_NODE_TYPE_BOOL:
        {
            bool value;
            error = reinterpret_cast<ConfigBool *>(root)->get(value);
            if(!error)
            {
                murmur3_hash_update(hash, &value, sizeof(value));
            }
            break;
        }
        case CONFIG_NODE_TYPE_FLOAT:
        {
            double value;
            error = reinterpret_cast<ConfigFloat *>(root)->get(value);
            if(!error)
            {
                murmur3_hash_update(hash, &value, sizeof(value));
            }
            break;
        }
        case CONFIG_NODE_TYPE_STRING:
        {
            const char *value;
            error = reinterpret_cast<ConfigString *>(root)->get(value);
            if(!error)
            {
                murmur3_hash_update(hash, value, strlen(value));
            }
            break;
        }
        case CONFIG_NODE_TYPE_STRING_ENUM:
        {
            const char *value;
            error = reinterpret_cast<ConfigStringEnum *>(root)->get(value);
            if(!error)
            {
                murmur3_hash_update(hash, value, strlen(value));
            }
        }
        case CONFIG_NODE_TYPE_ARRAY:
            break;
        case CONFIG_NODE_TYPE_UNKNOWN:
            break;
        case CONFIG_NODE_TYPE_OBJECT:
        {
            auto object_node = reinterpret_cast<ConfigObject *>(root);

            error = object_node->enter(false);
            if(!error)
            {
                if(root->name())
                {
                    murmur3_hash_update(hash, root->name(), strlen(root->name()));
                }

                for(int i=0; i < object_node->child_count(); i++)
                {
                    auto child = object_node->child(i);
                    if(child->name())
                    {
                        _config_hash(child, hash);
                    }
                }
                object_node->exit(false, error);
            }
            break;
        }
    }
}

void config_hash(ConfigNode *root, murmur3_hash_t &hash)
{
    murmur3_hash_start(hash, 0);
    _config_hash(root, hash);
    murmur3_hash_finalize(hash);
}

int config_get_int32_cb(int32_t &value, const void *context)
{
    value = *(int32_t *)context;

    return 0;
}

int config_set_int32_cb(int32_t value, const void *context)
{
    *(int32_t *)context = value;

    return 0;
}

int config_get_float_cb(double &value, const void *context)
{
    value = *(double *)context;

    return 0;

}

int config_set_float_cb(double value, const void *context)
{
    *(double *)context = value;

    return 0;
}

int config_get_bool_cb(bool &value, const void *context)
{
    value = *(bool *)context;

    return 0;
}

int config_set_bool_cb(bool value, const void *context)
{
    *(bool *)context = value;

    return 0;
}

int config_set_string_cb(const char *value, const void *context)
{
    char *s = (char *) context;

    // TODO: This makes me so very nervous but range-checking for the value
    // string to fit into s already checked before callback execution.
    // I suppose if the string was mutated in some fashion in between the check
    // and the copy could still allow for a buffer overflow...
    strcpy(s, value);

    return 0;
}

int config_get_string_cb(const char * &value, const void *context)
{
    value = (const char *) context;

    return 0;
}
