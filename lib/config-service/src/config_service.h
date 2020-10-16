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

#include "config_service_nodes.h"

#include "cloud_service.h"

#include "murmur3.h"

#include <list>

#ifndef CONFIG_SERVICE_FS_PATH
    #define CONFIG_SERVICE_FS_PATH "/usr/config"
#endif
#define CONFIG_SERVICE_FS_VERSION_KEY "version"
#define CONFIG_SERVICE_FS_SYNC_HASH_KEY "hash"
#define CONFIG_SERVICE_FS_VERSION (1)

extern Logger config_service_log;

typedef struct config_service_desc_t {
    ConfigNode *root;
    // current hash of the config module
    murmur3_hash_t hash;
    // as most recently synchronized successfully with the cloud
    // on mismatch with primary hash will trigger cfg pub to cloud
    murmur3_hash_t sync_hash;
    // as most recently saved to file
    // on mismatch will trigger save to file
    murmur3_hash_t file_hash;
    // as most recently synchronized with the cloud and then saved to file
    // on load this will allow optimization to skip republish of all cfg that
    // are up to date
    // on mismatch will trigger save to file
    murmur3_hash_t file_sync_hash;
} config_service_desc_t;

class ConfigService
{
    public:
        /**
         * @brief Return instance of the config service
         *
         * @retval ConfigService&
         */
        static ConfigService &instance()
        {
            if(!_instance)
            {
                _instance = new ConfigService();
            }
            return *_instance;
        }

        void init();

        void tick();

        int registerModule(ConfigNode &root);

        void resetToFactory();

        void flush();

    private:
        ConfigService();
        static ConfigService *_instance;

        std::list<config_service_desc_t> configs;

        std::list<config_service_desc_t>::iterator get_module(const char *name);

        int get_cfg_cb(CloudServiceStatus status, JSONValue *root, const void *context);
        int set_cfg_cb(CloudServiceStatus status, JSONValue *root, const void *context);
        int reset_to_factory_cb(CloudServiceStatus status, JSONValue *root, const void *context);

        int sync_ack_cb(CloudServiceStatus status, JSONValue *root, const char *req_event, const void *context);
        int config_sync_ack_cb(CloudServiceStatus status, JSONValue *root, const char *req_event, const void *context);

        // process infrequent actions
        void tick_sec();

        void save_all(bool force=false);
        int save(const char *name, bool force=false);

        int _save(config_service_desc_t &config_desc, bool force=false);
        int _load(config_service_desc_t &config_desc);
        String _get_filename(const char *name);

        uint32_t last_tick_sec;

        bool fs_ok;

        bool sync_pending;
        bool sync_ok;

        config_service_desc_t *config_sync_pending_object;
        murmur3_hash_t config_sync_pending_hash;
};
