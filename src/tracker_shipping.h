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

#include "cloud_service.h"

typedef std::function<int(void)> shipping_mode_shutdown_cb_t;

class TrackerShipping
{
    public:
        TrackerShipping() : shutdown_cb(nullptr) {}
        
        void init();

        int enter();

        int regShutdownCallback(shipping_mode_shutdown_cb_t cb);
    private:
        shipping_mode_shutdown_cb_t shutdown_cb;

        int enter_cb(CloudServiceStatus status, JSONValue *root, const void *context);
        static void shutdown();
};
