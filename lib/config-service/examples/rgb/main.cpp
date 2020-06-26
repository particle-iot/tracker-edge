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
#include "config_service.h"
#include "background_publish.h"


// some dummy config modules for testing
#include "rgb_dummy.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

PRODUCT_ID(10909);
PRODUCT_VERSION(1);

// SerialLogHandler logHandler(Serial, LOG_LEVEL_TRACE);

SerialLogHandler logHandler(115200, LOG_LEVEL_INFO, {
    { "app", LOG_LEVEL_ALL },
});

// high-level class to manage the JSON pub/sub semantics defined for the config
// servce and related
CloudService *cloud_service;

// high level class to expose configuration of registered modules in a generic
// fashion (including a JSON/etc interface via CloudService)
ConfigService *config_service;

// low-level class to manage publishes in a non-blocking fashion via a
// background thread
BackgroundPublish *background_publish;

void setup()
{
    Serial.begin();
    waitFor(Serial.isConnected, 10000);

    background_publish = new BackgroundPublish();
    background_publish->start();

    cloud_service = new CloudService(*background_publish);

    config_service = new ConfigService(*cloud_service);

    rgb_dummy_init(config_service);

    // do Particle.connect last
    // otherwise Particle.variable/Particle.function/Particle.subscribe all
    // turn into blocking calls
    Particle.connect();
}

void loop()
{
    static uint32_t last_sec = System.uptime();

    if(last_sec != System.uptime())
    {
        last_sec = System.uptime();
    }

    // background publish runs in its own thread and does not have a tick
    cloud_service->tick();
    config_service->tick();
    rgb_dummy_tick();
}
