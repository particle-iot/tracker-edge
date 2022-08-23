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

#define MEMFAULT_PARTICLE_PORT_CPP_ONLY_SYSTEM_VERSION  (0)

// Reserve space for exception traces
#define MEMFAULT_PLATFORM_COREDUMP_STORAGE_RAM_SIZE     (2048)

//! WARNING: This should only be set to "1" for development builds & testing
#define MEMFAULT_PARTICLE_PORT_DEBUG_API_ENABLE         (1)

// The name reported into Memfault for this application
#define MEMFAULT_PARTICLE_PORT_SOFTWARE_TYPE            "tracker-edge"

//! WARNING: Only uncomment this if the user wants to override default settings
#define MEMFAULT_METRICS_HEARTBEAT_INTERVAL_SECS        (60)

//! WARNING: Enabling logging will increase data usage and program memory size
#define MEMFAULT_PARTICLE_PORT_LOG_STORAGE_ENABLE       (0)
#define MEMFAULT_PARTICLE_PORT_LOGGING_ENABLE           (0)

//! WARNING: This overrides a default storage size in the library
#define MEMFAULT_PARTICLE_PORT_LOG_STORAGE_SIZE         (1024)
