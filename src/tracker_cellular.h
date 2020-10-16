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

// delay between checking cell strength when no errors detected
#define TRACKER_CELLULAR_PERIOD_SUCCESS_MS (1000)
// delay between checking cell strength when errors detected
// longer than success to minimize thrashing on the cell interface which could
// delay recovery in Device-OS
#define TRACKER_CELLULAR_PERIOD_ERROR_MS (10000)

// cell updates need to be at least this often or flagged as an error
#define TRACKER_CELLULAR_DEFAULT_MAX_AGE_SEC (10)

class TrackerCellular
{
    public:
        int getSignal(CellularSignal &signal, unsigned int max_age=TRACKER_CELLULAR_DEFAULT_MAX_AGE_SEC);
        unsigned int getSignalUpdate();

        void lock() {mutex.lock();}
        void unlock() {mutex.unlock();}

        static TrackerCellular &instance()
        {
            if(!_instance)
            {
                _instance = new TrackerCellular();
            }
            return *_instance;
        }
    private:
        TrackerCellular();

        CellularSignal _signal;
        unsigned int _signal_update;

        RecursiveMutex mutex;
        Thread * thread;

        void thread_f();

        static TrackerCellular *_instance;
};
