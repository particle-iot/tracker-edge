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

#include "tracker_cellular.h"

TrackerCellular *TrackerCellular::_instance = nullptr;

TrackerCellular::TrackerCellular() : _signal_update(0), thread(nullptr)
{
    thread = new Thread("tracker_cellular", [this]() {TrackerCellular::thread_f();}, OS_THREAD_PRIORITY_DEFAULT);
}

// a thread to capture cellular signal strength in a non-blocking fashion
void TrackerCellular::thread_f()
{
    while(true)
    {
        if(Cellular.ready())
        {
            CellularSignal rssi = Cellular.RSSI();

            if(rssi.getStrengthValue() < 0)
            {
                auto uptime = System.uptime();
                WITH_LOCK(mutex)
                {
                    _signal = rssi;
                    _signal_update = uptime;
                }
                delay(TRACKER_CELLULAR_PERIOD_SUCCESS_MS);
            }
            else
            {
                // wait longer on error to prevent overuse of cellular modem
                delay(TRACKER_CELLULAR_PERIOD_ERROR_MS);
            }
        }
        else
        {
            _signal_update = 0;
            delay(TRACKER_CELLULAR_PERIOD_SUCCESS_MS);
        }
    }
}

int TrackerCellular::getSignal(CellularSignal &signal, unsigned int max_age)
{
    const std::lock_guard<RecursiveMutex> lg(mutex);

    if(!_signal_update || System.uptime() - _signal_update > max_age)
    {
        return -ENODATA;
    }

    signal = _signal;
    return 0;
}

unsigned int TrackerCellular::getSignalUpdate()
{
    return _signal_update;
}
