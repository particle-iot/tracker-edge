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

#ifndef ARRAY_SIZE
    #define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif

TrackerCellular *TrackerCellular::_instance = nullptr;

TrackerCellular::TrackerCellular() : _signal_update(0), _thread(nullptr)
{
    os_queue_create(&_commandQueue, sizeof(TrackerCellularCommand), 1, nullptr);
    _thread = new Thread("tracker_cellular", [this]() {TrackerCellular::thread_f();}, OS_THREAD_PRIORITY_DEFAULT);
}

int TrackerCellular::startScan() {
    auto event = TrackerCellularCommand::Measure;
    CHECK_FALSE(os_queue_put(_commandQueue, &event, 0, nullptr), SYSTEM_ERROR_BUSY);

    return SYSTEM_ERROR_NONE;
}

int TrackerCellular::parseServeCell(const char* in, CellularServing& out) {
    CellularServing ret;
    char state[16] = {};
    char rat[16] = {};

    out = {};
    auto nitems = sscanf(in, " +QENG: \"servingcell\",\"%15[^\"]\",\"%15[^\"]\",\"%*15[^\"]\","
            "%u,%u,%lX,"
            "%*15[^,],%*15[^,],%*15[^,],%*15[^,],%*15[^,],%X,%d",
            state, rat,
            &ret.mcc, &ret.mnc, &ret.cellId, &ret.tac, &ret.signalPower);

    if (nitems < 7) {
        return SYSTEM_ERROR_NOT_ENOUGH_DATA;
    }

    if (!strncmp(rat, "CAT-M", 5) || !strncmp(rat, "eMTC", 4)) {
        out.rat = RadioAccessTechnology::LTE_CAT_M1;
    }
    else if (!strncmp(rat, "LTE", 3)) {
        out.rat = RadioAccessTechnology::LTE;
    }
    else if (!strncmp(rat, "CAT-NB", 6)) {
        out.rat = RadioAccessTechnology::LTE_NB_IOT;
    }
    else {
        return SYSTEM_ERROR_NOT_SUPPORTED;
    }

    out.mcc = ret.mcc;
    out.mnc = ret.mnc;
    out.cellId = ret.cellId;
    out.tac = ret.tac;
    out.signalPower = ret.signalPower;

    return SYSTEM_ERROR_NONE;
}

int TrackerCellular::serving_cb(int type, const char* buf, int len, TrackerCellular* context) {
    if (type == TYPE_OK) {
        return RESP_OK;
    }

    (void)parseServeCell(buf, context->_servingTower);
    return WAIT;
}

int TrackerCellular::parseCell(const char* in, CellularNeighbor& out) {
    CellularNeighbor ret;
    char rat[16] = {0};

    auto nitems = sscanf(in, " +QENG: \"neighbourcell %*15[^\"]\",\"%15[^\"]\",%lu,%lu,%d,%d,%d",
            rat,
            &ret.earfcn, &ret.neighborId, &ret.signalQuality, &ret.signalPower, &ret.signalStrength);

    if (nitems < 6) {
        return SYSTEM_ERROR_NOT_ENOUGH_DATA;
    }

    if (!strncmp(rat, "CAT-M", 5) || !strncmp(rat, "eMTC", 4)) {
        out.rat = RadioAccessTechnology::LTE_CAT_M1;
    }
    else if (!strncmp(rat, "LTE", 3)) {
        out.rat = RadioAccessTechnology::LTE;
    }
    else if (!strncmp(rat, "CAT-NB", 6)) {
        out.rat = RadioAccessTechnology::LTE_NB_IOT;
    }
    else {
        return SYSTEM_ERROR_NOT_SUPPORTED;
    }

    out.earfcn = ret.earfcn;
    out.neighborId = ret.neighborId;
    out.signalQuality = ret.signalQuality;
    out.signalPower = ret.signalPower;
    out.signalStrength = ret.signalStrength;

    return SYSTEM_ERROR_NONE;
}

int TrackerCellular::neighbor_cb(int type, const char* buf, int len, TrackerCellular* context) {
    if (type == TYPE_OK) {
        return RESP_OK;
    }

    CellularNeighbor neighbor {};
    if (parseCell(buf, neighbor) == SYSTEM_ERROR_NONE) {
        context->addNeighborList(neighbor);
    }

    return WAIT;
}

void TrackerCellular::resetNeighborList() {
    _towerListSize = 0;
}

int TrackerCellular::addNeighborList(const CellularNeighbor& neighbor) {
    if (0 > _towerListSize) {
        resetNeighborList();
    }
    if (ARRAY_SIZE(_towerList) > (size_t)_towerListSize) {
        _towerList[_towerListSize++] = neighbor;
        return SYSTEM_ERROR_NONE;
    }

    return SYSTEM_ERROR_NO_MEMORY;
}

TrackerCellularCommand TrackerCellular::waitOnEvent(system_tick_t timeout) {
    TrackerCellularCommand event {TrackerCellularCommand::None};
    auto ret = os_queue_take(_commandQueue, &event, timeout, nullptr);
    if (ret) {
        event = TrackerCellularCommand::None;
    }

    return event;
}

// a thread to capture cellular signal strength in a non-blocking fashion
void TrackerCellular::thread_f()
{
    auto loop = true;
    while (loop) {
        // Look for requests and provide a loop delay
        auto event = waitOnEvent(TRACKER_CELLULAR_PERIOD_SUCCESS_MS);

        if (Cellular.ready()) {
            // Grab the cellular strength on every loop iteration
            auto rssi = Cellular.RSSI();

            if (rssi.getStrengthValue() < 0) {
                auto uptime = System.uptime();
                WITH_LOCK(mutex) {
                    _signal = rssi;
                    _signal_update = uptime;
                }
            } else {
                _signal_update = 0;
            }
        }

        switch (event) {
            case TrackerCellularCommand::None:
                // Do nothing
                break;

            case TrackerCellularCommand::Exit:
                // Get out of main loop and join
                loop = false;
                break;

            case TrackerCellularCommand::Measure: {
                // Access to this data will always be requested in advance.  We just need
                // to take inventory of what has been collected and data from the operation.

                if (!Cellular.ready()) {
                    WITH_LOCK(mutex) {
                        _userServingTower = {};
                        _userTowerListSize = 0;
                     }
                    // The cellular modem is not even ready (maybe not powered) so leave
                    break;
                }

                auto serveRet = Cellular.command(serving_cb, this, 10000, "AT+QENG=\"servingcell\"\r\n");
                resetNeighborList(); // Clears the list
                auto neighborRet = Cellular.command(neighbor_cb, this, 10000, "AT+QENG=\"neighbourcell\"\r\n");
                // Simple copies for thread safety and to avoid very long holds on the mutex
                WITH_LOCK(mutex) {
                    if (RESP_OK == serveRet) {
                        _userServingTower = _servingTower;
                    } else {
                        _userServingTower = {};
                    }
                    if (RESP_OK == neighborRet) {
                        _userTowerListSize = _towerListSize;
                        for (int i = 0;i < _towerListSize;++i) {
                            _userTowerList[i] = _towerList[i];
                        }
                    } else {
                        _userTowerListSize = 0;
                    }
                }
                break;
            }

            default:
                break;
        }
    }

    // Kill the thread if we get here
    _thread->cancel();
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

int TrackerCellular::getServingTower(CellularServing& serving) {
    WITH_LOCK(mutex) {
        serving = _userServingTower;
    }

    return SYSTEM_ERROR_NONE;
}

int TrackerCellular::getNeighborTowers(Vector<CellularNeighbor>& neigbors) {
    WITH_LOCK(mutex) {
        for (int i = 0;i < _userTowerListSize;++i) {
            neigbors.append(_userTowerList[i]);
        }
    }

    return SYSTEM_ERROR_NONE;
}
