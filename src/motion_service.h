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

/**
 * @brief Type of source for the given event.
 *
 */
enum class MotionSource {
    MOTION_NONE,                    /**< No movement, periodic timeout */
    MOTION_MOVEMENT,                /**< Movement detected */
    MOTION_HIGH_G,                  /**< High-G movement detected */
    MOTION_ORIENTATION,             /**< Orientation change detected */
};


/**
 * @brief Event reporting structure.
 *
 */
struct MotionEvent {
    MotionSource source;            /**< Type of source generating this event */
    system_tick_t timestamp;        /**< Timestamp, in milliseconds, of the event */
};

/**
 * @brief Sensitivity confuration for motion detection.
 *
 */
enum class MotionDetectionMode {
    NONE,                           /**< No motion detection */
    LOW_SENSITIVITY,                /**< Low sensitivity motion detection */
    MEDIUM_SENSITIVITY,             /**< Medium sensitivity motion detection */
    HIGH_SENSITIVITY,               /**< High sensitivity motion detection */
};

/**
 * @brief Statistics reporting structure.
 *
 */
struct MotionCounters {

    size_t noneEvents;              /**< Count of no movement or periodic timeouts taking from queue */
    size_t syncEvents;              /**< Count of interrupt events from inertial motion units */
    size_t motionEvents;            /**< Count of motion events from inertial motion units */
    size_t highGEvents;             /**< Count of high G events from inertial motion units */
    size_t breakEvents;             /**< Count of graceful thread exits */
};

/**
 * @brief Configuration for high G detection.
 *
 */
enum class HighGDetectionMode {
    DISABLE,                        /**< Disabled high G detection */
    ENABLE,                         /**< Enabled high G detection */
};


/**
 * @brief Motion service class to configure and service intertial motion unit events.
 *
 */
class MotionService {
public:
    static constexpr system_tick_t MOTION_TIMEOUT_DEFAULT = 5*60*1000;
    static constexpr system_tick_t MOTION_EVENTS_DEFAULT = 10;

    /**
     * @brief Return instance of the motion service
     *
     * @retval MotionService&
     */
    static MotionService &instance()
    {
        if(!_instance)
        {
            _instance = new MotionService();
        }
        return *_instance;
    }

    /**
     * @brief Start the motion service
     *
     * @param eventDepth Count of maximum events in queue.
     * @retval SYSTEM_ERROR_NONE
     * @retval SYSTEM_ERROR_INVALID_STATE
     * @retval SYSTEM_ERROR_INTERNAL
     * @retval SYSTEM_ERROR_INVALID_ARGUMENT
     * @retval SYSTEM_ERROR_IO
     */
    int start(size_t eventDepth = MOTION_EVENTS_DEFAULT);

    /**
     * @brief Stop the motion service, gracefully
     *
     * @retval SYSTEM_ERROR_NONE
     * @retval SYSTEM_ERROR_INVALID_STATE
     * @retval SYSTEM_ERROR_UNKNOWN
     */
    int stop();

    /**
     * @brief Stop the motion service, forcibly
     *
     * @retval SYSTEM_ERROR_NONE
     */
    int kill();

    /**
     * @brief Join the motion service thread
     *
     * @retval SYSTEM_ERROR_NONE
     */
    int join();

    /**
     * @brief Enable (and disable) motion detection mode for given sensitivity
     *
     * @param mode One of NONE, LOW_SENSITIVITY, MEDIUM_SENSITIVITY, HIGH_SENSITIVITY
     * @retval SYSTEM_ERROR_NONE
     */
    int enableMotionDetection(MotionDetectionMode mode);

    /**
     * @brief Disable motion detection
     *
     * @retval SYSTEM_ERROR_NONE
     */
    int disableMotionDetection();

    /**
     * @brief Get configured motion detection mode
     *
     * @retval One of NONE, LOW_SENSITIVITY, MEDIUM_SENSITIVITY, HIGH_SENSITIVITY
     */
    MotionDetectionMode getMotionDetection();

    /**
     * @brief Enable high G detection mode
     *
     * @retval SYSTEM_ERROR_NONE
     */
    int enableHighGDetection();

    /**
     * @brief Disable high G detection mode
     *
     * @retval SYSTEM_ERROR_NONE
     */
    int disableHighGDetection();

    /**
     * @brief Get configured high G detection mode
     *
     * @retval One of DISABLED, ENABLED
     */
    HighGDetectionMode getHighGDetection();

    /**
     * @brief Wait and take event items from queue
     *
     * @param event Returned event information
     * @param timeout Timeout in milliseconds to wait for events.  Use 0 to return immediately if no event available.
     * @retval SYSTEM_ERROR_NONE
     */
    int waitOnEvent(MotionEvent& event, system_tick_t timeout);

    /**
     * @brief Get MotionService statistics
     *
     * @param stats Returned structure of MotionService statistics
     */
    void getStatistics(MotionCounters& stats);

    /**
     * @brief Get the event queue depth
     *
     * @return size_t Queue capacity
     */
    size_t getQueueDepth();

    /**
     * @brief Indicate if any IMU module is awake
     *
     * @return true At least one module is keeping the IMU from sleeping
     * @return false No modules are keeping the IMU from sleeping
     */
    bool isAnyAwake();

private:

    MotionService();
    static MotionService *_instance;

    /**
     * @brief MotionService main thread to receive, process, and send events
     *
     * @param context MotionService instance pointer
     */
    static void thread(void* context);

    /**
     * @brief Set the particuler awake flag
     *
     * @param bits Bitmap of flags to set
     */
    void setAwakeFlag(uint32_t bits);

    /**
     * @brief Clear the particuler awake flag
     *
     * @param bits Bitmap of flags to clear
     */
    void clearAwakeFlag(uint32_t bits);

    os_thread_t thread_;
    MotionCounters counters_;
    os_queue_t motionEventQueue_;
    MotionDetectionMode mode_;
    HighGDetectionMode highGMode_;
    uint32_t awakeFlags_;
    size_t eventDepth_;
};
