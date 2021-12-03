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

#include "ubloxGPS.h"


/**
 * @brief Type of location point structure
 *
 */
enum class LocationType {
    NONE,                           /**< Initial and default type */
    DEVICE,                         /**< Location point came from the device */
    CLOUD,                          /**< Location point came from the cloud */
};

/**
 * @brief Location source for coordinate
 *
 */
enum class LocationSource {
    NONE,                           /**< Initial and default source */
    CELL,                           /**< Geocoordinate sourced from cellular towers */
    WIFI,                           /**< Geocoordinate sourced from WiFi access points */
    GNSS,                           /**< Geocoordinate sourced from GNSS satellites */
};

/**
 * @brief Timescale relevant to epoch time
 *
 */
enum class LocationTimescale {
    TIMESCALE_UTC,                  /**< Coordinated Universal Time */
    TIMESCALE_TAI,                  /**< International Atomic Time */
    TIMESCALE_GPS,                  /**< Global Positioning System */
    TIMESCALE_GLOSNASS,             /**< GLObal NAvigation System */
    TIMESCALE_GS,                   /**< Galileo System */
    TIMESCALE_BD,                   /**< BeiDou */
};

/**
 * @brief Type of point coordinates of the given event.
 *
 */
struct LocationPoint {
    LocationType type;              /**< Type of location point */
    Vector<LocationSource> sources; /**< List of location sources sorted by highest accuracy */
    int locked;                     /**< Indication of GNSS locked status */
    unsigned int lockedDuration;    /**< Duration of the current GNSS lock (if applicable) */
    bool stable;                    /**< Indication if GNNS lock is stable (if applicable) */
    time_t epochTime;               /**< Epoch time from device sources */
    LocationTimescale timeScale;    /**< Epoch timescale */
    double latitude;                /**< Point latitude in degrees */
    double longitude;               /**< Point longitude in degrees */
    float altitude;                 /**< Point altitude in meters */
    float speed;                    /**< Point speed in meters per second */
    float heading;                  /**< Point heading in degrees */
    float horizontalAccuracy;       /**< Point horizontal accuracy in meters */
    float horizontalDop;            /**< Point horizontal dilution of precision */
    float verticalAccuracy;         /**< Point vertical accuracy in meters */
    float verticalDop;              /**< Point vertical dilution of precision */
};

/**
 * @brief Type of point coordinates for waypoint evaluation
 *
 */
struct PointThreshold {
    float radius;
    double latitude;
    double longitude;
};

struct LocationStatus {
    int powered;                    /**< Indication of GNSS power status */
    int locked;                     /**< Indication of GNSS locked status */
    int error;                      /**< Indication of GNSS module error */
};

/**
 * @brief Motion service class to configure and service intertial motion unit events.
 *
 */
class LocationService {
public:
    static constexpr system_tick_t LOCATION_PERIOD_DEFAULT = 60*1000; // Once per minute
    static constexpr system_tick_t LOCATION_EVENTS_DEFAULT = 32;
    static constexpr system_tick_t LOCATION_STARTUP_PERIOD_DEFAULT = 1*1000; // One second
    static constexpr system_tick_t LOCATION_LOCK_PERIOD_DEFAULT = 1*1000; // One second
    static constexpr double LOCATION_LOCK_HDOP_MAX_DEFAULT = 20.0;

    /**
     * @brief Return instance of the LocationService
     *
     * @retval LocationService&
     */
    static LocationService &instance()
    {
        if(!_instance)
        {
            _instance = new LocationService();
        }
        return *_instance;
    }

    /**
     * @brief Initialize the location service
     *
     * @retval SYSTEM_ERROR_NONE
     * @retval SYSTEM_ERROR_INVALID_STATE
     * @retval SYSTEM_ERROR_INTERNAL
     * @retval SYSTEM_ERROR_INVALID_ARGUMENT
     * @retval SYSTEM_ERROR_IO
     */
    int begin(bool fastLock = false);

    /**
     * @brief Set the GNSS fast lock
     *
     * @param enable Enable faster GNSS lock
     */
    void setFastLock(bool enable);

    /**
     * @brief Get the GNSS fast lock
     *
     * @return true Faster GNSS lock is enabled
     * @return false Faster GNSS lock is disabled
     */
    bool getFastLock();

    /**
     * @brief Start the location service
     *
     * @param  restart Optional flag to restart the module
     * @retval SYSTEM_ERROR_NONE
     * @retval SYSTEM_ERROR_INVALID_STATE
     */
    int start(bool restart = false);

    /**
     * @brief Stop the location service
     *
     * @retval SYSTEM_ERROR_NONE
     * @retval SYSTEM_ERROR_INVALID_STATE
     */
    int stop();

    /**
     * @brief Get the location point
     *
     * @param point Returned LocationPoint object contianing location coordinates
     * @retval SYSTEM_ERROR_NONE
     */
    int getLocation(LocationPoint& point);

    /**
     * @brief Get the radius threshold for point event triggering
     *
     * @param radius Returned radius threshold
     * @retval SYSTEM_ERROR_NONE
     */
    int getRadiusThreshold(float& radius);

    /**
     * @brief Set the radius threshold for point event triggering
     *
     * @param radius Radius threshold
     * @retval SYSTEM_ERROR_NONE
     */
    int setRadiusThreshold(float radius);

    /**
     * @brief Get the starting point coordinates to compare for radius thresholding
     *
     * @param latitude Returned latitude in degrees
     * @param longitude Returned longitude in degrees
     * @retval SYSTEM_ERROR_NONE
     * @retval SYSTEM_ERROR_INVALID_STATE
     */
    int getWayPoint(float& latitude, float& longitude);

    /**
     * @brief Set the starting point coordinates to compare for radius thresholding
     *
     * @param latitude Latitude in degrees
     * @param longitude Longitude in degrees
     * @retval SYSTEM_ERROR_NONE
     */
    int setWayPoint(float latitude, float longitude);

    /**
     * @brief Get the distance, in meters, between two location points
     *
     * @param distance Returned distance in meters
     * @param wayPoint Reference location point
     * @param point Measured location point
     * @retval SYSTEM_ERROR_NONE
     * @retval SYSTEM_ERROR_INVALID_STATE
     */
    int getDistance(float& distance, const PointThreshold& wayPoint, const LocationPoint& point);

    /**
     * @brief Evaluate given location point
     *
     * @param outside Returned true if outside radius; otherwise, inside radius
     * @param point Location point to evaluate
     * @retval SYSTEM_ERROR_NONE
     * @retval SYSTEM_ERROR_INVALID_STATE
     */
    int isOutsideRadius(bool& outside, const LocationPoint& point);

    /**
     * @brief Get the GNSS status
     *
     * @param status Status information
     * @retval SYSTEM_ERROR_NONE
     * @retval SYSTEM_ERROR_INVALID_STATE
     */
    int getStatus(LocationStatus& status);

    /**
     * @brief Is GNSS lock stable
     *
     * @retval TRUE if locked
     * @retval FALSE if not locked
     */
    bool isLockStable() {
        return gps_->isLockStable();
    }

    /**
     * @brief Indicate whether the GNSS module is active and sending NMEA/UBX data
     *
     * @return true Is active
     * @return false Is not active
     */
    bool isActive() {
        return gps_->is_active();
    };

private:

    LocationService();
    static LocationService *_instance;

    void cleanup();

    /**
     * @brief Assert SPI chip select
     *
     * @param select true to assert, false to deassert
     * @return true Always
     */
    bool assertSelect(bool select);

    /**
     * @brief Assert power enable
     *
     * @param select true to enabled power, false to disable power
     * @return true Always
     */
    bool assertEnable(bool enable);

    /**
     * @brief Get the Starting Point object
     *
     * @param point Returned starting point object
     * @retval SYSTEM_ERROR_NONE
     * @retval SYSTEM_ERROR_INVALID_STATE
     */
    int getWayPoint(PointThreshold& point);

    RecursiveMutex pointMutex_;
    uint16_t selectPin_;
    uint16_t enablePin_;
    ubloxGPS* gps_;
    PointThreshold pointThreshold_;
    bool pointThresholdConfigured_;
    bool fastGnssLock_;
};
