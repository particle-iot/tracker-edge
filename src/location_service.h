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
 * @brief Number of satellite descriptors to store
 *
 */
constexpr uint32_t NUM_SAT_DESC = 12*NUM_GSV_TYPES*NUM_SAT_BANDS;

/**
 * @brief GPS module type
 *
 */
enum class GnssModuleType {
    GNSS_NONE,
    GNSS_UBLOX,
};

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
    unsigned int satsInUse;         /**< Point satellites in use */
    unsigned int satsInView;        /**< Point satellites in view */
    gps_sat_t sats_in_view_desc[NUM_SAT_DESC]; /**< Collection of satellites in view */
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

#ifndef LOCATION_CONFIG_ENABLE_FAST_LOCK
// Enable or disable faster GNSS lock based on HDOP, see LocationServiceConfiguration below
#define LOCATION_CONFIG_ENABLE_FAST_LOCK            (false)
#endif

#ifndef LOCATION_CONFIG_ENABLE_UDR
// Enable or Disable Untethered Dead Reckoning, see LocationServiceConfiguration below
#define LOCATION_CONFIG_ENABLE_UDR                  (false)
#endif

#ifndef LOCATION_CONFIG_UDR_DYNAMIC_MODEL
// Set Dynamic Model for Untethered Dead Reckoning, see LocationServiceConfiguration below
#define LOCATION_CONFIG_UDR_DYNAMIC_MODEL           (UBX_DYNAMIC_MODEL_PORTABLE)
#endif

#ifndef LOCATION_CONFIG_IMU_ORIENTATION_YAW
// Set IMU orientation yaw angles, see LocationServiceConfiguration below
#define LOCATION_CONFIG_IMU_ORIENTATION_YAW         (0.0)
#endif

#ifndef LOCATION_CONFIG_IMU_ORIENTATION_PITCH
// Set IMU orientation pitch angles, see LocationServiceConfiguration below
#define LOCATION_CONFIG_IMU_ORIENTATION_PITCH       (0.0)
#endif

#ifndef LOCATION_CONFIG_IMU_ORIENTATION_ROLL
// Set IMU orientation roll angles, see LocationServiceConfiguration below
#define LOCATION_CONFIG_IMU_ORIENTATION_ROLL        (0.0)
#endif

#ifndef LOCATION_CONFIG_ENABLE_AUTO_IMU_ALIGNMENT
// Enable or disable automatic IMU alignment, see LocationServiceConfiguration below
#define LOCATION_CONFIG_ENABLE_AUTO_IMU_ALIGNMENT   (false)
#endif

#ifndef LOCATION_CONFIG_ENABLE_HOT_START_ON_WAKE
// Enable or disable GNSS Hot Start on Wake, see LocationServiceConfiguration below
#define LOCATION_CONFIG_ENABLE_HOT_START_ON_WAKE    (true)
#endif

#ifndef LOCATION_CONFIG_ENABLE_ASSISTNOW_AUTONOMOUS
// Enable or disable uBlox AssistNow Autonomous setting, see LocationServiceConfiguration below
#define LOCATION_CONFIG_ENABLE_ASSISTNOW_AUTONOMOUS (true)
#endif

/**
 * @brief LocationServiceConfiguration class to configure the tracker device in application
 *
 */
class LocationServiceConfiguration {
public:
    /**
     * @brief Construct a new Tracker Configuration object
     *
     */
    LocationServiceConfiguration() :
        _enableFastLock(LOCATION_CONFIG_ENABLE_FAST_LOCK),
        _enableUDR(LOCATION_CONFIG_ENABLE_UDR),
        _udrDynamicModel(LOCATION_CONFIG_UDR_DYNAMIC_MODEL),
        _imuYaw(LOCATION_CONFIG_IMU_ORIENTATION_YAW),
        _imuPitch(LOCATION_CONFIG_IMU_ORIENTATION_PITCH),
        _imuRoll(LOCATION_CONFIG_IMU_ORIENTATION_ROLL),
        _enableIMUAutoAlignment(LOCATION_CONFIG_ENABLE_AUTO_IMU_ALIGNMENT) ,
        _enableHotStartOnWake(LOCATION_CONFIG_ENABLE_HOT_START_ON_WAKE),
        _enableAssistNowAutonomous(LOCATION_CONFIG_ENABLE_ASSISTNOW_AUTONOMOUS) {
    }

    /**
     * @brief Construct a new Tracker Configuration object
     *
     */
    LocationServiceConfiguration(LocationServiceConfiguration&&) = default;

    /**
     * @brief Enable or disable faster GNSS lock based on HDOP.  May result in poor horizontal accuracy.
     *
     * @param enable Use faster method for GNSS lock state
     * @return LocationServiceConfiguration&
     */
    LocationServiceConfiguration& enableFastLock(bool enable) {
        _enableFastLock = enable;
        return *this;
    }

    /**
     * @brief Indicate if faster GNSS lock based on HDOP is enabled.
     *
     * @return true Faster GNSS lock is enabled
     * @return false Faster GNSS lock is disabled
     */
    bool enableFastLock() const {
        return _enableFastLock;
    }

    /**
     * @brief Enable or disable Untethered Dead Reckoning
     *
     * @@param[in] enable Enable or disable Untethered Dead Reckoning
     * @return LocationServiceConfiguration&
     */
    LocationServiceConfiguration& enableUDR(bool enable) {
        _enableUDR = enable;
        return *this;
    }

    /**
     * @brief Indicate if faster GNSS lock based on HDOP is enabled.
     *
     * @return true Faster GNSS lock is enabled
     * @return false Faster GNSS lock is disabled
     */
    bool enableUDR() const {
        return _enableUDR;
    }

    /**
     * @brief Set the Dynamic Model used for Untethered Dead Reckoning
     *
     * @@param[in] model Untethered Dead Reckoning model
     * @return LocationServiceConfiguration&
     */
    LocationServiceConfiguration& udrModel(ubx_dynamic_model_t model) {
        _udrDynamicModel = model;
        return *this;
    }

    /**
     * @brief Get model configured for Untethered Dead Reckoning
     *
     * @return ubx_dynamic_model_t
     */
    ubx_dynamic_model_t udrModel() const {
        return _udrDynamicModel;
    }

    /**
     * @brief Enable or disable automatic IMU alignment process
     *
     * @@param[in] enable Enable or disable Untethered Dead Reckoning
     * @return LocationServiceConfiguration&
     */
    LocationServiceConfiguration& enableIMUAutoAlignment(bool enable) {
        _enableIMUAutoAlignment = enable;
        return *this;
    }

    /**
     * @brief Indicate if automatic IMU alignment process is enabled
     *
     * @return true IMU auto alignment is enabled
     * @return false  IMU auto alignment is disabled
     */
    bool enableIMUAutoAlignment() const {
        return _enableIMUAutoAlignment;
    }

    /**
     * @brief Set orientation angles for manual IMU alignment
     *
     * @param[in] yaw Yaw angle of IMU
     * @param[in] pitch Pitch angle of IMU
     * @param[in] roll Roll angle of IMU
     * @return LocationServiceConfiguration&
     */
    LocationServiceConfiguration& imuOrientationAngles(double yaw, double pitch, double roll) {
        _imuYaw = yaw;
        _imuPitch = pitch;
        _imuRoll = roll;
        return *this;
    }

    /**
     * @brief Set Yaw angle for manual IMU alignment
     *
     * @param[in] yaw Yaw angle of IMU
     * @return LocationServiceConfiguration&
     */
    LocationServiceConfiguration& imuYaw(double yaw) {
        _imuYaw = yaw;
        return *this;
    }

    /**
     * @brief Get IMU orientation yaw angle
     *
     * @return yaw angle
     */
    double imuYaw() const {
        return _imuYaw;
    }

    /**
     * @brief Set Pitch angle for manual IMU alignment
     *
     * @param[in] pitch Pitch angle of IMU
     * @return LocationServiceConfiguration&
     */
    LocationServiceConfiguration& imuPitch(double pitch) {
        _imuPitch = pitch;
        return *this;
    }

    /**
     * @brief Get IMU orientation pitch angle
     *
     * @return pitch angle
     */
    double imuPitch() const {
        return _imuPitch;
    }

    /**
     * @brief Set Roll angle for manual IMU alignment
     *
     * @param[in] roll Roll angle of IMU
     * @return LocationServiceConfiguration&
     */
    LocationServiceConfiguration& imuRoll(double roll) {
        _imuRoll = roll;
        return *this;
    }

    /**
     * @brief Get IMU orientation roll angle
     *
     * @return roll angle
     */
    double imuRoll() const {
        return _imuRoll;
    }

    /**
     * @brief Set IMU to Vehicle Reference Point distance
     *
     * @param[in] x IMU to VRP X distance (cm)
     * @param[in] y IMU to VRP Y distance (cm)
     * @param[in] z IMU to VRP Z distance (cm)
     * @return LocationServiceConfiguration&
     */
    LocationServiceConfiguration& imuToVRP(int16_t x, int16_t y, int16_t z) {
        _imuVRPX = x;
        _imuVRPY = y;
        _imuVRPZ = z;
        return *this;
    }

    /**
     * @brief Set IMU to Vehicle Reference Point distance, X-dimension
     *
     * @param[in] x IMU to VRP X distance (cm)
     * @return LocationServiceConfiguration&
     */
    LocationServiceConfiguration& imuToVRPX(int16_t x) {
        _imuVRPX = x;
        return *this;
    }

    /**
     * @brief Get IMU to Vehicle Reference Point distance, X-dimension
     *
     * @return IMU to VRP X distance (cm)
     */
    int16_t imuToVRPX() const {
        return _imuVRPX;
    }

    /**
     * @brief Set IMU to Vehicle Reference Point distance, y dimension
     *
     * @param[in] y IMU to VRP Y distance (cm)
     * @return LocationServiceConfiguration&
     */
    LocationServiceConfiguration& imuToVRPY(int16_t y) {
        _imuVRPY = y;
        return *this;
    }

    /**
     * @brief Get IMU to Vehicle Reference Point distance, Y-dimension
     *
     * @return IMU to VRP Y distance (cm)
     */
    int16_t imuToVRPY() const {
        return _imuVRPY;
    }

    /**
     * @brief Set IMU to Vehicle Reference Point distance, Z-dimension
     *
     * @param[in] x IMU to VRP Z distance (cm)
     * @return LocationServiceConfiguration&
     */
    LocationServiceConfiguration& imuToVRPZ(int16_t z) {
        _imuVRPZ = z;
        return *this;
    }

    /**
     * @brief Get IMU to Vehicle Reference Point distance, Z-dimension
     *
     * @return IMU to VRP Z distance (cm)
     */
    int16_t imuToVRPZ() const {
        return _imuVRPZ;
    }

    /**
     * @brief Return if "Hot Start on Wake" is enabled
     *
     * @return Is "Hot Start on Wake" enabled?
     */
    bool enableHotStartOnWake() const {
        return _enableHotStartOnWake;
    }

    /**
     * @brief Set GNSS "Hot Start on Wake" enable
     *
     * @return LocationServiceConfiguration& object
     */
    LocationServiceConfiguration& enableHotStartOnWake(bool enable) {
        _enableHotStartOnWake = enable;
        return *this;
    }

    /**
     * @brief Return if "AssistNow Autonomous" is enabled
     *
     * @return Is "AssistNow Autonomous" enabled?
     */
    bool enableAssistNowAutonomous() const {
        return _enableAssistNowAutonomous;
    }

    /**
     * @brief Set GNSS "AssistNow Autonomous" enable
     *
     * @return LocationServiceConfiguration& object
     */
    LocationServiceConfiguration& enableAssistNowAutonomous(bool enable) {
        _enableAssistNowAutonomous = enable;
        return *this;
    }

    LocationServiceConfiguration& operator=(const LocationServiceConfiguration& rhs) {
        if (this == &rhs) {
            return *this;
        }
        this->_enableFastLock = rhs._enableFastLock;
        this->_enableUDR = rhs._enableUDR;
        this->_udrDynamicModel = rhs._udrDynamicModel;
        this->_imuYaw = rhs._imuYaw;
        this->_imuPitch = rhs._imuPitch;
        this->_imuRoll = rhs._imuRoll;
        this->_enableIMUAutoAlignment = rhs._enableIMUAutoAlignment;
        this->_imuVRPX = rhs._imuVRPX;
        this->_imuVRPY = rhs._imuVRPY;
        this->_imuVRPZ = rhs._imuVRPZ;
        this->_enableHotStartOnWake = rhs._enableHotStartOnWake;
        this->_enableAssistNowAutonomous = rhs._enableAssistNowAutonomous;

        return *this;
    }

    // TODO might want a comparison "==" operator to help with cases in which the configuration
    // is updated in-flight and needs to be verified against the current configuration

private:
    bool _enableFastLock;

    // Untethered Dead Reckoning config
    bool _enableUDR;
    ubx_dynamic_model_t _udrDynamicModel;
    double _imuYaw, _imuPitch, _imuRoll;
    bool _enableIMUAutoAlignment;
    int16_t _imuVRPX, _imuVRPY, _imuVRPZ;
    bool _enableHotStartOnWake;
    bool _enableAssistNowAutonomous;
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
     * @brief Assign appropriate GNSS module used by this platform
     *
     */
    void setModuleType(void);

    /**
     * @brief Initialize the location service
     *
     * @retval SYSTEM_ERROR_NONE
     * @retval SYSTEM_ERROR_INVALID_STATE
     * @retval SYSTEM_ERROR_INTERNAL
     * @retval SYSTEM_ERROR_INVALID_ARGUMENT
     * @retval SYSTEM_ERROR_IO
     */
    int begin(const LocationServiceConfiguration& config);

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
    bool isLockStable();

    /**
     * @brief Indicate whether the GNSS module is active and sending NMEA/UBX data
     *
     * @return true Is active
     * @return false Is not active
     */
    bool isActive();

private:

    LocationService();
    static LocationService *_instance;

    LocationServiceConfiguration _deviceConfig;

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

    /**
     * @brief Set up the GPS with advanced configuration options
     *
     * @return true success
     * @return false failure
     */
    bool configureGPS(LocationServiceConfiguration& config);

    RecursiveMutex pointMutex_;
    uint16_t selectPin_;
    uint16_t enablePin_;
    ubloxGPS* ubloxGps_;
    PointThreshold pointThreshold_;
    bool pointThresholdConfigured_;
    bool fastGnssLock_;
    bool enableHotStartOnWake_;
    GnssModuleType gnssType_;
};
