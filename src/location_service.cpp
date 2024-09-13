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

#include <functional>
#include <atomic>
#include <cmath>
#include "Particle.h"
#include "tracker_config.h"
#include "location_service.h"
#include "EdgePlatform.h"

using namespace spark;
using namespace particle;
using namespace std::placeholders;

namespace {

} // anonymous namespace

LocationService *LocationService::_instance = nullptr;

LocationService::LocationService()
    : ubloxGps_(nullptr),
      pointThreshold_({0}),
      pointThresholdConfigured_(false),
      fastGnssLock_(false),
      gnssType_(GnssModuleType::GNSS_NONE) {

}

void LocationService::setModuleType(void)
{
    gnssType_ = GnssModuleType::GNSS_UBLOX;
}

int LocationService::begin(const LocationServiceConfiguration& config) {

    int ret = SYSTEM_ERROR_NONE;

    // Assign the GNSS hardware variant
    setModuleType();

    _deviceConfig = config;

    CHECK_FALSE(ubloxGps_, SYSTEM_ERROR_INVALID_STATE);

    pinMode(UBLOX_CS_PIN, OUTPUT);
    pinMode(UBLOX_PWR_EN_PIN, OUTPUT);
    pinMode(UBLOX_RESETN_PIN, OUTPUT);
    digitalWrite(UBLOX_RESETN_PIN, LOW);

    CHECK_TRUE(assertEnable(false), SYSTEM_ERROR_IO);
    CHECK_TRUE(assertSelect(false), SYSTEM_ERROR_IO);

    selectPin_ = UBLOX_CS_PIN;
    enablePin_ = UBLOX_PWR_EN_PIN;

    do {
        ubloxGps_ = new ubloxGPS(UBLOX_SPI_INTERFACE,
                            std::bind(&LocationService::assertSelect, this, _1),
                            std::bind(&LocationService::assertEnable, this, _1),
                            UBLOX_TX_READY_MCU_PIN,
                            UBLOX_TX_READY_GPS_PIN);
        if (!ubloxGps_) {
            Log.error("ubloxGPS instantiation failed");
            ret = SYSTEM_ERROR_INTERNAL;
            break;
        }

        return SYSTEM_ERROR_NONE;
    } while (false);

    // Cleanup
    cleanup();

    return ret;
}

void LocationService::cleanup() {
    if (ubloxGps_) {
        delete ubloxGps_;
    }
}

void LocationService::setFastLock(bool enable) {
    if (ubloxGps_) {
        if (enable) {
            ubloxGps_->setLockMethod(ubloxGpsLockMethod::HorizontalDop);
            ubloxGps_->setLockHdopThreshold(LOCATION_LOCK_HDOP_MAX_DEFAULT);
        } else {
            ubloxGps_->setLockMethod(ubloxGpsLockMethod::HorizontalAccuracy);
        }
    }
}

bool LocationService::getFastLock() {
    if (ubloxGps_) {
        return (ubloxGpsLockMethod::HorizontalDop == ubloxGps_->getLockMethod());
    }

    return false;
}

bool LocationService::configureGPS(LocationServiceConfiguration& config) {
    enableHotStartOnWake_ = config.enableHotStartOnWake();

    bool ret = true;

    WITH_LOCK(*ubloxGps_) {
        setFastLock(_deviceConfig.enableFastLock());
        ret &= ubloxGps_->setMode(_deviceConfig.udrModel());
        ret &= ubloxGps_->setIMUAlignmentAngles(
            _deviceConfig.imuYaw(),
            _deviceConfig.imuPitch(),
            _deviceConfig.imuRoll()
        );
        ret &= ubloxGps_->setIMUAutoAlignment(_deviceConfig.enableIMUAutoAlignment());
        ret &= ubloxGps_->setUDREnable(_deviceConfig.enableUDR());
        ret &= ubloxGps_->setIMUtoVRP(
            _deviceConfig.imuToVRPX(),
            _deviceConfig.imuToVRPY(),
            _deviceConfig.imuToVRPZ()
        );
        ret &= ubloxGps_->setAOPSettings(_deviceConfig.enableAssistNowAutonomous());
    }
    return ret;
}

int LocationService::start(bool restart) {
    CHECK_TRUE(ubloxGps_, SYSTEM_ERROR_INVALID_STATE);

    if (restart && ubloxGps_->isOn()) {
        if (enableHotStartOnWake_) {
            CHECK_TRUE(ubloxGps_->saveOnShutdown(), SYSTEM_ERROR_INVALID_STATE);
        }
        ubloxGps_->off();
    }

    if (!ubloxGps_->isOn()) {
        auto ret = ubloxGps_->on();
        if (ret) {
            Log.error("Error %d when turning GNSS on", ret);
            return ret;
        }
        Log.info("GNSS Start");
        CHECK_TRUE(configureGPS(_deviceConfig), SYSTEM_ERROR_INVALID_STATE);
    }

    return SYSTEM_ERROR_NONE;
}

int LocationService::stop() {
    int ret = SYSTEM_ERROR_NONE;

    CHECK_TRUE(ubloxGps_, SYSTEM_ERROR_INVALID_STATE);

    if (ubloxGps_->isOn()) {
        Log.info("Turning GNSS off");
        if (enableHotStartOnWake_) {
            CHECK_TRUE(ubloxGps_->saveOnShutdown(), SYSTEM_ERROR_INVALID_STATE);
        }
        ret = ubloxGps_->off();
    }

    return ret;
}

int LocationService::getLocation(LocationPoint& point) {
    point.type = LocationType::DEVICE;
    point.sources.append(LocationSource::GNSS);

    WITH_LOCK(*ubloxGps_) {
        point.locked = (ubloxGps_->getLock()) ? 1 : 0;
        point.stable = ubloxGps_->isLockStable();
        point.lockedDuration = ubloxGps_->getLockDuration();
        point.epochTime = (time_t)ubloxGps_->getUTCTime();
        point.timeScale = LocationTimescale::TIMESCALE_UTC;
        point.satsInUse = ubloxGps_->getSatellites();
        point.satsInView = ubloxGps_->getSatellitesDesc(point.sats_in_view_desc);
        if (point.locked) {
            point.latitude = ubloxGps_->getLatitude();
            point.longitude = ubloxGps_->getLongitude();
            point.altitude = ubloxGps_->getAltitude();
            point.speed = ubloxGps_->getSpeed(GPS_SPEED_UNIT_MPS);
            point.heading = ubloxGps_->getHeading();
            point.horizontalAccuracy = ubloxGps_->getHorizontalAccuracy();
            point.horizontalDop = ubloxGps_->getHDOP();
            point.verticalAccuracy = ubloxGps_->getVerticalAccuracy();
            point.verticalDop = ubloxGps_->getVDOP();
        }
    }

    return SYSTEM_ERROR_NONE;
}

int LocationService::getRadiusThreshold(float& radius) {
    const std::lock_guard<RecursiveMutex> lock(pointMutex_);
    radius = pointThreshold_.radius;
    return SYSTEM_ERROR_NONE;
}

int LocationService::setRadiusThreshold(float radius) {
    const std::lock_guard<RecursiveMutex> lock(pointMutex_);
    pointThreshold_.radius = std::fabs(radius);
    return SYSTEM_ERROR_NONE;
}

int LocationService::getWayPoint(float& latitude, float& longitude) {
    const std::lock_guard<RecursiveMutex> lock(pointMutex_);
    CHECK_TRUE(pointThresholdConfigured_, SYSTEM_ERROR_INVALID_STATE);
    latitude = pointThreshold_.latitude;
    longitude = pointThreshold_.longitude;
    return SYSTEM_ERROR_NONE;
}

int LocationService::setWayPoint(float latitude, float longitude) {
    const std::lock_guard<RecursiveMutex> lock(pointMutex_);
    pointThreshold_.latitude = latitude;
    pointThreshold_.longitude = longitude;
    pointThresholdConfigured_ = true;
    return SYSTEM_ERROR_NONE;
}

int LocationService::getWayPoint(PointThreshold& point) {
    const std::lock_guard<RecursiveMutex> lock(pointMutex_);
    CHECK_TRUE(pointThresholdConfigured_, SYSTEM_ERROR_INVALID_STATE);
    point = pointThreshold_;
    return SYSTEM_ERROR_NONE;
}

int LocationService::getDistance(float& distance, const PointThreshold& wayPoint, const LocationPoint& point) {
    CHECK_TRUE(pointThresholdConfigured_, SYSTEM_ERROR_INVALID_STATE);

    CHECK_TRUE(ubloxGps_, SYSTEM_ERROR_INVALID_STATE);

    distance = fabs(ubloxGps_->getDistance(
        wayPoint.latitude, wayPoint.longitude,
        point.latitude, point.longitude));

    return SYSTEM_ERROR_NONE;
}

int LocationService::isOutsideRadius(bool& outside, const LocationPoint& point) {
    CHECK_TRUE(pointThresholdConfigured_, SYSTEM_ERROR_INVALID_STATE);

    float distance = 0.0;
    PointThreshold current;
    getWayPoint(current);

    CHECK_TRUE(ubloxGps_, SYSTEM_ERROR_INVALID_STATE);

    distance = fabs(ubloxGps_->getDistance(
        current.latitude, current.longitude,
        point.latitude, point.longitude));

    if (distance > current.radius) {
            outside = true;
    } else {
        outside = false;
    }

    return SYSTEM_ERROR_NONE;
}

int LocationService::getStatus(LocationStatus& status) {
    CHECK_TRUE(ubloxGps_, SYSTEM_ERROR_INVALID_STATE);

    auto gpsState = ubloxGps_->getGpsStatus();

    switch (gpsState) {
        case GPS_STATUS_OFF: {
            status.locked = 0;
            status.powered = 0;
            status.error = 0;
            break;
        }

        case GPS_STATUS_FIXING: {
            status.locked = 0;
            status.powered = 1;
            status.error = 0;
            break;
        }

        case GPS_STATUS_LOCK: {
            status.locked = 1;
            status.powered = 1;
            status.error = 0;
            break;
        }

        case GPS_STATUS_ERROR: {
            status.locked = 0;
            status.powered = 0;
            status.error = 1;
            break;
        }

        default: break;
    }

    return SYSTEM_ERROR_NONE;
}

bool LocationService::isLockStable() {
    return ubloxGps_->isLockStable();
}

bool LocationService::isActive() {
    return ubloxGps_->is_active();
};

bool LocationService::assertSelect(bool select)
{
    digitalWrite(selectPin_, (select) ? LOW : HIGH);
    return true;
}

bool LocationService::assertEnable(bool enable)
{
    //       UP               DOWN
    //         ____   ____   ______
    // VCC  __/    \_/             \____
    //      ____   _______   ___   _____
    // RST      \_/             \_/

    if (enable) {
        digitalWrite(enablePin_, HIGH);
        delay(1000);
        digitalWrite(UBLOX_RESETN_PIN, LOW);
        delay(100);
        digitalWrite(UBLOX_RESETN_PIN, HIGH);
        digitalWrite(enablePin_, LOW);
        delay(100);
        digitalWrite(enablePin_, HIGH);
    } else {
        digitalWrite(UBLOX_RESETN_PIN, LOW);
        delay(100);
        digitalWrite(UBLOX_RESETN_PIN, HIGH);
        digitalWrite(enablePin_, LOW);
    }

    return true;
}
