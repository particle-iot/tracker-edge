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

using namespace spark;
using namespace particle;
using namespace std::placeholders;

namespace {

} // anonymous namespace

LocationService *LocationService::_instance = nullptr;

LocationService::LocationService()
    : gps_(nullptr),
      pointThreshold_({0}),
      pointThresholdConfigured_(false),
      fastGnssLock_(false) {

}

int LocationService::begin(bool fastLock) {
    CHECK_FALSE(gps_, SYSTEM_ERROR_INVALID_STATE);

    pinMode(UBLOX_CS_PIN, OUTPUT);
    pinMode(UBLOX_PWR_EN_PIN, OUTPUT);
    pinMode(UBLOX_RESETN_PIN, OUTPUT);
    digitalWrite(UBLOX_RESETN_PIN, LOW);

    CHECK_TRUE(assertEnable(false), SYSTEM_ERROR_IO);
    CHECK_TRUE(assertSelect(false), SYSTEM_ERROR_IO);

    selectPin_ = UBLOX_CS_PIN;
    enablePin_ = UBLOX_PWR_EN_PIN;

    int ret = SYSTEM_ERROR_NONE;

    do {
        gps_ = new ubloxGPS(UBLOX_SPI_INTERFACE,
                            std::bind(&LocationService::assertSelect, this, _1),
                            std::bind(&LocationService::assertEnable, this, _1),
                            UBLOX_TX_READY_MCU_PIN,
                            UBLOX_TX_READY_GPS_PIN);
        if (!gps_) {
            Log.error("ubloxGPS instantiation failed");
            ret = SYSTEM_ERROR_INTERNAL;
            break;
        }

        setFastLock(fastLock);
        return SYSTEM_ERROR_NONE;
    } while (false);

    // Cleanup
    cleanup();

    return ret;
}

void LocationService::cleanup() {
    if (gps_) {
        delete gps_;
    }
}

void LocationService::setFastLock(bool enable) {
    if (gps_) {
        if (enable) {
            gps_->setLockMethod(ubloxGpsLockMethod::HorizontalDop);
            gps_->setLockHdopThreshold(LOCATION_LOCK_HDOP_MAX_DEFAULT);
        } else {
            gps_->setLockMethod(ubloxGpsLockMethod::HorizontalAccuracy);
        }
    }
}

bool LocationService::getFastLock() {
    if (gps_) {
        return (ubloxGpsLockMethod::HorizontalDop == gps_->getLockMethod());
    }

    return false;
}

int LocationService::start(bool restart) {
    CHECK_TRUE(gps_, SYSTEM_ERROR_INVALID_STATE);

    if (restart && gps_->isOn()) {
        gps_->off();
    }

    if (!gps_->isOn()) {
        auto ret = gps_->on();
        if (ret) {
            return ret;
        }
    }

    return SYSTEM_ERROR_NONE;
}

int LocationService::stop() {
    CHECK_TRUE(gps_, SYSTEM_ERROR_INVALID_STATE);
    int ret = SYSTEM_ERROR_NONE;

    if (gps_->isOn()) {
        ret = gps_->off();
    }

    return ret;
}

int LocationService::getLocation(LocationPoint& point) {
    point.type = LocationType::DEVICE;
    point.sources.append(LocationSource::GNSS);

    WITH_LOCK(*gps_) {
        point.locked = (gps_->getLock()) ? 1 : 0;
        point.stable = gps_->isLockStable();
        point.lockedDuration = gps_->getLockDuration();
        point.epochTime = (time_t)gps_->getUTCTime();
        point.timeScale = LocationTimescale::TIMESCALE_UTC;
        if (point.locked) {
            point.latitude = gps_->getLatitude();
            point.longitude = gps_->getLongitude();
            point.altitude = gps_->getAltitude();
            point.speed = gps_->getSpeed(GPS_SPEED_UNIT_MPS);
            point.heading = gps_->getHeading();
            point.horizontalAccuracy = gps_->getHorizontalAccuracy();
            point.horizontalDop = gps_->getHDOP();
            point.verticalAccuracy = gps_->getVerticalAccuracy();
            point.verticalDop = gps_->getVDOP();
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
    CHECK_TRUE(gps_, SYSTEM_ERROR_INVALID_STATE);
    CHECK_TRUE(pointThresholdConfigured_, SYSTEM_ERROR_INVALID_STATE);

    distance = fabs(gps_->getDistance(
        wayPoint.latitude, wayPoint.longitude,
        point.latitude, point.longitude));

    return SYSTEM_ERROR_NONE;
}

int LocationService::isOutsideRadius(bool& outside, const LocationPoint& point) {
    CHECK_TRUE(gps_, SYSTEM_ERROR_INVALID_STATE);
    CHECK_TRUE(pointThresholdConfigured_, SYSTEM_ERROR_INVALID_STATE);

    PointThreshold current;
    getWayPoint(current);

    float distance = fabs(gps_->getDistance(
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
    CHECK_TRUE(gps_, SYSTEM_ERROR_INVALID_STATE);

    auto gpsState = gps_->getGpsStatus();

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
