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
      pointThresholdConfigured_(false) {

}

int LocationService::begin(SPIClass& spi, uint16_t chipSelectPin, uint16_t powerEnablePin, uint16_t txReadyMCUPin, uint16_t txReadyGPSPin) {
    CHECK_FALSE(gps_, SYSTEM_ERROR_INVALID_STATE);

    pinMode(chipSelectPin, OUTPUT);
    pinMode(powerEnablePin, OUTPUT);

    CHECK_TRUE(assertEnable(false), SYSTEM_ERROR_IO);
    CHECK_TRUE(assertSelect(false), SYSTEM_ERROR_IO);

    selectPin_ = chipSelectPin;
    enablePin_ = powerEnablePin;

    int ret = SYSTEM_ERROR_NONE;

    do {
        gps_ = new ubloxGPS(spi,
                            std::bind(&LocationService::assertSelect, this, _1),
                            std::bind(&LocationService::assertEnable, this, _1),
                            txReadyMCUPin,
                            txReadyGPSPin);
        if (!gps_) {
            LOG(ERROR, "ubloxGPS instantiation failed");
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
    if (gps_) {
        delete gps_;
    }
}

int LocationService::start() {
    CHECK_TRUE(gps_, SYSTEM_ERROR_INVALID_STATE);

    gps_->on();

    return SYSTEM_ERROR_NONE;
}

int LocationService::stop() {
    CHECK_TRUE(gps_, SYSTEM_ERROR_INVALID_STATE);

    gps_->off();

    return SYSTEM_ERROR_NONE;
}

int LocationService::getLocation(LocationPoint& point) {
    WITH_LOCK(*gps_) {
        point.locked = (gps_->getLock()) ? 1 : 0;
        point.epochTime = (time_t)gps_->getUTCTime();
        point.timeScale = LocationTimescale::TIMESCALE_UTC;
        if (point.locked) {
            point.latitude = (float)gps_->getLatitude();
            point.longitude = (float)gps_->getLongitude();
            point.altitude = gps_->getAltitude();
            point.speed = gps_->getSpeed(GPS_SPEED_UNIT_MPS);
            point.heading = gps_->getHeading();
            point.horizontalAccuracy = gps_->getHorizontalAccuracy();
            point.verticalAccuracy = gps_->getVerticalAccuracy();
        }
    }

    return SYSTEM_ERROR_NONE;
}

int LocationService::getRadiusThreshold(float& radius) {
    const std::lock_guard<std::recursive_mutex> lock(pointMutex_);
    radius = pointThreshold_.radius;
    return SYSTEM_ERROR_NONE;
}

int LocationService::setRadiusThreshold(float radius) {
    const std::lock_guard<std::recursive_mutex> lock(pointMutex_);
    pointThreshold_.radius = std::fabs(radius);
    return SYSTEM_ERROR_NONE;
}

int LocationService::getWayPoint(float& latitude, float& longitude) {
    const std::lock_guard<std::recursive_mutex> lock(pointMutex_);
    CHECK_TRUE(pointThresholdConfigured_, SYSTEM_ERROR_INVALID_STATE);
    latitude = pointThreshold_.latitude;
    longitude = pointThreshold_.longitude;
    return SYSTEM_ERROR_NONE;
}

int LocationService::setWayPoint(float latitude, float longitude) {
    const std::lock_guard<std::recursive_mutex> lock(pointMutex_);
    pointThreshold_.latitude = latitude;
    pointThreshold_.longitude = longitude;
    pointThresholdConfigured_ = true;
    return SYSTEM_ERROR_NONE;
}

int LocationService::getWayPoint(PointThreshold& point) {
    const std::lock_guard<std::recursive_mutex> lock(pointMutex_);
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
            break;
        }

        case GPS_STATUS_FIXING: {
            status.locked = 0;
            status.powered = 1;
            break;
        }

        case GPS_STATUS_LOCK: {
            status.locked = 1;
            status.powered = 1;
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
    digitalWrite(enablePin_, (enable) ? HIGH : LOW);
    return true;
}
