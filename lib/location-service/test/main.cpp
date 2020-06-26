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

#include <cmath>
#include "Particle.h"
#include "tracker_config.h"
#include "location_service.h"

#define LOCK_LED                (A0)

/*
LOG_LEVEL_ALL   : special value that can be used to enable logging of all messages
LOG_LEVEL_TRACE : verbose output for debugging purposes
LOG_LEVEL_INFO  : regular information messages
LOG_LEVEL_WARN  : warnings and non-critical errors
LOG_LEVEL_ERROR : error messages
LOG_LEVEL_NONE  : special value that can be used to disable logging of any messages
*/
SerialLogHandler logHandler(115200, LOG_LEVEL_ALL,
                            {
                                {"app", LOG_LEVEL_ALL},
                            });

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(MANUAL);

LocationService service;

void locationVacation() {
    LocationPoint point;

    auto ret = service.getLocation(point);
    if (ret) {
        Serial1.println("ERROR: getLocation()");
        return;
    }

    if (point.locked) {
        digitalWrite(LOCK_LED, HIGH);

        PointThreshold waypoint;
        ret = service.getWayPoint(waypoint.latitude, waypoint.longitude);
        if (ret) {
            Serial1.println("ERROR: getWayPoint()");
            return;
        }

        float distance = 0.0;
        if (service.getDistance(distance, waypoint, point)) {
            Serial1.println("ERROR: getDistance()");
            return;
        }

        bool outside = false;
        if (service.isOutsideRadius(outside, point)) {
            Serial1.println("ERROR: isOutsideRadius()");
            return;
        }

        Serial1.printlnf("%lu: %.2f %d {\"loc\":{\"lcl\":1,\"time\":%lu,\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.2f,\"hd\":%.2f,\"h_acc\":%.2f,\"v_acc\":%.2f}}",
            Time.now(),
            distance, (outside) ? 1 : 0,
            point.epochTime,
            point.latitude,
            point.longitude,
            point.altitude,
            point.speed,
            point.heading,
            point.horizontalAccuracy,
            point.verticalAccuracy
        );

    } else {
        digitalWrite(LOCK_LED, LOW);
        Serial1.printlnf("%lu: {\"loc\":{\"lcl\":0,\"time\":%lu}}",
            Time.now(),
            point.epochTime
        );

    }
}

void setup() {
    Serial1.begin(115200);
    Serial.begin();

    SPI.setDataMode(SPI_MODE0);
    SPI.setClockSpeed(100 * 1000);
    SPI.begin();

#ifdef LOCK_LED
    pinMode(LOCK_LED, OUTPUT);
    digitalWrite(LOCK_LED, LOW);
#endif

    Serial1.println("Starting LocationService");
    auto ret = service.begin(UBLOX_SPI_INTERFACE, UBLOX_CS_PIN, UBLOX_PWR_EN_PIN);
    if (ret) {
        Serial1.println("Failed to begin location service");
    }

    delay(1000);
    service.start();
}

static volatile system_tick_t locTick = 0;
static volatile system_tick_t movementTick = 0;
static volatile system_tick_t updateRate = 1 * 1000;
static volatile float velocityDeg = 0.0; // deg/s
static volatile float heading = 0.0; // compass 0.0 North, 270.0 West

void loop() {
    int ret = SYSTEM_ERROR_NONE;

    if (Serial.isConnected() && (Serial.available() > 0)) {
        int c = Serial.read();
        ret = SYSTEM_ERROR_NONE;
        Serial.printlnf("Running '%c'", (char)c);
        switch ((char)c) {
            case 'r': ret = service.start(); break;
            case 's': ret = service.stop(); break;
            case '1': ret = service.setRadiusThreshold(1.0); break;
            case '2': ret = service.setRadiusThreshold(10.0); break;
            case '3': ret = service.setRadiusThreshold(100.0); break;
            case '4': velocityDeg = 0.0; heading = 0.0; break;
            case '5': velocityDeg = 0.000005; heading = 0.0; break; // walking
            case '6': velocityDeg = 0.000060; heading = 0.0; break; // bicycle
            case '7': velocityDeg = 0.000300; heading = 0.0; break; // highway speed
            case '8': velocityDeg = 0.000500; heading = 0.0; break; // small aircraft
            case 'w':{
                LocationPoint point;
                ret = service.getLocation(point);
                if (ret) {
                    Serial1.println("ERROR: getLocation()");
                    break;
                }

                ret = service.setWayPoint(point.latitude, point.longitude);
                if (ret) {
                    Serial1.println("ERROR: setWayPoint()");
                    break;
                }

                float latVerify, lonVerify;
                ret = service.getWayPoint(latVerify, lonVerify);
                if (ret) {
                    Serial1.println("ERROR: getWayPoint()");
                    break;
                }

                Serial1.printlnf("Set {%.6f,%.6f} to {%.6f,%.6f}",
                    point.latitude, point.longitude,
                    latVerify, lonVerify
                );
                break;
            }
        }
        if (ret) {
            Serial.printlnf("'%c' returned %d", (char)c, ret);
        }
    }

    if (millis() - movementTick >= 1 * 1000) {
        movementTick = millis();
        if (velocityDeg != 0.0) {
            float lat = 0.0;
            float lon = 0.0;
            service.getWayPoint(lat, lon);
            lat += velocityDeg * sinf((90.0 - heading) * M_PI / 180.0);
            lat += velocityDeg * cosf((90.0 - heading) * M_PI / 180.0);
            service.setWayPoint(lat, lon);
        }
    }

    if (millis() - locTick >= updateRate) {
        locTick = millis();
        locationVacation();
    }
}
