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

#include "Particle.h"
#include "tracker_config.h"
#include "motion_service.h"
#include "bmi160.h"

using namespace spark;
using namespace particle;

namespace {

// TODO: This should probably be a constant expression
Bmi160AccelerometerConfig bmi160AccelConfig = {
    .rate               = 100.0,    // Hz [0.78Hz -> 1600Hz]
    .range              = 16.0,     // g [2g, 4g, 8g, 16g]
};

Bmi160AccelMotionConfig bmi160MotionConfigs[] = {
    // Low sensitivity motion detection settings
    {
    .mode               = Bmi160AccelMotionMode::ACCEL_MOTION_MODE_SIGNIFICANT,
    .motionThreshold    = 1,      // g [up to range]
    .motionDuration     = 4,        // counts of rate period [1 -> 4]
    .motionSkip         = Bmi160AccelSignificantMotionSkip::SIG_MOTION_SKIP_1_5_S,      // seconds [1.5s, 3s, 6s, 12s]
    .motionProof        = Bmi160AccelSignificantMotionProof::SIG_MOTION_PROOF_0_25_S,   // seconds [0.25s, 0.5s, 1s, 2s]
    },

    // Medium sensitivity motion detection settings
    {
    .mode               = Bmi160AccelMotionMode::ACCEL_MOTION_MODE_ANY,
    .motionThreshold    = 0.5,      // g [up to range]
    .motionDuration     = 4,        // counts of rate period [1 -> 4]
    .motionSkip         = Bmi160AccelSignificantMotionSkip::SIG_MOTION_SKIP_1_5_S,      // seconds [1.5s, 3s, 6s, 12s]
    .motionProof        = Bmi160AccelSignificantMotionProof::SIG_MOTION_PROOF_0_25_S,   // seconds [0.25s, 0.5s, 1s, 2s]
    },

    // High sensitivity motion detection settings
    {
    .mode               = Bmi160AccelMotionMode::ACCEL_MOTION_MODE_ANY,
    .motionThreshold    = 0.1,      // g [up to range]
    .motionDuration     = 1,        // counts of rate period [1 -> 4]
    .motionSkip         = Bmi160AccelSignificantMotionSkip::SIG_MOTION_SKIP_1_5_S,      // seconds [1.5s, 3s, 6s, 12s]
    .motionProof        = Bmi160AccelSignificantMotionProof::SIG_MOTION_PROOF_0_25_S,   // seconds [0.25s, 0.5s, 1s, 2s]
    },
};

Bmi160AccelHighGConfig bmi160HighGConfig = {
    .threshold          = 4.0,          // g [up to range]
    .duration           = 0.0025,       // seconds
    .hysteresis         = 16.0 / 16.0,  // g [up to range]
};

enum MotionAwakeFlags : uint32_t {
    MOTION_AWAKE_NONE       = 0,            // Nothing is awake
    MOTION_AWAKE_HIGH_G     = (1UL << 1),   // High G is awake
    MOTION_AWAKE_SIGANY     = (1UL << 2),   // Significant/any motion is awake
};

} // anonymous namespace

MotionService *MotionService::_instance = nullptr;

MotionService::MotionService()
    : thread_(nullptr),
      counters_({0}),
      motionEventQueue_(nullptr),
      mode_(MotionDetectionMode::NONE),
      highGMode_(HighGDetectionMode::DISABLE),
      awakeFlags_(0),
      eventDepth_(0) {

}

int MotionService::start(size_t eventDepth) {
    CHECK_FALSE(thread_, SYSTEM_ERROR_INVALID_STATE);

    int ret = SYSTEM_ERROR_NONE;

    // Retrieve (if first time) instance of the BMI160 IMU device
    ret = BMI160.begin(BMI160_SPI_INTERFACE, BMI160_SPI_CS_PIN, BMI160_INT_PIN);
    if (ret != SYSTEM_ERROR_NONE) {
        Log.error("BMI160.begin() failed");
        return ret;
    }

    // Clear all configuration to defaults
    BMI160.reset();
    awakeFlags_ = MOTION_AWAKE_NONE;
    CHECK(BMI160.initAccelerometer(bmi160AccelConfig));

    // Create the motion event queue if it hasn't been created yet
    eventDepth_ = eventDepth;
    if (!motionEventQueue_ && os_queue_create(&motionEventQueue_, sizeof(MotionEvent), eventDepth, nullptr)) {
        motionEventQueue_ = nullptr;
        Log.error("os_queue_create() failed");
        return SYSTEM_ERROR_INTERNAL;
    }

    // Start the main MotionService thread
    ret = os_thread_create(&thread_, "MOTSERV", OS_THREAD_PRIORITY_DEFAULT, MotionService::thread, this, OS_THREAD_STACK_SIZE_DEFAULT);
    if (ret) {
        Log.error("os_thread_create() failed");
        return ret;
    }

    return SYSTEM_ERROR_NONE;
}

int MotionService::stop() {
    CHECK_TRUE(thread_, SYSTEM_ERROR_INVALID_STATE);
    CHECK_FALSE(BMI160.syncEvent(Bmi160::Bmi160EventType::BREAK), SYSTEM_ERROR_UNKNOWN);
    return SYSTEM_ERROR_NONE;
}

int MotionService::kill() {
    CHECK_TRUE(thread_, SYSTEM_ERROR_INVALID_STATE);
    CHECK_FALSE(os_thread_exit(thread_), SYSTEM_ERROR_INVALID_STATE);
    return SYSTEM_ERROR_NONE;
}

int MotionService::join() {
    CHECK_TRUE(thread_, SYSTEM_ERROR_INVALID_STATE);
    CHECK_FALSE(os_thread_join(thread_), SYSTEM_ERROR_UNKNOWN);
    thread_ = nullptr;
    return SYSTEM_ERROR_NONE;
}

int MotionService::enableMotionDetection(MotionDetectionMode mode) {
    // The motion service is simply configured from the outside using a handful of
    // abstracted configuration modes: low, medium, and high.
    switch (mode) {
        case MotionDetectionMode::NONE: {
            CHECK(BMI160.stopMotionDetect());
            clearAwakeFlag(MOTION_AWAKE_SIGANY);
            if (!isAnyAwake()) {
                CHECK(BMI160.sleep());
            }
            mode_ = mode;
            return SYSTEM_ERROR_NONE;
        }

        case MotionDetectionMode::LOW_SENSITIVITY: {
            BMI160.initMotion(bmi160MotionConfigs[0], false);

            break;
        }

        case MotionDetectionMode::MEDIUM_SENSITIVITY: {
            BMI160.initMotion(bmi160MotionConfigs[1], false);
            break;
        }

        case MotionDetectionMode::HIGH_SENSITIVITY: {
            BMI160.initMotion(bmi160MotionConfigs[2], false);
            break;
        }

        default: {
            return SYSTEM_ERROR_UNKNOWN;
        }
    }

    if (!isAnyAwake()) {
        CHECK(BMI160.wakeup());
    }
    setAwakeFlag(MOTION_AWAKE_SIGANY);
    CHECK(BMI160.startMotionDetect());

    mode_ = mode;

    return SYSTEM_ERROR_NONE;
}

int MotionService::disableMotionDetection() {
    return enableMotionDetection(MotionDetectionMode::NONE);
}

MotionDetectionMode MotionService::getMotionDetection() {
    return mode_;
}

int MotionService::enableHighGDetection() {
    if (!isAnyAwake()) {
        CHECK(BMI160.wakeup());
    }
    setAwakeFlag(MOTION_AWAKE_HIGH_G);
    BMI160.initHighG(bmi160HighGConfig, false);
    CHECK(BMI160.startHighGDetect());
    highGMode_ = HighGDetectionMode::ENABLE;

    return SYSTEM_ERROR_NONE;
}

int MotionService::disableHighGDetection() {
    CHECK(BMI160.stopHighGDetect());
    highGMode_ = HighGDetectionMode::DISABLE;
    clearAwakeFlag(MOTION_AWAKE_HIGH_G);
    if (!isAnyAwake()) {
        CHECK(BMI160.sleep());
    }

    return SYSTEM_ERROR_NONE;
}

HighGDetectionMode MotionService::getHighGDetection() {
    return highGMode_;
}

int MotionService::waitOnEvent(MotionEvent& event, system_tick_t timeout) {
    auto ret = os_queue_take(motionEventQueue_, &event, timeout, nullptr);
    if (ret) {
        event.source = MotionSource::MOTION_NONE;
    }

    return SYSTEM_ERROR_NONE;
}

void MotionService::getStatistics(MotionCounters& stats) {
    stats = counters_;
}

// TODO: Migrate to conditional variables when they become available from the device OS
void MotionService::thread(void* context) {
    MotionService* self = static_cast<MotionService*>(context);

    // This thread is not expected to exit but provisions were made here to allow for the thread
    // to die on its own using the BREAK event.
    bool exitLoop = false;
    while (!exitLoop) {
        Bmi160::Bmi160EventType event;
        BMI160.waitOnEvent(event, MotionService::MOTION_TIMEOUT_DEFAULT);
        switch (event) {

            // This event may be a result of a timeout of the waitOnEvent() call if
            // a timeout value was given.  It can also be a mechanism to poke the service
            // to perform some kind of housekeeping.
            case Bmi160::Bmi160EventType::NONE: {
                self->counters_.noneEvents++;
                break;
            }

            // This event comes directly from the IMU device driver as a result of one of the
            // several interrupts supported on the device.
            // Capture the device event, inquire as to what caused the interrupt, and wrap this
            // extra information to the queue consumer.
            case Bmi160::Bmi160EventType::SYNC: {
                self->counters_.syncEvents++;
                uint32_t status = 0;
                BMI160.getStatus(status, true);

                if (BMI160.isHighGDetect(status)) {
                    self->counters_.highGEvents++;
                    MotionEvent event = { .source = MotionSource::MOTION_HIGH_G };
                    os_queue_put(self->motionEventQueue_, &event, 0, nullptr);
                }
                if (BMI160.isMotionDetect(status)) {
                    self->counters_.motionEvents++;
                    MotionEvent event = { .source = MotionSource::MOTION_MOVEMENT };
                    os_queue_put(self->motionEventQueue_, &event, 0, nullptr);
                }
                break;
            }

            // This is an explicit request to exit the thread
            case Bmi160::Bmi160EventType::BREAK: {
                self->counters_.breakEvents++;
                exitLoop = true;
                break;
            }

            default: {
                break;
            }
        }
    }

    os_thread_exit(nullptr);
}

size_t MotionService::getQueueDepth() {
    return eventDepth_;
}

void MotionService::setAwakeFlag(uint32_t bits) {
    awakeFlags_ |= bits;
}

void MotionService::clearAwakeFlag(uint32_t bits) {
    awakeFlags_ &= ~bits;
}

bool MotionService::isAnyAwake() {
    return (awakeFlags_ == MOTION_AWAKE_NONE) ? false : true;
}
