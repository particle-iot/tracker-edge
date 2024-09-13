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

namespace particle {

enum class BmiVariant {
    IMU_BMI160,
    IMU_INVALID
};

enum class BmiEventType {
    NONE,
    BREAK,
    SYNC,
};

enum class BmiPowerState {
    PMU_SUSPEND,
    PMU_NORMAL,
    PMU_LOW_POWER,
    PMU_FAST_STARTUP,
};

enum class BmiAccelMotionMode {
    ACCEL_MOTION_MODE_ANY,
    ACCEL_MOTION_MODE_SIGNIFICANT,
};

enum class BmiAccelSignificantMotionSkip {
    SIG_MOTION_SKIP_1_5_S           = 0,
    SIG_MOTION_SKIP_3_0_S           = 1,
    SIG_MOTION_SKIP_6_0_S           = 2,
    SIG_MOTION_SKIP_12_S            = 3,
};

enum class BmiAccelSignificantMotionProof {
    SIG_MOTION_PROOF_0_25_S         = 0,
    SIG_MOTION_PROOF_0_5_S          = 1,
    SIG_MOTION_PROOF_1_S            = 2,
    SIG_MOTION_PROOF_2_S            = 3,
};

struct BmiAccelerometerConfig {
    float rate;
    float range;
};

struct BmiAccelerometer {
    float x;
    float y;
    float z;
};

struct BmiAccelHighGConfig {
    float threshold;
    float duration;
    float hysteresis;
};

struct BmiAccelMotionConfig {
    BmiAccelMotionMode mode;
    float motionThreshold;
    unsigned motionDuration;
    BmiAccelSignificantMotionSkip motionSkip;
    BmiAccelSignificantMotionProof motionProof;
};


/**
 * @brief Class used to abstract IMU.
 */
class TrackerImu {
public:
    public:
    /**
     * @brief Return instance of the TrackerImu
     *
     * @return TrackerImu&
     */
    static TrackerImu &instance()
    {
        if(!_instance)
        {
            _instance = new TrackerImu();
        }
        return *_instance;
    }

    /**
     * @brief Get the IMU variant for this platform
     *
     * @return BmiVariant
     */
    BmiVariant getImuType(void);

    // Common methods
    int begin(const TwoWire* interface, uint8_t address, pin_t interruptPin, size_t eventDepth = 8);
    int begin(const SPIClass& interface, pin_t selectPin, pin_t interruptPin, size_t eventDepth = 8);
    int end();
    int reset();
    int sleep();
    int wakeup();
    int syncEvent(BmiEventType event);
    int waitOnEvent(BmiEventType& event, system_tick_t timeout);

    int initAccelerometer(BmiAccelerometerConfig& config, bool feedback = false);
    int getAccelerometer(BmiAccelerometer& data);
    int getAccelerometerPmu(BmiPowerState& pmu);
    int initMotion(BmiAccelMotionConfig& config, bool feedback = false);
    int initHighG(BmiAccelHighGConfig& config, bool feedback = false);
    int startMotionDetect();
    int stopMotionDetect();
    int startHighGDetect();
    int stopHighGDetect();

    int getStatus(uint32_t& val, bool clear = false);
    bool isMotionDetect(uint32_t val);
    bool isHighGDetect(uint32_t val);

private:
    TrackerImu();
    static TrackerImu *_instance;
    bool          isInitialized_ {false};
    BmiVariant    imu_ {BmiVariant::IMU_INVALID};

}; // class TrackerImu

#define IMU TrackerImu::instance()

} // namespace particle
