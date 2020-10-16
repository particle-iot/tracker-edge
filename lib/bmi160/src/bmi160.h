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

struct Bmi160Accelerometer {
    float x;
    float y;
    float z;
};

enum class Bmi160AccelSignificantMotionSkip {
    SIG_MOTION_SKIP_1_5_S           = 0,
    SIG_MOTION_SKIP_3_0_S           = 1,
    SIG_MOTION_SKIP_6_0_S           = 2,
    SIG_MOTION_SKIP_12_S            = 3,
};

enum class Bmi160AccelSignificantMotionProof {
    SIG_MOTION_PROOF_0_25_S         = 0,
    SIG_MOTION_PROOF_0_5_S          = 1,
    SIG_MOTION_PROOF_1_S            = 2,
    SIG_MOTION_PROOF_2_S            = 3,
};

enum class Bmi160AccelMotionMode {
    ACCEL_MOTION_MODE_ANY,
    ACCEL_MOTION_MODE_SIGNIFICANT,
};

struct Bmi160AccelMotionConfig {
    Bmi160AccelMotionMode mode;
    float motionThreshold;
    unsigned motionDuration;
    Bmi160AccelSignificantMotionSkip motionSkip;
    Bmi160AccelSignificantMotionProof motionProof;
};

struct Bmi160AccelHighGConfig {
    float threshold;
    float duration;
    float hysteresis;
};

struct Bmi160AccelerometerConfig {
    float rate;
    float range;
};

enum class Bmi160InterruptSource {
    INTR_NONE,
    INTR_STEP,
    INTR_SIGNIFICANT_MOTION,
    INTR_ANY_MOTION,
    INTR_PMU_TRIGGER,
    INTR_DOUBLE_TAP,
    INTR_SINGLE_TAP,
    INTR_ORIENTATION,
    INTR_FLAT,
    INTR_HIGH_G,
    INTR_LOW_G,
    INTR_DATA_READY,
    INTR_FIFO_FULL,
    INTR_FIFO_WATERMARK,
    INTR_NO_MOTION,
};

enum Bmi160PmuAccel: uint8_t {
    PMU_STATUS_ACC_SUSPEND              = 0x0,
    PMU_STATUS_ACC_NORMAL               = 0x1,
    PMU_STATUS_ACC_LOW                  = 0x2,
};

enum Bmi160PmuGyro: uint8_t {
    PMU_STATUS_GYRO_SUSPEND             = 0x0,
    PMU_STATUS_GYRO_NORMAL              = 0x1,
    PMU_STATUS_GYRO_FAST_START_UP       = 0x3,
};


class Bmi160 {
public:
    enum class Bmi160EventType {
        NONE,
        BREAK,
        SYNC,
    };

    enum class Bmi160PowerState {
        PMU_SUSPEND,
        PMU_NORMAL,
        PMU_LOW_POWER,
        PMU_FAST_STARTUP,
    };

    // TODO - remove Bmi160InterruptSourceRaw
    enum Bmi160InterruptSourceRaw : uint32_t {
        BMI_INTR_BIT_STEP                   = (1UL << 0),
        BMI_INTR_BIT_SIGNIFICANT_MOTION     = (1UL << 1),
        BMI_INTR_BIT_ANY_MOTION             = (1UL << 2),
        BMI_INTR_BIT_PMU_TRIGGER            = (1UL << 3),
        BMI_INTR_BIT_DOUBLE_TAP             = (1UL << 4),
        BMI_INTR_BIT_SINGLE_TAP             = (1UL << 5),
        BMI_INTR_BIT_ORIENTATION            = (1UL << 6),
        BMI_INTR_BIT_FLAT                   = (1UL << 7),
        // 8 + 0
        // 8 + 1
        BMI_INTR_BIT_HIGH_G                 = (1UL << (8 + 2)),
        BMI_INTR_BIT_LOW_G                  = (1UL << (8 + 3)),
        BMI_INTR_BIT_DATA_READY             = (1UL << (8 + 4)),
        BMI_INTR_BIT_FIFO_FULL              = (1UL << (8 + 5)),
        BMI_INTR_BIT_FIFO_WATERMARK         = (1UL << (8 + 6)),
        BMI_INTR_BIT_NO_MOTION              = (1UL << (8 + 7)),

        BMI_INTR_BIT_ANY_FIRST_X            = (1UL << (16 + 0)),
        BMI_INTR_BIT_ANY_FIRST_Y            = (1UL << (16 + 1)),
        BMI_INTR_BIT_ANY_FIRST_Z            = (1UL << (16 + 2)),
        BMI_INTR_BIT_ANY_SIGN               = (1UL << (16 + 3)),
        BMI_INTR_BIT_TAP_FIRST_X            = (1UL << (16 + 4)),
        BMI_INTR_BIT_TAP_FIRST_Y            = (1UL << (16 + 5)),
        BMI_INTR_BIT_TAP_FIRST_Z            = (1UL << (16 + 6)),
        BMI_INTR_BIT_TAP_SIGN               = (1UL << (16 + 7)),

        BMI_INTR_BIT_HIGH_FIRST_X           = (1UL << (24 + 0)),
        BMI_INTR_BIT_HIGH_FIRST_Y           = (1UL << (24 + 1)),
        BMI_INTR_BIT_HIGH_FIRST_Z           = (1UL << (24 + 2)),
        BMI_INTR_BIT_HIGH_SIGN              = (1UL << (24 + 3)),
        BMI_INTR_BIT_ORIENT_0               = (1UL << (24 + 4)),
        BMI_INTR_BIT_ORIENT_1               = (1UL << (24 + 5)),
        BMI_INTR_BIT_ORIENT_2               = (1UL << (24 + 6)),
        BMI_INTR_BIT_ORIENT_FLAT            = (1UL << (24 + 7)),
    };

    int begin(const TwoWire* interface, uint8_t address, pin_t interruptPin, size_t eventDepth = 8);
    int begin(const SPIClass& interface, pin_t selectPin, pin_t interruptPin, size_t eventDepth = 8);
    int end();
    int reset();
    int sleep();
    int wakeup();
    int syncEvent(Bmi160EventType event);
    int waitOnEvent(Bmi160EventType& event, system_tick_t timeout);

    int getChipId(uint8_t& val);

    int initAccelerometer(Bmi160AccelerometerConfig& config, bool feedback = false);
    int getAccelerometer(Bmi160Accelerometer& data);
    int getAccelerometerPmu(Bmi160PowerState& pmu);
    int initMotion(Bmi160AccelMotionConfig& config, bool feedback = false);
    int initHighG(Bmi160AccelHighGConfig& config, bool feedback = false);
    int startMotionDetect();
    int stopMotionDetect();
    int startHighGDetect();
    int stopHighGDetect();

    int getStatus(uint32_t& val, bool clear = false);
    bool isMotionDetect(uint32_t val);
    bool isHighGDetect(uint32_t val);

    static Bmi160& getInstance();

private:
    enum class InterfaceType {
        BMI_I2C,
        BMI_SPI
    };

    Bmi160();
    ~Bmi160();

    int setSpiMode();
    int initialize();
    int cleanup();
    void bmi160Handler();

    float convertValue(float val, float toRange, float fromRange);
    uint8_t convertRateToOdr(float rate);
    float convertOdrToRate(uint8_t odr);
    int setAccelRange(float& range, bool feedback = false);
    int setAccelRate(float& rate, bool feedback = false);
    int setAccelMotionThreshold(float& threshold, bool feedback = false);
    int setAccelMotionDuration(unsigned& duration, bool feedback = false);
    int setAccelMotionSkip(Bmi160AccelSignificantMotionSkip skip);
    int setAccelMotionProof(Bmi160AccelSignificantMotionProof proof);
    int setAccelHighGThreshold(float& threshold, bool feedback = false);
    int setAccelHighGDuration(float& duration, bool feedback = false);
    int setAccelHighGHysteresis(float& hysteresis, bool feedback = false);

    int writeRegister(uint8_t reg, uint8_t val);
    int readRegister(uint8_t reg, uint8_t* val, int length = 1);

    InterfaceType type_;
    TwoWire* wire_;
    uint8_t address_;
    SPIClass* spi_;
    pin_t csPin_;
    const __SPISettings spiSettings_;
    pin_t intPin_;
    bool initialized_;
    Bmi160PmuAccel accelPmu_;
    Bmi160PmuGyro gyroPmu_;
    int rangeAccel_;
    float rateAccel_;
    uint8_t latchShadow_;
    os_queue_t motionSyncQueue_;
    static RecursiveMutex mutex_;
}; // class Bmi160

#define BMI160 Bmi160::getInstance()

} // namespace particle
