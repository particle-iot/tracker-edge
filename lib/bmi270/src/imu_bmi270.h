/*
 * Copyright (c) 2022 Particle Industries, Inc.
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
#include "Bosch/bmi270.h"
#include "Bosch/bmi270_legacy.h"

namespace particle {

struct Bmi270Accelerometer {
    float x;
    float y;
    float z;
};

struct Bmi270Gyrometer {
    float x;
    float y;
    float z;
};

enum class Bmi270AccelSignificantMotionSkip {
    SIG_MOTION_SKIP_1_5_S           = 0,
    SIG_MOTION_SKIP_3_0_S           = 1,
    SIG_MOTION_SKIP_6_0_S           = 2,
    SIG_MOTION_SKIP_12_S            = 3,
};

enum class Bmi270AccelSignificantMotionProof {
    SIG_MOTION_PROOF_0_25_S         = 0,
    SIG_MOTION_PROOF_0_5_S          = 1,
    SIG_MOTION_PROOF_1_S            = 2,
    SIG_MOTION_PROOF_2_S            = 3,
};

enum class Bmi270AccelMotionMode {
    ACCEL_MOTION_MODE_ANY,
    ACCEL_MOTION_MODE_SIGNIFICANT,
};

struct Bmi270AccelMotionConfig {
    Bmi270AccelMotionMode mode;
    float motionThreshold;
    unsigned motionDuration;
    Bmi270AccelSignificantMotionSkip motionSkip;
    Bmi270AccelSignificantMotionProof motionProof;
};

struct Bmi270AccelHighGConfig {
    float threshold;
    float duration;
    float hysteresis;
};

struct Bmi270AccelerometerConfig {
    float rate;
    float range;
};

enum class Bmi270InterruptSource {
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

enum class Bmi270PmuAccel: uint8_t {
    PMU_STATUS_ACC_SUSPEND              = 0x0,
    PMU_STATUS_ACC_NORMAL               = 0x1,
    PMU_STATUS_ACC_LOW                  = 0x2,
};

enum class Bmi270PmuGyro: uint8_t {
    PMU_STATUS_GYRO_SUSPEND             = 0x0,
    PMU_STATUS_GYRO_NORMAL              = 0x1,
    PMU_STATUS_GYRO_FAST_START_UP       = 0x3,
};


class Bmi270 {
public:
    enum class Bmi270EventType {
        NONE,
        BREAK,
        SYNC,
    };

    enum class Bmi270PowerState {
        PMU_SUSPEND,
        PMU_NORMAL,
        PMU_LOW_POWER,
        PMU_FAST_STARTUP,
    };

    int begin(const TwoWire* interface, uint8_t address, pin_t interruptPin, size_t eventDepth = 8);
    int begin(const SPIClass& interface, pin_t selectPin, pin_t interruptPin, size_t eventDepth = 8);
    int end();
    int reset();
    int sleep();
    int wakeup();
    int syncEvent(Bmi270EventType event);
    int waitOnEvent(Bmi270EventType& event, system_tick_t timeout);

    int getChipId(uint8_t& val);

    int initialize();
    int initAccelerometer(Bmi270AccelerometerConfig& config, bool feedback = false);
    int getAccelerometer(Bmi270Accelerometer& data);
    int getGyrometer(Bmi270Gyrometer& data);
    int getAccelerometerPmu(Bmi270PowerState& pmu);
    int initMotion(Bmi270AccelMotionConfig& config, bool feedback = false);
    int initHighG(Bmi270AccelHighGConfig& config, bool feedback = false);
    int startMotionDetect();
    int stopMotionDetect();
    int startHighGDetect();
    int stopHighGDetect();

    int getStatus(uint32_t& val, bool clear = false);
    bool isMotionDetect(uint32_t val);
    bool isHighGDetect(uint32_t val);

    static Bmi270& getInstance();

private:
    enum class InterfaceType {
        BMI_I2C,
        BMI_SPI
    };

    Bmi270();
    ~Bmi270();

    int setSpiMode();
    int cleanup();
    void bmi270Handler();

    float convertValue(float val, float toRange, float fromRange);
    uint8_t convertRateToOdr(float rate);
    float convertOdrToRate(uint8_t odr);
    int setAccelRange(float& range, bool feedback = false);
    int setAccelRate(float& rate, bool feedback = false);
    int setAccelMotionThreshold(float& threshold, bool feedback = false);
    int setAccelMotionDuration(unsigned& duration, bool feedback = false);
    int setAccelMotionSkip(Bmi270AccelSignificantMotionSkip skip);
    int setAccelMotionProof(Bmi270AccelSignificantMotionProof proof);
    int setAccelHighGThreshold(float& threshold, bool feedback = false);
    int setAccelHighGDuration(float& duration, bool feedback = false);
    int setAccelHighGHysteresis(float& hysteresis, bool feedback = false);

    static int8_t bmi2SpiRead(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
    static int8_t bmi2SpiWrite(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
    static int8_t bmi2I2cRead(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
    static int8_t bmi2I2cWrite(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
    static void   bmi2DelayUs(uint32_t period, void *intf_ptr);

    InterfaceType type_;
    inline static TwoWire* wire_;
    uint8_t address_;
    inline static SPIClass* spi_;
    pin_t csPin_;
    const __SPISettings spiSettings_;
    pin_t intPin_;
    struct bmi2_dev bmi2_;
    bool initialized_;
    Bmi270PmuAccel accelPmu_;
    Bmi270PmuGyro gyroPmu_;
    int rangeAccel_;
    float rateAccel_;
    uint8_t latchShadow_;
    os_queue_t motionSyncQueue_;
    static RecursiveMutex mutex_;
}; // class Bmi270

#define BMI270 Bmi270::getInstance()

} // namespace particle