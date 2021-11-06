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

#include "bmi160.h"
#include "bmi160regs.h"
#include "endian_util.h"
#include <cmath>

using namespace spark;
using namespace particle;

namespace {

} // anonymous namespace


Bmi160::Bmi160()
        : type_(InterfaceType::BMI_I2C),
          address_(INVALID_I2C_ADDRESS),
          spiSettings_(4*MHZ, MSBFIRST, SPI_MODE0),
          initialized_(false),
          accelPmu_(PMU_STATUS_ACC_SUSPEND),
          gyroPmu_(PMU_STATUS_GYRO_SUSPEND),
          rangeAccel_(BMI160_ACCEL_RANGE_DEFAULT),
          rateAccel_(BMI160_ACCEL_RATE_DEFAULT),
          latchShadow_(0),
          motionSyncQueue_(nullptr) {

}

Bmi160::~Bmi160() {

}

Bmi160& Bmi160::getInstance() {
    static Bmi160 newSensor;
    return newSensor;
}

int Bmi160::setSpiMode() {
    uint8_t dummy = 0;
    CHECK(readRegister(SPI_MODE_ADDR, &dummy, 1));
    delay(BMI160_SPI_SELECT_TIME);
    return SYSTEM_ERROR_NONE;
}

int Bmi160::initialize() {
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    uint8_t reg = 0;

    // Disable INT2 and enable INT1.  Latch interrupt for 160 ms
    CHECK(readRegister(Bmi160Register::INT_LATCH_ADDR, &reg));
    reg &= INT_LATCH_INT2_INPUT_EN_MASK;
    reg |= INT_LATCH_INT1_INPUT_EN_MASK | (IRQ_LATCH_LATCHED << INT_LATCH_MODE_SHIFT);
    CHECK(writeRegister(Bmi160Register::INT_LATCH_ADDR, reg));
    latchShadow_ = reg;

    // Configure interrupt pin 1 for active-low, level, push-pull output
    CHECK(writeRegister(Bmi160Register::INT_OUT_CTRL_ADDR,
        INT_OUT_CTRL_INT1_OE_MASK |
        (IRQ_DRIVE_PUSH_PULL << INT_OUT_CTRL_INT1_OD_SHIFT) |
        (IRQ_LEVEL_ACTIVE_LOW << INT_OUT_CTRL_INT1_LVL_SHIFT) |
        (IRQ_EDGE_LEVEL << INT_OUT_CTRL_INT1_EDGE_SHIFT)
        ));

    // Map all interrupts for INT1
    CHECK(writeRegister(Bmi160Register::INT_MAP_0_ADDR,
        INT_MAP_0_INT1_FLAT_MASK |
        INT_MAP_0_INT1_ORIENT_MASK |
        INT_MAP_0_INT1_S_TAP_MASK |
        INT_MAP_0_INT1_D_TAP_MASK |
        INT_MAP_0_INT1_NO_MOT_MASK |
        INT_MAP_0_INT1_ANYS_MOT_MASK |
        INT_MAP_0_INT1_HIGH_G_MASK |
        INT_MAP_0_INT1_LOW_G_MASK
        ));
    CHECK(writeRegister(Bmi160Register::INT_MAP_1_ADDR,
        INT_MAP_1_INT1_DATA_MASK |
        INT_MAP_1_INT1_FIFO_W_MASK |
        INT_MAP_1_INT1_FIFO_F_MASK |
        INT_MAP_1_INT1_PMU_MASK
        ));
    CHECK(writeRegister(Bmi160Register::INT_MAP_2_ADDR, 0x00));

    return SYSTEM_ERROR_NONE;
}

int Bmi160::cleanup() {
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    // Disable interrupts, set defaults
    CHECK(reset());

    // Power down modules
    CHECK(writeRegister(Bmi160Register::CMD_ADDR, Bmi160Command::CMD_ACC_PMU_MODE_SUSPEND));
    delay(BMI160_ACC_PMU_CMD_TIME);

    return SYSTEM_ERROR_NONE;
}

int Bmi160::begin(const TwoWire* interface, uint8_t address, pin_t interruptPin, size_t eventDepth) {
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_FALSE(initialized_, SYSTEM_ERROR_NONE);
    CHECK_TRUE(interface, SYSTEM_ERROR_INVALID_ARGUMENT);

    if (os_queue_create(&motionSyncQueue_, sizeof(Bmi160EventType), eventDepth, nullptr)) {
        motionSyncQueue_ = nullptr;
        Log.error("os_queue_create() failed");
        return SYSTEM_ERROR_INTERNAL;
    }

    type_ = InterfaceType::BMI_I2C;
    wire_ = const_cast<TwoWire*>(interface);
    intPin_ = interruptPin;

    wire_->begin();
    wire_->beginTransmission(address);
    if (wire_->endTransmission() != 0) {
        Log.error("address invalid or device failed");
        return SYSTEM_ERROR_IO;
    }

    pinMode(intPin_, INPUT);
    CHECK_TRUE(attachInterrupt(intPin_, &Bmi160::bmi160Handler, this, FALLING), SYSTEM_ERROR_INTERNAL);

    address_ = address;
    initialized_ = true;

    CHECK(reset());
    return SYSTEM_ERROR_NONE;
}

int Bmi160::begin(const SPIClass& interface, pin_t selectPin, pin_t interruptPin, size_t eventDepth) {
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_FALSE(initialized_, SYSTEM_ERROR_NONE);

    if (os_queue_create(&motionSyncQueue_, sizeof(Bmi160EventType), eventDepth, nullptr)) {
        motionSyncQueue_ = nullptr;
        Log.error("os_queue_create() failed");
        return SYSTEM_ERROR_INTERNAL;
    }

    type_ = InterfaceType::BMI_SPI;
    spi_ = (SPIClass*)&interface;
    csPin_ = selectPin;
    intPin_ = interruptPin;

    pinMode(csPin_, OUTPUT);
    digitalWrite(csPin_, HIGH);

    pinMode(intPin_, INPUT);
    CHECK_TRUE(attachInterrupt(intPin_, &Bmi160::bmi160Handler, this, FALLING), SYSTEM_ERROR_INTERNAL);
    initialized_ = true;

    CHECK(reset());
    return SYSTEM_ERROR_NONE;
}

int Bmi160::end() {
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_NONE);

    cleanup();

    os_queue_destroy(motionSyncQueue_, nullptr);
    address_ = INVALID_I2C_ADDRESS;
    rangeAccel_ = BMI160_ACCEL_RANGE_DEFAULT;
    rateAccel_ = BMI160_ACCEL_RATE_DEFAULT;

    initialized_ = true;

    return SYSTEM_ERROR_NONE;
}

int Bmi160::reset() {
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);
    if (type_ == InterfaceType::BMI_SPI) {
        CHECK(setSpiMode());
    }
    CHECK(writeRegister(Bmi160Register::CMD_ADDR, Bmi160Command::CMD_SOFT_RESET));
    delay(BMI160_SOFT_RESET_CMD_TIME);
    accelPmu_ = PMU_STATUS_ACC_SUSPEND;
    gyroPmu_ = PMU_STATUS_GYRO_SUSPEND;

    if (type_ == InterfaceType::BMI_SPI) {
        CHECK(setSpiMode());
    }
    CHECK(initialize());
    return SYSTEM_ERROR_NONE;
}

int Bmi160::sleep() {
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);
    CHECK(writeRegister(Bmi160Register::CMD_ADDR, Bmi160Command::CMD_ACC_PMU_MODE_SUSPEND));
    delay(BMI160_ACC_PMU_CMD_TIME);
    accelPmu_ = PMU_STATUS_ACC_SUSPEND;
    return SYSTEM_ERROR_NONE;
}

int Bmi160::wakeup() {
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);
    CHECK(writeRegister(Bmi160Register::CMD_ADDR, Bmi160Command::CMD_ACC_PMU_MODE_LOW));
    delay(BMI160_ACC_PMU_CMD_TIME);
    accelPmu_ = PMU_STATUS_ACC_LOW;
    return SYSTEM_ERROR_NONE;
}

void Bmi160::bmi160Handler() {
    this->syncEvent(Bmi160EventType::SYNC);
}

int Bmi160::syncEvent(Bmi160EventType event) {
    if (motionSyncQueue_) {
        CHECK_FALSE(os_queue_put(motionSyncQueue_, &event, 0, nullptr), SYSTEM_ERROR_BUSY);
    }

    return SYSTEM_ERROR_NONE;
}

int Bmi160::waitOnEvent(Bmi160EventType& event, system_tick_t timeout) {
    Bmi160EventType eventReceive = Bmi160EventType::NONE;
    auto ret = os_queue_take(motionSyncQueue_, &eventReceive, timeout, nullptr);
    if (ret) {
        event = Bmi160EventType::NONE;
    }
    else {
        event = eventReceive;
    }

    return SYSTEM_ERROR_NONE;
}

int Bmi160::setAccelRange(float& range, bool feedback) {
    auto rangeEnum = Bmi160AccelRange::ACCEL_RANGE_16G;
    auto workRange = range;

    // Assumed lock already acquired
    if (workRange <= ACCEL_RANGE_2G_F) {
        workRange = ACCEL_RANGE_2G_F;
        rangeEnum = Bmi160AccelRange::ACCEL_RANGE_2G;
    }
    else if (workRange <= ACCEL_RANGE_4G_F) {
        workRange = ACCEL_RANGE_4G_F;
        rangeEnum = Bmi160AccelRange::ACCEL_RANGE_4G;
    }
    else if (workRange <= ACCEL_RANGE_8G_F) {
        workRange = ACCEL_RANGE_8G_F;
        rangeEnum = Bmi160AccelRange::ACCEL_RANGE_8G;
    }
    else { // less than, equal, or greater than 16
        workRange = ACCEL_RANGE_16G_F;
        rangeEnum = Bmi160AccelRange::ACCEL_RANGE_16G;
    }

    CHECK(writeRegister(Bmi160Register::ACC_RANGE_ADDR, rangeEnum));
    rangeAccel_ = (int)workRange;

    if (feedback) {
        range = workRange;
    }

    return SYSTEM_ERROR_NONE;
}

uint8_t Bmi160::convertRateToOdr(float rate) {
    // <rate> quaranteed not to be zero
    auto value = ceilf((float)ACCEL_RATE_ODR_BIT_MIRROR - log2f(ACCEL_RATE_ODR_PERCENT/rate));
    return static_cast<uint8_t>(value);
}

float Bmi160::convertOdrToRate(uint8_t odr) {
    return ACCEL_RATE_ODR_PERCENT / powf(2.0, (float)(ACCEL_RATE_ODR_BIT_MIRROR - (int)odr));
}

int Bmi160::setAccelRate(float& rate, bool feedback) {
    // Assumed lock already acquired
    auto workRate = rate;

    if (workRate <= 0.0) {
        workRate = __FLT_EPSILON__;
    }
    else if (workRate >= ACCEL_RATE_MAX) {
        workRate = ACCEL_RATE_MAX;
    }

    auto odr = convertRateToOdr(workRate);

    // Enable undersampling
    uint8_t reg = ACC_CONF_USAMPLE_MASK | odr | (ACCEL_CONF_BWP_NORMAL << ACC_CONF_BWP_SHIFT);

    CHECK(writeRegister(Bmi160Register::ACC_CONF_ADDR, reg));
    rateAccel_ = convertOdrToRate(odr);

    if (feedback) {
        rate = workRate;
    }

    return SYSTEM_ERROR_NONE;
}

int Bmi160::setAccelMotionThreshold(float& threshold, bool feedback) {
    // Assumed lock already acquired
    CHECK_FALSE(rangeAccel_ == 0, SYSTEM_ERROR_INVALID_STATE);

    auto workThreshold = threshold;
    float lsbThreshold = (float)rangeAccel_ / INTMO_1_ANYM_TH_LOWEST_RES;
    float minThreshold = (float)rangeAccel_ / INTMO_1_ANYM_TH_RES;
    uint8_t reg = 0;

    if (workThreshold <= minThreshold) {
        workThreshold = minThreshold;
    }
    else if (workThreshold >= (float)rangeAccel_) {
        workThreshold = lsbThreshold * (float)INTMO_1_ANYM_TH_MAX;
        reg = INTMO_1_ANYM_TH_MAX;
    }
    else {
        auto regThreshold = ceilf(workThreshold / lsbThreshold);
        reg = (uint8_t)regThreshold;
        workThreshold = lsbThreshold * reg;
    }

    // IN_MOTION[1] int_anym_th<7:0>
    CHECK(writeRegister(Bmi160Register::INT_MOTION_1_ADDR, reg));

    if (feedback) {
        threshold = workThreshold;
    }

    return SYSTEM_ERROR_NONE;
}


int Bmi160::setAccelMotionDuration(unsigned& duration, bool feedback) {
    // Assumed lock already acquired
    auto workDuration = duration;

    if (workDuration < INTMO_0_ANYM_DUR_MIN) {
        workDuration = INTMO_0_ANYM_DUR_MIN;
    }
    else if (workDuration > INTMO_0_ANYM_DUR_MAX) {
        workDuration = INTMO_0_ANYM_DUR_MAX;
    }

    // IN_MOTION[0] int_anym_dur<1:0>
    uint8_t reg = 0;
    CHECK(readRegister(Bmi160Register::INT_MOTION_0_ADDR, &reg));
    reg &= ~INTMO_0_ANYM_DUR_MASK;
    reg |= INTMO_0_ANYM_DUR_MASK & ((uint8_t)workDuration - 1);
    CHECK(writeRegister(Bmi160Register::INT_MOTION_0_ADDR, reg));

    if (feedback) {
        duration = workDuration;
    }

    return SYSTEM_ERROR_NONE;
}

int Bmi160::setAccelMotionSkip(Bmi160AccelSignificantMotionSkip skip) {
    // Assumed lock already acquired

    auto skipValue = static_cast<uint8_t>(skip);

    // IN_MOTION[3] int_sig_mot_skip<1:0>
    uint8_t reg = 0;
    CHECK(readRegister(Bmi160Register::INT_MOTION_3_ADDR, &reg));
    reg &= ~INTMO_3_SIG_MOT_SKIP_MASK;
    reg |= INTMO_3_SIG_MOT_SKIP_MASK & (skipValue << INTMO_3_SIG_MOT_SKIP_SHIFT);
    CHECK(writeRegister(Bmi160Register::INT_MOTION_3_ADDR, reg));

    return SYSTEM_ERROR_NONE;
}

int Bmi160::setAccelMotionProof(Bmi160AccelSignificantMotionProof proof) {
    // Assumed lock already acquired

    auto proofValue = static_cast<uint8_t>(proof);

    // IN_MOTION[3] int_sig_mot_proof<1:0>
    uint8_t reg = 0;
    CHECK(readRegister(Bmi160Register::INT_MOTION_3_ADDR, &reg));
    reg &= ~INTMO_3_SIG_MOT_PROOF_MASK;
    reg |= INTMO_3_SIG_MOT_PROOF_MASK & (proofValue << INTMO_3_SIG_MOT_PROOF_SHIFT);
    CHECK(writeRegister(Bmi160Register::INT_MOTION_3_ADDR, reg));

    return SYSTEM_ERROR_NONE;
}

int Bmi160::setAccelHighGThreshold(float& threshold, bool feedback) {
    // Assumed lock already acquired
    CHECK_FALSE(rangeAccel_ == 0, SYSTEM_ERROR_INVALID_STATE);

    auto workThreshold = threshold;
    float lsbThreshold = (float)rangeAccel_ / INTLH_4_HIGH_TH_LOWEST_RES;
    float minThreshold = (float)rangeAccel_ / INTLH_4_HIGH_TH_RES;
    uint8_t reg = 0;

    if (workThreshold <= minThreshold) {
        workThreshold = minThreshold;
    }
    else if (workThreshold >= (float)rangeAccel_) {
        workThreshold = lsbThreshold * (float)INTLH_4_HIGH_TH_MAX;
        reg = INTLH_4_HIGH_TH_MAX;
    }
    else {
        auto regThreshold = ceilf(workThreshold / lsbThreshold);
        reg = (uint8_t)regThreshold;
        workThreshold = lsbThreshold * reg;
    }

    // IN_MOTION[1] int_anym_th<7:0>
    CHECK(writeRegister(Bmi160Register::INT_LOWHIGH_4_ADDR, reg));

    if (feedback) {
        threshold = workThreshold;
    }

    return SYSTEM_ERROR_NONE;
}

int Bmi160::setAccelHighGDuration(float& duration, bool feedback) {
    // Assumed lock already acquired
    auto workDuration = duration;

    if (workDuration < INTLH_3_HIGH_DUR_MIN) {
        workDuration = INTLH_3_HIGH_DUR_MIN;
    }
    else if (workDuration > INTLH_3_HIGH_DUR_MAX) {
        workDuration = INTLH_3_HIGH_DUR_MAX;
    }

    // INT_LOWHIGH[3] int_high_dur<7:0>
    uint8_t reg = (uint8_t)(workDuration / INTLH_3_HIGH_DUR_MIN) - 1;
    CHECK(writeRegister(Bmi160Register::INT_LOWHIGH_3_ADDR, reg));

    if (feedback) {
        duration = workDuration;
    }

    return SYSTEM_ERROR_NONE;
}

int Bmi160::setAccelHighGHysteresis(float& hysteresis, bool feedback) {
    // Assumed lock already acquired
    CHECK_FALSE(rangeAccel_ == 0, SYSTEM_ERROR_INVALID_STATE);

    auto workHysteresis = hysteresis;
    float lsbHysteresis = (float)rangeAccel_ / INTLH_2_HIGH_HYST_RES;
    uint8_t reg = 0;

    auto regHysteresis = ceilf(workHysteresis / lsbHysteresis);
    reg = (uint8_t)regHysteresis;
    if (reg > INTLH_2_HIGH_HYST_MAX) {
        reg = INTLH_2_HIGH_HYST_MAX;
    }
    workHysteresis = lsbHysteresis * reg;

    // INT_LOWHIGH[2] int_high_hy<1:0>
    uint8_t val = 0;
    CHECK(readRegister(Bmi160Register::INT_LOWHIGH_2_ADDR, &val));
    val &= ~(INTLH_2_HIGH_HYST_MASK);
    val |= (reg << INTLH_2_HIGH_HYST_SHIFT) & INTLH_2_HIGH_HYST_MASK;
    CHECK(writeRegister(Bmi160Register::INT_LOWHIGH_2_ADDR, val));

    if (feedback) {
        hysteresis = workHysteresis;
    }

    return SYSTEM_ERROR_NONE;
}

int Bmi160::initAccelerometer(Bmi160AccelerometerConfig& config, bool feedback) {
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    // Setting maximum range
    CHECK(setAccelRange(config.range, feedback));

    // Setting for sample rate
    CHECK(setAccelRate(config.rate, feedback));

    return SYSTEM_ERROR_NONE;
}

int Bmi160::getAccelerometer(Bmi160Accelerometer& data) {
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    uint8_t buffer[6];
    CHECK(readRegister(Bmi160Register::ACCEL_DATA_START_ADDR, buffer, arraySize(buffer)));
    auto x = littleEndianToNative<int16_t>(*(reinterpret_cast<int16_t*>(&buffer[0])));
    data.x = convertValue((float)x, (float)rangeAccel_, ACCEL_FULL_RANGE);
    auto y = littleEndianToNative<int16_t>(*(reinterpret_cast<int16_t*>(&buffer[2])));
    data.y = convertValue((float)y, (float)rangeAccel_, ACCEL_FULL_RANGE);
    auto z = littleEndianToNative<int16_t>(*(reinterpret_cast<int16_t*>(&buffer[4])));
    data.z = convertValue((float)z, (float)rangeAccel_, ACCEL_FULL_RANGE);

    return SYSTEM_ERROR_NONE;
}

int Bmi160::getAccelerometerPmu(Bmi160PowerState& pmu) {
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);
    uint8_t val = 0;
    CHECK(readRegister(Bmi160Register::PMU_STATUS_ADDR, &val));
    val &= PMU_STATUS_ACC_MASK;
    val >>= PMU_STATUS_ACC_SHIFT;
    switch(val) {
        case Bmi160PmuAccel::PMU_STATUS_ACC_SUSPEND: {
            pmu = Bmi160PowerState::PMU_SUSPEND;
            break;
        }

        case Bmi160PmuAccel::PMU_STATUS_ACC_NORMAL: {
            pmu = Bmi160PowerState::PMU_NORMAL;
            break;
        }

        case Bmi160PmuAccel::PMU_STATUS_ACC_LOW: {
            pmu = Bmi160PowerState::PMU_LOW_POWER;
            break;
        }

        default: {
            break;
        }
    }
    return SYSTEM_ERROR_NONE;
}

int Bmi160::startMotionDetect() {
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    uint8_t reg = 0;

    // INT_EN_0_ADDR[0] int_anymo_xyz_en
    CHECK(readRegister(Bmi160Register::INT_EN_0_ADDR, &reg));
    reg |= INT_EN_0_ANYMO_Z_MASK | INT_EN_0_ANYMO_Y_MASK | INT_EN_0_ANYMO_X_MASK;
    CHECK(writeRegister(Bmi160Register::INT_EN_0_ADDR, reg));

    return SYSTEM_ERROR_NONE;
}

int Bmi160::stopMotionDetect() {
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    uint8_t reg = 0;

    // INT_EN_0_ADDR[0] int_anymo_xyz_en
    CHECK(readRegister(Bmi160Register::INT_EN_0_ADDR, &reg));
    reg &= ~(INT_EN_0_ANYMO_Z_MASK | INT_EN_0_ANYMO_Y_MASK | INT_EN_0_ANYMO_X_MASK);
    CHECK(writeRegister(Bmi160Register::INT_EN_0_ADDR, reg));

    return SYSTEM_ERROR_NONE;
}

int Bmi160::initMotion(Bmi160AccelMotionConfig& config, bool feedback) {
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    // Setting motion threshold
    CHECK(setAccelMotionThreshold(config.motionThreshold, feedback));

    // Setting motion duration
    CHECK(setAccelMotionDuration(config.motionDuration, feedback));

    // Setting motion skip for significant motion
    CHECK(setAccelMotionSkip(config.motionSkip));

    // Setting motion proof for significant motion
    CHECK(setAccelMotionProof(config.motionProof));

    // Any-motion detection does not rely on the skip nor proof parameters.
    uint8_t reg = 0;
    CHECK(readRegister(Bmi160Register::INT_MOTION_3_ADDR, &reg));
    reg &= ~INTMO_3_SIG_MOT_SEL_MASK;
    reg |= (config.mode == Bmi160AccelMotionMode::ACCEL_MOTION_MODE_SIGNIFICANT) ? INTMO_3_SIG_MOT_SEL_MASK : 0x0;
    CHECK(writeRegister(Bmi160Register::INT_MOTION_3_ADDR, reg));

    return SYSTEM_ERROR_NONE;
}

int Bmi160::initHighG(Bmi160AccelHighGConfig& config, bool feedback) {
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    // Setting high G threshold
    CHECK(setAccelHighGThreshold(config.threshold, feedback));

    // Setting high G duration
    CHECK(setAccelHighGDuration(config.duration, feedback));

    // Setting high G hysteresis
    CHECK(setAccelHighGHysteresis(config.hysteresis, feedback));

    return SYSTEM_ERROR_NONE;
}

int Bmi160::startHighGDetect() {
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    uint8_t reg = 0;

    // INT_EN_0_ADDR[1] int_high_xyz_en
    CHECK(readRegister(Bmi160Register::INT_EN_1_ADDR, &reg));
    reg |= INT_EN_1_HIGH_G_Z_MASK | INT_EN_1_HIGH_G_Y_MASK | INT_EN_1_HIGH_G_X_MASK;
    CHECK(writeRegister(Bmi160Register::INT_EN_1_ADDR, reg));

    return SYSTEM_ERROR_NONE;
}

int Bmi160::stopHighGDetect() {
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    uint8_t reg = 0;

    // INT_EN_0_ADDR[0] int_high_xyz_en
    CHECK(readRegister(Bmi160Register::INT_EN_1_ADDR, &reg));
    reg &= ~(INT_EN_1_HIGH_G_Z_MASK | INT_EN_1_HIGH_G_Y_MASK | INT_EN_1_HIGH_G_X_MASK);
    CHECK(writeRegister(Bmi160Register::INT_EN_1_ADDR, reg));

    return SYSTEM_ERROR_NONE;
}

int Bmi160::getStatus(uint32_t& val, bool clear) {
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);
    CHECK(readRegister(Bmi160Register::INT_STATUS_0_ADDR, reinterpret_cast<uint8_t*>(&val), sizeof(val)));

    // Perform clear of latched interrupts by changing latch interval (as opposed to disabling and re-enabling interrupts)
    if (clear && ((latchShadow_ & INT_LATCH_MODE_MASK) == IRQ_LATCH_LATCHED)) {
        CHECK(writeRegister(Bmi160Register::INT_LATCH_ADDR, IRQ_LATCH_312_5_US));
        delayMicroseconds(BMI160_INT_LATCH_CLEAR_TIME);
        CHECK(writeRegister(Bmi160Register::INT_LATCH_ADDR, latchShadow_));
    }

    return SYSTEM_ERROR_NONE;
}

bool Bmi160::isMotionDetect(uint32_t val) {
    return (val & (BMI_INTR_BIT_SIGNIFICANT_MOTION | BMI_INTR_BIT_ANY_MOTION)) ? true : false;
}

bool Bmi160::isHighGDetect(uint32_t val) {
    return (val & (BMI_INTR_BIT_HIGH_G)) ? true : false;
}

int Bmi160::getChipId(uint8_t& val) {
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);
    return readRegister(Bmi160Register::CHIPID_ADDR, &val);
}

float Bmi160::convertValue(float val, float toRange, float fromRange) {
    if (0.0 == fromRange) {
        return NAN;
    }
    return val * toRange / fromRange;
}

int Bmi160::writeRegister(uint8_t reg, uint8_t val) {
    if (type_ == InterfaceType::BMI_I2C) {
        uint8_t buf[2];
        buf[0] = reg;
        buf[1] = val;
        wire_->beginTransmission(address_);
        wire_->write(buf, sizeof(buf));
        auto ret = wire_->endTransmission();
        if ((accelPmu_ == PMU_STATUS_ACC_LOW) ||
            (accelPmu_ == PMU_STATUS_ACC_SUSPEND) ||
            (gyroPmu_ == PMU_STATUS_GYRO_SUSPEND) ||
            (gyroPmu_ == PMU_STATUS_GYRO_FAST_START_UP)) {
            delayMicroseconds(BMI160_I2C_IDLE_TIME);
        }
        return ret;
    }
    else if (type_ == InterfaceType::BMI_SPI) {
        spi_->beginTransaction(spiSettings_);
        digitalWrite(csPin_, LOW);
        spi_->transfer(reg & 0x7f);
        spi_->transfer(val);
        digitalWrite(csPin_, HIGH);
        spi_->endTransaction();
        if ((accelPmu_ == PMU_STATUS_ACC_LOW) ||
            (accelPmu_ == PMU_STATUS_ACC_SUSPEND) ||
            (gyroPmu_ == PMU_STATUS_GYRO_SUSPEND) ||
            (gyroPmu_ == PMU_STATUS_GYRO_FAST_START_UP)) {
            delayMicroseconds(BMI160_SPI_IDLE_TIME);
        }
        return SYSTEM_ERROR_NONE;
    }

    return SYSTEM_ERROR_INVALID_STATE;
}

int Bmi160::readRegister(uint8_t reg, uint8_t* val, int length) {
    auto regAddress = reg;

    if (type_ == InterfaceType::BMI_I2C) {
        while (0 < length) {
            wire_->beginTransmission(address_);
            wire_->write(&regAddress, 1);
            CHECK_TRUE(wire_->endTransmission(false) == 0, SYSTEM_ERROR_INTERNAL);

            auto remaining = std::min<int>(length, I2C_BUFFER_LENGTH);
            length -= remaining;
            regAddress += remaining; // It is possible to overflow, allow it
            auto readLength = (int)wire_->requestFrom((int)address_, remaining);
            if (readLength != remaining) {
                wire_->endTransmission();
                return SYSTEM_ERROR_INTERNAL;
            }

            while (wire_->available() && remaining--) {
                *val++ = wire_->read();
            }
        }

        return SYSTEM_ERROR_NONE;
    }
    else if (type_ == InterfaceType::BMI_SPI) {
        spi_->beginTransaction(spiSettings_);
        digitalWrite(csPin_, LOW);
        spi_->transfer(reg | 0x80);
        while (length > 0) {
            *(val++) = spi_->transfer(0xff);
            length--;
        }
        digitalWrite(csPin_, HIGH);
        spi_->endTransaction();

        return SYSTEM_ERROR_NONE;
    }

    return SYSTEM_ERROR_INVALID_STATE;
}

RecursiveMutex Bmi160::mutex_;
