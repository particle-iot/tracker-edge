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


//***************** INCLUDES *********************
#include "Particle.h"
#include "tracker_imu.h"
#include "EdgePlatform.h"
#include "bmi160.h"


//***************** DEFINES **********************


//***************** CONSTANTS ********************



//***************** GLOBALS **********************
TrackerImu *TrackerImu::_instance = nullptr;


//************ CLASS: TrackerImu ************
TrackerImu::TrackerImu()
    : isInitialized_(false),
      imu_(BmiVariant::IMU_INVALID)
{

}

BmiVariant TrackerImu::getImuType(void)
{
    auto imu = EdgePlatform::instance().getImu();

    // Set the IMU type
    switch(imu)
    {
        case EdgePlatform::ImuVariant::eBMI160:
            imu_ = BmiVariant::IMU_BMI160;
            break;
        default:
            imu_ = BmiVariant::IMU_INVALID;
            break;
    }

    isInitialized_ = true;

    return imu_;
}

int TrackerImu::begin(const TwoWire* interface, uint8_t address, pin_t interruptPin, size_t eventDepth)
{
    CHECK_TRUE(isInitialized_, SYSTEM_ERROR_INVALID_STATE);

    switch(imu_)
    {
        case BmiVariant::IMU_BMI160:
        {
            return BMI160.begin(interface, address, interruptPin, eventDepth);
            break;
        }
    }

    return SYSTEM_ERROR_INVALID_STATE;
}


int TrackerImu::begin(const SPIClass& interface, pin_t selectPin, pin_t interruptPin, size_t eventDepth)
{
    CHECK_TRUE(isInitialized_, SYSTEM_ERROR_INVALID_STATE);

    switch(imu_)
    {
        case BmiVariant::IMU_BMI160:
        {
            return BMI160.begin(interface, selectPin, interruptPin, eventDepth);
            break;
        }
    }

    return SYSTEM_ERROR_INVALID_STATE;
}


int TrackerImu::end()
{
    CHECK_TRUE(isInitialized_, SYSTEM_ERROR_INVALID_STATE);

    switch(imu_)
    {
        case BmiVariant::IMU_BMI160:
        {
            return BMI160.end();
            break;
        }
    }

    return SYSTEM_ERROR_INVALID_STATE;
}

int TrackerImu::reset()
{
    CHECK_TRUE(isInitialized_, SYSTEM_ERROR_INVALID_STATE);

    switch(imu_)
    {
        case BmiVariant::IMU_BMI160:
        {
            return BMI160.reset();
            break;
        }
    }

    return SYSTEM_ERROR_INVALID_STATE;
}

int TrackerImu::sleep()
{
    CHECK_TRUE(isInitialized_, SYSTEM_ERROR_INVALID_STATE);

    switch(imu_)
    {
        case BmiVariant::IMU_BMI160:
        {
            return BMI160.sleep();
            break;
        }
    }

    return SYSTEM_ERROR_INVALID_STATE;
}

int TrackerImu::wakeup()
{
    CHECK_TRUE(isInitialized_, SYSTEM_ERROR_INVALID_STATE);

    switch(imu_)
    {
        case BmiVariant::IMU_BMI160:
        {
            return BMI160.wakeup();
            break;
        }
    }

    return SYSTEM_ERROR_INVALID_STATE;
}

int TrackerImu::syncEvent(BmiEventType event)
{
    CHECK_TRUE(isInitialized_, SYSTEM_ERROR_BUSY);

    switch(imu_)
    {
        case BmiVariant::IMU_BMI160:
        {
            return BMI160.syncEvent(static_cast<Bmi160::Bmi160EventType>(event));
            break;
        }
    }

    return SYSTEM_ERROR_BUSY;
}

int TrackerImu::waitOnEvent(BmiEventType& event, system_tick_t timeout)
{
    CHECK_TRUE(isInitialized_, SYSTEM_ERROR_NONE);

    switch(imu_)
    {
        case BmiVariant::IMU_BMI160:
        {
            Bmi160::Bmi160EventType evt160;
            auto status = BMI160.waitOnEvent(evt160, timeout);
            event = static_cast<BmiEventType>(evt160);
            return status;
            break;
        }
    }

    return SYSTEM_ERROR_NONE;
}

int TrackerImu::initAccelerometer(BmiAccelerometerConfig& config, bool feedback)
{
    CHECK_TRUE(isInitialized_, SYSTEM_ERROR_INVALID_STATE);

    switch(imu_)
    {
        case BmiVariant::IMU_BMI160:
        {
            Bmi160AccelerometerConfig cfg160{0};
            cfg160.rate = config.rate;
            cfg160.range = config.range;
            return BMI160.initAccelerometer(cfg160, feedback);
            break;
        }
    }

    return SYSTEM_ERROR_INVALID_STATE;
}

int TrackerImu::getAccelerometer(BmiAccelerometer& data)
{
    CHECK_TRUE(isInitialized_, SYSTEM_ERROR_INVALID_STATE);

    switch(imu_)
    {
        case BmiVariant::IMU_BMI160:
        {
            Bmi160Accelerometer data160{0};
            auto retval = BMI160.getAccelerometer(data160);
            data.x = data160.x;
            data.y = data160.y;
            data.z = data160.z;
            return retval;
            break;
        }
    }

    return SYSTEM_ERROR_INVALID_STATE;
}

int TrackerImu::getAccelerometerPmu(BmiPowerState& pmu)
{
    CHECK_TRUE(isInitialized_, SYSTEM_ERROR_INVALID_STATE);

    switch(imu_)
    {
        case BmiVariant::IMU_BMI160:
        {
            Bmi160::Bmi160PowerState data160{0};
            auto retval = BMI160.getAccelerometerPmu(data160);
            pmu = (BmiPowerState)data160;
            return retval;
            break;
        }
    }

    return SYSTEM_ERROR_INVALID_STATE;
}

int TrackerImu::initMotion(BmiAccelMotionConfig& config, bool feedback)
{
    CHECK_TRUE(isInitialized_, SYSTEM_ERROR_INVALID_STATE);

    switch(imu_)
    {
        case BmiVariant::IMU_BMI160:
        {
            Bmi160AccelMotionConfig cfg160{};
            cfg160.mode            = (Bmi160AccelMotionMode)config.mode;
            cfg160.motionDuration  = config.motionDuration;
            cfg160.motionProof     = (Bmi160AccelSignificantMotionProof)config.motionProof;
            cfg160.motionSkip      = (Bmi160AccelSignificantMotionSkip)config.motionSkip;
            cfg160.motionThreshold = config.motionThreshold;
            return BMI160.initMotion(cfg160, feedback);
            break;
        }
    }

    return SYSTEM_ERROR_INVALID_STATE;
}

int TrackerImu::initHighG(BmiAccelHighGConfig& config, bool feedback)
{
    CHECK_TRUE(isInitialized_, SYSTEM_ERROR_INVALID_STATE);

    switch(imu_)
    {
        case BmiVariant::IMU_BMI160:
        {
            Bmi160AccelHighGConfig cfg160{};
            cfg160.duration   = config.duration;
            cfg160.hysteresis = config.hysteresis;
            cfg160.threshold  = config.threshold;
            return BMI160.initHighG(cfg160, feedback);
            break;
        }
    }

    return SYSTEM_ERROR_INVALID_STATE;
}

int TrackerImu::startMotionDetect()
{
    CHECK_TRUE(isInitialized_, SYSTEM_ERROR_INVALID_STATE);

    switch(imu_)
    {
        case BmiVariant::IMU_BMI160:
        {
            return BMI160.startMotionDetect();
            break;
        }
    }

    return SYSTEM_ERROR_INVALID_STATE;
}

int TrackerImu::stopMotionDetect()
{
    CHECK_TRUE(isInitialized_, SYSTEM_ERROR_INVALID_STATE);

    switch(imu_)
    {
        case BmiVariant::IMU_BMI160:
        {
            return BMI160.stopMotionDetect();
            break;
        }
    }

    return SYSTEM_ERROR_INVALID_STATE;
}

int TrackerImu::startHighGDetect()
{
    CHECK_TRUE(isInitialized_, SYSTEM_ERROR_INVALID_STATE);

    switch(imu_)
    {
        case BmiVariant::IMU_BMI160:
        {
            return BMI160.startHighGDetect();
            break;
        }
    }

    return SYSTEM_ERROR_INVALID_STATE;
}

int TrackerImu::stopHighGDetect()
{
    CHECK_TRUE(isInitialized_, SYSTEM_ERROR_INVALID_STATE);

    switch(imu_)
    {
        case BmiVariant::IMU_BMI160:
        {
            return BMI160.stopHighGDetect();
            break;
        }
    }

    return SYSTEM_ERROR_INVALID_STATE;
}

int  TrackerImu::getStatus(uint32_t& val, bool clear)
{
    CHECK_TRUE(isInitialized_, SYSTEM_ERROR_INVALID_STATE);

    switch(imu_)
    {
        case BmiVariant::IMU_BMI160:
        {
            return BMI160.getStatus(val, clear);
            break;
        }
    }

    return SYSTEM_ERROR_INVALID_STATE;
}

bool TrackerImu::isMotionDetect(uint32_t val)
{
    CHECK_TRUE(isInitialized_, false);

    switch(imu_)
    {
        case BmiVariant::IMU_BMI160:
        {
            return BMI160.isMotionDetect(val);
            break;
        }
    }

    return false;
}

bool TrackerImu::isHighGDetect(uint32_t val)
{
    CHECK_TRUE(isInitialized_, false);

    switch(imu_)
    {
        case BmiVariant::IMU_BMI160:
        {
            return BMI160.isHighGDetect(val);
            break;
        }
    }

    return false;
}
