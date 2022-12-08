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

#include "imu_bmi270.h"
#include "imu_bmi270regs.h"
#include "endian_util.h"
#include <cmath>
#include <map>

#define _LOG_CHECKED_ERROR(_expr, _ret)

#define CHECK(_expr) \
        ({ \
            const auto _ret = _expr; \
            if (_ret < 0) { \
                _LOG_CHECKED_ERROR(_expr, _ret); \
                return _ret; \
            } \
            _ret; \
        })

/**
 * Check the result of a predicate expression.
 *
 * @see @ref check_macros
 */
#define CHECK_TRUE(_expr, _ret) \
        do { \
            const bool _ok = (bool)(_expr); \
            if (!_ok) { \
                _LOG_CHECKED_ERROR(_expr, _ret); \
                return _ret; \
            } \
        } while (false)

/**
 * Check the result of a predicate expression.
 *
 * @see @ref check_macros
 */
#define CHECK_FALSE(_expr, _ret) \
        CHECK_TRUE(!(_expr), _ret)

using namespace spark;
using namespace particle;

namespace {

} // anonymous namespace

//**************** CONSTANTS ****************
constexpr float MILLI_G_PER_G           = 1000.0f;
constexpr float MILLI_G_PER_COUNT       = 0.48f;
constexpr float DURATION_COUNTS_PER_MS  = (1000.0f / 5.0f);
constexpr float HYSTERESIS_COUNTS_PER_G = (2.0f * 1000.0f / 0.49f);
constexpr float THRESHOLD_COUNTS_PER_G  = (10000.0f/4.9f);

constexpr uint32_t INVALID_VALUE        = 0xFFFF;


//**************** GLOBALS ******************
constexpr uint8_t MAX_NUM_SENSOR_TYPES = 2U;
std::map<float,uint32_t> rateTable = {{0.78f, BMI2_ACC_ODR_0_78HZ},
                                      {1.56f, BMI2_ACC_ODR_1_56HZ},
                                      {3.12f, BMI2_ACC_ODR_3_12HZ},
                                      {6.25f, BMI2_ACC_ODR_6_25HZ},
                                      {12.5f, BMI2_ACC_ODR_12_5HZ},
                                      {25.0f, BMI2_ACC_ODR_25HZ},
                                      {60.0f, BMI2_ACC_ODR_50HZ},
                                      {100.0f,BMI2_ACC_ODR_100HZ},
                                      {200.0f,BMI2_ACC_ODR_200HZ},
                                      {400.0f,BMI2_ACC_ODR_400HZ},
                                      {800.0f,BMI2_ACC_ODR_800HZ}, 
                                      {1600.0f,BMI2_ACC_ODR_1600HZ}};

std::map<float,uint32_t> rangeTable = {{2.0f, BMI2_ACC_RANGE_2G},
                                       {4.0f, BMI2_ACC_RANGE_4G},                         
                                       {8.0f, BMI2_ACC_RANGE_8G},                         
                                       {16.0f,BMI2_ACC_RANGE_16G}};

Bmi270::Bmi270()
        : type_(InterfaceType::BMI_I2C),
          address_(INVALID_I2C_ADDRESS),
          spiSettings_(4*MHZ, MSBFIRST, SPI_MODE0),
          initialized_(false),
          accelPmu_(Bmi270PmuAccel::PMU_STATUS_ACC_SUSPEND),
          gyroPmu_(Bmi270PmuGyro::PMU_STATUS_GYRO_SUSPEND),
          rangeAccel_(BMI270_ACCEL_RANGE_DEFAULT),
          rateAccel_(BMI270_ACCEL_RATE_DEFAULT),
          latchShadow_(0),
          motionSyncQueue_(nullptr) {

}

Bmi270::~Bmi270() {

}

Bmi270& Bmi270::getInstance() 
{
    static Bmi270 newSensor;
    return newSensor;
}

int Bmi270::setSpiMode() 
{
    delay(BMI270_SPI_SELECT_TIME);
    return SYSTEM_ERROR_NONE;
}

int Bmi270::initialize() 
{
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);
    return SYSTEM_ERROR_NONE;
}

int Bmi270::cleanup() 
{
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);
    return SYSTEM_ERROR_NONE;
}

int Bmi270::begin(const TwoWire* interface, uint8_t address, pin_t interruptPin, size_t eventDepth) 
{
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_FALSE(initialized_, SYSTEM_ERROR_NONE);
    CHECK_TRUE(interface, SYSTEM_ERROR_INVALID_ARGUMENT);

    if (os_queue_create(&motionSyncQueue_, sizeof(Bmi270EventType), eventDepth, nullptr)) 
    {
        motionSyncQueue_ = nullptr;
        Log.error("os_queue_create() failed");
        return SYSTEM_ERROR_INTERNAL;
    }

    type_ = InterfaceType::BMI_I2C;
    wire_ = const_cast<TwoWire*>(interface);
    intPin_ = interruptPin;

    wire_->begin();
    
    // Configure IMU interface
    address_ = BMI2_I2C_PRIM_ADDR;
    
    // Initialize the user I2C function 
    bmi2_.read = Bmi270::bmi2I2cRead;
    bmi2_.write = Bmi270::bmi2I2cWrite;
    bmi2_.intf = BMI2_I2C_INTF;
    bmi2_.intf_ptr = &address_;
    bmi2_.delay_us = Bmi270::bmi2DelayUs;
    bmi2_.read_write_len = 30; // Limitation of the Wire library
    bmi2_.config_file_ptr = NULL; // Use the default BMI270 config file

    // Write the configuration file
    if( BMI2_OK != bmi270_legacy_init(&bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }

    // Setup the interrupts
    pinMode(intPin_, INPUT);
    CHECK_TRUE(attachInterrupt(intPin_, &Bmi270::bmi270Handler, this, CHANGE), SYSTEM_ERROR_INTERNAL);

    address_ = address;
    initialized_ = true;

    return SYSTEM_ERROR_NONE;
}

int Bmi270::begin(const SPIClass& interface, pin_t selectPin, pin_t interruptPin, size_t eventDepth) 
{
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_FALSE(initialized_, SYSTEM_ERROR_NONE);

    if (os_queue_create(&motionSyncQueue_, sizeof(Bmi270EventType), eventDepth, nullptr)) 
    {
        motionSyncQueue_ = nullptr;
        Log.error("os_queue_create() failed");
        return SYSTEM_ERROR_INTERNAL;
    }

    type_ = InterfaceType::BMI_SPI;
    spi_ = (SPIClass*)&interface;
    csPin_ = selectPin;
    intPin_ = interruptPin;

    spi_->begin();

    pinMode(csPin_, OUTPUT);
    digitalWrite(csPin_, HIGH);

    pinMode(intPin_, INPUT);
    CHECK_TRUE(attachInterrupt(intPin_, &Bmi270::bmi270Handler, this, FALLING), SYSTEM_ERROR_INTERNAL);
    initialized_ = true;


    /* To initialize the user SPI function */
    bmi2_.intf = BMI2_SPI_INTF;
    bmi2_.read = Bmi270::bmi2SpiRead;
    bmi2_.write = Bmi270::bmi2SpiWrite;
    bmi2_.intf_ptr = this;
    bmi2_.delay_us = Bmi270::bmi2DelayUs;
    bmi2_.read_write_len = 30; // Limitation of the Wire library
    bmi2_.config_file_ptr = NULL; // Use the default BMI270 config file

    // Write the configuration file
    if( BMI2_OK != bmi270_legacy_init(&bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }

    return SYSTEM_ERROR_NONE;
}

int Bmi270::end() 
{
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_NONE);

    cleanup();

    os_queue_destroy(motionSyncQueue_, nullptr);
    address_ = INVALID_I2C_ADDRESS;

    initialized_ = true;

    return SYSTEM_ERROR_NONE;
}

int Bmi270::reset() 
{
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);
    if (type_ == InterfaceType::BMI_SPI) {
        CHECK(setSpiMode());
    }
    
    // Perform soft rest using Bosch API
    if( BMI2_OK != bmi2_soft_reset(&bmi2_) ) {
        return SYSTEM_ERROR_INTERNAL;
    }

    return SYSTEM_ERROR_NONE;
}

int Bmi270::sleep() 
{
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    // From the BMI270 Datasheet, Suspend mode is:
       // PWR_CTRL.acc_en = 0
       // PWR_CTRL.gyr_en = 0
       // POWER_CONF.adv_power_save = 1
    
    // Variable to store register values 
    uint8_t regData = 0;

    // Perform Read->Modify->Write for the above list of registers
    if( BMI2_OK != bmi2_get_regs(BMI2_PWR_CTRL_ADDR, &regData, sizeof(regData), &bmi2_) ) {
        return SYSTEM_ERROR_INTERNAL;
    }
    regData &= ~(BMI2_ACC_EN_MASK | BMI2_GYR_EN_MASK);
    if( BMI2_OK != bmi2_set_regs(BMI2_PWR_CTRL_ADDR, &regData, sizeof(regData), &bmi2_) ) {
        return SYSTEM_ERROR_INTERNAL;
    }

    if( BMI2_OK != bmi2_get_regs(BMI2_PWR_CONF_ADDR, &regData, sizeof(regData), &bmi2_) ) {
        return SYSTEM_ERROR_INTERNAL;
    }
    if( !(regData & BMI2_ADV_POW_EN_MASK) ) {
        // Set the POWER_CONF register only if the relevant bit is not set
        regData |= BMI2_ADV_POW_EN_MASK;
        if( BMI2_OK != bmi2_set_regs(BMI2_PWR_CONF_ADDR, &regData, sizeof(regData), &bmi2_) ) {
            return SYSTEM_ERROR_INTERNAL;
        }
    }

    return SYSTEM_ERROR_NONE;
}

int Bmi270::wakeup() 
{
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    // From the BMI270 Datasheet, Low Power mode is:
       // PWR_CTRL.acc_en = 1
       // PWR_CTRL.gyr_en = 1
       // ACC_CONF.acc_filter_perf = 0
       // GYR_CONF.gyr_filter_perf = 0
       // GYR_CONF.gyr_noise_perf = 0
       // POWER_CONF.adv_power_save = 1

    // Variable to store register values 
    uint8_t regData = 0;

    // Perform Read->Modify->Write for the above list of registers
    if( BMI2_OK != bmi2_get_regs(BMI2_PWR_CTRL_ADDR, &regData, sizeof(regData), &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }
    regData |= BMI2_ACC_EN_MASK | BMI2_GYR_EN_MASK;
    if( BMI2_OK != bmi2_set_regs(BMI2_PWR_CTRL_ADDR, &regData, sizeof(regData), &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }
    
    if( BMI2_OK != bmi2_get_regs(BMI2_ACC_CONF_ADDR, &regData, sizeof(regData), &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }
    regData &= ~BMI2_ACC_FILTER_PERF_MODE_MASK;
    if( BMI2_OK != bmi2_set_regs(BMI2_ACC_CONF_ADDR, &regData, sizeof(regData), &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }

    if( BMI2_OK != bmi2_get_regs(BMI2_GYR_CONF_ADDR, &regData, sizeof(regData), &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }
    regData &= ~(BMI2_GYR_FILTER_PERF_MODE_MASK | BMI2_GYR_NOISE_PERF_MODE_MASK);
    if( BMI2_OK != bmi2_set_regs(BMI2_GYR_CONF_ADDR, &regData, sizeof(regData), &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }

    if( BMI2_OK != bmi2_get_regs(BMI2_PWR_CONF_ADDR, &regData, sizeof(regData), &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }
    if( !(regData & BMI2_ADV_POW_EN_MASK) ) 
    {
        // Set the POWER_CONF register only if the relevant bit is not set
        regData |= BMI2_ADV_POW_EN_MASK;
        if( BMI2_OK != bmi2_set_regs(BMI2_PWR_CONF_ADDR, &regData, sizeof(regData), &bmi2_) ) 
        {
            return SYSTEM_ERROR_INTERNAL;
        }
    }

    accelPmu_ = Bmi270PmuAccel::PMU_STATUS_ACC_LOW;
    
    return SYSTEM_ERROR_NONE;
}

void Bmi270::bmi270Handler() 
{
    this->syncEvent(Bmi270EventType::SYNC);
}

int Bmi270::syncEvent(Bmi270EventType event) 
{
    if (motionSyncQueue_) {
        CHECK_FALSE(os_queue_put(motionSyncQueue_, &event, 0, nullptr), SYSTEM_ERROR_BUSY);
    }

    return SYSTEM_ERROR_NONE;
}

int Bmi270::waitOnEvent(Bmi270EventType& event, system_tick_t timeout) 
{
    Bmi270EventType eventReceive = Bmi270EventType::NONE;
    auto ret = os_queue_take(motionSyncQueue_, &eventReceive, timeout, nullptr);
    if (ret) {
        event = Bmi270EventType::NONE;
    }
    else {
        event = eventReceive;
    }

    return SYSTEM_ERROR_NONE;
}

int Bmi270::setAccelRange(float& range, bool feedback) 
{
    auto rangeEnum = Bmi270AccelRange::ACCEL_RANGE_16G;
    auto workRange = range;
    struct bmi2_sens_config sensCfg;
    uint8_t numSensors = 1;

    // Get the current configuration using the Bosch API
    sensCfg.type = BMI2_ACCEL;
    if( BMI2_OK != bmi2_get_sensor_config(&sensCfg, numSensors, &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }

    // Assign the new accelerometer range
    if (workRange <= ACCEL_RANGE_2G_F) 
    {
        workRange = ACCEL_RANGE_2G_F;
        rangeEnum = Bmi270AccelRange::ACCEL_RANGE_2G;
    }
    else if (workRange <= ACCEL_RANGE_4G_F) 
    {
        workRange = ACCEL_RANGE_4G_F;
        rangeEnum = Bmi270AccelRange::ACCEL_RANGE_4G;
    }
    else if (workRange <= ACCEL_RANGE_8G_F) 
    {
        workRange = ACCEL_RANGE_8G_F;
        rangeEnum = Bmi270AccelRange::ACCEL_RANGE_8G;
    }
    else 
    { // less than, equal, or greater than 16
        workRange = ACCEL_RANGE_16G_F;
        rangeEnum = Bmi270AccelRange::ACCEL_RANGE_16G;
    }
    sensCfg.cfg.acc.range = rangeEnum;

    // Update the accelerometer range using the Bosch API
    if( BMI2_OK != bmi2_set_sensor_config(&sensCfg, numSensors, &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }

    rangeAccel_ = (int)workRange;

    if (feedback) 
    {
        range = workRange;
    }

    return SYSTEM_ERROR_NONE;
}

uint8_t Bmi270::convertRateToOdr(float rate) 
{
    // <rate> guaranteed not to be zero
    auto value = ceilf((float)ACCEL_RATE_ODR_BIT_MIRROR - log2f(ACCEL_RATE_ODR_PERCENT/rate));
    return static_cast<uint8_t>(value);
}

float Bmi270::convertOdrToRate(uint8_t odr) 
{
    return ACCEL_RATE_ODR_PERCENT / powf(2.0, (float)(ACCEL_RATE_ODR_BIT_MIRROR - (int)odr));
}

int Bmi270::setAccelRate(float& rate, bool feedback) 
{
    // Assumed lock already acquired
    return SYSTEM_ERROR_NONE;
}

int Bmi270::setAccelMotionThreshold(float& threshold, bool feedback) 
{
    // Assumed lock already acquired
    CHECK_FALSE(rangeAccel_ == 0, SYSTEM_ERROR_INVALID_STATE);
    return SYSTEM_ERROR_NONE;
}


int Bmi270::setAccelMotionDuration(unsigned& duration, bool feedback) 
{
    // Assumed lock already acquired
    return SYSTEM_ERROR_NONE;
}

int Bmi270::setAccelMotionSkip(Bmi270AccelSignificantMotionSkip skip) 
{
    // Assumed lock already acquired
    return SYSTEM_ERROR_NONE;
}

int Bmi270::setAccelMotionProof(Bmi270AccelSignificantMotionProof proof) 
{
    // Assumed lock already acquired
    return SYSTEM_ERROR_NONE;
}

int Bmi270::setAccelHighGThreshold(float& threshold, bool feedback) 
{
    // Assumed lock already acquired
    CHECK_FALSE(rangeAccel_ == 0, SYSTEM_ERROR_INVALID_STATE);
    return SYSTEM_ERROR_NONE;
}

int Bmi270::setAccelHighGDuration(float& duration, bool feedback) 
{
    // Assumed lock already acquired
    return SYSTEM_ERROR_NONE;
}

int Bmi270::setAccelHighGHysteresis(float& hysteresis, bool feedback) 
{
    // Assumed lock already acquired
    CHECK_FALSE(rangeAccel_ == 0, SYSTEM_ERROR_INVALID_STATE);   
    return SYSTEM_ERROR_NONE;
}

int Bmi270::initAccelerometer(Bmi270AccelerometerConfig& config, bool feedback) 
{
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    // Structure to define accelerometer configuration. 
    struct bmi2_sens_config conf;

    // Configure the type of feature. 
    conf.type = BMI2_ACCEL;

    // Get default configurations for the type of feature selected. 
    if( BMI2_OK != bmi270_legacy_get_sensor_config(&conf, 1, &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }
  
    // NOTE: The user can change the following configuration parameters according to their requirement. 
    // Output Data Rate 
    conf.cfg.acc.odr = rateTable[config.rate]; //BMI2_ACC_ODR_100HZ;

    // Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). 
    conf.cfg.acc.range = rangeTable[config.range]; //BMI2_ACC_RANGE_2G;

    // The bandwidth parameter is used to configure the number of sensor samples that are averaged
        // if it is set to 2, then 2^(bandwidth parameter) samples
        // are averaged, resulting in 4 averaged samples.
        // Note1 : For more information, refer the datasheet.
        // Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
        // this has an adverse effect on the power consumed.
        //
    conf.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

    // Enable the filter performance mode where averaging of samples
        // will be done based on above set bandwidth and ODR.
        // There are two modes
        //  0 -> Ultra low power mode
        //  1 -> High performance mode(Default)
        // For more info refer datasheet.
        //
    conf.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    // Set the accel configurations. 
    if( BMI2_OK != bmi270_legacy_set_sensor_config(&conf, 1, &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }

    // Assign accel sensor to variable
    uint8_t sensorList = BMI2_ACCEL;
    if( BMI2_OK != bmi270_legacy_sensor_enable(&sensorList, 1, &bmi2_) )
    {
        return SYSTEM_ERROR_INTERNAL;
    }

    // Configure the interrupt pin
    struct bmi2_int_pin_config data_int_cfg;
    data_int_cfg.pin_type = BMI2_INT1;
    data_int_cfg.int_latch = BMI2_INT_NON_LATCH;
    data_int_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE; // Output enabled
    data_int_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;      // OpenDrain disabled
    data_int_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_LOW;      // Signal Low Active
    data_int_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;  // Input Disabled

    if( BMI2_OK != bmi2_set_int_pin_config( &data_int_cfg, &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }

    return SYSTEM_ERROR_NONE;
}

int Bmi270::getAccelerometer(Bmi270Accelerometer& data) 
{
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    // Read the sensor's data via Bosch API
    struct bmi2_sens_data sensorData = { {0} };
    if( BMI2_OK != bmi2_get_sensor_data(&sensorData, &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }

    // Scale and return the appropriate values
    data.x = convertValue((float)sensorData.acc.x, (float)rangeAccel_, ACCEL_FULL_RANGE);
    data.y = convertValue((float)sensorData.acc.y, (float)rangeAccel_, ACCEL_FULL_RANGE);
    data.z = convertValue((float)sensorData.acc.z, (float)rangeAccel_, ACCEL_FULL_RANGE);

    return SYSTEM_ERROR_NONE;
}

int Bmi270::getGyrometer(Bmi270Gyrometer& data) 
{
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    // Read the sensor's data via Bosch API
    struct bmi2_sens_data sensorData = { {0} };
    if( BMI2_OK != bmi2_get_sensor_data(&sensorData, &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }

    // Set the appropriate return values
    data.x = sensorData.gyr.x;
    data.y = sensorData.gyr.y;
    data.z = sensorData.gyr.z;

    return SYSTEM_ERROR_NONE;
}

int Bmi270::getAccelerometerPmu(Bmi270PowerState& pmu) 
{
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);
    
    return SYSTEM_ERROR_NONE;
}

int Bmi270::startMotionDetect() 
{
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    // Map any_motion feature interrupt to interrupt pin.
    struct bmi2_sens_int_config any_motion_int = { .type = BMI2_ANY_MOTION, .hw_int_pin = BMI2_INT1 };
    if( BMI2_OK != bmi270_legacy_map_feat_int(&any_motion_int, 1, &bmi2_) )
    {
        return SYSTEM_ERROR_INTERNAL;
    }    

    return SYSTEM_ERROR_NONE;
}

int Bmi270::stopMotionDetect() 
{
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    // Accel sensor and any-motion feature are listed in array.
    uint8_t sensList[1] = { BMI2_ANY_MOTION };

    if( BMI2_OK != bmi270_legacy_sensor_disable(sensList, sizeof(sensList), &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }
    
    return SYSTEM_ERROR_NONE;
}

int Bmi270::initMotion(Bmi270AccelMotionConfig& config, bool feedback) 
{
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);
    
    uint8_t mode {BMI2_ANY_MOTION};

    // Validate accelerometer mode
    if( Bmi270AccelMotionMode::ACCEL_MOTION_MODE_ANY == config.mode )
    {
        mode = BMI2_ANY_MOTION;
    }
    else if( Bmi270AccelMotionMode::ACCEL_MOTION_MODE_SIGNIFICANT == config.mode )
    {
        mode = BMI2_SIG_MOTION;
    }

    // Accel sensor and any-motion feature are listed in array.
    uint8_t sensList[2] = { BMI2_ACCEL, mode };
    if( BMI2_OK != bmi270_legacy_sensor_enable(sensList, sizeof(sensList), &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    } 

    // Configure the properties of the motion feature. 
    struct bmi2_sens_config sensConfig;
    sensConfig.type = mode;

    // NOTE: The user can change the following configuration parameters according to their requirement. 
    // 1LSB equals 20ms. Default is 100ms, setting to 80ms. 
    sensConfig.cfg.any_motion.duration = config.motionDuration * 4;

    // 1LSB equals to 0.48mg. Default is 83mg, setting to 50mg. 
    sensConfig.cfg.any_motion.threshold = (MILLI_G_PER_G * config.motionDuration) / MILLI_G_PER_COUNT;

    // Set new configurations. 
    if( BMI2_OK != bmi270_legacy_set_sensor_config(&sensConfig, 1, &bmi2_) )
    {
        return SYSTEM_ERROR_INTERNAL;
    }

    return SYSTEM_ERROR_NONE;
}

int Bmi270::initHighG(Bmi270AccelHighGConfig& config, bool feedback) 
{
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    // Accel sensor and high-g feature are listed in array. 
    uint8_t sensList[1] = { BMI2_HIGH_G };
    
    if( BMI2_OK != bmi270_legacy_sensor_enable(sensList, sizeof(sensList), &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }

    // Structure to define the type of sensor and its configurations. 
    struct bmi2_sens_config sensConfig;

    // Configure the type of High G feature.
    sensConfig.type = BMI2_HIGH_G;

    // Set parameters from input configuration
    sensConfig.cfg.high_g.duration = (uint16_t)(config.duration * DURATION_COUNTS_PER_MS);
    sensConfig.cfg.high_g.hysteresis = (uint16_t)(config.hysteresis * HYSTERESIS_COUNTS_PER_G);
    sensConfig.cfg.high_g.threshold = (uint16_t)(config.threshold * THRESHOLD_COUNTS_PER_G);
    sensConfig.cfg.high_g.select_x = 1;
    sensConfig.cfg.high_g.select_y = 1;
    sensConfig.cfg.high_g.select_z = 1;

    // Set new configurations. 
    if( BMI2_OK != bmi270_legacy_set_sensor_config(&sensConfig, 1, &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }

    return SYSTEM_ERROR_NONE;
}

int Bmi270::startHighGDetect() 
{
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    // Select features and their pins to be mapped to. 
    struct bmi2_sens_int_config sensInt = { .type = BMI2_HIGH_G, .hw_int_pin = BMI2_INT1 };

    // Map the feature to the interrupt pin
    if( BMI2_OK != bmi270_legacy_map_feat_int(&sensInt, 1, &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }

    return SYSTEM_ERROR_NONE;
}

int Bmi270::stopHighGDetect() 
{
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    // Accel sensor and high-g feature are listed in array. 
    uint8_t sensList[1] = { BMI2_HIGH_G };
    
    if( BMI2_OK != bmi270_legacy_sensor_disable(sensList, sizeof(sensList), &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }
    
    return SYSTEM_ERROR_NONE;
}

int Bmi270::getStatus(uint32_t& val, bool clear) 
{
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    // Read interrupt status
    if( BMI2_OK != bmi2_get_int_status(reinterpret_cast<uint16_t*>(&val), &bmi2_) ) 
    {
        return SYSTEM_ERROR_INTERNAL;
    }   

    return SYSTEM_ERROR_NONE;
}

bool Bmi270::isMotionDetect(uint32_t val) 
{
    // Validate the status value before masking
    if( INVALID_VALUE == val )
    {
        return false;
    }

    return ((uint16_t)val & BMI270_LEGACY_ANY_MOT_STATUS_MASK) ? true : false;
}

bool Bmi270::isHighGDetect(uint32_t val) 
{
    // Validate the status value before masking
    if( INVALID_VALUE == val )
    {
        return false;
    }

    return ((uint16_t)val & BMI270_LEGACY_HIGH_G_STATUS_MASK) ? true : false;
}

int Bmi270::getChipId(uint8_t& val) 
{
    const std::lock_guard<RecursiveMutex> lock(mutex_);
    CHECK_TRUE(initialized_, SYSTEM_ERROR_INVALID_STATE);

    val = bmi2_.chip_id;

    return SYSTEM_ERROR_NONE;
}

float Bmi270::convertValue(float val, float toRange, float fromRange) 
{
    if (0.0 == fromRange) 
    {
        return NAN;
    }
    return val * toRange / fromRange;
}

int8_t Bmi270::bmi2SpiRead(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    if ((data == NULL) || (len == 0))
    {
        return -1;
    }

    Bmi270 *periph = (Bmi270 *)intf_ptr;
    uint8_t dev_id = periph->csPin_;

    digitalWrite(dev_id, LOW);
    spi_->beginTransaction(periph->spiSettings_);
    spi_->transfer(reg_addr);
    for (uint16_t i = 0; i < len; i++)
    {
        data[i] = spi_->transfer(0xff);
    }
    spi_->endTransaction();
    digitalWrite(dev_id, HIGH);
    return 0;
}

int8_t Bmi270::bmi2SpiWrite(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    if ((data == NULL) || (len == 0))
    {
        return -1;
    }

    Bmi270 *periph = (Bmi270 *)intf_ptr;
    uint8_t dev_id = periph->csPin_;

    digitalWrite(dev_id, LOW);
    spi_->beginTransaction(periph->spiSettings_);
    spi_->transfer(reg_addr);
    for (uint16_t i = 0; i < len; i++) 
    {
        spi_->transfer(data[i]);
    }
    spi_->endTransaction();
    digitalWrite(dev_id, HIGH);
    return 0;
}

int8_t Bmi270::bmi2I2cRead(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    if ((data == NULL) || (len == 0) || (len > 32) || (intf_ptr == NULL) )    
    {
        return -1;
    }

    uint8_t bytes_received;
    uint8_t dev_id = *(uint8_t*)intf_ptr;

    wire_->beginTransmission(dev_id);
    wire_->write(reg_addr);
    if (wire_->endTransmission() == 0) 
    {
        bytes_received = wire_->requestFrom(dev_id, len);
        // Optionally, throw an error if bytes_received != len
        for (uint16_t i = 0; i < bytes_received; i++)
        {
            data[i] = wire_->read();
        }
    } 
    else 
    {
        return -1;
    }

    return 0;
}

int8_t Bmi270::bmi2I2cWrite(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    if ((data == NULL) || (len == 0) || (len > 32) || (intf_ptr == NULL) ) 
    {
        return -1;
    }

    uint8_t dev_id = *(uint8_t*)intf_ptr;

    wire_->beginTransmission(dev_id);
    wire_->write(reg_addr);
    for (uint16_t i = 0; i < len; i++)
    {
        wire_->write(data[i]);
    }
    if (wire_->endTransmission() != 0) 
    {
        return -1;
    }

    return 0;
}

void Bmi270::bmi2DelayUs(uint32_t period, void *intf_ptr)
{
    delayMicroseconds(period);
}

RecursiveMutex Bmi270::mutex_;