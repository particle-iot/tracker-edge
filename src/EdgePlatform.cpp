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

//***************** INCLUDES *********************
#include "EdgePlatform.h"
#include "tracker_config.h"


//***************** DEFINES **********************


//***************** CONSTANTS ********************
constexpr uint8_t GNSS_NEO_M8U  = 0b111;
constexpr uint8_t GNSS_LC29HBA  = 0b110;
constexpr uint8_t GNSS_NEO_M9V  = 0b101;
constexpr uint8_t GNSS_MASK     = 0x07;
constexpr uint8_t GNSS_SHIFT    = 0;

constexpr uint8_t IMU_BMI160    = 0b11;
constexpr uint8_t IMU_BMI270    = 0b10;
constexpr uint8_t IMU_MASK      = 0x18;
constexpr uint8_t IMU_SHIFT     = 3;

constexpr uint8_t GPIO_MCP23S17 = 0b11;
constexpr uint8_t GPIO_MASK     = 0x60;
constexpr uint8_t GPIO_SHIFT    = 5;

constexpr uint8_t CAN_MCP25625  = 0b111;
constexpr uint8_t CAN_MASK      = 0x80;

constexpr uint8_t FG_MAX17043   = 0b1;
constexpr uint8_t FG_SHIFT      = 6;

constexpr uint8_t WIFI_ESP32_D2WD  = 0b11;
constexpr uint8_t WIFI_ESP32_U4WDH = 0b10;
constexpr uint8_t WIFI_MASK        = 0x60;
constexpr uint8_t WIFI_SHIFT       = 4;

constexpr uint8_t THERMISTOR_SOFT  = 0b1;
constexpr uint8_t THERMISTOR_PMIC  = 0b0;
constexpr uint8_t THERMISTOR_MASK  = 0x08;
constexpr uint8_t THERMISTOR_SHIFT = 3;

constexpr uint8_t CURRENT_1_5_A    = 0b1;
constexpr uint8_t CURRENT_3_A      = 0b0;
constexpr uint8_t CURRENT_MASK     = 0x04;
constexpr uint8_t CURRENT_SHIFT    = 2;

constexpr uint32_t INVALID_DATA = 0xFFFFFFFF;


//***************** GLOBALS **********************
EdgePlatform *EdgePlatform::_instance = nullptr;

Logger EdgePlatformLogger("otp");                                                                                      


//***************** CLASS METHODS **********************
bool EdgePlatform::init()
{
    // Populate the lookup tables for this class
    modelTable_.insert({TRACKER_MODEL_BARE_SOM_DEFAULT, TrackerModel::eBARE_SOM_DEFAULT});
    modelTable_.insert({TRACKER_MODEL_BARE_SOM,         TrackerModel::eBARE_SOM});
    modelTable_.insert({TRACKER_MODEL_EVAL,             TrackerModel::eEVAL});
    modelTable_.insert({TRACKER_MODEL_TRACKERONE,       TrackerModel::eTRACKER_ONE});
    modelTable_.insert({TRACKER_MODEL_MONITORONE,       TrackerModel::eMONITOR_ONE});
    modelTable_.insert({TRACKER_MODEL_TRACKERM,         TrackerModel::eTRACKER_M});

    gnssTable_.insert({GNSS_NEO_M8U, GnssVariant::eNEO_M8U});
    gnssTable_.insert({GNSS_LC29HBA, GnssVariant::eLC29HBA});
    gnssTable_.insert({GNSS_NEO_M9V, GnssVariant::eNEO_M9V});

    imuTable_.insert({IMU_BMI160, ImuVariant::eBMI160});
    imuTable_.insert({IMU_BMI270, ImuVariant::eBMI270});

    wifiTable_.insert({WIFI_ESP32_D2WD, WiFiVariant::eESP32_D2WD});
    wifiTable_.insert({WIFI_ESP32_U4WDH,WiFiVariant::eESP32_U4WDH});

    currentTable_.insert({CURRENT_1_5_A, Ilim::eILIM_1_5});
    currentTable_.insert({CURRENT_3_A,   Ilim::eILIM_3});

    thermistorTable_.insert({THERMISTOR_SOFT, ThermistorType::eSOFTWARE});
    thermistorTable_.insert({THERMISTOR_PMIC, ThermistorType::ePMIC}); 

    // Flag the initialization as complete
    isInitialized_ = true;

    return true;
}

bool EdgePlatform::readHwInfo()
{
    CHECK_TRUE(isInitialized_, false);
 
    hal_device_hw_info info;
    uint32_t           res;
    uint8_t            byte2 {0};
    uint8_t            byte3 {0};

    // Read the OTP region using the DeviceOS API
    hal_get_device_hw_info(&info, &res);
    EdgePlatformLogger.info("BYTE 3: 0x%lx", info.features >> 8);
    EdgePlatformLogger.info("BYTE 2: 0x%lx", info.features & 0xFF);

    byte2 = info.features & 0xFF;
    byte3 = info.features >> 8;

#if (PLATFORM_ID == PLATFORM_TRACKER)   
    // Parse OTP area to determine module type
    model_ = modelTable_[info.model];

    // Parse out info.features:BYTE2 for GNSS, IMU and GPIO Expander
    // BYTE2:
    // +----+----+----+----+----+----+----+----+
    // |  7 |  6 |  5 |  4 |  3 |  2 |  1 |  0 |
    // +----+----+----+----+----+----+----+----+
    // | CAN|  GPIO   |   IMU   |   GNSS       |
    // +----+----+----+----+----+----+----+----+

    // Only one type of CAN interface has been defined
    canIface_ = EdgePlatform::CanXcvr::eMCP25625;

    if( GPIO_MCP23S17 == ((byte2 & GPIO_MASK) >> GPIO_SHIFT) )
    {
        gpioExpander_ = EdgePlatform::GpioExpander::eMCP23S17;
    }

    imu_  = imuTable_[(byte2 & IMU_MASK)>>IMU_SHIFT];
    
    gnss_ = gnssTable_[(byte2 & GNSS_MASK)];

    // Parse out info.features:BYTE3 for WIFI, Thermistor, Fuel Gauge, Current Limit
    // BYTE3:
    // +----+----+----+----+----+----+----+----+
    // |  7 |  6 |  5 |  4 |  3 |  2 |  1 |  0 |
    // +----+----+----+----+----+----+----+----+
    // | xxx| FG | WIFI    | TR |ILIM|   CAN   |
    // +----+----+----+----+----+----+----+----+

    // Only one type of fuel gauge has been defined
    fg_ = EdgePlatform::FuelGaugeType::eMAX17043;

    wifi_ = wifiTable_[(byte3 & WIFI_MASK) >> WIFI_SHIFT];

    tr_ = thermistorTable_[(byte3 & THERMISTOR_MASK) >> THERMISTOR_SHIFT];

    currentLimit_ = currentTable_[(byte3 & CURRENT_MASK) >> CURRENT_SHIFT];

    // Sensirion sensor type is currently not a field in OTP.
    // However, there may be a field in the future.
    sensirion_ = EdgePlatform::SensirionType::eSTS31;
#elif (PLATFORM_ID == PLATFORM_TRACKERM)
    // Tracker-M is fixed with these peripherals
    (void)byte2;
    (void)byte3;
    model_ = EdgePlatform::TrackerModel::eTRACKER_M;
    gnss_ = EdgePlatform::GnssVariant::eLC29HBA;
    imu_  = EdgePlatform::ImuVariant::eBMI270;
    fg_ = EdgePlatform::FuelGaugeType::eMAX17043;
    sensirion_ = EdgePlatform::SensirionType::eSTS31;
#endif

    return true;
}

EdgePlatform::TrackerModel EdgePlatform::getModel() const
{
    CHECK_TRUE(isInitialized_, TrackerModel::eMODEL_INVALID);

    return model_;
}

EdgePlatform::GnssVariant EdgePlatform::getGnss() const
{
    CHECK_TRUE(isInitialized_, GnssVariant::eGNSS_INVALID);

    return gnss_;
}

EdgePlatform::ImuVariant EdgePlatform::getImu() const
{
    CHECK_TRUE(isInitialized_, ImuVariant::eIMU_INVALID);

    return imu_;
}

EdgePlatform::GpioExpander EdgePlatform::getGpioExpander() const
{
    CHECK_TRUE(isInitialized_, GpioExpander::eEXPANDER_INVALID);

    return gpioExpander_;
}

EdgePlatform::CanXcvr EdgePlatform::getCanInterface() const
{
    CHECK_TRUE(isInitialized_, CanXcvr::eCAN_INVALID);

    return canIface_;
}

EdgePlatform::Ilim EdgePlatform::getCurrentLimit() const
{
    CHECK_TRUE(isInitialized_, Ilim::eILIM_INVALID);

    return currentLimit_;
}

EdgePlatform::ThermistorType EdgePlatform::getThermistor() const
{
    CHECK_TRUE(isInitialized_, ThermistorType::eTR_INVALID);

    return tr_;
}

EdgePlatform::WiFiVariant EdgePlatform::getWifiType() const
{
    CHECK_TRUE(isInitialized_, WiFiVariant::eWIFI_INVALID);

    return wifi_;
}

EdgePlatform::FuelGaugeType EdgePlatform::getFuelGaugeType() const
{
    CHECK_TRUE(isInitialized_, FuelGaugeType::eFG_INVALID);

    return fg_;
}

EdgePlatform::SensirionType EdgePlatform::getSensirionType() const
{
    CHECK_TRUE(isInitialized_, SensirionType::eSENSE_INVALID);

    return sensirion_;
}
