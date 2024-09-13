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
constexpr uint8_t GNSS_MASK     = 0x07;
constexpr uint8_t GNSS_SHIFT    = 0;
constexpr uint8_t GNSS_NEO_M8U  = 0b111 << GNSS_SHIFT;
constexpr uint8_t GNSS_LC29HBA  = 0b110 << GNSS_SHIFT;
constexpr uint8_t GNSS_NEO_M9V  = 0b101 << GNSS_SHIFT;

constexpr uint8_t IMU_MASK      = 0x18;
constexpr uint8_t IMU_SHIFT     = 3;
constexpr uint8_t IMU_BMI160    = 0b11 << IMU_SHIFT;
constexpr uint8_t IMU_BMI270    = 0b10 << IMU_SHIFT;

constexpr uint8_t GPIO_MASK     = 0x60;
constexpr uint8_t GPIO_SHIFT    = 5;
constexpr uint8_t GPIO_MCP23S17 = 0b11 << GPIO_SHIFT;

constexpr uint8_t CAN_MASK      = 0x80;
constexpr uint8_t CAN_MCP25625  = 0b111;

constexpr uint8_t FG_MAX17043   = 0b1;
constexpr uint8_t FG_SHIFT      = 6;

constexpr uint8_t WIFI_MASK        = 0x60;
constexpr uint8_t WIFI_SHIFT       = 4;
constexpr uint8_t WIFI_ESP32_D2WD  = 0b11 << WIFI_SHIFT;
constexpr uint8_t WIFI_ESP32_U4WDH = 0b10 << WIFI_SHIFT;

constexpr uint8_t THERMISTOR_MASK  = 0x08;
constexpr uint8_t THERMISTOR_SHIFT = 3;
constexpr uint8_t THERMISTOR_SOFT  = 0b1 << THERMISTOR_SHIFT;
constexpr uint8_t THERMISTOR_PMIC  = 0b0 << THERMISTOR_SHIFT;

constexpr uint8_t CURRENT_MASK     = 0x04;
constexpr uint8_t CURRENT_SHIFT    = 2;
constexpr uint8_t CURRENT_1_5_A    = 0b1 << CURRENT_SHIFT;
constexpr uint8_t CURRENT_3_A      = 0b0 << CURRENT_SHIFT;

//***************** GLOBALS **********************
EdgePlatform *EdgePlatform::_instance = nullptr;

Logger EdgePlatformLogger("otp");

void EdgePlatform::init(uint32_t model, uint32_t variant)
{
#ifdef TRACKER_HAS_HW_INFO
    hal_device_hw_info info;
    uint32_t           res;

    // Read the OTP region using the DeviceOS API
    hal_get_device_hw_info(&info, &res);
    EdgePlatformLogger.info("BYTE 3: 0x%lx", info.features >> 8);
    EdgePlatformLogger.info("BYTE 2: 0x%lx", info.features & 0xFF);

    uint8_t byte2 = info.features & 0xFF;
    uint8_t byte3 = info.features >> 8;

#else // TRACKER_HAS_HW_INFO
    uint8_t byte2 = 0xFF;
    uint8_t byte3 = 0xFF;
#endif // TRACKER_HAS_HW_INFO

    // Parse OTP area to determine module type
    switch (model) {
    case TRACKER_MODEL_BARE_SOM:
        model_ = TrackerModel::eBARE_SOM;
        break;
    case TRACKER_MODEL_EVAL:
        model_ = TrackerModel::eEVAL;
        break;
    case TRACKER_MODEL_TRACKERONE:
        model_ = TrackerModel::eTRACKER_ONE;
        break;
    case TRACKER_MODEL_BARE_SOM_DEFAULT:
    default:
        model_ = TrackerModel::eBARE_SOM_DEFAULT;
    }

    // Parse out info.features:BYTE2 for GNSS, IMU and GPIO Expander
    // BYTE2:
    // +----+----+----+----+----+----+----+----+
    // |  7 |  6 |  5 |  4 |  3 |  2 |  1 |  0 |
    // +----+----+----+----+----+----+----+----+
    // | CAN|  GPIO   |   IMU   |   GNSS       |
    // +----+----+----+----+----+----+----+----+

    // Only one type of CAN interface has been defined
    canIface_ = EdgePlatform::CanXcvr::eMCP25625;

    if ((byte2 & GPIO_MASK) == GPIO_MCP23S17) {
        gpioExpander_ = EdgePlatform::GpioExpander::eMCP23S17;
    }

    switch (byte2 & IMU_MASK) {
    case IMU_BMI160:
        imu_ = ImuVariant::eBMI160;
        break;
    }

    switch (byte2 & GNSS_MASK) {
    case GNSS_NEO_M8U:
        gnss_ = GnssVariant::eNEO_M8U;
        break;
    }

    // Parse out info.features:BYTE3 for WIFI, Thermistor, Fuel Gauge, Current Limit
    // BYTE3:
    // +----+----+----+----+----+----+----+----+
    // |  7 |  6 |  5 |  4 |  3 |  2 |  1 |  0 |
    // +----+----+----+----+----+----+----+----+
    // | xxx| FG | WIFI    | TR |ILIM|   CAN   |
    // +----+----+----+----+----+----+----+----+

    // Only one type of fuel gauge has been defined
    fg_ = EdgePlatform::FuelGaugeType::eMAX17043;

    switch (byte3 & WIFI_MASK) {
    case WIFI_ESP32_D2WD:
        wifi_ = WiFiVariant::eESP32_D2WD;
        break;
    case WIFI_ESP32_U4WDH:
        wifi_ = WiFiVariant::eESP32_U4WDH;
        break;
    }

    if ((byte3 & THERMISTOR_MASK) == THERMISTOR_SOFT) {
        tr_ = ThermistorType::eSOFTWARE;
    } else {
        tr_ = ThermistorType::ePMIC;
    }

    if ((byte3 & CURRENT_MASK) == CURRENT_1_5_A) {
        currentLimit_ = Ilim::eILIM_1_5;
    } else {
        currentLimit_ = Ilim::eILIM_3;
    }

    // Flag the initialization as complete
    isInitialized_ = true;
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
