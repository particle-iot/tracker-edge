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

/**
 * @brief Class used to read and parse OTP fields.
 */
class EdgePlatform
{
public:
    //***************** ENUMERATIONS **********************
    enum class TrackerModel
    {
        eBARE_SOM_DEFAULT,
        eBARE_SOM,
        eEVAL,
        eTRACKER_ONE,
        eMODEL_INVALID
    };

    enum class GnssVariant
    {
        eNEO_M8U,
        eGNSS_INVALID
    };

    enum class ImuVariant
    {
        eBMI160,
        eIMU_INVALID
    };

    enum class GpioExpander
    {
        eMCP23S17,
        eEXPANDER_INVALID
    };

    enum class CanXcvr
    {
        eMCP25625,
        eCAN_INVALID
    };

    enum class Ilim
    {
        eILIM_1_5,
        eILIM_3,
        eILIM_INVALID
    };

    enum class ThermistorType
    {
        eSOFTWARE,
        ePMIC,
        eTR_INVALID
    };

    enum class WiFiVariant
    {
        eESP32_D2WD,
        eESP32_U4WDH,
        eWIFI_INVALID
    };

    enum class FuelGaugeType
    {
        eMAX17043,
        eFG_INVALID
    };

    /**
     * @brief Return instance of the LocationService
     *
     * @return LocationService&
     */
    static EdgePlatform &instance()
    {
        if(!_instance)
        {
            _instance = new EdgePlatform();
        }
        return *_instance;
    }

    /**
     * @brief Read OTP region for hardware information
     *
     * @param model Model info from non-volatile memory
     * @param variant Variant info from non-volatile memory
     */
    void init(uint32_t model, uint32_t variant);

    /**
     * @brief Return Tracker model type
     *
     * @return TrackerModel
     */
    TrackerModel getModel() const;

    /**
     * @brief Return GNSS model type
     *
     * @return GnssVariant
     */
    GnssVariant getGnss() const;

    /**
     * @brief Return IMU model type
     *
     * @return ImuVariant
     */
    ImuVariant getImu() const;

    /**
     * @brief Return GPIO expander model type
     *
     * @return GpioExpander
     */
    GpioExpander getGpioExpander() const;

    /**
     * @brief Return CAN transceiver type
     *
     * @return CanXcvr
     */
    CanXcvr getCanInterface() const;

    /**
     * @brief Return current limit
     *
     * @return Ilim
     */
    Ilim getCurrentLimit() const;

    /**
     * @brief Return thermistor type
     *
     * @return ThermistorType
     */
    ThermistorType getThermistor() const;

    /**
     * @brief Return wifi type
     *
     * @return WiFiVariant
     */
    WiFiVariant getWifiType() const;

    /**
     * @brief Return fuel gauge type
     *
     * @return FuelGaugeType
     */
    FuelGaugeType getFuelGaugeType() const;

private:
    /**
     * Create a new Tracker Platform object
     * @brief Default constructor.
     */
    EdgePlatform() { };
    static EdgePlatform *_instance;
    bool                isInitialized_ {false};

    TrackerModel   model_         {TrackerModel::eMODEL_INVALID};
    GnssVariant    gnss_          {GnssVariant::eGNSS_INVALID};
    ImuVariant     imu_           {ImuVariant::eIMU_INVALID};
    GpioExpander   gpioExpander_  {GpioExpander::eEXPANDER_INVALID};
    CanXcvr        canIface_      {CanXcvr::eCAN_INVALID};
    Ilim           currentLimit_  {Ilim::eILIM_INVALID};
    ThermistorType tr_            {ThermistorType::eTR_INVALID};
    WiFiVariant    wifi_          {WiFiVariant::eWIFI_INVALID};
    FuelGaugeType  fg_            {FuelGaugeType::eFG_INVALID};
};
