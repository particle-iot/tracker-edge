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

#include <cmath>
#include "Particle.h"

namespace particle {

/**
 * @brief Resistor divider circuit used for thermistor.
 *
 */
enum class ThermistorCircuit {
  NONE,                                 /**< No configuration */
  LOW_SIDE_DIVIDER,                     /**< Thermistor is on the low side of the divider.  V = Vcc * Rt / (Rt + Rfixed) */
  HIGH_SIDE_DIVIDER,                    /**< Thermistor is on the high side of the divider.  V = Vcc * Rfixed / (Rt + Rfixed) */
};

/**
 * @brief Type of thermistor.
 *
 */
enum class ThermistorType {
  NONE,                                 /**< No configuration */
  POSITIVE_COEFF,                       /**< Positive temperature coefficient thermistor */
  NEGATIVE_COEFF,                       /**< Negative temperature coefficient thermistor */
};

/**
 * @brief Structure to configure thermistor reading.
 *
 */
struct ThermistorConfig {
  ThermistorCircuit circuit;            /**< Circuit details for resistor divider. */
  ThermistorType type;                  /**< Type of thermistor. */
  float beta;                           /**< Beta parameter for thermistor. */
  float t0;                             /**< Temperature for reference resistance. */
  float r0;                             /**< Reference resitance at reference temperature. */
  float fixedR;                         /**< Fixed resistor used in resistor divider with thermistor. */
  float adcResolution;                  /**< Maximum ADC value. */
  float minTemperature;                 /**< Minumum temperature that can be measured with sensor. */
  float maxTemperature;                 /**< Maximum temperature that can be measured with sensor. */
};

/**
 * @brief Thermistor class to read from various resistor topologies and types.
 *
 */
class Thermistor {
public:
  const float ThermistorError = -300.0; /**< Error value */

  /**
   * @brief Construct a new Thermistor object
   *
   */
  Thermistor() :
    inputPin_(PIN_INVALID),
    config_((ThermistorConfig){
      .circuit        = ThermistorCircuit::NONE,
      .type           = ThermistorType::NONE,
      .beta           = 1.0,
      .t0             = 1.0,
      .r0             = 1.0,
      .fixedR         = 1.0,
      .adcResolution  = 4096.0,
      .minTemperature = 1.0,
      .maxTemperature = 1.0}),
    ratioNormalize_(1.0f) {

  }

  /**
   * @brief Destroy the Thermistor object
   *
   */
  virtual ~Thermistor() {

  }

  /**
   * @brief Initialize thermistor settings for particular circuit and thermistor type.
   *
   * @param inputPin Analog input pin to read voltage from voltage divider.
   * @param config Configuration settings.
   * @retval SYSTEM_ERROR_NONE
   * @retval SYSTEM_ERROR_NOT_SUPPORTED
   * @retval SYSTEM_ERROR_INVALID_ARGUMENT
   * @retval SYSTEM_ERROR_ALREADY_EXISTS
   * @retval SYSTEM_ERROR_IO
   */
  int begin(pin_t inputPin, ThermistorConfig& config) {
    // Only negative coefficient thermistors are supported
    CHECK_TRUE((config.type == ThermistorType::NEGATIVE_COEFF), SYSTEM_ERROR_NOT_SUPPORTED);

    // Check the pin type
    pin_t checkPin = inputPin;
    CHECK_TRUE((checkPin >= FIRST_ANALOG_PIN), SYSTEM_ERROR_INVALID_ARGUMENT);
    CHECK_TRUE((checkPin < FIRST_ANALOG_PIN + TOTAL_ANALOG_PINS), SYSTEM_ERROR_INVALID_ARGUMENT);
    checkPin += FIRST_ANALOG_PIN;
    CHECK_TRUE(pinAvailable(checkPin), SYSTEM_ERROR_ALREADY_EXISTS);
    CHECK_TRUE((HAL_Validate_Pin_Function(checkPin, PF_ADC) == PF_ADC), SYSTEM_ERROR_IO);

    // Various configuration checks
    CHECK_TRUE((config.adcResolution > 1.0), SYSTEM_ERROR_INVALID_ARGUMENT);
    CHECK_TRUE((config.r0 > 0.0), SYSTEM_ERROR_INVALID_ARGUMENT);
    CHECK_TRUE((config.fixedR > 0.0), SYSTEM_ERROR_INVALID_ARGUMENT);
    CHECK_TRUE((config.t0 > -kelvinCelcius_), SYSTEM_ERROR_INVALID_ARGUMENT);

    inputPin_ = inputPin;
    config_ = config;

    // Compute constants so that they do not need to be calculated on every read
    ratioNormalize_ = config_.fixedR / config_.r0 * expf(config_.beta / (config_.t0 + kelvinCelcius_));

    return SYSTEM_ERROR_NONE;
  }

  /**
   * @brief Get the current temperature.
   *
   * @return float Temperature in degrees Celcius.
   */
  float getTemperature() {
    int32_t val = analogRead(inputPin_);
    float vRatio = val / config_.adcResolution;
    float rRatio = 1.0;

    switch (config_.circuit) {
      case ThermistorCircuit::LOW_SIDE_DIVIDER: {
        rRatio = vRatio / (1.0 - vRatio);
        break;
      }

      case ThermistorCircuit::HIGH_SIDE_DIVIDER: {
        rRatio = (1.0 - vRatio) / vRatio;
        break;
      }

      default: {
        return SYSTEM_ERROR_INVALID_STATE;
      }

    }

    // T = B / ln(rratio)
    rRatio *= ratioNormalize_;
    float temperature = config_.beta / logf(rRatio) - kelvinCelcius_;
    if ((temperature < config_.minTemperature) || (temperature > config_.maxTemperature)) {
      temperature = ThermistorError;
    }

    return temperature;
  }

private:
  const float kelvinCelcius_ = 273.15f; // 0 degree Kelvin = -273.15 degrees C

  pin_t inputPin_;
  ThermistorConfig config_;
  float ratioNormalize_;
}; // class Thermistor

} // namespace particle
