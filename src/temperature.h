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
#include "tracker_config.h"
#include "config_service.h"

enum class TemperatureChargeEvent {
  NORMAL,                 //< Normal temperature condition
  OVER_TEMPERATURE,       //< Over temperature condition
  UNDER_TEMPERATURE,      //< Under temperature condition
  OVER_CHARGE_REDUCTION,  //< Over temperature for reduced charge condition
};

using TemperatureCallback = std::function<int(TemperatureChargeEvent)>;


// Default temperature high threshold.
constexpr double TemperatureHighDefault = 25.0; // degrees celsius

// Default temperature low threshold.
constexpr double TemperatureLowDefault = 25.0; // degrees celsius

// Default temperature hysteresis
constexpr double TemperatureHysteresisDefault = 5.0; // degrees celsius

// High limit to disable battery charging (inclusive)
constexpr double ChargeTempHighLimit = 54.0; // degrees celsius

// Low limit to disable battery charging (inclusive)
constexpr double ChargeTempLowLimit = 0.0; // degrees celsius

// High limit to reduce battery charging current (inclusive)
constexpr double ChargeTempReducedALimit = 40.0; // degrees celsius

// Hysteresis applied to high/low limits to re-enable battery charging
constexpr double ChargeTempHyst = 2.0; // degrees celsius

// Rate, in seconds, to sample the temperature and evaluate battery charge enablement when awake
constexpr unsigned int ChargeTickAwakeEvalInterval = 30; // seconds

// Rate, in seconds, to sample the temperature and evaluate battery charge enablement when woken
constexpr unsigned int ChargeTickSleepEvalInterval = 1; // seconds

/**
 * @brief Get the current temperature
 *
 * @return float Current temperature in degrees celsius.
 */
float get_temperature();

/**
 * @brief Get the number of temperature high threshold events since last call to this function.
 *
 * @return size_t Number of events that have elapsed.
 */
size_t temperature_high_events();

/**
 * @brief Get the number of temperature low threshold events since last call to this function.
 *
 * @return size_t Number of events that have elapsed.
 */
size_t temperature_low_events();

/**
 * @brief Initialize the temperature sampling feature.
 *
 * @param [in]  analogPin     Analog pin to use for thermistor sampling.
 *
 * @retval SYSTEM_ERROR_NONE
 * @retval SYSTEM_ERROR_INVALID_ARGUMENT
 */
int temperature_init(pin_t analogPin, TemperatureCallback eventCallback);

/**
 * @brief Process the temperature loop tick.
 *
 * @retval SYSTEM_ERROR_NONE
 */
int temperature_tick();
