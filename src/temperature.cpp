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

#include <atomic>
#include "thermistor.h"
#include "temperature.h"


// Configuration based on Panasonic ERTJ1VR104FM NTC thermistor
ThermistorConfig _thermistorConfig = {
  .circuit              = ThermistorCircuit::HIGH_SIDE_DIVIDER,  // Thermistor is between VCC and the ADC input
  .type                 = ThermistorType::NEGATIVE_COEFF,        // NTC type
  .beta                 = 4200.0,       // B(25/50) figure
  .t0                   = 25.0,         // degrees C
  .r0                   = 100000.0,     // ohms
  .fixedR               = 100000.0,     // ohms
  .adcResolution        = 4096.0,       // values full-range
  .minTemperature       = -40.0,        // minimum temperature to represent
  .maxTemperature       = 150.0         // maximum temperature to represent
};

// Basic structure to hold all configuration fields
struct ConfigData {
  double highThreshold;
  bool highEnable;
  bool highLatch;
  double lowThreshold;
  bool lowEnable;
  bool lowLatch;
  double hysteresis;
};


static ConfigData _temperatureConfig = {
    .highThreshold = TemperatureHighDefault,
    .highEnable = false,
    .highLatch = true,
    .lowThreshold = TemperatureLowDefault,
    .lowEnable = false,
    .lowLatch = true,
    .hysteresis = TemperatureHysteresisDefault
};


// Configuration service node setup
// { "temp_trig" :
//     { "high": 25.0,
//       "high_en": false,
//       "high_latch": true,
//       "low": 25.0,
//       "low_en": false
//       "low_latch": true,
//       "hyst": 5.0
//      }
//  }

enum class TempState {
  UNKNOWN,        //< Initial state
  NORMAL,         //< Value is not outside limit and isn't pending a pass through the hysteresis limit
  OUTSIDE_LIMIT,  //< Value is outside of the given limit
  INSIDE_LIMIT,   //< Value is inside of the give limit and is pending a pass through the hysteresis limit
};


static Thermistor _thermistor;
static TemperatureCallback _enableCharging = nullptr;
static TemperatureCallback _disableCharging = nullptr;

float get_temperature() {
  return _thermistor.getTemperature();
}

int temperature_init(pin_t analogPin, TemperatureCallback chargeEnable, TemperatureCallback chargeDisable) {
  CHECK(_thermistor.begin(analogPin, _thermistorConfig));

  static ConfigObject _serviceObject
  (
      "temp_trig",
      {
        ConfigFloat("high", &_temperatureConfig.highThreshold, _thermistorConfig.minTemperature, _thermistorConfig.maxTemperature),
        ConfigBool("high_en", &_temperatureConfig.highEnable),
        ConfigBool("high_latch", &_temperatureConfig.highLatch),
        ConfigFloat("low", &_temperatureConfig.lowThreshold, _thermistorConfig.minTemperature, _thermistorConfig.maxTemperature),
        ConfigBool("low_en", &_temperatureConfig.lowEnable),
        ConfigBool("low_latch", &_temperatureConfig.lowLatch),
        ConfigFloat("hyst", &_temperatureConfig.hysteresis, 0.0, _thermistorConfig.maxTemperature - _thermistorConfig.minTemperature),
      }
  );

  CHECK(ConfigService::instance().registerModule(_serviceObject));

  _enableCharging = chargeEnable;
  _disableCharging = chargeDisable;

  return SYSTEM_ERROR_NONE;
}

// All state and variables related to high threshold evaluation
static TempState highState = TempState::NORMAL;
static std::atomic<size_t> highEvents(0);
static size_t highEventsLast = 0;
static bool highLatch = false;

size_t temperature_high_events() {
  auto eventsCapture = highEvents.load();
  auto eventsCount = eventsCapture - highEventsLast;
  highEventsLast = eventsCapture;
  return (_temperatureConfig.highLatch) ? highLatch : eventsCount;
}

// All state and variables related to low threshold evaluation
static TempState lowState = TempState::NORMAL;
static std::atomic<size_t> lowEvents(0);
static size_t lowEventsLast = 0;
static bool lowLatch = false;

size_t temperature_low_events() {
  auto eventsCapture = lowEvents.load();
  auto eventsCount = eventsCapture - lowEventsLast;
  lowEventsLast = eventsCapture;
  return (_temperatureConfig.lowLatch) ? lowLatch : eventsCount;
}

void evaluate_user_temperature(float temperature) {
  // Evaluate temperature against high threshold
  if (_temperatureConfig.highEnable) {
    switch (highState) {
      case TempState::UNKNOWN:
        highState = TempState::NORMAL;
        // Fall through
      case TempState::NORMAL: {
        if (temperature >= _temperatureConfig.highThreshold) {
          highEvents++;
          highLatch = true;
          highState = TempState::OUTSIDE_LIMIT;
        }
        break;
      }

      case TempState::OUTSIDE_LIMIT: {
        if (temperature < _temperatureConfig.highThreshold) {
          highState = TempState::INSIDE_LIMIT;
        }
        break;
      }

      case TempState::INSIDE_LIMIT: {
        if (temperature <= _temperatureConfig.highThreshold - _temperatureConfig.hysteresis) {
          highLatch = false;
          highState = TempState::NORMAL;
        }
        else if (temperature >= _temperatureConfig.highThreshold) {
          highState = TempState::OUTSIDE_LIMIT;
        }
        break;
      }
    }
  }

  // Evaluate temperature against low threshold
  if (_temperatureConfig.lowEnable) {
    switch (lowState) {
      case TempState::UNKNOWN:
        lowState = TempState::NORMAL;
        // Fall through
      case TempState::NORMAL: {
        if (temperature <= _temperatureConfig.lowThreshold) {
          lowEvents++;
          lowLatch = true;
          lowState = TempState::OUTSIDE_LIMIT;
        }
        break;
      }

      case TempState::OUTSIDE_LIMIT: {
        if (temperature > _temperatureConfig.lowThreshold) {
          lowState = TempState::INSIDE_LIMIT;
        }
        break;
      }

      case TempState::INSIDE_LIMIT: {
        if (temperature >= _temperatureConfig.lowThreshold + _temperatureConfig.hysteresis) {
          lowLatch = false;
          lowState = TempState::NORMAL;
        }
        else if (temperature <= _temperatureConfig.lowThreshold) {
          lowState = TempState::OUTSIDE_LIMIT;
        }
        break;
      }
    }
  }
}

void enableCharging(float temperature) {
  if (_enableCharging) {
    _enableCharging();
    Log.info("Battery charge temperature %0.1f C in range.  Enabling charge", temperature);
  }
}

void disableCharging(float temperature) {
  if (_disableCharging) {
    _disableCharging();
    Log.warn("Battery charge temperature %0.1f C out of range.  Disabling charge", temperature);
  }
}

void evaluate_charge_temperature(float temperature) {
  static unsigned int tick = 0;
  static TempState chargeTempState = TempState::UNKNOWN;

  if (System.uptime() - tick < ChargeTickRateSec) {
    return;
  }

  tick = System.uptime();

  bool isTempHigh = (temperature >= ChargeTempHighLimit); // Inclusive
  bool isTempBelowHighHyst = (temperature <= (ChargeTempHighLimit - ChargeTempHyst)); // Inclusive
  bool isTempLow = (temperature <= ChargeTempLowLimit); // Inclusive
  bool isTempAboveLowHyst = (temperature >= (ChargeTempLowLimit + ChargeTempHyst)); // Inclusive

  // The states are defined as follows:
    //
    //    OUTSIDE_LIMIT
    //  ----------------^ 50 degC
    //    INSIDE_LIMIT    49 degC
    //  ----------------v 48 degC
    //    NORMAL
    //  ----------------^ 2 degC
    //    INSIDE_LIMIT    1 degC
    //  ----------------v 0 degC
    //    OUTSIDE_LIMIT
    //

    switch (chargeTempState) {
      case TempState::UNKNOWN: {
        if (isTempHigh || isTempLow) {
          disableCharging(temperature);
          chargeTempState = TempState::OUTSIDE_LIMIT;
          break;
        }

        enableCharging(temperature);
        chargeTempState = TempState::NORMAL;
        break;
      }

      case TempState::NORMAL: {
        if (isTempHigh || isTempLow) {
          disableCharging(temperature);
          chargeTempState = TempState::OUTSIDE_LIMIT;
        }
        break;
      }

      case TempState::OUTSIDE_LIMIT: {
        if (isTempBelowHighHyst && isTempAboveLowHyst) {
          enableCharging(temperature);
          chargeTempState = TempState::NORMAL;
        }
        else if ((!isTempHigh && !isTempBelowHighHyst) || (!isTempLow && !isTempAboveLowHyst)) {
          chargeTempState = TempState::INSIDE_LIMIT;
        }
        break;
      }

      case TempState::INSIDE_LIMIT: {
        if (isTempHigh || isTempLow) {
          chargeTempState = TempState::OUTSIDE_LIMIT;
        }
        else if (isTempBelowHighHyst && isTempAboveLowHyst) {
          enableCharging(temperature);
          chargeTempState = TempState::NORMAL;
        }
        break;
      }
    }
}

int temperature_tick() {
  float temperature = get_temperature();

  evaluate_user_temperature(temperature);
  evaluate_charge_temperature(temperature);

  return SYSTEM_ERROR_NONE;
}
