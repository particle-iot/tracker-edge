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
  NORMAL,         //< Value is not outside limit and isn't pending a pass through the hysteresis limit
  OUTSIDE_LIMIT,  //< Valie is outside of the given limit
  INSIDE_LIMIT,   //< Value is inside of the give limit and is pending a pass through the hysteresis limit
};


static Thermistor _thermistor;

float get_temperature() {
  return _thermistor.getTemperature();
}

int temperature_init(pin_t analogPin) {  
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


int temperature_tick() {
  float temperature = get_temperature();

  // Evaluate temperature against high threshold
  if (_temperatureConfig.highEnable) {
    switch (highState) {
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

  return SYSTEM_ERROR_NONE;
}
