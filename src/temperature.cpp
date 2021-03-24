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
#include "tracker_sleep.h"


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

enum class TempChargeState {
  UNKNOWN,                //< Initial state
  NORMAL,                 //< Value is not outside limit and isn't pending a pass through the hysteresis limit
  OVER_TEMPERATURE,       //< Value is above the upper given limit
  UNDER_TEMPERATURE,      //< Value is below the lower given limit
  OVER_CHARGE_REDUCTION,  //< Value is above the intermediate given limit
};


static Thermistor _thermistor;
static TemperatureCallback _eventCallback = nullptr;
static unsigned int chargeEvalTick = 0;

void onWake(TrackerSleepContext context) {
  // Allow evaluation immediately after wake
  chargeEvalTick = 0;
}

float get_temperature() {
  return _thermistor.getTemperature();
}

int temperature_init(pin_t analogPin, TemperatureCallback eventCallback) {
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

  _eventCallback = eventCallback;

  void onWake(TrackerSleepContext context);

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

int alertEventListener(TemperatureChargeEvent event) {
  if (_eventCallback) {
    return _eventCallback(event);
  }

  return SYSTEM_ERROR_NOT_SUPPORTED;
}

TempChargeState toChargeState(TempChargeState state) {
  switch (state) {
    case TempChargeState::UNKNOWN:
      break;

    case TempChargeState::NORMAL:
      alertEventListener(TemperatureChargeEvent::NORMAL);
      break;

    case TempChargeState::OVER_TEMPERATURE:
      alertEventListener(TemperatureChargeEvent::OVER_TEMPERATURE);
      break;

    case TempChargeState::UNDER_TEMPERATURE:
      alertEventListener(TemperatureChargeEvent::UNDER_TEMPERATURE);
      break;

    case TempChargeState::OVER_CHARGE_REDUCTION:
      alertEventListener(TemperatureChargeEvent::OVER_CHARGE_REDUCTION);
      break;

  }

  return state;
}

void evaluate_charge_temperature(float temperature) {
  static TempChargeState chargeTempState = TempChargeState::UNKNOWN;

  unsigned int evalLoopInterval = TrackerSleep::instance().isSleepDisabled() ? ChargeTickAwakeEvalInterval : ChargeTickSleepEvalInterval;
  if (System.uptime() - chargeEvalTick < evalLoopInterval) {
    return;
  }

  chargeEvalTick = System.uptime();

  bool isTempHigh = (temperature >= ChargeTempHighLimit); // Inclusive
  bool isTempBelowHighHyst = (temperature <= (ChargeTempHighLimit - ChargeTempHyst)); // Inclusive
  bool isTempReducedA = (temperature >= ChargeTempReducedALimit); // Inclusive
  bool isTempReducedAHyst = (temperature <= (ChargeTempReducedALimit - ChargeTempHyst)); // Inclusive
  bool isTempLow = (temperature <= ChargeTempLowLimit); // Inclusive
  bool isTempAboveLowHyst = (temperature >= (ChargeTempLowLimit + ChargeTempHyst)); // Inclusive

  // The states are defined as follows:
  //
  //    OVER_TEMPERATURE
  //  --------------------------------------------------------^ 54 degC  (ChargeTempHighLimit)
  //    OVER_CHARGE_REDUCTION ^     OVER_TEMPERATURE v          53 degC
  //  --------------------------------------------------------v 52 degC  (ChargeTempHighLimit - ChargeTempHyst)
  //    OVER_CHARGE_REDUCTION
  //  --------------------------------------------------------^ 40 degC  (ChargeTempReducedALimit)
  //    NORMAL ^                    OVER_CHARGE_REDUCTION v     39 degC
  //  --------------------------------------------------------v 38 degC  (ChargeTempReducedALimit - ChargeTempHyst)
  //    NORMAL
  //  --------------------------------------------------------^ 2 degC   (ChargeTempLowLimit + ChargeTempHyst)
  //    UNDER_TEMPERATURE ^         NORMAL v                    1 degC
  //  --------------------------------------------------------v 0 degC   (ChargeTempLowLimit)
  //    UNDER_TEMPERATURE
  //

  switch (chargeTempState) {
    //
    // Initial state at boot
    //
    case TempChargeState::UNKNOWN: {
      if (isTempLow) {
        chargeTempState = toChargeState(TempChargeState::UNDER_TEMPERATURE);
      }
      else if (isTempHigh) {
        chargeTempState = toChargeState(TempChargeState::OVER_TEMPERATURE);
      }
      else if (isTempReducedA) {
        chargeTempState = toChargeState(TempChargeState::OVER_CHARGE_REDUCTION);
      }
      else {
        chargeTempState = toChargeState(TempChargeState::NORMAL);
      }

      break;
    }

    //
    // No limits were breached
    //
    case TempChargeState::NORMAL: {
      if (isTempLow) {
        chargeTempState = toChargeState(TempChargeState::UNDER_TEMPERATURE);
      }
      else if (isTempHigh) {
        chargeTempState = toChargeState(TempChargeState::OVER_TEMPERATURE);
      }
      else if (isTempReducedA) {
        chargeTempState = toChargeState(TempChargeState::OVER_CHARGE_REDUCTION);
      }

      break;
    }

    //
    // Under normal charging operation
    //
    case TempChargeState::UNDER_TEMPERATURE: {
      if (isTempHigh) {
        chargeTempState = toChargeState(TempChargeState::OVER_TEMPERATURE);
      }
      else if (isTempReducedA) {
        chargeTempState = toChargeState(TempChargeState::OVER_CHARGE_REDUCTION);
      }
      else if (isTempAboveLowHyst) {
        chargeTempState = toChargeState(TempChargeState::NORMAL);
      }

      break;
    }

    //
    // Above normal charging operation
    //
    case TempChargeState::OVER_TEMPERATURE: {
      if (isTempLow) {
        chargeTempState = toChargeState(TempChargeState::UNDER_TEMPERATURE);
      }
      else if (isTempBelowHighHyst && isTempReducedA) {
        chargeTempState = toChargeState(TempChargeState::OVER_CHARGE_REDUCTION);
      }
      else if (isTempReducedAHyst) {
        chargeTempState = toChargeState(TempChargeState::NORMAL);
      }

      break;
    }

    //
    // Above normal charging operation
    //
    case TempChargeState::OVER_CHARGE_REDUCTION: {
      if (isTempHigh) {
        chargeTempState = toChargeState(TempChargeState::OVER_TEMPERATURE);
      }
      else if (isTempLow) {
        chargeTempState = toChargeState(TempChargeState::UNDER_TEMPERATURE);
      }
      else if (isTempReducedAHyst) {
        chargeTempState = toChargeState(TempChargeState::NORMAL);
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
