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

#include "Particle.h"
#include "thermistor.h"

/*
LOG_LEVEL_ALL   : special value that can be used to enable logging of all messages
LOG_LEVEL_TRACE : verbose output for debugging purposes
LOG_LEVEL_INFO  : regular information messages
LOG_LEVEL_WARN  : warnings and non-critical errors
LOG_LEVEL_ERROR : error messages
LOG_LEVEL_NONE  : special value that can be used to disable logging of any messages
*/
// SerialLogHandler logHandler(115200, LOG_LEVEL_ALL,
//                             {
//                                 {"app", LOG_LEVEL_ALL},
//                             });

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(MANUAL);

#define TEST_THERMISTOR_PIN             (A0)
static unsigned long UPDATE_DELAY = 1000; // milliseconds

// Configuration based on Panasonic ERTJ1VR104FM NTC thermistor
ThermistorConfig config = {
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

Thermistor thermistor;

// !!! Note: your experience will be much more rewarding if you use Serial1 versus Serial

void setup() {
    Serial1.begin(115200);
    //while(!Serial.isConnected());

    volatile int ret = -1;
    ret = thermistor.begin(TEST_THERMISTOR_PIN, config);

    if (ret != SYSTEM_ERROR_NONE) {
        Serial1.printf("Thermistor.begin() failed.\r\n");
    }
}

volatile bool once = false;

void loop() {
    float temperature = thermistor.getTemperature();
    Serial1.printlnf("T(degC) = %.1f", temperature);

    delay(UPDATE_DELAY);
}
