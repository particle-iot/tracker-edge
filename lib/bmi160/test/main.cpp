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
#include "bmi160.h"

#define USE_BMI160_SPI              (1)
#define USE_BMI160_I2C              (0)

#if (USE_BMI160_SPI == USE_BMI160_I2C)
#error "SPI or I2C interface must be selected"
#endif

#define SPI_XFACE                   (SPI1)
#define SPI_CS_PIN                  (SEN_CS)

#define I2C_INTERFACE               (&Wire)
#define I2C_ADDRESS                 (0x68)

#define INT_PIN                     (SEN_INT)

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

Bmi160AccelerometerConfig config = {
    .rate               = 100.0,    // Hz [0.78Hz -> 1600Hz]
    .range              = 16.0,      // g [2g, 4g, 8g, 16g]
};

Bmi160AccelMotionConfig configMotion = {
    .mode               = Bmi160AccelMotionMode::ACCEL_MOTION_MODE_ANY,
    .motionThreshold    = 0.25,      // g [up to range]
    .motionDuration     = 4,        // counts of rate period [1 -> 4]
    .motionSkip         = Bmi160AccelSignificantMotionSkip::SIG_MOTION_SKIP_1_5_S,      // seconds [1.5s, 3s, 6s, 12s]
    .motionProof        = Bmi160AccelSignificantMotionProof::SIG_MOTION_PROOF_0_25_S,   // seconds [0.25s, 0.5s, 1s, 2s]
};

Bmi160AccelHighGConfig configHighG = {
    .threshold          = 4.0,      // g [up to range]
    .duration           = 0.0025,      // seconds
    .hysteresis         = 16.0 / 16.0, // g [up to range]
};

static int WAKEUP_TIME = 60; // seconds
static int MOTION_TIMOUT = 20 * 1000; // milliseconds
static bool SLEEP_ENABLED = false;
static unsigned long UPDATE_DELAY = 10; // milliseconds


// !!! Note: your experience will be much more rewarding if you use Serial1 versus Serial

void setup() {
    Serial1.begin(115200);
    //while(!Serial.isConnected());

    int ret = 0;

#if USE_BMI160_I2C
    I2C_INTERFACE->setSpeed(CLOCK_SPEED_100KHZ);

    if (BMI160.begin(I2C_INTERFACE, I2C_ADDRESS, INT_PIN) != SYSTEM_ERROR_NONE) {
        Serial1.printf("I2C BMI160.begin() failed.\r\n");
    }

#elif USE_BMI160_SPI
    SPI_XFACE.begin();

    ret = BMI160.begin(SPI_XFACE, SPI_CS_PIN, INT_PIN);
    if (ret != SYSTEM_ERROR_NONE) {
        Serial1.printlnf("SPI BMI160.begin() failed. %d", ret);
    }

#else
#error "Interface not recognized"
#endif // USE_BMI160_XFACE

    uint8_t chipId;
    if (BMI160.getChipId(chipId) != SYSTEM_ERROR_NONE) {
        Serial1.printf("BMI160.getChipId() failed.\r\n");
    }

    BMI160.reset();
    BMI160.initAccelerometer(config);
    BMI160.initMotion(configMotion);
    BMI160.initHighG(configHighG);
    BMI160.wakeup();
}

void loop() {
    static system_tick_t sleepTime = 0;
    int ret = 0;

    if (Serial1.available()) {
        int c = Serial1.read();
        Serial1.printlnf("%c (%d)", (char)c, c);
        switch ((char)c) {
            case '0': {
                BMI160.reset();
                BMI160.initAccelerometer(config);
                BMI160.initMotion(configMotion);
                ret = BMI160.initHighG(configHighG);
                break;
            }

            case '1':   ret = BMI160.wakeup(); break;
            case '2':   ret = BMI160.sleep(); break;
            case '3':   ret = BMI160.startMotionDetect(); break;
            case '4':   ret = BMI160.stopMotionDetect(); break;
            case '5':   ret = BMI160.startHighGDetect(); break;
            case '6':   ret = BMI160.stopHighGDetect(); break;

        }
        if (ret) {
            Serial1.printlnf("'%c' returned %d", (char)c, ret);
        }
    }

    Bmi160Accelerometer accel = {0};
    BMI160.getAccelerometer(accel);
    Bmi160::Bmi160PowerState pmu;
    BMI160.getAccelerometerPmu(pmu);

    uint32_t val = 0;
    bool intr = false;
    Bmi160::Bmi160EventType event;
    BMI160.waitOnEvent(event, 0);
    if (event == Bmi160::Bmi160EventType::SYNC) {
        intr = true;
        BMI160.getStatus(val, true);
    }
    else {
        BMI160.getStatus(val, false);
    }
    // Serial1.printf("%2.3f,%2.3f,%2.3f,%u,%u,%u,%08lx\r\n",
    //     accel.x, accel.y, accel.z, (intr) ? 1 : 0, BMI160.isMotionDetect(val), BMI160.isHighGDetect(val), val);
    Serial1.printf("%2.3f,%2.3f,%2.3f,%u,%u,%u\r\n",
        accel.x, accel.y, accel.z, (intr) ? 1 : 0, BMI160.isMotionDetect(val), BMI160.isHighGDetect(val));

    // Keep from going to sleep by moving timeout ahead
    if (intr) {
        sleepTime = millis();
    }

    delay(UPDATE_DELAY);

    if (SLEEP_ENABLED && (millis() - sleepTime >= MOTION_TIMOUT)) {
        SystemSleepConfiguration config;

        config.mode(SystemSleepMode::ULTRA_LOW_POWER)
            .gpio(INT_PIN, FALLING)
            .duration(WAKEUP_TIME);

        System.sleep(config);
        sleepTime = millis();
    }
}
