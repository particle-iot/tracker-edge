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
#include "imu_bmi270.h"

#if (PLATFORM_ID == PLATFORM_TRACKER)
    #define SPI_XFACE                   (SPI1)
    #define SPI_CS_PIN                  (SEN_CS)

    #define INT_PIN                     (SEN_INT)
#elif (PLATFORM_ID == PLATFORM_TRACKERM)
    #define SPI_XFACE                   (SPI)
    #define SPI_CS_PIN                  (Y3)

    #define INT_PIN                     (IO_EXP_B1)
#endif    

/*
LOG_LEVEL_ALL   : special value that can be used to enable logging of all messages
LOG_LEVEL_TRACE : verbose output for debugging purposes
LOG_LEVEL_INFO  : regular information messages
LOG_LEVEL_WARN  : warnings and non-critical errors
LOG_LEVEL_ERROR : error messages
LOG_LEVEL_NONE  : special value that can be used to disable logging of any messages
*/
SerialLogHandler logHandler(115200, LOG_LEVEL_ALL,
                             {
                                 {"app", LOG_LEVEL_ALL},
                             });

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(MANUAL);

Bmi270AccelerometerConfig config = {
    .rate               = 100.0,    // Hz [0.78Hz -> 1600Hz]
    .range              = 2.0,      // g [2g, 4g, 8g, 16g]
};

Bmi270AccelMotionConfig configMotion = {
    .mode               = Bmi270AccelMotionMode::ACCEL_MOTION_MODE_ANY,
    .motionThreshold    = 0.1,      // g [up to range]
    .motionDuration     = 1,        // counts of rate period [1 -> 4]
    .motionSkip         = Bmi270AccelSignificantMotionSkip::SIG_MOTION_SKIP_1_5_S,      // seconds [1.5s, 3s, 6s, 12s]
    .motionProof        = Bmi270AccelSignificantMotionProof::SIG_MOTION_PROOF_0_25_S,   // seconds [0.25s, 0.5s, 1s, 2s]
};

Bmi270AccelHighGConfig configHighG = {
    .threshold          = 4.0,      // g [up to range]
    .duration           = 0.0025,      // seconds
    .hysteresis         = 16.0 / 16.0, // g [up to range]
};

static int WAKEUP_TIME = 60 * 1000; // milliseconds
static int MOTION_TIMOUT = 60 * 1000; // milliseconds
static bool SLEEP_ENABLED = true;
static unsigned long UPDATE_DELAY = 10; // milliseconds


// !!! Note: your experience will be much more rewarding if you use Serial1 versus Serial

void setup() {
    Serial1.begin(115200);
    
    delay(5000);

    // Configure the IMU
    auto ret = BMI270.begin(SPI_XFACE, SPI_CS_PIN, INT_PIN);
    if (ret != SYSTEM_ERROR_NONE) 
    {
        Log.error("SPI BMI270.begin() failed. %d", ret);
    }
    uint8_t chipId;
    if(SYSTEM_ERROR_NONE != BMI270.getChipId(chipId)) 
    {
        Log.error("BMI270.getChipId() failed.\r\n");
    }
    if(SYSTEM_ERROR_NONE != BMI270.reset())
    { 
        Log.error("BMI270.reset() failed.");
    }
    if(SYSTEM_ERROR_NONE != BMI270.initialize())
    {
        Log.error("BMI270.initialize() failed.");
    }
    if(SYSTEM_ERROR_NONE != BMI270.initAccelerometer(config))
    { 
        Log.error("BMI270.initAccelerometer() failed.");
    }
    if(SYSTEM_ERROR_NONE != BMI270.wakeup())
    { 
        Log.error("BMI270.wakeup() failed.");
    }

	Log.info("setup done");
}

void loop() {
    static system_tick_t sleepTime = 0;
    int ret = 0;

    if (Serial1.available()) {
        int c = Serial1.read();
        Serial1.printlnf("%c (%d)", (char)c, c);
        switch ((char)c) {
            case '0': {
                BMI270.reset();
                BMI270.initAccelerometer(config);
                BMI270.initMotion(configMotion);
                ret = BMI270.initHighG(configHighG);
                break;
            }

            case '1':   ret = BMI270.wakeup(); break;
            case '2':   ret = BMI270.sleep(); break;
            case '3':   ret = BMI270.startMotionDetect(); break;
            case '4':   ret = BMI270.stopMotionDetect(); break;
            case '5':   ret = BMI270.startHighGDetect(); break;
            case '6':   ret = BMI270.stopHighGDetect(); break;

        }
        if (ret) {
            Serial1.printlnf("'%c' returned %d", (char)c, ret);
        }
    }

    Bmi270Accelerometer accel = {0};
    BMI270.getAccelerometer(accel);

    uint32_t val = 0;
    bool intr = false;
    Bmi270::Bmi270EventType event;
    BMI270.waitOnEvent(event, 0);
    if (event == Bmi270::Bmi270EventType::SYNC) {
        Log.info("Interrupt detected");
        intr = true;
        BMI270.getStatus(val, true);
    }
    else {
        BMI270.getStatus(val, false);
    }

#if 0
    Serial1.printf("%2.3f,%2.3f,%2.3f,%u,%u,%u\r\n",
        accel.x, accel.y, accel.z, (intr) ? 1 : 0, BMI270.isMotionDetect(val), BMI270.isHighGDetect(val));
#endif

    if( BMI270.isMotionDetect(val) )
    {
        Serial1.println("Motion");
    }

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