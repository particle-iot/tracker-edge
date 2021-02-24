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

#include "tracker_shipping.h"
#include "tracker.h"

#define SHIPPING_MODE_LED_CYCLE_PERIOD_MS       (250)
#define SHIPPING_MODE_LED_CYCLE_DURATION_MS     (5000)
#define SHIPPING_MODE_DEFER_DURATION_MS         (5000) // 5 seconds
#define SHIPPING_MODE_SAMPLE_MS                 (1000) // 1 second
#define SHIPPING_MODE_TIMEOUT                   (60) // SHIPPING_MODE_SAMPLE_MS intervals

TrackerShipping *TrackerShipping::_instance = nullptr;

int TrackerShipping::regShutdownBeginCallback(ShippingModeCb begin)
{
    _beginCallback = begin;

    return 0;
}

int TrackerShipping::regShutdownIoCallback(ShippingModeCb io)
{
    _ioCallback = io;

    return 0;
}

int TrackerShipping::regShutdownFinalCallback(ShippingModeCb final)
{
    _finalCallback = final;

    return 0;
}

void TrackerShipping::pmicHandler()
{
    TrackerShipping::instance()._pmicFire = true;
}

void TrackerShipping::shutdown()
{
    if(_ioCallback)
    {
        (void)_ioCallback();
    }

    // blink RGB to signal entering shipping mode
    RGB.control(true);
    RGB.brightness(255);
    for(int i=0;
        i < (SHIPPING_MODE_LED_CYCLE_DURATION_MS / SHIPPING_MODE_LED_CYCLE_PERIOD_MS);
        i++)
    {
        // cycle between primary colors
        RGB.color(((uint32_t) 0xFF) << ((i % 3) * 8));
        HAL_Delay_Milliseconds(SHIPPING_MODE_LED_CYCLE_PERIOD_MS);
    }

    auto shipping = &TrackerShipping::instance();
    if (shipping->_checkPower)
    {
        // Attach and own the PMIC interrupt in order to provide the quickest
        // way to figure out changes in PMIC input power right before going into
        // shipping mode.
        attachInterrupt(PMIC_INT, &TrackerShipping::pmicHandler, FALLING);
    }

    // The PMIC will be locked from this point forward with no further changes allowed
    PMIC pmic(true);

    // Disable charging will ensure there are no asynchronous events that may interrupt
    // entering of shipping mode
    pmic.disableCharging();

    // Clear all faults
    (void)pmic.getFault();
    (void)pmic.getFault();

    pmic.disableWatchdog();
    if (shipping->_checkPower && shipping->_pmicFire)
    {
        // If the PMIC interrupted us then reset instead of going into shipping mode because
        // the power is likely to be applied between when the mode was commanded and the delayed
        // response of this particular handler.
        System.reset();
    }

    pmic.disableBATFET();

    // Wait for the PMIC to exit DPDM detection before shutting down
    int timeout = SHIPPING_MODE_TIMEOUT;
    while (pmic.isInDPDM() && (timeout-- > 0))
    {
        delay(SHIPPING_MODE_SAMPLE_MS);
    }

    // Clear all faults
    (void)pmic.getFault();
    (void)pmic.getFault();

    RGB.brightness(0);

    if(_finalCallback)
    {
        (void)_finalCallback();
    }

    // Enter sleep with lower level sleep API so power management doesn't override PMIC settings
    SystemSleepConfiguration config;
    config.mode(SystemSleepMode::HIBERNATE)
          .gpio(PMIC_INT, FALLING);
    hal_sleep_enter(config.halConfig(), nullptr, nullptr);

    // shouldn't hit these lines as never coming back from sleep but out of an
    // abundance of paranoia force a reset so we don't get stuck in some weird
    // pseudo-shutdown state
    System.reset();
}

int TrackerShipping::enter(bool checkPower)
{
    if(_beginCallback)
    {
        int rval = _beginCallback();

        if(rval)
        {
            return rval;
        }
    }

    // This flag will allow the shipping mode code to check power state before shutting down
    _checkPower = checkPower;

    // Timer call will shutdown device so don't worry about dynamic memory
    auto deferredShutdown = new Timer(SHIPPING_MODE_DEFER_DURATION_MS, &TrackerShipping::shutdown, *this, true);
    deferredShutdown->start();

    return 0; // compiler warnings about no return...
}

int TrackerShipping::enter_cb(CloudServiceStatus status, JSONValue *root, const void *context)
{
    return enter();
}

void TrackerShipping::init()
{
    CloudService::instance().regCommandCallback("enter_shipping", &TrackerShipping::enter_cb, this);
}
