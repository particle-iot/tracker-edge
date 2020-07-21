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

#include "dct.h"

#include "tracker.h"
#include "tracker_cellular.h"


void ctrl_request_custom_handler(ctrl_request* req)
{
    auto result = SYSTEM_ERROR_NOT_SUPPORTED;
    if (Tracker::instance().isUsbCommandEnabled())
    {
        String command(req->request_data, req->request_size);
        if (CloudService::instance().dispatchCommand(command))
        {
            result = SYSTEM_ERROR_NONE;
        }
        else
        {
            result = SYSTEM_ERROR_INVALID_ARGUMENT;
        }
    }
  
    system_ctrl_set_result(req, result, nullptr, nullptr, nullptr);
}

Tracker *Tracker::_instance = nullptr;

Tracker::Tracker() :
    cloudService(CloudService::instance()),
    configService(ConfigService::instance()),
    locationService(LocationService::instance()),
    motionService(MotionService::instance()),
    rtc(AM1805_PIN_INVALID, RTC_AM1805_I2C_INSTANCE, RTC_AM1805_I2C_ADDR),
    location(TrackerLocation::instance()),
    motion(TrackerMotion::instance()),
    shipping(),
    rgb(TrackerRGB::instance())
{
    _config =
    {
        .UsbCommandEnable = true,
    };
}

int Tracker::registerConfig()
{
    static ConfigObject tracker_config("tracker", {
        ConfigBool("usb_cmd", &_config.UsbCommandEnable),
    });
    ConfigService::instance().registerModule(tracker_config);

    return 0;
}

void Tracker::init()
{
    last_loop_sec = System.uptime();

    // mark setup as complete to skip mobile app commissioning flow
    uint8_t val = 0;
    if(!dct_read_app_data_copy(DCT_SETUP_DONE_OFFSET, &val, DCT_SETUP_DONE_SIZE) && val != 1)
    {
        val = 1;
        dct_write_app_data(&val, DCT_SETUP_DONE_OFFSET, DCT_SETUP_DONE_SIZE);
    }

    int ret = hal_get_device_hw_model(&_model, &_variant, nullptr);
    if (ret)
    {
        Log.error("Failed to read device model and variant");
    }
    else
    {
        Log.info("Tracker model = %04lX, variant = %04lX", _model, _variant);
    }

    // PIN_INVALID to disable unnecessary config of a default CS pin
    SPI.begin(PIN_INVALID);

    CloudService::instance().init();

    ConfigService::instance().init();

    // Register our own configuration settings
    registerConfig();

    ret = LocationService::instance().begin(UBLOX_SPI_INTERFACE,
        UBLOX_CS_PIN,
        UBLOX_PWR_EN_PIN,
        UBLOX_TX_READY_MCU_PIN,
        UBLOX_TX_READY_GPS_PIN);
    if (ret)
    {
        Log.error("Failed to begin location service");
    }

    LocationService::instance().start();

    // Check for Tracker One hardware
    if (_model == TRACKER_MODEL_TRACKERONE)
    {
        (void)GnssLedInit();
        temperature_init(TRACKER_THERMISTOR);
    }

    MotionService::instance().start();

    location.init();

    motion.init();

    shipping.init();
    shipping.regShutdownCallback(std::bind(&Tracker::stop, this));

    rgb.init();

    rtc.begin();
#ifdef RTC_WDT_DISABLE
    rtc.disable_wdt();
#else
    // watchdog at 1 minute
    rtc.configure_wdt(true, 15, AM1805_WDT_REGISTER_WRB_QUARTER_HZ);
#endif

    location.regLocGenCallback(loc_gen_cb);
}

void Tracker::loop()
{
    uint32_t cur_sec = System.uptime();

    // slow operations for once a second
    if(last_loop_sec != cur_sec)
    {
        last_loop_sec = cur_sec;

        #ifndef RTC_WDT_DISABLE
            wdt_rtc->reset_wdt();
        #endif
    }

    // fast operations for every loop
    CloudService::instance().tick();
    ConfigService::instance().tick();
    TrackerMotion::instance().loop();

    // Check for Tracker One hardware
    if (_model == TRACKER_MODEL_TRACKERONE)
    {
        temperature_tick();

        if (temperature_high_events())
        {
            location.triggerLocPub(Trigger::NORMAL,"temp_h");
        }

        if (temperature_low_events())
        {
            location.triggerLocPub(Trigger::NORMAL,"temp_l");
        }
    }

    location.loop();
}

int Tracker::stop()
{
    LocationService::instance().stop();
    MotionService::instance().stop();

    return 0;
}

void Tracker::loc_gen_cb(JSONWriter& writer, LocationPoint &loc, const void *context)
{

    if(TrackerLocation::instance().getMinPublish())
    {
        // only add additional fields when not on minimal publish
        return;
    }

    // add cellular signal strength if available
    CellularSignal signal;
    if(!TrackerCellular::instance().getSignal(signal))
    {
        writer.name("cell").value(signal.getStrength(), 1);
    }

    // add lipo battery charge if available
    int bat_state = System.batteryState();
    if(bat_state == BATTERY_STATE_NOT_CHARGING ||
        bat_state == BATTERY_STATE_CHARGING ||
        bat_state == BATTERY_STATE_DISCHARGING ||
        bat_state == BATTERY_STATE_CHARGED)
    {
        float bat = System.batteryCharge();
        if(bat >= 0 && bat <= 100)
        {
            writer.name("batt").value(bat, 1);
        }
    }

    // Check for Tracker One hardware
    if (Tracker::instance().getModel() == TRACKER_MODEL_TRACKERONE)
    {
        writer.name("temp").value(get_temperature(), 1);
    }
}
