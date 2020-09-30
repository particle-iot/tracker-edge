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

constexpr int TrackerLowBatteryCutoff = 8; // percent of battery charge
constexpr int TrackerLowBatteryCutoffCorrection = 1; // percent of battery charge
constexpr int TrackerLowBatteryWarning = 15; // percent of battery charge
constexpr int TrackerLowBatteryWarningHyst = 1; // percent of battery charge
constexpr unsigned int TrackerLowBatteryEvalTime = 15 * 60; // seconds to sample for low battery condition
constexpr system_tick_t TrackerPostChargeSettleTime = 500; // milliseconds
constexpr unsigned int TrackerLowBatteryStartTime = 20; // seconds to debounce low battery condition
constexpr unsigned int TrackerLowBatteryDebounceTime = 5; // seconds to debounce low battery condition

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
    shipping(TrackerShipping::instance()),
    rgb(TrackerRGB::instance()),
    _model(TRACKER_MODEL_BARE_SOM),
    _variant(0),
    last_loop_sec(0),
    _pastWarnLimit(false),
    _evalTick(0),
    _lastBatteryCharging(false),
    _delayedBatteryCheck(true),
    _delayedBatteryCheckTick(0),
    _pendingChargeStatus{.uptime = 0, .state = TrackerChargeState::CHARGE_INIT},
    _chargeStatus(TrackerChargeState::CHARGE_INIT),
    _lowBatteryEvent(0)
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

void Tracker::enterLowBatteryShippingMode() {
    // Publish then shutdown
    Particle.publishVitals();
    TrackerLocation::instance().triggerLocPub(Trigger::IMMEDIATE,"batt_low");
    shipping.enter(true);
}

void Tracker::lowBatteryHandler(system_event_t event, int data) {
    Tracker::instance()._lowBatteryEvent = System.uptime();
}

TrackerChargeState Tracker::batteryDecode(battery_state_t state) {
    auto chargeStatus = TrackerChargeState::CHARGE_INIT;

    switch (state) {
        case BATTERY_STATE_UNKNOWN:
        // Fall through
        case BATTERY_STATE_FAULT:
        // Fall through
        case BATTERY_STATE_NOT_CHARGING:
        // Fall through
        case BATTERY_STATE_DISCHARGING: {
            chargeStatus = TrackerChargeState::CHARGE_CARE;
            break;
        }

        case BATTERY_STATE_CHARGING:
        // Fall through
        case BATTERY_STATE_CHARGED:
        // Fall through
        case BATTERY_STATE_DISCONNECTED: {
            chargeStatus = TrackerChargeState::CHARGE_DONT_CARE;
            break;
        }
    }

    return chargeStatus;
}

void Tracker::setPendingChargeStatus(unsigned int uptime, TrackerChargeState state) {
    const std::lock_guard<std::mutex> lock(_pendingLock);
    _pendingChargeStatus.uptime = uptime;
    _pendingChargeStatus.state = state;
}

TrackerChargeStatus Tracker::getPendingChargeStatus() {
    const std::lock_guard<std::mutex> lock(_pendingLock);
    return _pendingChargeStatus;
}

void Tracker::batteryStateHandler(system_event_t event, int data) {
    auto currentChargeStatus = Tracker::instance().batteryDecode(static_cast<battery_state_t>(data));

    Tracker::instance().setPendingChargeStatus(System.uptime(), currentChargeStatus);
}


void Tracker::initBatteryMonitor() {
    // To initialize the fuel gauge so that it provides semi-accurate readings we
    // want to ensure that the charging circuit is off when providing the
    // fuel gauge quick start command.
    // In order to disable charging safely we want to enable the PMIC watchdog so that
    // if anything happens during the procedure that the circuit can return to
    // normal operation in the event the MCU doesn't complete.
    PMIC pmic(true);
    FuelGauge fuelGauge;

    pmic.setWatchdog(0x1); // 40 seconds
    pmic.disableCharging();
    // Delay so that the bulk capacitance and battery can equalize
    delay(TrackerPostChargeSettleTime);

    fuelGauge.quickStart();
    // Must delay at least 175ms after quickstart, before calling
    // getSoC(), or reading will not have updated yet.
    delay(200);

    pmic.enableCharging();
    pmic.disableWatchdog();
}

void Tracker::evaluateBatteryCharge() {
    // This is delayed intialization for the fuel gauge threshold since power on
    // events may glitch between battery states easily.
    if (_delayedBatteryCheck) {
        if (System.uptime() >= TrackerLowBatteryStartTime) {
            _delayedBatteryCheck = false;
            FuelGauge fuelGauge;

            // Set the alert level for 8% - 1%.  This value will not be normalized but rather the raw
            // threshold value provided by the fuel gauge.
            // The fuel gauge will only give an alert when passing through this limit with decreasing
            // succesive charge amounts.  It is important to check whether we are already below this limit
            fuelGauge.setAlertThreshold((uint8_t)(TrackerLowBatteryCutoff - TrackerLowBatteryCutoffCorrection));
            fuelGauge.clearAlert();
            delay(100);

            // NOTE: This is a workaround in case the fuel gauge interrupt is not configured as an input
            pinMode(LOW_BAT_UC, INPUT_PULLUP);

            System.on(low_battery, lowBatteryHandler);
            System.on(battery_state, batteryStateHandler);
            if (_chargeStatus == TrackerChargeState::CHARGE_INIT) {
                setPendingChargeStatus(System.uptime(), batteryDecode(static_cast<battery_state_t>(System.batteryState())));
            }
        }
    }

    // Debounce the charge status here by looking at data collected by the interrupt handler and making sure that the
    // last state is present over a qualified amount of time.
    auto status = getPendingChargeStatus();
    if (status.uptime && ((System.uptime() - status.uptime) >= TrackerLowBatteryDebounceTime)) {
        _chargeStatus = status.state;
        setPendingChargeStatus(0, _chargeStatus);
        _evalTick = System.uptime();
    }

    // No further work necessary if we are still in the delayed battery check interval or not on a evaluation interval
    if (_delayedBatteryCheck ||
        (System.uptime() - _evalTick < TrackerLowBatteryEvalTime)) {

        return;
    }
    _evalTick = System.uptime();

    auto stateOfCharge = System.batteryCharge();

    // Skip errors
    if (stateOfCharge < 0.0) {
        return;
    }

    switch (_chargeStatus) {
        case TrackerChargeState::CHARGE_CARE: {
            if (_lowBatteryEvent || (stateOfCharge <= (float)TrackerLowBatteryCutoff)) {
                // Publish then shutdown
                Log.error("Battery charge of %0.1f%% is less than limit of %0.1f%%.  Entering shipping mode", stateOfCharge, (float)TrackerLowBatteryCutoff);
                enterLowBatteryShippingMode();
            }
            else if (!_pastWarnLimit && (stateOfCharge <= (float)TrackerLowBatteryWarning)) {
                _pastWarnLimit = true;
                // Publish once when falling through this value
                Particle.publishVitals();
                location.triggerLocPub(Trigger::IMMEDIATE,"batt_warn");
                Log.warn("Battery charge of %0.1f%% is less than limit of %0.1f%%.  Publishing warning", stateOfCharge, (float)TrackerLowBatteryWarning);
            }
            break;
        }

        case TrackerChargeState::CHARGE_DONT_CARE: {
            // There may be instances where the device is being charged but the battery is still being discharged
            if (_lowBatteryEvent) {
                // Publish then shutdown
                Log.error("Battery charge of %0.1f%% is less than limit of %0.1f%%.  Entering shipping mode", stateOfCharge, (float)TrackerLowBatteryCutoff);
                enterLowBatteryShippingMode();
            }
            else if (_pastWarnLimit && (stateOfCharge >= (float)(TrackerLowBatteryWarning + TrackerLowBatteryWarningHyst))) {
                _pastWarnLimit = false;
                // Publish again to announce that we are out of low battery warning
                Particle.publishVitals();
            }
        }
    }
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

    // Reset the fuel gauge state-of-charge, check if under thresholds
    if (_model == TRACKER_MODEL_TRACKERONE)
    {
        initBatteryMonitor();
    }

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
        temperature_init(TRACKER_THERMISTOR,
            [this](){ return enableCharging();},
            [this](){ return disableCharging();}
        );
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

    // Evaluate low battery conditions
    if (_model == TRACKER_MODEL_TRACKERONE)
    {
        evaluateBatteryCharge();
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

int Tracker::enableCharging() {
    PMIC pmic(true);
    return (pmic.enableCharging()) ? SYSTEM_ERROR_NONE : SYSTEM_ERROR_IO;
}

int Tracker::disableCharging() {
    PMIC pmic(true);
    return (pmic.disableCharging()) ? SYSTEM_ERROR_NONE : SYSTEM_ERROR_IO;
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
