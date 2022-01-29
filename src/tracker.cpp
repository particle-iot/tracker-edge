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
#include "mcp_can.h"

// Defines and constants
constexpr int CanSleepRetries = 10; // Based on a series of 10ms delays

constexpr int TrackerLowBatteryCutoff = 2; // percent of battery charge
constexpr int TrackerLowBatteryCutoffCorrection = 1; // percent of battery charge
constexpr int TrackerLowBatteryWarning = 8; // percent of battery charge
constexpr int TrackerLowBatteryWarningHyst = 1; // percent of battery charge
constexpr unsigned int TrackerLowBatteryAwakeEvalInterval = 2 * 60; // seconds to sample for low battery condition
constexpr unsigned int TrackerLowBatterySleepEvalInterval = 1; // seconds to sample for low battery condition
constexpr unsigned int TrackerLowBatterySleepWakeInterval = 15 * 60; // seconds to sample for low battery condition
constexpr system_tick_t TrackerPostChargeSettleTime = 500; // milliseconds
constexpr unsigned int TrackerLowBatteryStartTime = 20; // seconds to debounce low battery condition
constexpr unsigned int TrackerLowBatteryDebounceTime = 5; // seconds to debounce low battery condition
constexpr unsigned int TrackerChargingAwakeEvalTime = 10; // seconds to sample the PMIC charging state
constexpr unsigned int TrackerChargingSleepEvalTime = 1; // seconds to sample the PMIC charging state
constexpr uint16_t TrackerChargeCurrentHigh = 1024; // milliamps
constexpr uint16_t TrackerChargeCurrentLow = 512; // milliamps
constexpr uint16_t TrackerInputCurrent = 1500; // milliamps
constexpr unsigned int TrackerFailedOtaKeepAwake = 60; // seconds to stay awake after failed OTA
constexpr system_tick_t TrackerWatchdogExpireTime = 60 * 1000; // milliseconds to expire the WDT

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
    sleep(TrackerSleep::instance()),
    locationService(LocationService::instance()),
    motionService(MotionService::instance()),
    location(TrackerLocation::instance()),
    motion(TrackerMotion::instance()),
    shipping(TrackerShipping::instance()),
    rgb(TrackerRGB::instance()),
    _model(TRACKER_MODEL_BARE_SOM),
    _variant(0),
    _lastLoopSec(0),
    _canPowerEnabled(false),
    _pastWarnLimit(false),
    _evalTick(0),
    _lastBatteryCharging(false),
    _delayedBatteryCheck(true),
    _delayedBatteryCheckTick(0),
    _pendingChargeStatus{.uptime = 0, .state = TrackerChargeState::CHARGE_INIT},
    _chargeStatus(TrackerChargeState::CHARGE_INIT),
    _lowBatteryEvent(0),
    _evalChargingTick(0),
    _batteryChargeEnabled(true)
{
    _cloudConfig =
    {
        .UsbCommandEnable = true,
    };
}

int Tracker::registerConfig()
{
    static ConfigObject tracker_config("tracker", {
        ConfigBool("usb_cmd", &_cloudConfig.UsbCommandEnable),
    });
    configService.registerModule(tracker_config);

    return 0;
}

int Tracker::getPowerManagementConfig(hal_power_config& conf) {
    conf.size = sizeof(conf); // Size is provided for backwards compatibility
    auto err = dct_read_app_data_copy(DCT_POWER_CONFIG_OFFSET, &conf, conf.size);
    if (err) {
        return SYSTEM_ERROR_IO; // Read error
    }

    // Check if table contents are valid
    if ((conf.version == 0xff) || (conf.size == 0x00) || (conf.size == 0xff)) {
        return SYSTEM_ERROR_INVALID_STATE;
    }

    // Invert first byte of the flags to keep compatibility for HAL_POWER_PMIC_DETECTION flag
    uint32_t inverted = (~conf.flags) & 0x000000ff;
    conf.flags &= 0xffffff00;
    conf.flags |= inverted;

    return SYSTEM_ERROR_NONE;
}

int Tracker::setPowerManagementConfig(const hal_power_config& conf) {
    return system_power_management_set_config(&conf, nullptr);
}

int Tracker::enablePowerManagement() {
    // Gather power management configuration data that has been saved away to the DCT
    // and ensure the power managemen disable flag is clear without affecting any
    // other settings.  This will be performed inside of the device OS on later
    // versions.

    hal_power_config conf = {};
    auto err = getPowerManagementConfig(conf);

    if (err) {
        return err;
    }

    // Clear the disable flag if set but keep everything else
    if ((conf.flags & HAL_POWER_MANAGEMENT_DISABLE) == 0) {
        return SYSTEM_ERROR_NONE;
    }

    conf.flags &= ~HAL_POWER_MANAGEMENT_DISABLE;

    err = setPowerManagementConfig(conf);

    return err;
}

void Tracker::enableIoCanPower(bool enable)
{
    digitalWrite(MCP_CAN_PWR_EN_PIN, (_canPowerEnabled = enable) ? HIGH : LOW);
}

int Tracker::initEsp32()
{
    // ESP32 related GPIO
    pinMode(ESP32_BOOT_MODE_PIN, OUTPUT);
    digitalWrite(ESP32_BOOT_MODE_PIN, HIGH);
    pinMode(ESP32_PWR_EN_PIN, OUTPUT);
    digitalWrite(ESP32_PWR_EN_PIN, LOW); // power off device, first power off for ESP32 workaround for low power
    delay(50); // ESP32 workaround for low power
    digitalWrite(ESP32_PWR_EN_PIN, HIGH); // power on device, ESP32 workaround for low power
    delay(50); // ESP32 workaround for low power
    digitalWrite(ESP32_PWR_EN_PIN, LOW); // power off device
    pinMode(ESP32_CS_PIN, OUTPUT);
    digitalWrite(ESP32_CS_PIN, HIGH);

    return SYSTEM_ERROR_NONE;
}

int Tracker::initCan()
{
    // CAN related GPIO
    pinMode(MCP_CAN_STBY_PIN, OUTPUT);
    digitalWrite(MCP_CAN_STBY_PIN, LOW);
    pinMode(MCP_CAN_PWR_EN_PIN, OUTPUT);
    // Do not power the CAN interface on yet
    pinMode(MCP_CAN_RESETN_PIN, OUTPUT);
    digitalWrite(MCP_CAN_RESETN_PIN, HIGH);
    pinMode(MCP_CAN_INT_PIN, INPUT_PULLUP);
    pinMode(MCP_CAN_CS_PIN, OUTPUT);
    digitalWrite(MCP_CAN_CS_PIN, HIGH);

    // Reset CAN transceiver
    digitalWrite(MCP_CAN_RESETN_PIN, LOW);
    delay(50);
    digitalWrite(MCP_CAN_RESETN_PIN, HIGH);
    delay(50);

    digitalWrite(MCP_CAN_STBY_PIN, HIGH);

    // Initialize CAN device driver
    MCP_CAN can(MCP_CAN_CS_PIN, MCP_CAN_SPI_INTERFACE);
    if (can.minimalInit() != CAN_OK)
    {
        Log.error("CAN init failed");
    }

    if (_deviceConfig.enableIoCanPower()) {
        enableIoCanPower(true);
    }

    return SYSTEM_ERROR_NONE;
}

int Tracker::initIo()
{
    // Initialize basic Tracker GPIO to known inactive values until they are needed later
    (void)initEsp32();
    (void)initCan();

    return SYSTEM_ERROR_NONE;
}

void Tracker::enableWatchdog(bool enable) {
#ifndef RTC_WDT_DISABLE
    if (enable) {
        // watchdog at 1 minute
        hal_exrtc_enable_watchdog(TrackerWatchdogExpireTime, nullptr);
        hal_exrtc_feed_watchdog(nullptr);
    }
    else {
        hal_exrtc_disable_watchdog(nullptr);
    }
#else
    (void)enable;
#endif // RTC_WDT_DISABLE
}

void Tracker::startShippingMode() {
    // Always let the sleep framework manage dependencies on power state changes
    sleep.forceShutdown();
}

int Tracker::prepareSleep() {
    if (_deviceConfig.enableIoCanPowerSleep()) {
        enableIoCanPower(false);
    }
    return SYSTEM_ERROR_NONE;
}

int Tracker::prepareWake() {
    if (_deviceConfig.enableIoCanPowerSleep() && _deviceConfig.enableIoCanPower()) {
        enableIoCanPower(true);
    }
    return SYSTEM_ERROR_NONE;
}

void Tracker::startLowBatteryShippingMode() {
    if (sleep.isForcedShutdownPending()) {
        return;
    }

    // Publish then shutdown
    sleep.forcePublishVitals();
    location.triggerLocPub(Trigger::IMMEDIATE,"batt_low");
    startShippingMode();
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
    const std::lock_guard<Mutex> lock(_pendingLock);
    _pendingChargeStatus.uptime = uptime;
    _pendingChargeStatus.state = state;
}

TrackerChargeStatus Tracker::getPendingChargeStatus() {
    const std::lock_guard<Mutex> lock(_pendingLock);
    return _pendingChargeStatus;
}

void Tracker::batteryStateHandler(system_event_t event, int data) {
    auto currentChargeStatus = Tracker::instance().batteryDecode(static_cast<battery_state_t>(data));

    Tracker::instance().setPendingChargeStatus(System.uptime(), currentChargeStatus);
}


void Tracker::initBatteryMonitor() {
    auto powerConfig = System.getPowerConfiguration();
    // Start battery charging at low current state from boot then increase if necessary
    if ((powerConfig.batteryChargeCurrent() != TrackerChargeCurrentLow) ||
        (powerConfig.powerSourceMaxCurrent() != TrackerInputCurrent)) {

        powerConfig.batteryChargeCurrent(TrackerChargeCurrentLow);
        powerConfig.powerSourceMaxCurrent(TrackerInputCurrent);
        System.setPowerConfiguration(powerConfig);
    }

    // Keep a handy variable to check on battery charge enablement
    _batteryChargeEnabled = !powerConfig.isFeatureSet(SystemPowerFeature::DISABLE_CHARGING);

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

    if (_batteryChargeEnabled) {
        pmic.enableCharging();
    }
    pmic.disableWatchdog();
}

bool Tracker::getChargeEnabled() {
    return !System.getPowerConfiguration().isFeatureSet(SystemPowerFeature::DISABLE_CHARGING);
}

void Tracker::evaluateBatteryCharge() {
    // This is delayed intialization for the fuel gauge threshold since power on
    // events may glitch between battery states easily.
    if (_delayedBatteryCheck) {
        if (System.uptime() >= TrackerLowBatteryStartTime) {
            _delayedBatteryCheck = false;
            FuelGauge fuelGauge;

            // Set the alert level for <SET VALUE> - 1%.  This value will not be normalized but rather the raw
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
    unsigned int evalLoopInterval = sleep.isSleepDisabled() ? TrackerLowBatteryAwakeEvalInterval : TrackerLowBatterySleepEvalInterval;
    if (_delayedBatteryCheck ||
        (System.uptime() - _evalTick < evalLoopInterval)) {

        return;
    }

    _evalTick = System.uptime();

    auto stateOfCharge = System.batteryCharge();

    // Skip errors
    if (stateOfCharge < 0.0) {
        Log.info("Battery charge reporting error");
        return;
    }

    switch (_chargeStatus) {
        case TrackerChargeState::CHARGE_CARE: {
            if (_lowBatteryEvent || (stateOfCharge <= (float)TrackerLowBatteryCutoff)) {
                // Publish then shutdown
                Log.error("Battery charge of %0.1f%% is less than limit of %0.1f%%.  Entering shipping mode", stateOfCharge, (float)TrackerLowBatteryCutoff);
                startLowBatteryShippingMode();
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
                startLowBatteryShippingMode();
            }
            else if (_pastWarnLimit && (stateOfCharge >= (float)(TrackerLowBatteryWarning + TrackerLowBatteryWarningHyst))) {
                _pastWarnLimit = false;
                // Publish again to announce that we are out of low battery warning
                Particle.publishVitals();
            }
        }
    }
}

void Tracker::onSleepPrepare(TrackerSleepContext context)
{
    configService.flush();
    if (_model == TRACKER_MODEL_TRACKERONE) {
        TrackerSleep::instance().wakeAtSeconds(System.uptime() + TrackerLowBatterySleepWakeInterval);
    }
}

void Tracker::onSleep(TrackerSleepContext context)
{
    if (_model == TRACKER_MODEL_TRACKERONE) {
        GnssLedEnable(false);
    }
}

void Tracker::onWake(TrackerSleepContext context)
{
    if (_model == TRACKER_MODEL_TRACKERONE) {
        GnssLedEnable(true);
        // Ensure battery evaluation starts immediately after waking
        _evalTick = 0;
    }
}

void Tracker::onSleepStateChange(TrackerSleepContext context)
{
    if (context.reason == TrackerSleepReason::STATE_TO_SHUTDOWN) {
        // Consider any device shutdown here
    }
}

void Tracker::completeSetupDone()
{
    // mark setup as complete to skip mobile app commissioning flow
    uint8_t val = 0;
    if(!dct_read_app_data_copy(DCT_SETUP_DONE_OFFSET, &val, DCT_SETUP_DONE_SIZE) && val != 1)
    {
        val = 1;
        dct_write_app_data(&val, DCT_SETUP_DONE_OFFSET, DCT_SETUP_DONE_SIZE);
    }
}

void Tracker::otaHandler(system_event_t event, int param) {
    switch ((unsigned int)param) {
        case SystemEventsParam::firmware_update_complete: {
            // There will be an imminent system reset so disable the watchdog
            enableWatchdog(false);
        }
        break;

        case SystemEventsParam::firmware_update_begin: {
            if (!sleep.isSleepDisabled()) {
                // Don't allow the device to go asleep if an OTA has begun
                sleep.pauseSleep();
            }
        }
        break;

        case SystemEventsParam::firmware_update_failed: {
            if (!sleep.isSleepDisabled()) {
                // Allow the device to go asleep after a chance for the cloud to restart a failed OTA
                sleep.extendExecutionFromNow(TrackerFailedOtaKeepAwake);
                sleep.resumeSleep();
            }
        }
        break;

        default:
            break;
    }
}

void Tracker::startup()
{
    completeSetupDone();

    // Correct power manager states in the DCT
    enablePowerManagement();
}

int Tracker::init()
{
    int ret = 0;

    _lastLoopSec = System.uptime();

    // Disable OTA updates until after the system handler has been registered
    System.disableUpdates();

#ifndef TRACKER_MODEL_NUMBER
    ret = hal_get_device_hw_model(&_model, &_variant, nullptr);
    if (ret)
    {
        Log.error("Failed to read device model and variant");
    }
    else
    {
        Log.info("Tracker model = %04lX, variant = %04lX", _model, _variant);
    }
#else
    _model = TRACKER_MODEL_NUMBER;
#ifdef TRACKER_MODEL_VARIANT
    _variant = TRACKER_MODEL_VARIANT;
#else
    _variant = 0;
#endif // TRACKER_MODEL_VARIANT
#endif // TRACKER_MODEL_NUMBER

    // Initialize unused interfaces and pins
    (void)initIo();

    // Perform IO setup specific to Tracker One.  Reset the fuel gauge state-of-charge, check if under thresholds.
    if (_model == TRACKER_MODEL_TRACKERONE)
    {
        BLE.selectAntenna(BleAntennaType::EXTERNAL);
        initBatteryMonitor();
    }

    cloudService.init();

    configService.init();

    sleep.init([this](bool enable){ this->enableWatchdog(enable); });
    sleep.registerSleepPrepare([this](TrackerSleepContext context){ this->onSleepPrepare(context); });
    sleep.registerSleep([this](TrackerSleepContext context){ this->onSleep(context); });
    sleep.registerWake([this](TrackerSleepContext context){ this->onWake(context); });
    sleep.registerStateChange([this](TrackerSleepContext context){ this->onSleepStateChange(context); });

    // Register our own configuration settings
    registerConfig();

    ret = locationService.begin(_deviceConfig.enableFastLock());
    if (ret)
    {
        Log.error("Failed to begin location service");
    }

    // Check for Tracker One hardware
    if (_model == TRACKER_MODEL_TRACKERONE)
    {
        (void)GnssLedInit();
        GnssLedEnable(true);
        temperature_init(TRACKER_THERMISTOR,
            [this](TemperatureChargeEvent event){ return chargeCallback(event); }
        );
    }

    motionService.start();

    location.init(_deviceConfig.gnssRetryCount());

    motion.init();

    shipping.init();
    shipping.regShutdownBeginCallback(std::bind(&Tracker::stop, this));
    shipping.regShutdownIoCallback(std::bind(&Tracker::end, this));
    shipping.regShutdownFinalCallback(
        [this](){
            enableWatchdog(false);
            return 0;
        });

    rgb.init();

    enableWatchdog(true);

    // Associate handler to OTAs and pending resets to disable the watchdog
    System.on(reset_pending,
        [this](system_event_t event, int param){
            // Stop everything
            stop();
            end();
        }
    );
    System.on(firmware_update, [this](system_event_t event, int param){ otaHandler(event, param); });

    // Allow OTAs now that the firmware update handlers are registered
    System.enableUpdates();

    location.regLocGenCallback(loc_gen_cb);

    return SYSTEM_ERROR_NONE;
}

void Tracker::loop()
{
    uint32_t cur_sec = System.uptime();

    // slow operations for once a second
    if(_lastLoopSec != cur_sec)
    {
        _lastLoopSec = cur_sec;

#ifndef RTC_WDT_DISABLE
        hal_exrtc_feed_watchdog(nullptr);
#endif
    }

    // Evaluate low battery conditions
    if (_model == TRACKER_MODEL_TRACKERONE)
    {
        evaluateBatteryCharge();
    }

    // fast operations for every loop
    sleep.loop();
    motion.loop();

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


    // fast operations for every loop
    cloudService.tick();
    configService.tick();
    location.loop();
}

int Tracker::stop() {
    locationService.stop();
    motionService.stop();

    return SYSTEM_ERROR_NONE;
}

int Tracker::end() {
    enableIoCanPower(false);
    GnssLedEnable(false);
    enableWatchdog(false);

    return SYSTEM_ERROR_NONE;
}

int Tracker::reset() {
    stop();
    end();
    System.reset();

    return SYSTEM_ERROR_NONE;
}

int Tracker::enableCharging() {
    _batteryChargeEnabled = true;

    auto powerConfig = System.getPowerConfiguration();
    if (powerConfig.isFeatureSet(SystemPowerFeature::DISABLE_CHARGING)) {
        powerConfig.clearFeature(SystemPowerFeature::DISABLE_CHARGING);
        return System.setPowerConfiguration(powerConfig);
    }

    return SYSTEM_ERROR_NONE;
}

int Tracker::disableCharging() {
    _batteryChargeEnabled = false;

    auto powerConfig = System.getPowerConfiguration();
    if (!powerConfig.isFeatureSet(SystemPowerFeature::DISABLE_CHARGING)) {
        powerConfig.feature(SystemPowerFeature::DISABLE_CHARGING);
        return System.setPowerConfiguration(powerConfig);
    }

    return SYSTEM_ERROR_NONE;
}

int Tracker::setChargeCurrent(uint16_t current) {
    int ret = SYSTEM_ERROR_NONE;
    auto powerConfig = System.getPowerConfiguration();
    if (powerConfig.batteryChargeCurrent() != current) {
        powerConfig.batteryChargeCurrent(current);
        ret = System.setPowerConfiguration(powerConfig);
    }
    return ret;
}

int Tracker::chargeCallback(TemperatureChargeEvent event) {
    auto shouldCharge = true;

    switch (event) {
        case TemperatureChargeEvent::NORMAL: {
            setChargeCurrent(TrackerChargeCurrentHigh);
            shouldCharge = true;
            break;
        }

        case TemperatureChargeEvent::OVER_CHARGE_REDUCTION: {
            setChargeCurrent(TrackerChargeCurrentLow);
            shouldCharge = true;
            break;
        }

        case TemperatureChargeEvent::OVER_TEMPERATURE: {
            setChargeCurrent(TrackerChargeCurrentLow);
            shouldCharge = false;
            break;
        }

        case TemperatureChargeEvent::UNDER_TEMPERATURE: {
            setChargeCurrent(TrackerChargeCurrentLow);
            shouldCharge = false;
            break;
        }
    }

    // Check if anything needs to be changed for charging
    if (!shouldCharge && _batteryChargeEnabled) {
        disableCharging();
    }
    else if (shouldCharge && !_batteryChargeEnabled) {
        enableCharging();
    }

    return SYSTEM_ERROR_NONE;
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
