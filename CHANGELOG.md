## v19

### COMPATIBILTY

Must be built using device OS v3.2.0 or greater.

### FEATURES

- Added satellite diagnostics configuration to publish satellite count and mean CN0.
- Updated battery model for more accurate state-of-charge

### ENHANCEMENTS

- Source can be built for device OS 3.2.0 to 6.x.

### BUGFIXES

- Fixed thermistor characteristics for more accuracy.
- Fixed excessive shutdown current for GNSS ephemeris battery during shipping mode.
- Fixed Ublox M8U fix stability stability check to exclude bad acquistions.

### OTHER CHANGES

- Some function and file cleanup to allow for abstraction and organization of source code.
- Use of C++ templates to replace selected function pointer callbacks.


## v18

### COMPATIBILTY

Must be built using device OS v3.2.0 or greater.

### FEATURES

- Store and Forward feature for saving publish data while offline.
- Added support for Memfault device monitoring.
- Added battery charging configuration overrides.

### ENHANCEMENTS

- Added a new log category for TrackerSleep class.
- Added more control for GNSS module dead reckoning.
- Added GNSS enhancements for time-to-first-fix (TTFF).

### BUGFIXES

- Fix setting of HDOP to geofence zones.

### OTHER CHANGES

- Add configuration schema file to the project.
- Cleared several compile warnings.
- Changes for building under device OS versions above 3.x.


## v17

### COMPATIBILTY

Must be built using device OS v3.2.0 or greater.

### FEATURES

- Geofence feature with settings for 4 circular zones.

### ENHANCEMENTS

- Replaced application AM1805 RTC driver with new device OS watchdog calls.

### BUGFIXES

- Disable the hardware watchdog whenever performing firmware updates over OTA.

### OTHER CHANGES

- [TRACKER ONE] Changed the low battery evaluation interval to catch low state-of-charge sooner.


## v16

### COMPATIBILTY

Must be built using device OS v3.0.0 or greater.

### FEATURES

- Additional setting and implementation for GNSS initialization retries if failures are detected.

### ENHANCEMENTS

- The ublox GPS library has been updated to provide performance counters.

### BUGFIXES

- Fix to calculating sleep and execution times when the device is in areas of low connectivity.

### OTHER CHANGES

- Switched several logging calls from the LOG() macro to the functional Log() method.


## v15

### COMPATIBILTY

Must be built using device OS v3.0.0 or greater.

### FEATURES

- Additional setting for GNSS lock stability criteria based on HDOP rather than using existing horizonatal accuracy.
- Both HDOP and VDOP figures were added to outgoing location publishes.

### ENHANCEMENTS

- The CAN bus library can be initialized in listen-only mode.
- GPS ublox driver changes to detect errors during initialization and return immediately upon such errors.
- An additional trigger named "err" will be sent in location publishes if GNSS module errors are encountered.
- [TRACKER ONE] The GPS LED will flash rapidly if GNSS module errors are encountered.
- WiFi scanning time has been shortened in order to reduce power usage.

### BUGFIXES

- GNSS module power on and off sequence changes for modules that may power up in the wrong interface configuration.


## v14

### COMPATIBILTY

Must be built using device OS v3.0.0-rc.2 or greater.

### FEATURES

No new features.

### ENHANCEMENTS

No new enhancements.

### BUGFIXES

- Fuel gauge reports incorrect battery charge during sleep.


## v13

### COMPATIBILTY

Must be built using device OS v3.0.0-rc.2 or greater.

### FEATURES

- Added reset command from cloud.

### ENHANCEMENTS

- Updated ublox GNSS drivers to allow UDR model changes.

### BUGFIXES

- Increased I2C timeouts for the AM1805 RTC driver as a precaution against crashes.
- [TRACKER ONE] Fixed GNSS LED state to make sure it is off when going into shipping mode.


## v12

### COMPATIBILTY

Must be built using device OS v3.0.0-rc.2 or greater.

### FEATURES

- Added enhanced location services (Location Fusion).
- Added GNSS speed to location publishes.
- Added dynamic charge current control over temperature.
- [TRACKER ONE] Added IO/CAN power controls for configuration in application.

### ENHANCEMENTS

- Changed priorities of first and immediate publishes.
- Reduced application footprint by migrating to wiring library mutexes.
- Allow device OS to control battery charge enablement.

### BUGFIXES

- Fixed issue of default chip select SPI initialization effect on D5.
- Fixed power management issue with incorrectly written DCT values.
- Fixed issue with multiple publishes at boot.


## v11

### COMPATIBILTY

Must be built using device OS v2.0.0-rc.3 or greater.

### FEATURES

- Added configurable setting to enable or disable parsing of location publish acknowledgements from the cloud.

### ENHANCEMENTS

- Changed CAN library enumerations for setting power modes so they do not collide with symbols in other device driver libraries.
- Changed CAN library so that cloud compiles are possible.

### BUGFIXES

- Fixed issue of no location publishes when GNSS signal quality is low and out-of-lock.
- [TRACKER ONE] Changed BLE antenna configuration from internal to external.


## v10

### COMPATIBILTY

Must be built using device OS v2.0.0-rc.3 or greater.

### FEATURES

- Added sleep feature to place device into ultra low power mode sleep and wake periodically for timed and triggered events.

### ENHANCEMENTS

- Improved GPS lock status with a stability check of horizontal accuracy.  Location publishes will be held off until the accuracy is stable.
- Placed CAN and ESP32 devices into low power mode to reduce overall power consumption
- Enabled the RTC watchdog by default
- [TRACKER ONE] Lowered low battery detect shutoff from 8% state-of-charge to 2% to gain more operational time.  Lowered battery warning from 15% to 8%.

### BUGFIXES

- [TRACKER ONE] Fixed intermittent issues with battery charge enablement/disablement when lower than 0 degrees C and above 50 degrees C


## v9

### FEATURES

- Added lock trigger that signifies changes in GNSS lock status
- Added CAN bus library as submodule
- [TRACKER ONE] Disable battery charging when temperatures fall below 0 degrees C or rise above 50 degrees C in order to protect battery outside of operational temperatures
- [TRACKER ONE] Force device to enter shipping mode when battery charge falls below 8% and publish location message with battery low trigger.  Publish location message with battery warning when charge falls below 15%.

### ENHANCEMENTS

- Improved ease-of-use in git clones of repository submodules by shifting from SSH to HTTPS URL links
- Changed default minimum and maximum publish intervals to 15 minutes and 1 hour respectively in order to conserve user data limits

### BUGFIXES

- Fixed BMI160 (IMU) power consumption when placing devce into suspend mode (sleep)
- Fixed BMI160 (IMU) SPI bus access timing differences when operating in normal versus low power and suspend modes
- Fixed ublox GNSS module library warnings for packed vs unpacked structure casting
- Fixed issue with parsing input of malformed JSON configuration commands
- Fixed BMI160 (IMU) configuration settings to return to default values upon factory reset


## v8

### FEATURES

- Detect and differentiate Tracker One and evaluation board products
- Receive USB bus messages to configure parameters or command firmware

### ENHANCEMENTS

- Changed several tracker classes to singleton for ease-of-use

### BUGFIXES

 - Fixed memory leak with cloud service messages to be published
 - Fixed memory leak related to GNSS power cycling


## v7

### FEATURES

- Initial repository for Tracker application reference source
- Publish location on minimum/maximum publish interval times
- Publish location on motion events detected on the Inertial Measurement Unit (IMU)
- Publish location on high g events detected on the IMU
- Publish location on radius boundary since the last publish

### ENHANCEMENTS


### BUGFIXES
