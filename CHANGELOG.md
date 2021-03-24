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
