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
