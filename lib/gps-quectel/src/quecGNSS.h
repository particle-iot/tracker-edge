#ifndef __QUECTELGPS_H
#define __QUECTELGPS_H

//**************** INCLUDES ******************
#include "Particle.h"
#include "quectelGNSSCoreI2C.h"
#include "gps/gps.h"


//**************** CONSTANTS *****************


//*************** ENUMS *********************
enum class Dev_Resp_FlagStatus
{
    DEV_REP_SUCCESS = 0,
    DEV_REP_ERROR = 1
};


enum class gpsLockMethod 
{
    Lock,
    HorizontalAccuracy,
    HorizontalDop,
};

enum class gpsStartType
{
    FullCold,
    Cold,
    Warm,
    Hot
};

typedef enum
{
    FEATURE_DISABLE,
    FEATURE_ENABLE
} Feature_t;

typedef enum 
{
    NMEA_SEN_NONE = 0x00,
    NMEA_SEN_GGA  = 0x01,
    NMEA_SEN_GLL  = 0x02,
    NMEA_SEN_GSA  = 0x04,
    NMEA_SEN_GSV  = 0x08,
    NMEA_SEN_RMC  = 0x10,
    NMEA_SEN_VTG  = 0x20,
    NMEA_SEN_BASIC  = (NMEA_SEN_GGA | NMEA_SEN_GLL | NMEA_SEN_RMC | NMEA_SEN_VTG),
    NMEA_SEN_ALL  = (NMEA_SEN_GGA | NMEA_SEN_GLL | NMEA_SEN_GSA | NMEA_SEN_GSV | NMEA_SEN_RMC | NMEA_SEN_VTG)
} NMEA_Sentence_t;

enum class gpsLedStatus 
{
    GPS_STATUS_OFF,
    GPS_STATUS_FIXING,
    GPS_STATUS_LOCK,
    GPS_STATUS_ERROR,
};

enum class gpsSpeedUnit
{
    GPS_SPEED_UNIT_MPS = 0, // m/s
    GPS_SPEED_UNIT_MPH,     // m/h
    GPS_SPEED_UNIT_KMPH     // km/h
};

typedef enum 
{
    RATE_1HZ,
    RATE_10HZ
} fix_rate_t;


//*************** CLASS *********************
class quectelGPS
{

public:
    /**
     * @brief Construct a new quectelGPS object attached to an I2C bus
     *
     * @param i2c Reference to specific i2c bus
     * @param powerPin Power control pin
     * @param wakeupPin Wakeup from low-power pin
     */
    quectelGPS(TwoWire& bus, uint16_t powerPin, uint16_t wakeupPin);

    /**
     * @brief Enables or disables power to GNSS module 
     *
     * @param none 
     */
    Dev_Resp_FlagStatus quectelDevInit(bool reInit = false);

    /**
     * @brief Start data collection
     *
     */
    Dev_Resp_FlagStatus quectelStart(void);

    /**
     * @brief Stop data collection
     *
     */
    Dev_Resp_FlagStatus quectelStop(void);

    /**
     * @brief Set which NMEA sentence-types to be streamed
     *
     * @param nmeaTypes Bit settings for which NMEA sentences to enable/disable
     */
    Dev_Resp_FlagStatus quectelSetFilters(uint8_t nmeaTypes);

    /**
     * @brief Set NMEA sentence rate
     *
     * @param measRateHz Fix rate in Hz
     */
    Dev_Resp_FlagStatus quectelSetRate(fix_rate_t measRateHz);

    /**
     * @brief Perform cold-start reset
     *
     */
    Dev_Resp_FlagStatus quectelResetGnss(void);

    /**
     * @brief Save settings to NVRAM
     *
     */
    Dev_Resp_FlagStatus quectelSaveSettings(void);

    /**
     * @brief Enable or disable power to GNSS module
     *
     * @param state true = turn on module, false = turn off module
     */
    Dev_Resp_FlagStatus quectelModulePower(bool state);

    /**
     * @brief Enter low-power mode. 
     *        NOTE: The only way to wake up the
     *        module is to use the quectelWakeup() API
     *
     */
    Dev_Resp_FlagStatus quectelLowPower();

    /**
     * @brief Wakeup from low-power mode
     *
     */
    Dev_Resp_FlagStatus quectelWakeup();    

    /**
     * @brief Perform a module restart
     *
     * @param method type of restart
     */
    Dev_Resp_FlagStatus quectelStartMethod(gpsStartType startMode);

    /**
     * @brief Save the latest navigation data to flash
     *
     * @param method type of restart
     */
    Dev_Resp_FlagStatus quectelSaveLocationData();

    /**
     * @brief Return the last parsed latitude reading
     *
     */
    double getLatitude(void);

    /**
     * @brief Return the last parsed longitude reading
     *
     */
    double getLongitude(void);

    /**
     * @brief Return the last parsed altitude reading
     *
     */
    float getAltitude(void);

    /**
     * @brief Return the last calcuated fix quality reading
     *
     */
    uint8_t getFixQuality(void);

    /**
     * @brief Return the satellite lock status
     *
     */
    bool getLock(void);

    /**
     * @brief Return the last time a satellite lock was detected
     *
     */
    uint32_t getLockTime(void);

    /**
     * @brief Return the last parsed speed
     *
     */
    float getSpeed(uint8_t unit);

    /**
     * @brief Return the last parsed heading reading
     *
     */
    float getHeading(void);

    /**
     * @brief Return the last parsed date
     *
     */
    uint32_t getDate(void);

    /**
     * @brief Return the last parsed timestamp
     *
     */
    uint32_t getTime(void);

    /**
     * @brief Return the current UTC time
     *
     */
    uint32_t getUTCTime();

    /**
     * @brief Return the current number of satellites used for a fix
     *
     * @param desc array of satellite descriptors
     * @param numSatsInView number of satellites in array of satellite descriptors
     */
    uint8_t getSatellites(gps_sat_t desc[], uint32_t &numSatsInView);

    /**
     * @brief Return the GeoID height
     *
     */
    double getGeoIdHeight(void);

    /**
     * @brief Return the last horizontal dilution of precision
     *
     */
    double getHDOP(void);

    /**
     * @brief Return the last vertical dilution of precision
     *
     */
    double getVDOP(void);

    /**
     * @brief Return the last RSSI
     *
     */
    uint8_t getSignalStrength(void);

    /**
     * @brief Return the last horizontal accuracy
     *
     */
    double getHorizontalAccuracy(void);

    /**
     * @brief Return the last vertical accuracy
     *
     */
    double getVerticalAccuracy(void);

    /**
     * @brief Return the distance from the previous location reading
     *
     */
    float getDistance(double lat1, double long1, double lat2, double long2);

    /**
     * @brief Take a mutex lock
     *
     */
    void lock(void);

    /**
     * @brief Release a mutex lock
     *
     */
    void unlock(void);

    /**
     * @brief Is the GNSS receiver actively reporting data
     *
     */
    bool isActive(void);

    /**
     * @brief Is the GPS lock consistently stable
     *
     */
    bool isLockStable(void);

    /**
     * @brief Return the length of time a GPS lock has occurred
     *
     */
    uint32_t getLockDuration(void);

    /**
     * @brief Return the GPS fix state
     *
     */
    uint8_t getGpsStatus(void);

    /**
     * @brief Return the GPS module calibration state
     *
     */
    uint8_t getCalibrationState(void);

    /**
     * @brief Return the GPS fix mode (ie 2D, 3D)
     *
     */
    uint8_t getFixMode(void);

    /**
     * @brief Return the GPS mode indicator
     *
     */
    char getModeIndicator(void);

    /**
     * @brief Send raw bytes to GNSS device
     *
     * @param buf pointer to command/raw bytes
     * @param len number of bytes in command
     */
    void writeBytes(const char *buf, uint16_t len);

    /**
     * @brief Return the most recent GGA NMEA sentence
     *
     */
    const char* getLastGGA();

    /**
     * @brief Return the power state of the GNSS module
     *
     */
    bool isOn(void);

private:
    // Variables
    uint16_t        _powerPin;
    uint16_t        _wakeupPin;
    uint8_t         _filterList;
    uint32_t        _lastLockTime;
    uint32_t        _lastReceiveTime;
    uint32_t        _startLockUptime;
    bool            _enableDiag;
    bool            _initialized;
    bool            _initializing;
    bool            _running;
    bool            _isStable;
    bool            _msgDone;
    double          _hdopStability;
    gpsLedStatus    _gpsStatus;
    gps_t           _gpsParser;
    gpsLockMethod   _lockMethod;
    Thread*         _gpsThread;
    uint8_t         _msgBuf[512]; //TODO this size is based on i2c performance at 400 kHz

    quectelGNSSCoreI2C*      _i2cDriver;
    // Functions
  

    /**
     * @brief Calcuation of NMEA protocol checksum
     *
     * @param data pointer to NMEA-format sentence
     */
    uint8_t calculateChecksum(uint8_t *pData);

    /**
     * @brief NMEA-formatted command to send to Quectel device
     *
     * @param cmd pointer to NMEA protocol command to be sent to module
     * @param len number of bytes in cmd
     */
    void sendCommand(const char* cmd, uint32_t len);


    /**
     * @brief Thread function to read Quectel GNSS module
     *
     */
    void updateGPS(void);

    /**
     * @brief Hand off data to the parser
     *
     */
    void processGpsBytes(const uint8_t* buf, size_t len);


    /**
     * @brief Perform stability check of reading
     *        MUST BE CALLED WITH GPS LOCK ALREADY HELD
     *
     */
    void processLockStability(void);

};

#endif // __QUECTELGPS_H