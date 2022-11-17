/************** INCLUDES ****************************/
#include "quecGNSS.h"


/************** DEFINES ****************************/

#define QUECTEL_CMD_WAIT_US                 10000
#define QUECTEL_STARTUP_WAIT_US             (QUECTEL_CMD_WAIT_US * 500)

#define LOG_DEVICE                          (Serial1)

/************** ENUMS ****************************/


/************** GLOBALS **************************/
static RecursiveMutex gps_mutex;
Logger qgnss_local_log("qgnss");



/************** CONSTANTS ************************/
static constexpr uint8_t  NMEA_START_DELIMITER         = '$';
static constexpr uint8_t  NMEA_END_CHAR_1              = '\n';
static constexpr uint32_t NMEA_MAX_LENGTH              = 82;

static constexpr uint32_t MAX_GPS_AGE_MS               = 10000; // GPS location must be newer than this to be considered valid
static constexpr uint32_t MAX_GPS_ACTIVE_TIME_MS       = 5000;

static constexpr double   STABILITY_HDOP_THRESHOLD     = 20.0;


/************** MACROS ***************************/
/**
 * Check the result of a predicate expression.
 *
 * @see @ref check_macros
 */
#define CHECK_TRUE(_expr, _ret) \
        do { \
            const bool _ok = (bool)(_expr); \
            if (!_ok) { \
                _LOG_CHECKED_ERROR(_expr, _ret); \
                return _ret; \
            } \
        } while (false)

#define LOCK()      std::lock_guard<RecursiveMutex> __gps_guard(gps_mutex);


/************** STATIC FUNCTIONS *******************/
/**
 * @brief NMEA parser wrapper function to get system time
 *
 */
static int getTstamp()
{
    return System.uptime();
}


/**
 * @brief NMEA parser callback function invoked when NMEA sentence is parsed
 *
 * @param gh handle to the NMEA parser
 * @param res return status to check if sentence was valid
 */
static void nmeaDisplay(gps_t* gh, gps_statement_t res)
{
    if(res != STAT_CHECKSUM_FAIL)
    {
        LOG_DEVICE.printf("%.*s\r\n", gh->sentence_len, gh->sentence);
    }
}



/************** CONSTRUCTOR ***********************/
quectelGPS::quectelGPS(TwoWire& bus, uint16_t powerPin, uint16_t wakeupPin) :
        _powerPin(powerPin),
        _wakeupPin(wakeupPin), 
        _lastLockTime(0),
        _startLockUptime(0),
        _isStable(false),
        _msgDone(true),
        _hdopStability(STABILITY_HDOP_THRESHOLD),
        _gpsParser({0}),
        _lockMethod(gpsLockMethod::HorizontalDop),
        _i2cDriver(new quectelGNSSCoreI2C(bus))
{ 
    // Clear the NMEA filter list
    _filterList = NMEA_SEN_NONE;

    // Start with an unitialized device
    _initialized = false;
    _running = false;
}




uint8_t quectelGPS::calculateChecksum(uint8_t *pData)
{
    uint8_t *n = pData + 1; // Plus one, skip '$'
    uint8_t chk = 0;

    // While current char isn't '*' or sentence ending (newline) 
    while ('*' != *n && NMEA_END_CHAR_1 != *n) 
    {
        if ( ('\0' == *n) || (n - pData > NMEA_MAX_LENGTH) )
        {
            // Sentence too long or short 
            return 0;
        }

        chk ^= (uint8_t) *n;
        n++;
    }

    return chk;
}


void quectelGPS::sendCommand(const char* cmd, uint32_t len)
{
    static uint32_t num_valid_sends = 0;
    //qgnss_local_log.info("sendCommand ");
    // Validate before sending command
    if( (nullptr != cmd) && (0 != len) ) {
        size_t total_sent = 0;
        uint8_t retry_count = 0;
        // Wait until the current I2C transaction is done before sending the command
        while(!_msgDone);

        while (retry_count < 2) {
            size_t incr_sent = 0;
            LOCK();

            auto rc = _i2cDriver->quectelDevTransmit((uint8_t *)(cmd+total_sent), (len-total_sent), &incr_sent);
            if (QuecDriverStatus::QDEV_SUCCESS  == rc) {
                total_sent += incr_sent;
                delayMicroseconds(QUECTEL_CMD_WAIT_US);
                if (total_sent >= len) {
                    num_valid_sends += 1;
                    break;
                }
                else {
                    // This retry works around a problem where the LC29H reports a receive
                    // buffer available size that doesn't reflect its actual data consumption. 
                    qgnss_local_log.trace("xmit continue: %u / %u", total_sent, len);
                }
            }
            else {
                qgnss_local_log.warn("sendCommand rc: %d after: %u ok", (int)rc, num_valid_sends);
                num_valid_sends = 0;
                delay(1);
                retry_count++;
            }
        }
    }
}

void quectelGPS::updateGPS(void)
{
    // Fetch GNSS data
    while (true)   {
        if( _running )  {
            // Read as much GNSS data as available
            while (true)  {
                // only lock ownership with the gnss receiver for the minimum time required
                LOCK();
                size_t bytesReceived = 0;
                memset(_msgBuf, 0, sizeof(_msgBuf));
                auto rc = _i2cDriver->quectelDevReceive(_msgBuf, sizeof(_msgBuf), &bytesReceived);

                // Display results
                if (( QuecDriverStatus::QDEV_SUCCESS == rc) && (bytesReceived > 0))  {
                    // qgnss_local_log.info("processing: %u", bytesReceived);
                    processGpsBytes(_msgBuf, bytesReceived);
                }
                else {
                    break;
                }
            }

            _msgDone = false;
        }

        // Give back context to the scheduler
        _msgDone = true;
        delay(1);
    }

    // Should never exit
}

void quectelGPS::processGpsBytes(const uint8_t* buf, size_t len) {
    gps_process(&_gpsParser, buf, len, _enableDiag ? nmeaDisplay : nullptr);
    if (!_initializing)  {
        if (getLock())   {
            _lastLockTime = Time.now() - (System.uptime() - _gpsParser.time_timestamp);
            _gpsStatus = gpsLedStatus::GPS_STATUS_LOCK;
        }
        else  {
            _gpsStatus = gpsLedStatus::GPS_STATUS_FIXING;
        }
    }

    processLockStability();

    _lastReceiveTime = millis();
}


void quectelGPS::processLockStability()
{
    if(!getLock())
    {
        _isStable = false;
        _startLockUptime = 0;
        return;
    }

    if(!_startLockUptime)
    {
        _startLockUptime = System.uptime();
    }

    if (gpsLockMethod::HorizontalDop == _lockMethod) 
    {
        _isStable = (getHDOP() < _hdopStability);
    } 
    else 
    {
        _isStable = true;
    }

}


/************** PUBLIC METHODS ****************************/
Dev_Resp_FlagStatus quectelGPS::quectelDevInit(void)
{
    // Configure the module power enable
	pinMode(_powerPin, OUTPUT);
    pinMode(_wakeupPin, OUTPUT);

    // Turn on module
    digitalWrite(_powerPin, HIGH);
    digitalWrite(_wakeupPin, LOW);

    // Momentarily wait before setting bus properties
    delayMicroseconds(QUECTEL_CMD_WAIT_US);

    // In the event that there was a reset to the hardware after
    // the GNSS module was put to sleep, there is no other way to
    // wake it up except via hardware signaling. This call will
    // ensure that the GNSS module will wake up after a board reset.
    quectelWakeup();

    // Flag the device as ready for commands
    _initialized = true;
    _initializing = true;
    _enableDiag = true;
    _gpsStatus = gpsLedStatus::GPS_STATUS_FIXING;
 
    // After startup of the module, it will be unresponsive
    // Wait for a finite period before allowing additional commands.
    delayMicroseconds(QUECTEL_STARTUP_WAIT_US);

    // Intialize the GPS parser
    gps_init(&_gpsParser, getTstamp);

    _lastReceiveTime = 0;
        
    if (nullptr != _i2cDriver) {
        _i2cDriver->waitGNSSOnline();
        if (! _i2cDriver->waitGNSSOnline()) {
            qgnss_local_log.error("waitGNSSOnline failed");
            return Dev_Resp_FlagStatus::DEV_REP_ERROR;
        }
    }


    // Start the data collection thread
    _gpsThread = new Thread("gps", [this]() { updateGPS(); }, OS_THREAD_PRIORITY_DEFAULT);

    _initializing = false;

    // Enable proprietary message reporting
    const char *cmd1 = "$PAIR6010,-1,1*20\r\n";  //set defaults
    sendCommand( cmd1, strlen(cmd1));
    delayMicroseconds(QUECTEL_CMD_WAIT_US);

    // Enable vehicle message reporting
    const char *cmd2 = "$PAIR6010,0,1*0C\r\n"; //open PQTMVEHMSG
    sendCommand( cmd2, strlen(cmd2));
    delayMicroseconds(QUECTEL_CMD_WAIT_US);

    // Enable sensor message reporting
    const char *cmd3 = "$PAIR6010,1,1*0D\r\n"; //open PQTMSENMSG
    sendCommand( cmd3, strlen(cmd3));
    delayMicroseconds(QUECTEL_CMD_WAIT_US);

    // Enable calibration state reporting
    const char *cmd4 = "$PAIR6010,2,1*0E\r\n"; //open PQTMDRCAL
    sendCommand( cmd4, strlen(cmd4));
    delayMicroseconds(QUECTEL_CMD_WAIT_US);

    // Enable IMU reporting
    const char *cmd5 = "$PAIR6010,3,1*0F\r\n"; //open PQTMIMUTYPE
    sendCommand( cmd5, strlen(cmd5));
    delayMicroseconds(QUECTEL_CMD_WAIT_US);

    // Enable motion reporting
    const char *cmd6 = "$PAIR6010,4,1*08\r\n"; //open PQTMVEHMOT
    sendCommand( cmd6, strlen(cmd6));
    delayMicroseconds(QUECTEL_CMD_WAIT_US);

    // Enable estimated error reporting
    const char *cmd7 = "$PAIR6010,5,1*09\r\n"; //open PQTMEPE
    sendCommand( cmd7, strlen(cmd7));
    delayMicroseconds(QUECTEL_CMD_WAIT_US);

    const char *cmd8 = "$PQTMCFGMSGRATE,W,PQTMEPE,1,2*1D\r\n"; //open PQTMEPE (2WD)
    sendCommand( cmd8, strlen(cmd8));
    delayMicroseconds(QUECTEL_CMD_WAIT_US);

    // Save the settings to NVRAM
    const char *cmd9 = "$PQTMSAVEPAR*5A\r\n"; //save to nvram
    sendCommand( cmd9, strlen(cmd9));
    delayMicroseconds(QUECTEL_CMD_WAIT_US);

    // Set the NMEA types to stream
    quectelSetFilters( NMEA_SEN_ALL );

    return Dev_Resp_FlagStatus::DEV_REP_SUCCESS; 
}

Dev_Resp_FlagStatus quectelGPS::quectelStart(void)
{
    _gpsStatus = gpsLedStatus::GPS_STATUS_FIXING;

    // Start the multi-threading
    _running = true;

    return Dev_Resp_FlagStatus::DEV_REP_SUCCESS;
}

Dev_Resp_FlagStatus quectelGPS::quectelStop(void)
{
    // Disable thread processing
    _running = false;

    // Disable reading of all the NMEA sentence types
    char *disableList[6] = 
                                {
                                    (char *)"$PAIR062,0,0*3E\r\n", // Disable NMEA_SEN_GGA
                                    (char *)"$PAIR062,1,0*3F\r\n", // Disable NMEA_SEN_GLL
                                    (char *)"$PAIR062,2,0*3C\r\n", // Disable NMEA_SEN_GSA
                                    (char *)"$PAIR062,3,0*3D\r\n", // Disable NMEA_SEN_GSV
                                    (char *)"$PAIR062,4,0*3A\r\n", // Disable NMEA_SEN_RMC
                                    (char *)"$PAIR062,5,0*3B\r\n"  // Disable NMEA_SEN_VTG
                                }; 

    for(uint32_t ii = 0; ii < 6; ii++)
    {
        const char *cmd = disableList[ii];

        // Send the command individually for each NMEA type
        sendCommand( cmd, strlen(cmd));
    }

    _gpsStatus = gpsLedStatus::GPS_STATUS_OFF;

    return Dev_Resp_FlagStatus::DEV_REP_SUCCESS;
}

Dev_Resp_FlagStatus quectelGPS::quectelSetFilters(uint8_t nmeaTypes)
{
    CHECK_TRUE(_initialized, Dev_Resp_FlagStatus::DEV_REP_ERROR);

    const char *cmd;

    // Save a shadow of the NMEA types that should be streamed
    _filterList = nmeaTypes;

    // Keep a map of the NMEA types
    enum 
    {
        IDX_GGA,
        IDX_GLL,
        IDX_GSA,
        IDX_GSV,
        IDX_RMC,
        IDX_VTG,
        IDX_MAX
    };

    // List of commands to read the various NMEA sentence types
    char *enableList[IDX_MAX] = 
                                {
                                    (char *)"$PAIR062,0,1*3F\r\n", // Enable NMEA_SEN_GGA
                                    (char *)"$PAIR062,1,1*3E\r\n", // Enable NMEA_SEN_GLL
                                    (char *)"$PAIR062,2,1*3D\r\n", // Enable NMEA_SEN_GSA
                                    (char *)"$PAIR062,3,1*3C\r\n", // Enable NMEA_SEN_GSV
                                    (char *)"$PAIR062,4,1*3B\r\n", // Enable NMEA_SEN_RMC
                                    (char *)"$PAIR062,5,1*3A\r\n"  // Enable NMEA_SEN_VTG
                                }; 

    // List of commands to disable reading of the various NMEA sentence types
    char *disableList[IDX_MAX] = 
                                {
                                    (char *)"$PAIR062,0,0*3E\r\n", // Disable NMEA_SEN_GGA
                                    (char *)"$PAIR062,1,0*3F\r\n", // Disable NMEA_SEN_GLL
                                    (char *)"$PAIR062,2,0*3C\r\n", // Disable NMEA_SEN_GSA
                                    (char *)"$PAIR062,3,0*3D\r\n", // Disable NMEA_SEN_GSV
                                    (char *)"$PAIR062,4,0*3A\r\n", // Disable NMEA_SEN_RMC
                                    (char *)"$PAIR062,5,0*3B\r\n"  // Disable NMEA_SEN_VTG
                                }; 

    // Set NMEA filters in the GNSS module	
    for( uint32_t ii = 0; ii < IDX_MAX; ii++ )
    {
        switch(ii)
        {
            case IDX_GGA:
                if(_filterList & NMEA_SEN_GGA)
                {
                    cmd = enableList[IDX_GGA];
                }
                else
                {
                    cmd = disableList[IDX_GGA];
                }
                break;
            case IDX_GLL:
                if(_filterList & NMEA_SEN_GLL)
                {
                    cmd = enableList[IDX_GLL];
                }
                else
                {
                    cmd = disableList[IDX_GLL];
                }
                break;
            case IDX_GSA:
                if(_filterList & NMEA_SEN_GSA)
                {
                    cmd = enableList[IDX_GSA];
                }
                else
                {
                    cmd = disableList[IDX_GSA];
                }
                break;
            case IDX_GSV:
                if(_filterList & NMEA_SEN_GSV)
                {
                    cmd = enableList[IDX_GSV];
                }
                else
                {
                    cmd = disableList[IDX_GSV];
                }
                break;
            case IDX_RMC:
                if(_filterList & NMEA_SEN_RMC)
                {
                    cmd = enableList[IDX_RMC];
                }
                else
                {
                    cmd = disableList[IDX_RMC];
                }
                break;
            case IDX_VTG:
                if(_filterList & NMEA_SEN_VTG)
                {
                    cmd = enableList[IDX_VTG];
                }
                else
                {
                    cmd = disableList[IDX_VTG];
                }
                break;
        }

        // Send the command individually for each NMEA type
        sendCommand( cmd, strlen(cmd));
    }

    return Dev_Resp_FlagStatus::DEV_REP_SUCCESS;
}

Dev_Resp_FlagStatus quectelGPS::quectelSetRate(fix_rate_t measRateHz)
{
    CHECK_TRUE(_initialized, Dev_Resp_FlagStatus::DEV_REP_ERROR);

    const char *cmd;
    std::string command;

    switch( measRateHz )
    {
        case RATE_1HZ:
            command = "$PAIR050,1000*12\r\n";
            cmd = (const char *)command.c_str();
            break;

        case RATE_10HZ:
            command = "$PAIR050,100*22\r\n";
            cmd = (const char *)command.c_str();
            break;

        default:
            return Dev_Resp_FlagStatus::DEV_REP_ERROR;
    };

    sendCommand( cmd, strlen(cmd));

    return Dev_Resp_FlagStatus::DEV_REP_SUCCESS;
}

Dev_Resp_FlagStatus quectelGPS::quectelResetGnss()
{
    CHECK_TRUE(_initialized, Dev_Resp_FlagStatus::DEV_REP_ERROR);

    const char *cmd = "$PAIR007*3D\r\n";

    // Send the command
    sendCommand( cmd, strlen(cmd));

    // Restart the module
    delayMicroseconds(QUECTEL_CMD_WAIT_US);
    quectelStart();

    return Dev_Resp_FlagStatus::DEV_REP_SUCCESS;
}

Dev_Resp_FlagStatus quectelGPS::quectelSaveSettings()
{
    CHECK_TRUE(_initialized, Dev_Resp_FlagStatus::DEV_REP_ERROR);

    const char *cmd = "$PAIR513*3D\r\n";

    // Send the command
    sendCommand( cmd, strlen(cmd));

    return Dev_Resp_FlagStatus::DEV_REP_SUCCESS;
}

Dev_Resp_FlagStatus quectelGPS::quectelModulePower(bool state)
{
    CHECK_TRUE(_initialized, Dev_Resp_FlagStatus::DEV_REP_ERROR);

    if( true == state )
    {
        // Turn on module
        digitalWrite(_powerPin, HIGH);

        // Momentarily wait before setting bus properties
        delayMicroseconds(QUECTEL_CMD_WAIT_US);

        // Enable thread processing
        _running = true;
    }
    else
    {
        // Turn off module
        digitalWrite(_powerPin, LOW);

        // Disable thread processing
        _running = false;
    }

    return Dev_Resp_FlagStatus::DEV_REP_SUCCESS;
}

Dev_Resp_FlagStatus quectelGPS::quectelLowPower()
{
    CHECK_TRUE(_initialized, Dev_Resp_FlagStatus::DEV_REP_ERROR);

    const char *cmd = "$PAIR650,0*25\r\n";

    // Send the command
    sendCommand( cmd, strlen(cmd));
   
    return Dev_Resp_FlagStatus::DEV_REP_SUCCESS;
}

Dev_Resp_FlagStatus quectelGPS::quectelWakeup()
{
    // Toggle the wakeup pin (LOW->HIGH->LOW) for 10ms
    digitalWrite(_wakeupPin, HIGH);
    delayMicroseconds(QUECTEL_CMD_WAIT_US);
    digitalWrite(_wakeupPin, LOW);

    return Dev_Resp_FlagStatus::DEV_REP_SUCCESS;
}

Dev_Resp_FlagStatus quectelGPS::quectelStartMethod(gpsStartType startMode)
{
    CHECK_TRUE(_initialized, Dev_Resp_FlagStatus::DEV_REP_ERROR);

    const char *cmd;
    std::string command;

    switch( startMode )
    {
        case gpsStartType::FullCold:
            command = "$PAIR007*3D\r\n";
            cmd = (const char *)command.c_str();
            break;
        case gpsStartType::Cold:
            command = "$PAIR006*3C\r\n";
            cmd = (const char *)command.c_str();
            break;
        case gpsStartType::Warm:
            command = "$PAIR005*3F\r\n";
            cmd = (const char *)command.c_str();
            break;
        case gpsStartType::Hot:
            command = "$PAIR004*3E\r\n";
            cmd = (const char *)command.c_str();
            break;
        default:
            return Dev_Resp_FlagStatus::DEV_REP_ERROR;
    };

    sendCommand( cmd, strlen(cmd));

    // Restart the module
    delayMicroseconds(QUECTEL_CMD_WAIT_US);
    quectelStart();

    return Dev_Resp_FlagStatus::DEV_REP_SUCCESS;
}

Dev_Resp_FlagStatus quectelGPS::quectelSaveLocationData()
{
    CHECK_TRUE(_initialized, Dev_Resp_FlagStatus::DEV_REP_ERROR);

    const char *cmd = "$PAIR511*3F\r\n";

    // Send the command
    sendCommand( cmd, strlen(cmd));
   
    return Dev_Resp_FlagStatus::DEV_REP_SUCCESS;
}

double quectelGPS::getLatitude(void)
{
    return _gpsParser.latitude;
}

double quectelGPS::getLongitude(void)
{
    return _gpsParser.longitude;
}

float quectelGPS::getAltitude(void)
{
    return _gpsParser.altitude;
}

uint8_t quectelGPS::getFixQuality(void)
{
    //  0 = No fix,
    //  1 = Autonomous GNSS fix,
    //  2 = Differential GNSS fix,
    //  4 = RTK fixed,
    //  5 = RTK float,
    //  6 = Estimated/Dead reckoning fix
    return _gpsParser.fix;
}

bool quectelGPS::getLock(void)
{
    //LOCK();
    return ( (0 != _gpsParser.fix) || (0 != _gpsParser.is_valid));
    // return (_gpsParser.fix &&
    //         _gpsParser.pos_timestamp &&
    //     System.uptime() - _gpsParser.pos_timestamp < (MAX_GPS_AGE_MS / 1000));
}

uint32_t quectelGPS::getLockTime(void)
{
    return _lastLockTime;
}

float quectelGPS::getSpeed(uint8_t unit)
{
    constexpr float KNOTS_TO_MPS = 0.51444444444444;
    constexpr float KNOTS_TO_MPH = 1.1507794480235;
    constexpr float KNOTS_TO_KMPH = 1.852;
    float speed = _gpsParser.speed; // in knots

    switch ((gpsSpeedUnit)unit) {
    default:
    case gpsSpeedUnit::GPS_SPEED_UNIT_MPS:
        speed *= KNOTS_TO_MPS;
        break;
    case gpsSpeedUnit::GPS_SPEED_UNIT_MPH:
        speed *= KNOTS_TO_MPH;
        break;
    case gpsSpeedUnit::GPS_SPEED_UNIT_KMPH:
        speed *= KNOTS_TO_KMPH;
        break;
    }
    return speed;
}

float quectelGPS::getHeading(void)
{
    return _gpsParser.course;
}


uint32_t quectelGPS::getDate(void)
{
    // LOCK();
    return _gpsParser.year * 10000 + _gpsParser.month * 100 + _gpsParser.date;
}


uint32_t quectelGPS::getTime(void)
{
    // LOCK();
    return _gpsParser.hours * 10000 + _gpsParser.minutes * 100 + _gpsParser.seconds;
}


uint32_t quectelGPS::getUTCTime()
{
    struct tm t;

    memset(&t, 0, sizeof(t));
    WITH_LOCK(*this) {
        if (_gpsParser.time_valid && _gpsParser.date_valid) {
            t.tm_hour = _gpsParser.hours;
            t.tm_min = _gpsParser.minutes;
            t.tm_sec = _gpsParser.seconds;
            t.tm_mday = _gpsParser.date;
            // struct tm expects 0-11 mon, nmea parser gives 1-12
            t.tm_mon = _gpsParser.month - 1;
            // struct tm expect years since 1900, nmea parser gives year since 2000
            t.tm_year = _gpsParser.year + 100;
        }
    }

    return mktime(&t);
}


uint8_t quectelGPS::getSatellites(gps_sat_t desc[], uint32_t &numSatInView)
{
    numSatInView = _gpsParser.sats_in_view;

    for( uint32_t ii = 0; ii < numSatInView; ii++ )
    {
        desc[ii].num       = _gpsParser.sats_in_view_desc[ii].num,
        desc[ii].elevation = _gpsParser.sats_in_view_desc[ii].elevation,
        desc[ii].azimuth   = _gpsParser.sats_in_view_desc[ii].azimuth,
        desc[ii].snr       = _gpsParser.sats_in_view_desc[ii].snr;
        desc[ii].used      = _gpsParser.sats_in_view_desc[ii].used;
    }

    return _gpsParser.sats_in_use;
}


double quectelGPS::getGeoIdHeight(void)
{
    //the geoid's variation ranges from +85 m (Iceland) to −106 m (southern India)
    return (double) _gpsParser.geo_sep;
}


double quectelGPS::getHDOP(void)
{
    return (double) _gpsParser.dop_h;
}


double quectelGPS::getVDOP(void)
{
    return (double) _gpsParser.dop_v;
}


uint8_t quectelGPS::getSignalStrength(void)
{
    // take the initial satellites average cno as the signal strength
    LOCK();
    constexpr int MAX_SAT_COUNT = 4;
    uint16_t avgSignal = 0;
    int count = std::min((int)_gpsParser.sats_in_view, MAX_SAT_COUNT);
    int counted = 0;

    for (int i = 0; i < _gpsParser.sats_in_view && counted < count; i++)
    {
        if(_gpsParser.sats_in_view_desc[i].used)
        {
            avgSignal += _gpsParser.sats_in_view_desc[i].snr;
            counted++;
        }
    }
    if(counted)
    {
        avgSignal /= counted;
    }

    return (uint8_t)avgSignal;
}


double quectelGPS::getHorizontalAccuracy(void)
{
    return _gpsParser.epe_2d;
}


double quectelGPS::getVerticalAccuracy(void)
{
    return _gpsParser.v_accuracy;
}


float quectelGPS::getDistance(double lat1, double long1, double lat2, double long2)
{
    //default unit is m(meter)
    gps_float_t distance, bearing;
    gps_distance_bearing(lat1, long1, lat2, long2, &distance, &bearing);
    return distance;
}


void quectelGPS::lock(void)
{
    gps_mutex.lock();
}


void quectelGPS::unlock(void)
{
    gps_mutex.unlock();
}


bool quectelGPS::isActive(void)
{
    switch (_gpsStatus) 
    {
        case gpsLedStatus::GPS_STATUS_OFF:
        case gpsLedStatus::GPS_STATUS_ERROR:
            return false;
    }

    return( (millis() - _lastReceiveTime) < MAX_GPS_ACTIVE_TIME_MS );
}


bool quectelGPS::isLockStable(void)
{
    LOCK();

    return getLock() && _isStable;
}


uint32_t quectelGPS::getLockDuration(void)
{
    LOCK();

    if(!getLock())
    {
        return 0;
    }

    return System.uptime() - _startLockUptime;
}


uint8_t quectelGPS::getGpsStatus(void)
{
    return (uint8_t)_gpsStatus;
}


uint8_t quectelGPS::getCalibrationState(void)
{
    return _gpsParser.cal_state;
}


uint8_t quectelGPS::getFixMode(void)
{
    return _gpsParser.fix_mode;
}


char quectelGPS::getModeIndicator(void)
{
    return _gpsParser.mode_ind;
}

const char* quectelGPS::getLastGGA() {
    return _gpsParser.last_gga_sentence;
}

bool quectelGPS::isOn(void)
{
    return _running;
}


void quectelGPS::writeBytes(const char *buf, uint16_t len)
{
    bool was_running = _running;
    // Disable polling before the command is sent
    _running = false;

    // Send the command and get the response
    sendCommand( buf, len );

    // Re-enable polling 
    _running = was_running;
}