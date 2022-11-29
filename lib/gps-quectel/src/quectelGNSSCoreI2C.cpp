/************** INCLUDES ****************************/
#include "Particle.h"

#include "quectelGNSSCoreI2C.h"

/************** DEFINES ****************************/
#define QUECTEL_I2C_SLAVE_CR_CMD            0xaa51
#define QUECTEL_I2C_SLAVE_CW_CMD            0xaa53

#define QUECTEL_I2C_SLAVE_CMD_LEN           8
#define QUECTEL_I2C_INIT_CMD_LEN            4
#define QUECTEL_I2C_CMD_RESP_LEN           4
#define QUECTEL_I2C_SLAVE_TX_LEN_REG_OFFSET 0x08
#define QUECTEL_I2C_SLAVE_TX_BUF_REG_OFFSET 0x2000

#define QUECTEL_I2C_SLAVE_RX_LEN_REG_OFFSET 0x04
#define QUECTEL_I2C_SLAVE_RX_BUF_REG_OFFSET 0x1000

#define QUECTEL_I2C_SLAVE_ADDRESS_CR_OR_CW  0x50
#define QUECTEL_I2C_SLAVE_ADDRESS_R         0x54
#define QUECTEL_I2C_SLAVE_ADDRESS_W         0x58

#define MAX_ERROR_NUMBER                    1

#define QUECTEL_TRANSACTION_DELAY_US        5000

#define LOG_DEVICE                          (Serial1)

/************** ENUMS ****************************/


/************** GLOBALS **************************/

Logger i2c_local_log("qi2c");

// Re-definition of weak function to specify a larger
// I2C read buffer.  The default is 32 bytes but this
// fuction overrides and sets it to 512 bytes. DeviceOS
// uses this function to initialize the I2C driver.
// The Quectel GNSS driver requires read block size to be
// a minimum of 512 bytes.
static constexpr uint32_t I2C_READ_BUF_SIZE = 512;
#if (PLATFORM_ID == PLATFORM_TRACKER)
hal_i2c_config_t acquireWire1Buffer()
#elif (PLATFORM_ID == PLATFORM_TRACKERM)
hal_i2c_config_t acquireWireBuffer() 
#endif 
{
    hal_i2c_config_t config = {
        .size = sizeof(hal_i2c_config_t),
        .version = HAL_I2C_CONFIG_VERSION_1,
        .rx_buffer = new (std::nothrow) uint8_t[I2C_READ_BUF_SIZE],
        .rx_buffer_size = I2C_READ_BUF_SIZE,
        .tx_buffer = new (std::nothrow) uint8_t[I2C_READ_BUF_SIZE],
        .tx_buffer_size = I2C_READ_BUF_SIZE
    };
    return config;
}


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



/************** CONSTRUCTOR ***********************/
quectelGNSSCoreI2C::quectelGNSSCoreI2C(TwoWire& bus):
    _i2c(bus)
{

}


/************** PRIVATE METHODS ****************************/

I2c_Resp_FlagStatus quectelGNSSCoreI2C::i2cMasterReceive(uint8_t addr, uint8_t* pBuff, size_t reqLen, size_t* pReadLen)
{
    *pReadLen = 0;
    // Validate inputs
    if( (nullptr == pBuff) || (0 == reqLen) )  {
        return I2C_NACK;
    }

    WITH_LOCK(_i2c) // Other devices are on this bus using this same global handle
    {
        // Read the requested amount of data from the specified address
        size_t requestedBytes = _i2c.requestFrom(addr, reqLen);
        if (requestedBytes != reqLen) {
            // The request for data from the slave was NAK'd
            return I2C_NACK;
        }

        size_t nread = _i2c.readBytes((char*)pBuff, reqLen);
        if (nread != reqLen) {
            i2c_local_log.warn("nreq: %u nread: %u", reqLen, nread);
        }
        *pReadLen = nread;
    }

    return I2C_ACK;
}

I2c_Resp_FlagStatus quectelGNSSCoreI2C::i2cMasterTransmit(uint8_t addr, const uint8_t* pData, size_t length)
{
    // Validate inputs
    if( (nullptr == pData) || (0 == length) ) {
        return I2C_NACK;
    }

    WITH_LOCK(_i2c) // Other devices are on this bus using this same global handle
    {
        // Write data to the specified address
        _i2c.beginTransmission(addr);

        int nwrite = _i2c.write(pData, length);
        if (nwrite != (int)length) {
            i2c_local_log.warn("nwrite %u < len %u", nwrite, length);
            return I2C_NACK;
        }

        int etrc = _i2c.endTransmission();
        switch (etrc) {
            case 0:
                break;
            case 3:
                i2c_local_log.warn("i2c tx timeout");
                return I2C_NACK;
                break;
            default:
                i2c_local_log.warn("i2c tx err: %d on send: %u", etrc, length);
                return I2C_NACK;
                break;
        }
    }

    return I2C_ACK;
}

QuecDriverStatus quectelGNSSCoreI2C::quectelDevReceive(uint8_t* pData, size_t maxLength, size_t* pRecLength)
{
    uint32_t request_cmd[2];
    size_t received_len = 0;
    size_t avail_len = 0;
    size_t nreq = 0;
    static size_t prev_avail_max = 0;

    uint8_t i2c_master_receive_error_counter = 0;
    I2c_Resp_FlagStatus status;
    
    *pRecLength = 0;

    //step 1_a
    request_cmd[0] = (uint32_t)((uint32_t)(QUECTEL_I2C_SLAVE_CR_CMD << 16) | QUECTEL_I2C_SLAVE_TX_LEN_REG_OFFSET);
    request_cmd[1] = 4; 

    while(true) {
        delayMicroseconds(QUECTEL_TRANSACTION_DELAY_US);
        status = i2cMasterTransmit(QUECTEL_I2C_SLAVE_ADDRESS_CR_OR_CW, (uint8_t *)request_cmd, QUECTEL_I2C_SLAVE_CMD_LEN);
        if(status == I2C_ACK)  {
            break;
        }
        else {
            Log.warn("Configure read cmd failed");
            return QuecDriverStatus::QDEV_ERROR;
        }
    }

    //step 1_b
    while(true)  {
        delayMicroseconds(QUECTEL_TRANSACTION_DELAY_US);
        status = i2cMasterReceive(QUECTEL_I2C_SLAVE_ADDRESS_R, (uint8_t*)&avail_len, QUECTEL_I2C_CMD_RESP_LEN, &received_len);
        if ((status == I2C_ACK) && (received_len == QUECTEL_I2C_CMD_RESP_LEN))  {
            if (avail_len > prev_avail_max) {
                i2c_local_log.trace("qavail: %u prev_avail_max: %u", avail_len, prev_avail_max);
                prev_avail_max = avail_len;
            }
            break;
        }
        else {
            // don't bother to retry because it consistently fails
            Log.warn("Read available bytes failed");
            return QuecDriverStatus::QDEV_ERROR;
        }
    }

    if (avail_len == 0) {
        // Having zero bytes of data available to read is a valid response
        return QuecDriverStatus::QDEV_SUCCESS;
    }

    nreq = avail_len;
    if (nreq > maxLength) {
        //Log.warn("Available bytes %d larger than buffer size %d", nreq, maxLength);
        nreq = maxLength;
    }

    //step 2_a
    request_cmd[0] = (uint32_t)(QUECTEL_I2C_SLAVE_CR_CMD << 16) | QUECTEL_I2C_SLAVE_TX_BUF_REG_OFFSET;
    request_cmd[1] = nreq;
    while (true) {
        delayMicroseconds(QUECTEL_TRANSACTION_DELAY_US);
        status = i2cMasterTransmit(QUECTEL_I2C_SLAVE_ADDRESS_CR_OR_CW, (uint8_t *)request_cmd, QUECTEL_I2C_SLAVE_CMD_LEN);
        if(status == I2C_ACK) {
            break;
        }
        else {
            Log.warn("Read bytes command failed");
            return QuecDriverStatus::QDEV_ERROR;
        }
    }

    //step 2_b
    while(true)  {
        delayMicroseconds(QUECTEL_TRANSACTION_DELAY_US);
        status = i2cMasterReceive(QUECTEL_I2C_SLAVE_ADDRESS_R, pData, nreq, &received_len);
        if (status == I2C_ACK)  { // we allow a shorter response in case our i2c buffer is smaller
            *pRecLength = received_len;
            //i2c_local_log.info("avail: %u max: %u nreq: %u nrecv: %u", avail_len, maxLength, nreq, received_len);
            return QuecDriverStatus::QDEV_SUCCESS;
        }
        Log.warn("Read bytes failed");
        i2c_master_receive_error_counter++;
        if(i2c_master_receive_error_counter > MAX_ERROR_NUMBER)  {
            return QuecDriverStatus::QDEV_ERROR;
        }
    }

    return QuecDriverStatus::QDEV_SUCCESS;
}

QuecDriverStatus quectelGNSSCoreI2C::quectelDevTransmit(const uint8_t *pData, size_t dataLength, size_t* pSentLength)
{
    uint32_t request_cmd[2];
    uint16_t rxBuffLength = 0;
    size_t received_len = 0;
    size_t nsend = 0;
    uint8_t i2c_master_receive_error_counter = 0;
    I2c_Resp_FlagStatus status;
    static size_t prev_ravail_max = 0;
    static size_t total_sent = 0;

    //no data has been confirmed sent
    *pSentLength = 0;

    //step 1_a
    request_cmd[0] = (uint32_t)((QUECTEL_I2C_SLAVE_CR_CMD << 16) | QUECTEL_I2C_SLAVE_RX_LEN_REG_OFFSET);
    request_cmd[1] = 4; 

    while(true) {
        delayMicroseconds(QUECTEL_TRANSACTION_DELAY_US);
        status = i2cMasterTransmit(QUECTEL_I2C_SLAVE_ADDRESS_CR_OR_CW, (uint8_t *)request_cmd, QUECTEL_I2C_SLAVE_CMD_LEN);
        if(status == I2C_ACK)  {
            break;
        }
        else {
            // don't bother to retry because this is fatal
            Log.warn("devTransmit query recv buf failed");
            return QuecDriverStatus::QDEV_ERROR;
        }
    }

    //step 1_b
    while(true)  {
        delayMicroseconds(QUECTEL_TRANSACTION_DELAY_US);
        status = i2cMasterReceive(QUECTEL_I2C_SLAVE_ADDRESS_R, (uint8_t*)&rxBuffLength, QUECTEL_I2C_CMD_RESP_LEN, &received_len);
        if ((status == I2C_ACK) && (received_len == QUECTEL_I2C_CMD_RESP_LEN))  {
            if (rxBuffLength > prev_ravail_max) {
                i2c_local_log.trace("new ravail_max: %u", rxBuffLength);
                prev_ravail_max = rxBuffLength;
            }
            break;
        }
        else {
            // don't bother to retry because it consistently fails
            Log.warn("devTransmit query recv buf failed");
            return QuecDriverStatus::QDEV_ERROR;
        }
    }

    //i2c_local_log.trace("ravail %u len %u  total_sent: %u", rxBuffLength, dataLength, total_sent);

    if (dataLength > rxBuffLength) {
        i2c_local_log.trace("ravail %u < %u on total_sent: %u", rxBuffLength, dataLength, total_sent);
        nsend = rxBuffLength; // this truncates the message but we now let our caller know the actual sent length
    }
    else {
        nsend = dataLength;
    }

    //step 2_a
    request_cmd[0] = (uint32_t)(QUECTEL_I2C_SLAVE_CW_CMD << 16) | QUECTEL_I2C_SLAVE_RX_BUF_REG_OFFSET;
    request_cmd[1] = nsend;  
    i2c_master_receive_error_counter = 0;
    while(true) {
        delayMicroseconds(QUECTEL_TRANSACTION_DELAY_US);
        status = i2cMasterTransmit(QUECTEL_I2C_SLAVE_ADDRESS_CR_OR_CW, (uint8_t *)request_cmd, QUECTEL_I2C_SLAVE_CMD_LEN);
        if(status == I2C_ACK) {
            break;
        }
        i2c_master_receive_error_counter++;
        if(i2c_master_receive_error_counter > MAX_ERROR_NUMBER){
            return QuecDriverStatus::QDEV_ERROR;
        }
    }

    //step 2_b
    i2c_master_receive_error_counter = 0;
    while(true) {
        delayMicroseconds(QUECTEL_TRANSACTION_DELAY_US);
        status = i2cMasterTransmit(QUECTEL_I2C_SLAVE_ADDRESS_W, pData, nsend);
        if(status == I2C_ACK) {
            *pSentLength = nsend;
            total_sent +=  nsend; 
            if (total_sent == 4096) {
                //i2c_local_log.trace("wrapping total_sent!");
                total_sent = 0;
            }
            break;
        }
        i2c_master_receive_error_counter++;
        if(i2c_master_receive_error_counter > MAX_ERROR_NUMBER) {
            i2c_local_log.error("sendfail: %u", dataLength);
            return QuecDriverStatus::QDEV_ERROR;
        }
    }

    return QuecDriverStatus::QDEV_SUCCESS;
}


bool quectelGNSSCoreI2C::waitGNSSOnline() {
    size_t try_count = 0;
    bool online = false;
    uint32_t start_time = millis();
    while (try_count < 100) {
        int rc = 0;

        WITH_LOCK(_i2c) {
            _i2c.beginTransmission(QUECTEL_I2C_SLAVE_ADDRESS_CR_OR_CW);
            _i2c.write(0);
            rc = _i2c.endTransmission();
        }
        if (rc == SYSTEM_ERROR_NONE) {
            online = true;
            break;
        }
       else {
            Log.warn("gnss offline: %d", (int)rc);
            try_count++;
            delay(1);
        }
    }

    uint32_t total_time = millis() - start_time;
    if (total_time > 0) {
        i2c_local_log.trace("waitGNSSOnline took: %lu", total_time);
    }

    return online;
}










