#ifndef __quectelGNSSCoreI2C_H
#define __quectelGNSSCoreI2C_H

//**************** INCLUDES ******************
#include "Particle.h"


//**************** CONSTANTS *****************



//*************** ENUMS *********************

enum class QuecDriverStatus
{
    QDEV_SUCCESS = 0,
    QDEV_ERROR = 1
};

typedef enum
{
    I2C_ACK = 0,
    I2C_NACK = 1
} I2c_Resp_FlagStatus;


//*************** CLASS *********************
class quectelGNSSCoreI2C
{

public:
    /**
     * @brief Construct a new Quectel GNSS I2C protocol-handling object
     *
     * @param bus Reference to a specific I2C bus
     */
    quectelGNSSCoreI2C(TwoWire& bus);


    /**
     * @brief Receive GNSS data 
     *
     * @param pData (out) pointer to buffer containing data returned by the read request
     * @param maxLenth (in) maximum buffer size for pData
     * @param pRecLength (out) actual number of bytes read from GNSS module
     */
    QuecDriverStatus quectelDevReceive(uint8_t* pData, size_t maxLength, size_t* pRecLength);

    /**
     * @brief Transmit data to GNSS module
     *
     * @param pData (in) pointer to data being sent to module
     * @param dataLength (in) number of bytes being transmitted to module
     * @param pSentLength (out) ctual number of bytes sent to the module
     */
    QuecDriverStatus quectelDevTransmit(const uint8_t *pData, size_t dataLength, size_t* pSentLength);

    /**
     * 
     * @return true If the device can be seen on the i2c bus
     */
    bool waitGNSSOnline();



private:
    // Variables
    TwoWire&        _i2c;

    // Methods
    /**
     * @brief Wrapper of Wiring API to read I2C bus
     *
     * @param addr 7-bit I2C device address to request data from
     * @param pBuff (out) Buffer for returning received bytes
     * @param reqLen (in) number of bytes to request from the module
     * @param pReadLen (out) Actual number of bytes read
     * 
     * @return Error if data could not be received. Note that partial data may be returned
     */
    I2c_Resp_FlagStatus i2cMasterReceive(uint8_t addr, uint8_t* pBuff, size_t reqLen, size_t* pReadLen);

    /**
     * @brief Wrapper of Wiring API to write to I2C bus
     *
     * @param addr 7-bit I2C device address
     * @param data pointer to data being sent to module
     * @param length number of bytes to send to module
     * 
     * @return Error if the full length could not be sent
     */
    I2c_Resp_FlagStatus i2cMasterTransmit(uint8_t addr, const uint8_t* pData, size_t length);


};

#endif // __quectelGNSSCoreI2C_H