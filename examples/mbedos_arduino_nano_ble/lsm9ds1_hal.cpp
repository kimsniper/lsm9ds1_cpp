#include "lsm9ds1_hal.h"

Lsm9ds1Hal::Lsm9ds1Hal()
    : i2c(I2C_SDA1, I2C_SCL1),
      pulluppin(p32),
      acc_pwr(VDD_ENV)
{

    i2c.frequency(100000);
    i2c.start();

    acc_pwr = 1;
	pulluppin = 1;
}

Lsm9ds1Hal::~Lsm9ds1Hal() { i2c.stop(); }

/*==================================================================================================
*                                       PUBLIC FUNCTIONS
==================================================================================================*/

/*================================================================================================*/
/**
* @brief        Execute I2C read.
* @details      Execute I2C read sequence.
*
* @param[in]    address     Slave address.
* @param[in]    reg         Register address.
* @param[in]    count       Number of bytes to read.
* @param[out]   aRxBuffer   Array to which data will be stored.
*
* @return       Lsm9ds1_Error_t     Return code.
*
*/
Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1Hal::lsm9ds1_i2c_hal_read(const std::uint8_t address, std::uint8_t *reg, std::uint8_t *aRxBuffer, const std::uint16_t count)
{
    /* Param checking */
    if(count == 0U)
    {
        return LSM9DS1_ERR;
    }
    else
    {
        auto i2c_res = i2c.write((int)address, (char *)reg, 1, 1);
        if(i2c_res == i2c.Result::NACK)
        {
            return LSM9DS1_ERR;
        }

        i2c_res = i2c.read((int)address, (char *)aRxBuffer, (int)count, 0);
        if(i2c_res == i2c.Result::NACK)
        {
            return LSM9DS1_ERR;
        }
    }

    return LSM9DS1_OK;
}

/*================================================================================================*/
/**
 * @brief        Execute I2C write.
 * @details      Execute I2C write sequence.
 *
 * @param[in]    address     Slave address.
 * @param[in]    reg         Register address.
 * @param[out]   TxBuffer    Data that will be written.
 *
 * @return       Lsm9ds1_Error_t     Return code.
 *
 */
Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1Hal::lsm9ds1_i2c_hal_write(const std::uint8_t address, std::uint8_t *TxBuffer, const std::uint16_t count)
{
    const auto i2c_res = i2c.write(address, (char *)TxBuffer, int(count), 0);

    if(i2c_res == i2c.Result::NACK)
    {
        return LSM9DS1_ERR;
    }

    return LSM9DS1_OK;
}

/*================================================================================================*/
/**
* @brief        Execute ms delay.
* @details      Execute ms delay for hal usage.
*
* @param[in]    ms      Time in milliseconds.
*
* @return       void
*
*/
void Lsm9ds1Hal::lsm9ds1_i2c_hal_ms_delay(std::uint32_t ms) {

    /* User implementation here */
    std::chrono::milliseconds ms_delay(ms);
    ThisThread::sleep_for(ms_delay);
    
}
