#ifndef LSM9DS1_HAL_H
#define LSM9DS1_HAL_H

/* Hardware Specific Components */
#include "mbed.h"

class Lsm9ds1Hal {
public:
    Lsm9ds1Hal();
    
    ~Lsm9ds1Hal();

    typedef int16_t Lsm9ds1_Error_t;

    static constexpr Lsm9ds1_Error_t LSM9DS1_ERR    = -1;
    static constexpr Lsm9ds1_Error_t LSM9DS1_OK     = 0;

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
    Lsm9ds1_Error_t lsm9ds1_i2c_hal_read(const std::uint8_t address, std::uint8_t *reg, std::uint8_t *aRxBuffer, const std::uint16_t count);

    /**
     * @brief        Execute I2C write.
     * @details      Execute I2C write sequence.
     *
     * @param[in]    reg         Register address.
     * @param[out]   TxBuffer    Data that will be written.
     *
     * @return       Lsm9ds1_Error_t     Return code.
     *
     */
    Lsm9ds1_Error_t lsm9ds1_i2c_hal_write(const std::uint8_t address, std::uint8_t *TxBuffer, const std::uint16_t count);

    /**
     * @brief        Execute ms delay.
     * @details      Execute ms delay for hal usage.
     *
     * @param[in]    ms      Time in milliseconds.
     *
     * @return       void
     *
     */
    void lsm9ds1_i2c_hal_ms_delay(std::uint32_t ms);

 private:

    I2C i2c;
    DigitalOut pulluppin;
    DigitalOut acc_pwr;

};


#endif  // LSM9DS1_HAL_H
