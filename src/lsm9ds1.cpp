/*
 * Copyright (c) 2024, Mezael Docoy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "lsm9ds1.h"

Lsm9ds1::Lsm9ds1()
{
    std::uint8_t dev_id {0};
    lsm9ds1_i2c = std::make_unique<Lsm9ds1Hal>();

    Lsm9ds1Hal::Lsm9ds1_Error_t ret = Lsm9ds1Hal::LSM9DS1_OK;

    // /* Startup time */
    lsm9ds1_i2c->lsm9ds1_i2c_hal_ms_delay(50);

    /* Perform device reset */
    ret |= Lsm9ds1_Reset();

    lsm9ds1_i2c->lsm9ds1_i2c_hal_ms_delay(50);

    ret |= Lsm9ds1_GetDevideId(dev_id);

    /* Turn on accelerometer */
    ret |= Lsm9ds1_AccelerometerOdrSelect(ODR_ACCEL_50HZ);

    /* Turn on Gyroscope */
    ret |= Lsm9ds1_GyroOdrSelect(ODR_GYRO_59_5HZ);

    if(Lsm9ds1Hal::LSM9DS1_OK == ret && dev_id == WHO_AM_I_VAL)
    {
        /* Do something */
    }
    else
    {
        /* Do something */
    }

    lsm9ds1_i2c->lsm9ds1_i2c_hal_ms_delay(50);
}

Lsm9ds1::~Lsm9ds1() = default;

/*==================================================================================================
*                                       PRIVATE MEMBER FUNCTIONS
==================================================================================================*/

/* Read accelerometer's raw data */
Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1::Lsm9ds1_GetAccelRawData(Lsm9ds1_AccelRawData_t &AccelRawData)
{
    Lsm9ds1Hal::Lsm9ds1_Error_t ret = Lsm9ds1Hal::LSM9DS1_OK;
    std::uint8_t reg = REG_OUT_X_L_XL;
    std::uint8_t data[6U] = {0U};

    ret |= lsm9ds1_i2c->lsm9ds1_i2c_hal_read(I2C_ADDRESS_LSM9DS1, &reg, data, 6U);

    if (Lsm9ds1Hal::LSM9DS1_OK == ret)
    {
        AccelRawData.Accel_X_Raw = static_cast<int16_t>((data[1U] << MSB_8BIT_SHIFT) | data[0U]);
        AccelRawData.Accel_Y_Raw = static_cast<int16_t>((data[3U] << MSB_8BIT_SHIFT) | data[2U]);
        AccelRawData.Accel_Z_Raw = static_cast<int16_t>((data[5U] << MSB_8BIT_SHIFT) | data[4U]);
    }

    return ret;
}

/* Read gyroscope's raw data */
Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1::Lsm9ds1_GetGyroRawData(Lsm9ds1_GyroRawData_t &GyroRawData)
{
    Lsm9ds1Hal::Lsm9ds1_Error_t ret = Lsm9ds1Hal::LSM9DS1_OK;
    std::uint8_t reg = REG_OUT_X_L_G;
    std::uint8_t data[6U] = {0U};

    ret |= lsm9ds1_i2c->lsm9ds1_i2c_hal_read(I2C_ADDRESS_LSM9DS1, &reg, data, 6U);

    if (Lsm9ds1Hal::LSM9DS1_OK == ret)
    {
        GyroRawData.Gyro_X_Raw = static_cast<int16_t>((data[1U] << MSB_8BIT_SHIFT) | data[0U]);
        GyroRawData.Gyro_Y_Raw = static_cast<int16_t>((data[3U] << MSB_8BIT_SHIFT) | data[2U]);
        GyroRawData.Gyro_Z_Raw = static_cast<int16_t>((data[5U] << MSB_8BIT_SHIFT) | data[4U]);
    }

    return ret;
}

/*==================================================================================================
*                                       PUBLIC MEMBER FUNCTIONS
==================================================================================================*/

/*================================================================================================*/
/**
* @brief        Set output data rate.
* @details      Set output data rate and power mode for accelerometer in accelermeter-only mode.
*
* @param[in]    Odr     Output data rate.
*
* @return       Lsm9ds1Hal::Lsm9ds1_Error_t     Return code.
*
*/
Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1::Lsm9ds1_AccelerometerOdrSelect(Lsm9ds1_Accel_Odr_t Odr)
{
    Lsm9ds1Hal::Lsm9ds1_Error_t ret = Lsm9ds1Hal::LSM9DS1_OK;
    std::uint8_t reg = REG_CTRL_REG6_XL;
    std::uint8_t cfg = 0U;
    std::uint8_t data[2] = {0U};

    /* Parameter check */
    if(Odr > ODR_ACCEL_952HZ)
    {
        ret |= Lsm9ds1Hal::LSM9DS1_ERR;
    }
    else
    {
        ret |= lsm9ds1_i2c->lsm9ds1_i2c_hal_read(I2C_ADDRESS_LSM9DS1, &reg, &cfg, 1);

        if (Lsm9ds1Hal::LSM9DS1_OK == ret)
        {
            data[0] = reg;
            data[1] = cfg & ~XL_ODR_MASK;
            data[1] |= (static_cast<std::uint8_t>(Odr) << XL_ODR_SHIFT);

            ret |= lsm9ds1_i2c->lsm9ds1_i2c_hal_write(I2C_ADDRESS_LSM9DS1, data, 2);
        }
    }

    return ret;
}

/*================================================================================================*/
/**
* @brief        Set output data rate.
* @details      Set output data rate and power mode for gyroscope in accelerometer + gyro mode.
*
* @param[in]    Odr     Output data rate.
*
* @return       Lsm9ds1Hal::Lsm9ds1_Error_t     Return code.
*
*/
Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1::Lsm9ds1_GyroOdrSelect(Lsm9ds1_Gyro_Odr_t Odr)
{
    Lsm9ds1Hal::Lsm9ds1_Error_t ret = Lsm9ds1Hal::LSM9DS1_OK;
    std::uint8_t reg = REG_CTRL_REG1_G;
    std::uint8_t cfg = 0U;
    std::uint8_t data[2] = {0U};

    /* Parameter check */
    if(Odr > ODR_GYRO_952HZ)
    {
        ret |= Lsm9ds1Hal::LSM9DS1_ERR;
    }
    else
    {
        ret |= lsm9ds1_i2c->lsm9ds1_i2c_hal_read(I2C_ADDRESS_LSM9DS1, &reg, &cfg, 1);

        if (Lsm9ds1Hal::LSM9DS1_OK == ret)
        {
            data[0] = reg;
            data[1] = cfg & ~XL_ODR_MASK;
            data[1] |= (static_cast<std::uint8_t>(Odr) << XL_ODR_SHIFT);

            ret |= lsm9ds1_i2c->lsm9ds1_i2c_hal_write(I2C_ADDRESS_LSM9DS1, data, 2);
        }
    }

    return ret;
}

/*================================================================================================*/
/**
* @brief        Set sleep mode for gyroscope.
* @details      Gyroscope sleep mode enable.
*
* @param[in]    ModeEnable     Enable/Disable sleep mode.
*
* @return       Lsm9ds1Hal::Lsm9ds1_Error_t     Return code.
*
*/
Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1::Lsm9ds1_GyroscopeSleepMode(std::uint8_t ModeEnable)
{
    Lsm9ds1Hal::Lsm9ds1_Error_t ret = Lsm9ds1Hal::LSM9DS1_OK;
    std::uint8_t reg = REG_CTRL_REG9;
    std::uint8_t cfg = 0U;
    std::uint8_t data[2] = {0U};

    /* Parameter check */
    if(ModeEnable > ENABLE)
    {
        ret |= Lsm9ds1Hal::LSM9DS1_ERR;
    }
    else
    {
        ret |= lsm9ds1_i2c->lsm9ds1_i2c_hal_read(I2C_ADDRESS_LSM9DS1, &reg, &cfg, 1);

        if (Lsm9ds1Hal::LSM9DS1_OK == ret)
        {
            data[0] = reg;
            data[1] = (ModeEnable == DISABLE) ? (cfg & ~GYRO_SLEEP_MODE_MASK) : (cfg & ~GYRO_SLEEP_MODE_MASK);

            ret |= lsm9ds1_i2c->lsm9ds1_i2c_hal_write(I2C_ADDRESS_LSM9DS1, data, 2);
        }
    }

    return ret;
}

/*================================================================================================*/
/**
* @brief        Reset device.
* @details      Perform device reset.
*
* @return       Lsm9ds1Hal::Lsm9ds1_Error_t     Return code.
*
*/
Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1::Lsm9ds1_Reset()
{
    Lsm9ds1Hal::Lsm9ds1_Error_t ret = Lsm9ds1Hal::LSM9DS1_OK;
    std::uint8_t reg = REG_CTRL_REG8;
    std::uint8_t cfg = 0U;
    std::uint8_t data[2] = {0U};

    ret |= lsm9ds1_i2c->lsm9ds1_i2c_hal_read(I2C_ADDRESS_LSM9DS1, &reg, &cfg, 1);

    if (Lsm9ds1Hal::LSM9DS1_OK == ret)
    {
        data[0] = reg;
        data[1] = cfg | DEV_RESET_MASK;

        ret |= lsm9ds1_i2c->lsm9ds1_i2c_hal_write(I2C_ADDRESS_LSM9DS1, data, 2);
    }

    return ret;
}

/*================================================================================================*/
/**
* @brief        Full scale range setting.
* @details      Configure the accelerometer's full scale range.
*
* @param[in]    eFsSel_Xl     Accelerometer full scale range value.
*
* @return       Lsm9ds1Hal::Lsm9ds1_Error_t     Return code.
*
*/
Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1::Lsm9ds1_AccelFsSel(Lsm9ds1_FsSel_Xl_t eFsSel_Xl)
{
    Lsm9ds1Hal::Lsm9ds1_Error_t ret = Lsm9ds1Hal::LSM9DS1_OK;
    std::uint8_t reg = REG_CTRL_REG6_XL;
    std::uint8_t cfg = 0U;
    std::uint8_t data[2] = {0U};

    /* Parameter check */
    if(eFsSel_Xl > FS_XL_16)
    {
        ret |= Lsm9ds1Hal::LSM9DS1_ERR;
    }
    else
    {
        ret |= lsm9ds1_i2c->lsm9ds1_i2c_hal_read(I2C_ADDRESS_LSM9DS1, &reg, &cfg, 1);

        if (Lsm9ds1Hal::LSM9DS1_OK == ret)
        {
            data[0] = reg;
            data[1] = cfg & ~FS_SEL_MASK;
            data[1] |= (static_cast<std::uint8_t>(eFsSel_Xl) << FS_SEL_SHIFT);

            ret |= lsm9ds1_i2c->lsm9ds1_i2c_hal_write(I2C_ADDRESS_LSM9DS1, data, 2);

            /* Confirm acceleration FsSel value and store it in the variable */
            cfg = 0U;
            ret |= lsm9ds1_i2c->lsm9ds1_i2c_hal_read(I2C_ADDRESS_LSM9DS1, &reg, &cfg, 1);


            if (Lsm9ds1Hal::LSM9DS1_OK == ret)
            {
                u8AccelFsSelCurrentVal = (cfg & FS_SEL_MASK) >> FS_SEL_SHIFT;
            }
        }
    }

    return ret;
}

/*================================================================================================*/
/**
* @brief        Full scale range setting.
* @details      Configure the gyroscope's full scale range.
*
* @param[in]    eFsSel_G      Gyroscope full scale range value.
*
* @return       Lsm9ds1Hal::Lsm9ds1_Error_t     Return code.
*
*/
Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1::Lsm9ds1_GyroFsSel(Lsm9ds1_FsSel_G_t eFsSel_G)
{
    Lsm9ds1Hal::Lsm9ds1_Error_t ret = Lsm9ds1Hal::LSM9DS1_OK;
    std::uint8_t reg = REG_CTRL_REG1_G;
    std::uint8_t cfg = 0U;
    std::uint8_t data[2] = {0U};

    /* Parameter check */
    if(eFsSel_G > FS_G_2000)
    {
        ret |= Lsm9ds1Hal::LSM9DS1_ERR;
    }
    else
    {
        ret |= lsm9ds1_i2c->lsm9ds1_i2c_hal_read(I2C_ADDRESS_LSM9DS1, &reg, &cfg, 1);

        if (Lsm9ds1Hal::LSM9DS1_OK == ret)
        {
            data[0] = reg;
            data[1] = cfg & ~FS_SEL_MASK;
            data[1] |= (static_cast<std::uint8_t>(eFsSel_G) << FS_SEL_SHIFT);

            ret |= lsm9ds1_i2c->lsm9ds1_i2c_hal_write(I2C_ADDRESS_LSM9DS1, data, 2);

            /* Confirm gyroscope FsSel value and store it in the variable */
            cfg = 0U;
            ret |= lsm9ds1_i2c->lsm9ds1_i2c_hal_read(I2C_ADDRESS_LSM9DS1, &reg, &cfg, 1);

            if (Lsm9ds1Hal::LSM9DS1_OK == ret)
            {
                u8GyroFsSelCurrentVal = (cfg & FS_SEL_MASK) >> FS_SEL_SHIFT;
            }
        }
    }

    return ret;
}

/*================================================================================================*/
/**
* @brief        Get accelerometer data.
* @details      Get 3 axis accelerometer data.
*
* @param[in]    AccelData   Pointer where the data will be stored.
*
* @return       Lsm9ds1Hal::Lsm9ds1_Error_t     Return code.
*
*/
Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1::Lsm9ds1_GetAccelData(Lsm9ds1_AccelData_t &AccelData)
{
    Lsm9ds1Hal::Lsm9ds1_Error_t ret = Lsm9ds1Hal::LSM9DS1_OK;
    Lsm9ds1_AccelRawData_t AccelRawData;

    ret |= Lsm9ds1_GetAccelRawData(AccelRawData);

    if (Lsm9ds1Hal::LSM9DS1_OK == ret)
    {
        AccelData.Accel_X = static_cast<double>((static_cast<double>(AccelRawData.Accel_X_Raw) * ACCEL_GRAVITY) / aAccelFsSelVal[u8AccelFsSelCurrentVal]);
        AccelData.Accel_Y = static_cast<double>((static_cast<double>(AccelRawData.Accel_Y_Raw) * ACCEL_GRAVITY) / aAccelFsSelVal[u8AccelFsSelCurrentVal]);
        AccelData.Accel_Z = static_cast<double>((static_cast<double>(AccelRawData.Accel_Z_Raw) * ACCEL_GRAVITY) / aAccelFsSelVal[u8AccelFsSelCurrentVal]);
    }

    return ret;
}

/*================================================================================================*/
/**
* @brief        Get gyroscope data.
* @details      Get 3 axis gyroscope data.
*
* @param[in]    GyroData    Pointer where the data will be stored.
*
* @return       Lsm9ds1Hal::Lsm9ds1_Error_t     Return code.
*
*/
Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1::Lsm9ds1_GetGyroData(Lsm9ds1_GyroData_t &GyroData)
{
    Lsm9ds1Hal::Lsm9ds1_Error_t ret = Lsm9ds1Hal::LSM9DS1_OK;
    Lsm9ds1_GyroRawData_t GyroRawData;

    ret |= Lsm9ds1_GetGyroRawData(GyroRawData);

    if (Lsm9ds1Hal::LSM9DS1_OK == ret)
    {
        GyroData.Gyro_X = static_cast<double>(static_cast<double>(GyroRawData.Gyro_X_Raw) / aGyroFsSelVal[u8GyroFsSelCurrentVal]);
        GyroData.Gyro_Y = static_cast<double>(static_cast<double>(GyroRawData.Gyro_Y_Raw) / aGyroFsSelVal[u8GyroFsSelCurrentVal]);
        GyroData.Gyro_Z = static_cast<double>(static_cast<double>(GyroRawData.Gyro_Z_Raw) / aGyroFsSelVal[u8GyroFsSelCurrentVal]);
    }

    return ret;
}

/*================================================================================================*/
/**
* @brief        Read device ID.
* @details      Read 8bit device id.
*
* @param[in]    pId         Pointer to which the data will be stored.
*
* @return       Lsm9ds1Hal::Lsm9ds1_Error_t     Return code.
*
*/
Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1::Lsm9ds1_GetDevideId(std::uint8_t &pId)
{
    Lsm9ds1Hal::Lsm9ds1_Error_t ret = Lsm9ds1Hal::LSM9DS1_OK;
    std::uint8_t reg = REG_WHO_AM_I;
    std::uint8_t data = 0U;

    ret |= lsm9ds1_i2c->lsm9ds1_i2c_hal_read(I2C_ADDRESS_LSM9DS1, &reg, &data, 1);

    if (Lsm9ds1Hal::LSM9DS1_OK == ret)
    {
        pId = data;
    }

    return ret;
}
