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

#ifndef LSM9DS1_H
#define LSM9DS1_H

#include "lsm9ds1_hal.h"

#include <string>
#include <memory>
#include <cstdint>

class Lsm9ds1 {
public:
    explicit Lsm9ds1();

    ~Lsm9ds1();

    typedef enum{
        DLPF_CFG_260_256 = 0x00U,
        DLPF_CFG_184_188 = 0x01U,
        DLPF_CFG_94_98 = 0x02U,
        DLPF_CFG_44_42 = 0x03U,
        DLPF_CFG_21_20 = 0x04U,
        DLPF_CFG_10_10 = 0x05U,
        DLPF_CFG_5_5 = 0x06U
    } Lsm9ds1_DlpfCfg_t;

    typedef enum{
        FS_G_245 = 0x00U,
        FS_G_500 = 0x01U,
        FS_G_2000 = 0x03U
    } Lsm9ds1_FsSel_G_t;

    typedef enum{
        FS_XL_2 = 0x00U,
        FS_XL_4 = 0x02U,
        FS_XL_8 = 0x03U,
        FS_XL_16 = 0x01U
    } Lsm9ds1_FsSel_Xl_t;

    typedef enum{
        PWR_MODE_NORMAL = 0x00U,
        PWR_MODE_SLEEP = 0x01U,
        PWR_MODE_CYCLE = 0x02U,
    } Lsm9ds1_PwrMode_t;

    typedef enum{
        ODR_ACCEL_POWER_DOWN = 0x00U,
        ODR_ACCEL_10HZ = 0x01U,
        ODR_ACCEL_50HZ = 0x02U,
        ODR_ACCEL_119HZ = 0x03U,
        ODR_ACCEL_238HZ = 0x04U,
        ODR_ACCEL_476HZ = 0x05U,
        ODR_ACCEL_952HZ = 0x06U,
    } Lsm9ds1_Accel_Odr_t;

    typedef enum{
        ODR_GYRO_POWER_DOWN = 0x00U,
        ODR_GYRO_14_9HZ = 0x01U,
        ODR_GYRO_59_5HZ = 0x02U,
        ODR_GYRO_119HZ = 0x03U,
        ODR_GYRO_238HZ = 0x04U,
        ODR_GYRO_476HZ = 0x05U,
        ODR_GYRO_952HZ = 0x06U,
    } Lsm9ds1_Gyro_Odr_t;

    /**
     * @brief Accelerometer raw data
     */
    typedef struct{
        int16_t Accel_X_Raw;     /** @brief Accel X raw data */
        int16_t Accel_Y_Raw;     /** @brief Accel Y raw data */
        int16_t Accel_Z_Raw;     /** @brief Accel Z raw data */
    } Lsm9ds1_AccelRawData_t;

    /**
     * @brief Accelerometer data
     */
    typedef struct{
        double Accel_X;     /** @brief Accel X data */
        double Accel_Y;     /** @brief Accel Y data */
        double Accel_Z;     /** @brief Accel Z data */
    } Lsm9ds1_AccelData_t;

    /**
     * @brief Gyroscope raw data
     */
    typedef struct{
        int16_t Gyro_X_Raw;     /** @brief Gyro X raw data */
        int16_t Gyro_Y_Raw;     /** @brief Gyro Y raw data */
        int16_t Gyro_Z_Raw;     /** @brief Gyro Z raw data */
    } Lsm9ds1_GyroRawData_t;

    /**
     * @brief Gyroscope data
     */
    typedef struct{
        double Gyro_X;     /** @brief Gyro X data */
        double Gyro_Y;     /** @brief Gyro Y data */
        double Gyro_Z;     /** @brief Gyro Z data */
    } Lsm9ds1_GyroData_t;

    /*==================================================================================================
    *                                       FUNCTION Prototypes
    ==================================================================================================*/

    /**
    * @brief        Set output data rate.
    * @details      Set output data rate and power mode for accelerometer.
    *
    * @param[in]    Odr     Output data rate.
    *
    * @return       Lsm9ds1Hal::Lsm9ds1_Error_t     Return code.
    *
    */
    Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1_AccelerometerOdrSelect(Lsm9ds1_Accel_Odr_t Odr);

    /**
    * @brief        Set output data rate.
    * @details      Set output data rate and power mode for gyroscope in accelerometer + gyro mode.
    *
    * @param[in]    Odr     Output data rate.
    *
    * @return       Lsm9ds1Hal::Lsm9ds1_Error_t     Return code.
    *
    */
    Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1_GyroOdrSelect(Lsm9ds1_Gyro_Odr_t Odr);

    /**
    * @brief        Set sleep mode for gyroscope.
    * @details      Gyroscope sleep mode enable.
    *
    * @param[in]    ModeEnable     Enable/Disable sleep mode.
    *
    * @return       Lsm9ds1Hal::Lsm9ds1_Error_t     Return code.
    *
    */
    Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1_GyroscopeSleepMode(std::uint8_t ModeEnable);

    /**
    * @brief        Reset device.
    * @details      Perform device reset.
    *
    *
    * @return       Lsm9ds1Hal::Lsm9ds1_Error_t     Return code.
    *
    */
    Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1_Reset();

    /**
    * @brief        Full scale range setting.
    * @details      Configure the accelerometer's full scale range.
    *
    * @param[in]    eFsSel_Xl     Accelerometer full scale range value.
    *
    * @return       Lsm9ds1Hal::Lsm9ds1_Error_t     Return code.
    *
    */
    Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1_AccelFsSel(Lsm9ds1_FsSel_Xl_t eFsSel_Xl);

    /**
    * @brief        Full scale range setting.
    * @details      Configure the gyroscope's full scale range.
    *
    * @param[in]    eFsSel_G      Gyroscope full scale range value.
    *
    * @return       Lsm9ds1Hal::Lsm9ds1_Error_t     Return code.
    *
    */
    Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1_GyroFsSel(Lsm9ds1_FsSel_G_t eFsSel_G);

    /**
    * @brief        Get accelerometer data.
    * @details      Get 3 axis accelerometer data.
    *
    * @param[in]    AccelData   Pointer where the data will be stored.
    *
    * @return       Lsm9ds1Hal::Lsm9ds1_Error_t     Return code.
    *
    */
    Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1_GetAccelData(Lsm9ds1_AccelData_t &AccelData);
    /**
    * @brief        Get gyroscope data.
    * @details      Get 3 axis gyroscope data.
    *
    * @param[in]    GyroData    Pointer where the data will be stored.
    *
    * @return       Lsm9ds1Hal::Lsm9ds1_Error_t     Return code.
    *
    */
    Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1_GetGyroData(Lsm9ds1_GyroData_t &GyroData);

    /**
    * @brief        Read device ID.
    * @details      Read 8bit device id.
    *
    * @param[in]    pId         Pointer to which the data will be stored.
    *
    * @return       Lsm9ds1Hal::Lsm9ds1_Error_t     Return code.
    *
    */
    Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1_GetDevideId(std::uint8_t &pId);

private:
    Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1_GetAccelRawData(Lsm9ds1_AccelRawData_t &AccelRawData);
    Lsm9ds1Hal::Lsm9ds1_Error_t Lsm9ds1_GetGyroRawData(Lsm9ds1_GyroRawData_t &GyroRawData);

    std::unique_ptr<Lsm9ds1Hal> lsm9ds1_i2c;

    std::uint8_t u8AccelFsSelCurrentVal = static_cast<std::uint8_t>(FS_XL_2);
    std::uint8_t u8GyroFsSelCurrentVal = static_cast<std::uint8_t>(FS_G_245);
    const std::array<double, 3> aGyroFsSelVal {114.29, 57.14, 14.19};
    const std::array<double, 4> aAccelFsSelVal {16393.442, 1366.12, 8196.72, 4098.36};

    /**
     * @brief LSM9DS1 accelerometer/gyroscope I2C slave address when pin AD0 is low
     */
    static constexpr std::uint8_t I2C_ADDRESS_LSM9DS1             = 0xD6;

    /**
     * @brief LSM9DS1 magnetometer I2C slave address when pin AD0 is low
     */
    static constexpr std::uint8_t I2C_ADDRESS_LSM9DS1_M           = 0x3C;

    /**
     * @brief LSM9DS1 command code registers
     */

    /* Accelerometer & gyroscope */
    static constexpr std::uint8_t REG_ACT_THS                     = 0x04;
    static constexpr std::uint8_t REG_ACT_DUR                     = 0x05;
    static constexpr std::uint8_t REG_INT_GEN_CFG_XL              = 0x06;
    static constexpr std::uint8_t REG_INT_GEN_THS_X_XL            = 0x07;
    static constexpr std::uint8_t REG_INT_GEN_THS_Y_XL            = 0x08;
    static constexpr std::uint8_t REG_INT_GEN_THS_Z_XL            = 0x09;
    static constexpr std::uint8_t REG_INT_GEN_DUR_XL              = 0x0A;
    static constexpr std::uint8_t REG_REFERENCE_G                 = 0x0B;
    static constexpr std::uint8_t REG_INT1_CTRL                   = 0x0C;
    static constexpr std::uint8_t REG_INT2_CTRL                   = 0x0D;
    static constexpr std::uint8_t REG_WHO_AM_I                    = 0x0F;
    static constexpr std::uint8_t REG_CTRL_REG1_G                 = 0x10;
    static constexpr std::uint8_t REG_CTRL_REG2_G                 = 0x11;
    static constexpr std::uint8_t REG_CTRL_REG3_G                 = 0x12;
    static constexpr std::uint8_t REG_ORIENT_CFG_G                = 0x13;
    static constexpr std::uint8_t REG_INT_GEN_SRC_G               = 0x14;
    static constexpr std::uint8_t REG_STATUS_REG                  = 0x17;
    static constexpr std::uint8_t REG_OUT_X_L_G                   = 0x18;
    static constexpr std::uint8_t REG_CTRL_REG4                   = 0x1E;
    static constexpr std::uint8_t REG_CTRL_REG5_XL                = 0x1F;
    static constexpr std::uint8_t REG_CTRL_REG6_XL                = 0x20;
    static constexpr std::uint8_t REG_CTRL_REG7_XL                = 0x21;
    static constexpr std::uint8_t REG_CTRL_REG8                   = 0x22;
    static constexpr std::uint8_t REG_CTRL_REG9                   = 0x23;
    static constexpr std::uint8_t REG_CTRL_REG10                  = 0x24;
    static constexpr std::uint8_t REG_INT_GEN_SRC_XL              = 0x26;
    static constexpr std::uint8_t REG_OUT_X_L_XL                  = 0x28;
    static constexpr std::uint8_t REG_FIFO_CTRL                   = 0x2E;
    static constexpr std::uint8_t REG_FIFO_SRC                    = 0x2F;
    static constexpr std::uint8_t REG_INT_GEN_CFG_G               = 0x30;
    static constexpr std::uint8_t REG_INT_GEN_THS_XH_G            = 0x31;
    static constexpr std::uint8_t REG_INT_GEN_THS_XL_G            = 0x32;
    static constexpr std::uint8_t REG_INT_GEN_THS_YH_G            = 0x33;
    static constexpr std::uint8_t REG_INT_GEN_THS_YL_G            = 0x34;
    static constexpr std::uint8_t REG_INT_GEN_THS_ZH_G            = 0x35;
    static constexpr std::uint8_t REG_INT_GEN_THS_ZL_G            = 0x36;
    static constexpr std::uint8_t REG_INT_GEN_DUR_G               = 0x37;

    /* Magnetometer */

    static constexpr std::uint8_t REG_OFFSET_X_REG_L_M            = 0x05;
    static constexpr std::uint8_t REG_OFFSET_X_REG_H_M            = 0x06;
    static constexpr std::uint8_t REG_OFFSET_Y_REG_L_M            = 0x07;
    static constexpr std::uint8_t REG_OFFSET_Y_REG_H_M            = 0x08;
    static constexpr std::uint8_t REG_OFFSET_Z_REG_L_M            = 0x09;
    static constexpr std::uint8_t REG_OFFSET_Z_REG_H_M            = 0x0A;
    static constexpr std::uint8_t REG_WHO_AM_I_M                  = 0x0F;
    static constexpr std::uint8_t REG_CTRL_REG1_M                 = 0x20;
    static constexpr std::uint8_t REG_CTRL_REG2_M                 = 0x21;
    static constexpr std::uint8_t REG_CTRL_REG3_M                 = 0x22;
    static constexpr std::uint8_t REG_CTRL_REG4_M                 = 0x23;
    static constexpr std::uint8_t REG_CTRL_REG5_M                 = 0x24;
    static constexpr std::uint8_t REG_STATUS_REG_M                = 0x27;
    static constexpr std::uint8_t REG_OUT_X_L_M                   = 0x28;
    static constexpr std::uint8_t REG_OUT_X_H_M                   = 0x29;
    static constexpr std::uint8_t REG_OUT_Y_L_M                   = 0x2A;
    static constexpr std::uint8_t REG_OUT_Y_H_M                   = 0x2B;
    static constexpr std::uint8_t REG_OUT_Z_L_M                   = 0x2C;
    static constexpr std::uint8_t REG_OUT_Z_H_M                   = 0x2D;
    static constexpr std::uint8_t REG_INT_CFG_M                   = 0x30;
    static constexpr std::uint8_t REG_INT_SRC_M                   = 0x31;
    static constexpr std::uint8_t REG_INT_THS_L_M                 = 0x32;
    static constexpr std::uint8_t REG_INT_THS_H_M                 = 0x33;

    /**
     * @brief Mask and shift variables
     */
    static constexpr std::uint8_t FIFO_EN_MASK                    = 0x40;
    static constexpr std::uint8_t FIFO_RESET_MASK                 = 0x04;
    static constexpr std::uint8_t FIFO_TEMP_EN_MASK               = 0x80;
    static constexpr std::uint8_t FIFO_XG_EN_MASK                 = 0x40;
    static constexpr std::uint8_t FIFO_YG_EN_MASK                 = 0x20;
    static constexpr std::uint8_t FIFO_ZG_EN_MASK                 = 0x10;
    static constexpr std::uint8_t FIFO_ACCEL_EN_MASK              = 0x08;
    static constexpr std::uint8_t FIFO_SLV2_EN_MASK               = 0x04;
    static constexpr std::uint8_t FIFO_SLV1_EN_MASK               = 0x02;
    static constexpr std::uint8_t FIFO_SLV0_EN_MASK               = 0x01;

    static constexpr std::uint8_t FS_SEL_MASK                     = 0x18;
    static constexpr std::uint8_t FS_SEL_SHIFT                    = 0x03;

    static constexpr std::uint8_t DLPF_CFG_MASK                   = 0x07;

    static constexpr std::uint8_t XL_ODR_MASK                     = 0xE0;
    static constexpr std::uint8_t XL_ODR_SHIFT                    = 0x05;

    static constexpr std::uint8_t DEV_RESET_MASK                  = 0x01;


    static constexpr std::uint8_t GYRO_SLEEP_MODE_MASK            = 0x40;

    static constexpr std::uint8_t MSB_8BIT_SHIFT                  = 0x08;

    /**
     * @brief Other variables
     */
    static constexpr std::uint8_t WHO_AM_I_VAL                    = 0x68;
    static constexpr std::uint8_t WHO_AM_I_M_VAL                  = 0x3D;
    static constexpr std::uint8_t ENABLE                          = 0x01;
    static constexpr std::uint8_t DISABLE                         = 0x00;
    static constexpr double ACCEL_GRAVITY                         = 9.81;
};

#endif  // LSM9DS1_H
