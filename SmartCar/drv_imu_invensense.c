/**
 * Copyright 2018 - 2021 HITSIC
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/**
 * @file    :   drv_imu_invensense.c
 * @author  :   17616&C.M.
 *
 * @version :   v1.0
 *
 * @date        v1.0      2021.04.17
 */

#include "drv_imu_invensense.h"

#define SYSLOG_A(...) INV_PRINTF("\r\n");
#define SYSLOG_E(...) INV_PRINTF("\r\n");
#define SYSLOG_W(...) INV_PRINTF("\r\n");
#define SYSLOG_I(...) INV_PRINTF("\r\n");
#define SYSLOG_D(...) INV_PRINTF("\r\n");
#define SYSLOG_V(...) INV_PRINTF("\r\n");

inv_imu_handle_t _IMU_ConstructI2C(inv_i2c_t _i2c, uint8_t _addr) {
    inv_imu_handle_t rtv = INV_MALLOC(sizeof(inv_imu_t));
    memset(rtv, 0, sizeof(inv_imu_t));
    rtv->isSPI = false;
    rtv->i2c = _i2c;
    rtv->i2cTransfer.slaveAddress = _addr;
    rtv->i2cTransfer.subAddressSize = 1;
    if (_addr == IMU_SlaveAddressAutoDetect) {
        rtv->addrAutoDetect = true;
    } else {
        rtv->addrAutoDetect = false;
    }
    return rtv;
}
inv_imu_handle_t _IMU_ConstructSPI(inv_spi_t _spi) {
    inv_imu_handle_t rtv = INV_MALLOC(sizeof(inv_imu_t));
    memset(rtv, 0, sizeof(inv_imu_t));
    rtv->isSPI = true;
    rtv->spi = _spi;
    return rtv;
}


int IMU_WriteReg(inv_imu_handle_t _this, uint8_t reg, uint8_t val) {
    int res = 0;
    if (!_this->isSPI) {
        _this->i2cTransfer.subAddress = reg;
        _this->i2cTransfer.data = &val;
        _this->i2cTransfer.dataSize = 1;
        _this->i2cTransfer.direction = inv_i2c_direction_Write;
        res = _this->i2c.masterTransferBlocking(&_this->i2cTransfer);
        if (res != 0) {
            SYSLOG_D("i2c write return code = %d", res);
        }
    } else {
        uint8_t txb[2];
        uint8_t rxb[2];
        txb[0] = (reg & 0x7fU);
        txb[1] = val;
        _this->spiTransfer.dataSize = 2;
        _this->spiTransfer.rxData = rxb;
        _this->spiTransfer.txData = txb;
        res = _this->spi.masterTransferBlocking(&_this->spiTransfer);
        if (res != 0) {
            SYSLOG_D("spi write return code = %d", res);
        }
    }
    return res;
}
int IMU_WriteRegVerified(inv_imu_handle_t _this, uint8_t reg, uint8_t val) {
    uint8_t regVal;
    int res = 0;
    res |= IMU_WriteReg(_this, reg, val);
    res |= IMU_ReadReg(_this, reg, &regVal);
    if (res == 0 && val != regVal) {
        res |= IMU_WriteReg(_this, reg, val);
        res |= IMU_ReadReg(_this, reg, &regVal);
        if (res == 0 && val != regVal) {
            SYSLOG_D("imu  rw error");
            res |= -1;
        }
    }
    return res;
}
int IMU_ReadReg(inv_imu_handle_t _this, uint8_t reg, uint8_t *val) {
    int res = 0;
    if (!_this->isSPI) {
        _this->i2cTransfer.subAddress = reg;
        _this->i2cTransfer.data = val;
        _this->i2cTransfer.dataSize = 1;
        _this->i2cTransfer.direction = inv_i2c_direction_Read;
        res = _this->i2c.masterTransferBlocking(&_this->i2cTransfer);
        if (res != 0) {
            SYSLOG_D("i2c read return code = %d", res);
        }
    } else {
        uint8_t txb[2];
        uint8_t rxb[2];
        txb[0] = (1U << 7U) | (reg & 0x7f);
        _this->spiTransfer.dataSize = 2;
        _this->spiTransfer.rxData = rxb;
        _this->spiTransfer.txData = txb;
        res = _this->spi.masterTransferBlocking(&_this->spiTransfer);
        if (res != 0) {
            SYSLOG_D("spi read return code = %d", res);
        } else {
            *val = rxb[1];
        }

    }
    return res;
}
int IMU_ModifyReg(inv_imu_handle_t _this, uint8_t reg, uint8_t val, uint8_t mask) {
    uint8_t regVal;
    int res = 0;
    res |= IMU_ReadReg(_this, reg, &regVal);
    res |= IMU_WriteRegVerified(_this, reg, (regVal & (~mask)) | (val & mask));
    res |= IMU_ReadReg(_this, reg, &regVal);
    if ((regVal & mask) != (val & mask)) {
        SYSLOG_D("imu rw error");
        res |= -1;
    }
    return res;
}


const float mpu_accel_fs_map_key[] =
{
        MPU_FS_2G,
        MPU_FS_4G,
        MPU_FS_8G,
        MPU_FS_16G,
};
const int mpu_accel_fs_map_val[] =
{
        0,
        1,
        2,
        3,
};
const struct _inv_weak_map_int_t mpu_accel_fs_map =
{
        .key=mpu_accel_fs_map_key,
        .val =mpu_accel_fs_map_val,
        .n = sizeof(mpu_accel_fs_map_val) / sizeof(int)
};


const float mpu_gyro_fs_map_key[] =
{
        MPU_FS_250dps,
        MPU_FS_500dps,
        MPU_FS_1000dps,
        MPU_FS_2000dps,
};
const int mpu_gyro_fs_map_val[] =
{
        0,
        1,
        2,
        3,
};
const struct _inv_weak_map_int_t mpu_gyro_fs_map =
{
        .key=mpu_gyro_fs_map_key,
        .val =mpu_gyro_fs_map_val,
        .n = sizeof(mpu_gyro_fs_map_val) / sizeof(int)
};


const float mpu_accel_unit_G_map_key[] =
{
        MPU_FS_2G,
        MPU_FS_4G,
        MPU_FS_8G,
        MPU_FS_16G,
};
const float mpu_accel_unit_G_map_val[] =
{
        2.0 / 32768.0,
        4.0 / 32768.0,
        8.0 / 32768.0,
        16.0 / 32768.0,
};
const struct _inv_weak_map_float_t mpu_accel_unit_G_map =
{
        .key=mpu_accel_unit_G_map_key,
        .val =mpu_accel_unit_G_map_val,
        .n = sizeof(mpu_accel_unit_G_map_val) / sizeof(float)
};


const float mpu_gyro_unit_dps_map_key[] =
{
        MPU_FS_250dps,
        MPU_FS_500dps,
        MPU_FS_1000dps,
        MPU_FS_2000dps,
};
const float mpu_gyro_unit_dps_map_val[] =
{
        250.0 / 32768.0,
        500.0 / 32768.0,
        1000.0 / 32768.0,
        2000.0 / 32768.0,
};
const struct _inv_weak_map_float_t mpu_gyro_unit_dps_map =
{
        .key=mpu_gyro_unit_dps_map_key,
        .val =mpu_gyro_unit_dps_map_val,
        .n = sizeof(mpu_gyro_unit_dps_map_val) / sizeof(float)
};


const float mpu_accel_unit_from_G_map_key[] =
{
        MPU_UNIT_MetersPerSquareSecond,
        MPU_UNIT_G,
        MPU_UNIT_mG,
};
const float mpu_accel_unit_from_G_map_val[] =
{
        9.8,
        1,
        1000
};
const struct _inv_weak_map_float_t mpu_accel_unit_from_G_map =
{
        .key=mpu_accel_unit_from_G_map_key,
        .val =mpu_accel_unit_from_G_map_val,
        .n = sizeof(mpu_accel_unit_from_G_map_val) / sizeof(float)
};


const float mpu_gyro_unit_from_dps_map_key[] =
{
        MPU_UNIT_DegPerSec,
        MPU_UNIT_RadPerSec,
        MPU_UNIT_RevolutionsPerMinute,
};
const float mpu_gyro_unit_from_dps_map_val[] =
{
        1,
        3.14159265358979323846 / 180.0,
        60.0 / 360.0,
};
const struct _inv_weak_map_float_t mpu_gyro_unit_from_dps_map =
{
        .key=mpu_gyro_unit_from_dps_map_key,
        .val =mpu_gyro_unit_from_dps_map_val,
        .n = sizeof(mpu_gyro_unit_from_dps_map_val) / sizeof(float)
};


const float MPU9250_GBW_MAP_key[] =
{
        250,
        184,
        92,
        41,
        20,
        10,
        5,
};
const int MPU9250_GBW_MAP_val[] =
{
        0,
        1,
        2,
        3,
        4,
        5,
        6,
};
const struct _inv_weak_map_int_t MPU9250_GBW_MAP =
{
        .key=MPU9250_GBW_MAP_key,
        .val =MPU9250_GBW_MAP_val,
        .n = sizeof(MPU9250_GBW_MAP_val) / sizeof(int)
};


const float MPU9250_ABW_MAP_key[] =
{
        218.1,
        99,
        44.8,
        21.2,
        10.2,
        5.05,
        420,
};
const int MPU9250_ABW_MAP_val[] =
{
        1,
        2,
        3,
        4,
        5,
        6,
        7,
};
const struct _inv_weak_map_int_t MPU9250_ABW_MAP =
{
        .key=MPU9250_ABW_MAP_key,
        .val =MPU9250_ABW_MAP_val,
        .n = sizeof(MPU9250_ABW_MAP_val) / sizeof(int)
};


const float ICM20948_GBW_MAP_key[] =
{
        196.6,
        151.8,
        119.5,
        51.2,
        23.9,
        11.6,
        5.7,
        361.4,
};
const int ICM20948_GBW_MAP_val[] =
{
        0,
        1,
        2,
        3,
        4,
        5,
        6,
        7,
};
const struct _inv_weak_map_int_t ICM20948_GBW_MAP =
{
        .key=ICM20948_GBW_MAP_key,
        .val =ICM20948_GBW_MAP_val,
        .n = sizeof(ICM20948_GBW_MAP_val) / sizeof(int)
};
inv_imu_config_t IMU_ConfigDefault() {
    inv_imu_config_t result;
    result.gyroFullScale = MPU_FS_2000dps;
    result.gyroBandwidth = MPU_GBW_92;
    result.gyroUnit = MPU_UNIT_DegPerSec;
    result.accelFullScale = MPU_FS_8G;
    result.accelBandwidth = MPU_ABW_99;
    result.accelUnit = MPU_UNIT_MetersPerSquareSecond;


    return result;
}
const int IMU_SlaveAddressAutoDetect = 0;
void _IMU_Destruct(inv_imu_handle_t _this) { INV_FREE(_this); }
boolean _IMU_IsOpen(inv_imu_handle_t _this) { return _this->isOpen; }

const uint16_t sSelfTestEquation[256] =
{
        2620, 2646, 2672, 2699, 2726, 2753, 2781, 2808,
        2837, 2865, 2894, 2923, 2952, 2981, 3011, 3041,
        3072, 3102, 3133, 3165, 3196, 3228, 3261, 3293,
        3326, 3359, 3393, 3427, 3461, 3496, 3531, 3566,
        3602, 3638, 3674, 3711, 3748, 3786, 3823, 3862,
        3900, 3939, 3979, 4019, 4059, 4099, 4140, 4182,
        4224, 4266, 4308, 4352, 4395, 4439, 4483, 4528,
        4574, 4619, 4665, 4712, 4759, 4807, 4855, 4903,
        4953, 5002, 5052, 5103, 5154, 5205, 5257, 5310,
        5363, 5417, 5471, 5525, 5581, 5636, 5693, 5750,
        5807, 5865, 5924, 5983, 6043, 6104, 6165, 6226,
        6289, 6351, 6415, 6479, 6544, 6609, 6675, 6742,
        6810, 6878, 6946, 7016, 7086, 7157, 7229, 7301,
        7374, 7448, 7522, 7597, 7673, 7750, 7828, 7906,
        7985, 8065, 8145, 8227, 8309, 8392, 8476, 8561,
        8647, 8733, 8820, 8909, 8998, 9088, 9178, 9270,
        9363, 9457, 9551, 9647, 9743, 9841, 9939, 10038,
        10139, 10240, 10343, 10446, 10550, 10656, 10763, 10870,
        10979, 11089, 11200, 11312, 11425, 11539, 11654, 11771,
        11889, 12008, 12128, 12249, 12371, 12495, 12620, 12746,
        12874, 13002, 13132, 13264, 13396, 13530, 13666, 13802,
        13940, 14080, 14221, 14363, 14506, 14652, 14798, 14946,
        15096, 15247, 15399, 15553, 15709, 15866, 16024, 16184,
        16346, 16510, 16675, 16842, 17010, 17180, 17352, 17526,
        17701, 17878, 18057, 18237, 18420, 18604, 18790, 18978,
        19167, 19359, 19553, 19748, 19946, 20145, 20347, 20550,
        20756, 20963, 21173, 21385, 21598, 21814, 22033, 22253,
        22475, 22700, 22927, 23156, 23388, 23622, 23858, 24097,
        24338, 24581, 24827, 25075, 25326, 25579, 25835, 26093,
        26354, 26618, 26884, 27153, 27424, 27699, 27976, 28255,
        28538, 28823, 29112, 29403, 29697, 29994, 30294, 30597,
        30903, 31212, 31524, 31839, 32157, 32479, 32804
};


