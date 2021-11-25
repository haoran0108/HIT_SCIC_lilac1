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
 * @file    :   drv_imu_invensense.h
 * @author  :   17616&C.M.
 *
 * @version :   v1.0
 *
 * @date        v1.0      2021.04.17
 */

#ifndef EMT1_DRV_IMU_INVENSENSE_H_
#define EMT1_DRV_IMU_INVENSENSE_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <float.h>
#include "common.h"
#include "drv_imu_invensense_port.h"
#include "SmartCar_Assert.h"

typedef enum mpu_accel_fs
{
    MPU_FS_2G = 2,
    MPU_FS_4G = 4,
    MPU_FS_8G = 8,
    MPU_FS_16G = 16,
}mpu_accel_fs_t;

typedef enum mpu_accel_bw
{
    MPU_ABW_420 = 420,
    MPU_ABW_218 = 218,
    MPU_ABW_99 = 99,
    MPU_ABW_45 = 45,
    MPU_ABW_21 = 21,
    MPU_ABW_10 = 10,
    MPU_ABW_5 = 5,
}mpu_accel_bw_t;

typedef enum mpu_gyro_fs
{
    MPU_FS_125dps = 125,
    MPU_FS_250dps = 250,
    MPU_FS_500dps = 500,
    MPU_FS_1000dps = 1000,
    MPU_FS_2000dps = 2000,
}mpu_gyro_fs_t;

typedef enum mpu_gyro_bw
{
    MPU_GBW_361 = 361,
    MPU_GBW_250 = 250,
    MPU_GBW_176 = 176,
    MPU_GBW_92 = 92,
    MPU_GBW_41 = 41,
    MPU_GBW_20 = 20,
    MPU_GBW_10 = 10,
    MPU_GBW_5 = 5,
}mpu_gyro_bw_t;

typedef enum mpu_gyro_unit
{
    MPU_UNIT_DegPerSec,
    MPU_UNIT_RadPerSec,
    MPU_UNIT_RevolutionsPerMinute,
}mpu_gyro_unit_t;

typedef enum mpu_accel_unit
{
    MPU_UNIT_MetersPerSquareSecond,
    MPU_UNIT_G,
    MPU_UNIT_mG
}mpu_accel_unit_t;

typedef struct __imu_config
{
    mpu_accel_fs_t accelFullScale;
    mpu_accel_bw_t accelBandwidth;
    mpu_gyro_fs_t gyroFullScale;
    mpu_gyro_bw_t gyroBandwidth;
    mpu_gyro_unit_t gyroUnit;
    mpu_accel_unit_t accelUnit;
} inv_imu_config_t;

inv_imu_config_t IMU_ConfigDefault(void);

typedef struct __inv_imu_vector_table
{
    int (*Init)(void *_this, inv_imu_config_t _cfg);
    boolean (*Detect)(void *_this);
    int (*SelfTest)(void *_this);
    const char *(*Report)(void *_this);
    boolean (*DataReady)(void *_this);
    int (*EnableDataReadyInt)(void *_this);
    int (*SoftReset)(void *_this);
    int (*ReadSensorBlocking)(void *_this);
    int (*ReadSensorNonBlocking)(void *_this);
    int (*Convert)(void *_this, float array[9]);
    int (*ConvertRaw)(void *_this, int16_t raw[9]);
    int (*ConvertTemp)(void *_this, float *temp);
    boolean (*IsOpen)(void *_this);
    void (*Destruct)(void *_this);
} inv_imu_vector_table_t;

typedef struct __inv_imu
{
    inv_imu_vector_table_t *vtable;
    union
    {
        inv_i2c_t i2c;
        inv_spi_t spi;
    };
    union
    {
        inv_i2c_transfer_t i2cTransfer;
        inv_spi_transfer_t spiTransfer;
    };
    bool isSPI;
    bool addrAutoDetect;
    bool isOpen;
    inv_imu_config_t cfg;
} inv_imu_t, *inv_imu_handle_t;


static inline int IMU_Init(inv_imu_handle_t _this, inv_imu_config_t _cfg)
{
    SMARTCAR_ASSERT(_this);
    SMARTCAR_ASSERT(_this->vtable);
    return _this->vtable->Init(_this, _cfg);
}
static inline bool IMU_Detect(inv_imu_handle_t _this) { return _this->vtable->Detect(_this); }
static inline int IMU_SelfTest(inv_imu_handle_t _this) { return _this->vtable->SelfTest(_this); }
static inline const char *IMU_Report(inv_imu_handle_t _this) { return _this->vtable->Report(_this); }
static inline bool IMU_DataReady(inv_imu_handle_t _this) { return _this->vtable->DataReady(_this); }
static inline int IMU_EnableDataReadyInt(inv_imu_handle_t _this) { return _this->vtable->EnableDataReadyInt(_this); }
static inline int IMU_SoftReset(inv_imu_handle_t _this) { return _this->vtable->SoftReset(_this); }
static inline int IMU_ReadSensorBlocking(inv_imu_handle_t _this) { return _this->vtable->ReadSensorBlocking(_this); }
static inline int IMU_ReadSensorNonBlocking(inv_imu_handle_t _this) { return _this->vtable->ReadSensorNonBlocking(_this); }
//椤哄簭鏄� accel(xyz) gyro(xyz) mag(xyz)
static inline int IMU_Convert(inv_imu_handle_t _this, float array[9]) { return _this->vtable->Convert(_this, array); };
static inline int IMU_ConvertRaw(inv_imu_handle_t _this, int16_t raw[9]) { return _this->vtable->ConvertRaw(_this, raw); }
static inline int IMU_ConvertTemp(inv_imu_handle_t _this, float *temp) { return _this->vtable->ConvertTemp(_this, temp); }
static inline bool IMU_IsOpen(inv_imu_handle_t _this) { return _this->vtable->IsOpen(_this); }
static inline void IMU_Destruct(inv_imu_handle_t _this) { return _this->vtable->Destruct(_this); }

void _IMU_Destruct(inv_imu_handle_t _this);
inv_imu_handle_t _IMU_ConstructI2C(inv_i2c_t _i2c, uint8_t _addr);
inv_imu_handle_t _IMU_ConstructSPI(inv_spi_t _spi);
int IMU_WriteReg(inv_imu_handle_t _this, uint8_t reg, uint8_t val);
int IMU_WriteRegVerified(inv_imu_handle_t _this, uint8_t reg, uint8_t val);
int IMU_ReadReg(inv_imu_handle_t _this, uint8_t reg, uint8_t *val);
int IMU_ModifyReg(inv_imu_handle_t _this, uint8_t reg, uint8_t val, uint8_t mask);
boolean _IMU_IsOpen(inv_imu_handle_t _this);
inv_imu_handle_t IMU_AutoConstructI2C(inv_i2c_t _i2c, uint8_t _addr);
inv_imu_handle_t IMU_AutoConstructSPI(inv_spi_t _spi);

struct _inv_weak_map_int_t
{
    const float *key;
    const int *val;
    int n;
};

struct _inv_weak_map_float_t
{
    const float *key;
    const float *val;
    int n;
};

#define _InvGetMapVal(map, _key, the_val) { \
int n = 0;\
float distance = FLT_MAX;\
float buf;\
for (int i = 0; i < map.n; ++i) {\
buf = map.key[i] - _key;\
buf *= buf;\
if (buf < distance) {\
distance = buf;\
n = i;\
}\
}  the_val = map.val[n];}

extern const int IMU_SlaveAddressAutoDetect;
extern const struct _inv_weak_map_int_t mpu_accel_fs_map;
extern const struct _inv_weak_map_int_t mpu_gyro_fs_map;
extern const struct _inv_weak_map_float_t mpu_accel_unit_G_map;
extern const struct _inv_weak_map_float_t mpu_gyro_unit_dps_map;
extern const struct _inv_weak_map_float_t mpu_accel_unit_from_G_map;
extern const struct _inv_weak_map_float_t mpu_gyro_unit_from_dps_map;
extern const struct _inv_weak_map_int_t MPU9250_GBW_MAP;
extern const struct _inv_weak_map_int_t MPU9250_ABW_MAP;
extern const struct _inv_weak_map_int_t ICM20948_GBW_MAP;

#define MPU6050_GBW_MAP   MPU9250_GBW_MAP
#define ICM20602_GBW_MAP  MPU9250_GBW_MAP
#define ICM20602_ABW_MAP  MPU9250_ABW_MAP
#define ICM20948_ABW_MAP  MPU9250_ABW_MAP

extern const uint16_t sSelfTestEquation[256];

#endif /* _DRV_IMU_INVENSENSE_H_ */
