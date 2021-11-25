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
 * @file    :   drv_imu_inv_icm20602.h
 * @author  :   17616&C.M.
 *
 * @version :   v1.0
 *
 * @date        v1.0      2021.04.18
 */

#ifndef _DRV_IMU_INV_ICM20602_H_
#define _DRV_IMU_INV_ICM20602_H_

#include "drv_imu_invensense.h"
#include "SmartCar_Assert.h"

typedef struct __inv_icm20602 {
    inv_imu_t parents;
    float gyroUnit;
    float accelUnit;
    uint8_t *buf;
    uint8_t txbuf[15];
    uint8_t rxbuf[15];
} inv_icm20602_t, *inv_icm20602_handle_t;

extern inv_icm20602_handle_t icm20602;
extern float inv_gyro[3] ;

inline void ICM20602_Destruct(inv_icm20602_handle_t _this) { _IMU_Destruct((void *) _this); }
inv_icm20602_handle_t ICM20602_ConstructI2C(inv_i2c_t _i2c, uint8_t _addr);
inv_icm20602_handle_t ICM20602_ConstructSPI(inv_spi_t _spi);


int ICM20602_Init(inv_icm20602_handle_t _this, inv_imu_config_t _cfg);
bool ICM20602_Detect(inv_icm20602_handle_t _this);
int ICM20602_SelfTest(inv_icm20602_handle_t _this);
inline const char *ICM20602_Report(inv_icm20602_handle_t _this) { return " icm20602"; }
bool ICM20602_DataReady(inv_icm20602_handle_t _this);
int ICM20602_EnableDataReadyInt(inv_icm20602_handle_t _this);
int ICM20602_SoftReset(inv_icm20602_handle_t _this);
int ICM20602_ReadSensorBlocking(inv_icm20602_handle_t _this);
int ICM20602_ReadSensorNonBlocking(inv_icm20602_handle_t _this);
int ICM20602_Convert(inv_icm20602_handle_t _this, float array[9]);
int ICM20602_ConvertRaw(inv_icm20602_handle_t _this, int16_t raw[9]);
int ICM20602_ConvertTemp(inv_icm20602_handle_t _this, float *temp);


enum ICM20602_RegMap {
    ICM20602_XG_OFFS_TC_H = 0x4,            // READ/ WRITE
    ICM20602_XG_OFFS_TC_L = 0x5,            // READ/ WRITE
    ICM20602_YG_OFFS_TC_H = 0x7,            // READ/ WRITE
    ICM20602_YG_OFFS_TC_L = 0x8,            // READ/ WRITE
    ICM20602_ZG_OFFS_TC_H = 0x0A,        // READ/ WRITE
    ICM20602_ZG_OFFS_TC_L = 0x0B,        // READ/ WRITE
    ICM20602_SELF_TEST_X_ACCEL = 0x0D,    // READ/ WRITE
    ICM20602_SELF_TEST_Y_ACCEL = 0x0E,    // READ/ WRITE
    ICM20602_SELF_TEST_Z_ACCEL = 0x0F,    // READ/ WRITE
    ICM20602_XG_OFFS_USRH = 0x13,        // READ/ WRITE
    ICM20602_XG_OFFS_USRL = 0x14,        // READ/ WRITE
    ICM20602_YG_OFFS_USRH = 0x15,        // READ/ WRITE
    ICM20602_YG_OFFS_USRL = 0x16,        // READ/ WRITE
    ICM20602_ZG_OFFS_USRH = 0x17,        // READ/ WRITE
    ICM20602_ZG_OFFS_USRL = 0x18,        // READ/ WRITE
    ICM20602_SMPLRT_DIV = 0x19,            // READ/ WRITE
    ICM20602_CONFIG = 0x1A,                // READ/ WRITE default value:0x80
    ICM20602_GYRO_CONFIG = 0x1B,            // READ/ WRITE
    ICM20602_ACCEL_CONFIG = 0x1C,        // READ/ WRITE
    ICM20602_ACCEL_CONFIG2 = 0x1D,        // READ/ WRITE
    ICM20602_LP_MODE_CFG = 0x1E,            // READ/ WRITE
    ICM20602_ACCEL_WOM_X_THR = 0x20,        // READ/ WRITE
    ICM20602_ACCEL_WOM_Y_THR = 0x21,        // READ/ WRITE
    ICM20602_ACCEL_WOM_Z_THR = 0x22,        // READ/ WRITE
    ICM20602_FIFO_EN = 0x23,                // READ/ WRITE
    ICM20602_FSYNC_INT = 0x36,            // READ to CLEAR
    ICM20602_INT_PIN_CFG = 0x37,            // READ/ WRITE
    ICM20602_INT_ENABLE = 0x38,            // READ/ WRITE
    ICM20602_FIFO_WM_INT_STATUS = 0x39,    // READ to CLEAR
    ICM20602_INT_STATUS = 0x3A,            // READ to CLEAR
    ICM20602_ACCEL_XOUT_H = 0x3B,        // READ
    ICM20602_ACCEL_XOUT_L = 0x3C,        // READ
    ICM20602_ACCEL_YOUT_H = 0x3D,        // READ
    ICM20602_ACCEL_YOUT_L = 0x3E,        // READ
    ICM20602_ACCEL_ZOUT_H = 0x3F,        // READ
    ICM20602_ACCEL_ZOUT_L = 0x40,        // READ
    ICM20602_TEMP_OUT_H = 0x41,            // READ
    ICM20602_TEMP_OUT_L = 0x42,            // READ
    ICM20602_GYRO_XOUT_H = 0x43,            // READ
    ICM20602_GYRO_XOUT_L = 0x44,            // READ
    ICM20602_GYRO_YOUT_H = 0x45,            // READ
    ICM20602_GYRO_YOUT_L = 0x46,            // READ
    ICM20602_GYRO_ZOUT_H = 0x47,            // READ
    ICM20602_GYRO_ZOUT_L = 0x48,            // READ
    ICM20602_SELF_TEST_X_GYRO = 0x50,    // READ/ WRITE
    ICM20602_SELF_TEST_Y_GYRO = 0x51,    // READ/ WRITE
    ICM20602_SELF_TEST_Z_GYRO = 0x52,    // READ/ WRITE
    ICM20602_FIFO_WM_TH1 = 0x60,            // READ/ WRITE
    ICM20602_FIFO_WM_TH2 = 0x61,            // READ/ WRITE
    ICM20602_SIGNAL_PATH_RESET = 0x68,    // READ/ WRITE
    ICM20602_ACCEL_INTEL_CTRL = 0x69,    // READ/ WRITE
    ICM20602_USER_CTRL = 0x6A,            // READ/ WRITE
    ICM20602_PWR_MGMT_1 = 0x6B,            // READ/ WRITE default value:0x41
    ICM20602_PWR_MGMT_2 = 0x6C,            // READ/ WRITE
    ICM20602_I2C_IF = 0x70,                // READ/ WRITE
    ICM20602_FIFO_COUNTH = 0x72,            // READ
    ICM20602_FIFO_COUNTL = 0x73,            // READ
    ICM20602_FIFO_R_W = 0x74,            // READ/ WRITE
    ICM20602_WHO_AM_I = 0x75,            // READ default value:0x12
    ICM20602_XA_OFFSET_H = 0x77,            // READ/ WRITE
    ICM20602_XA_OFFSET_L = 0x78,            // READ/ WRITE
    ICM20602_YA_OFFSET_H = 0x7A,            // READ/ WRITE
    ICM20602_YA_OFFSET_L = 0x7B,            // READ/ WRITE
    ICM20602_ZA_OFFSET_H = 0x7D,            // READ/ WRITE
    ICM20602_ZA_OFFSET_L = 0x7E,            // READ/ WRITE
};

#endif /* DRV_IMU_INV_ICM20602_H_ */
