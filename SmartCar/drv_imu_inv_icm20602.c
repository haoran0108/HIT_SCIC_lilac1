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
 * @file    :   drv_imu_inv_icm20602.c
 * @author  :   17616&C.M.
 *
 * @version :   v1.0
 *
 * @date        v1.0      2021.04.18
 */

#include "drv_imu_inv_icm20602.h"
#include "SmartCar_Assert.h"

#define SYSLOG_A(...) INV_PRINTF("\r\n");
#define SYSLOG_E(...) INV_PRINTF("\r\n");
#define SYSLOG_W(...) INV_PRINTF("\r\n");
#define SYSLOG_I(...) INV_PRINTF("\r\n");
#define SYSLOG_D(...) INV_PRINTF("\r\n");
#define SYSLOG_V(...) INV_PRINTF("\r\n");

inv_icm20602_handle_t icm20602;
const int DEF_ST_PRECISION = 1000;
const int DEF_GYRO_CT_SHIFT_DELTA = 500;
const int DEF_ACCEL_ST_SHIFT_DELTA = 500;
/* Gyro Offset Max Value (dps) */
const int DEF_GYRO_OFFSET_MAX = 20;
/* Gyro Self Test Absolute Limits ST_AL (dps) */
const int DEF_GYRO_ST_AL = 60;
/* Accel Self Test Absolute Limits ST_AL (mg) */
const int DEF_ACCEL_ST_AL_MIN = 225;
const int DEF_ACCEL_ST_AL_MAX = 675;

const inv_imu_vector_table_t icm20602_VectorTable =
        {
                .Init = (void *) ICM20602_Init,
                .Detect =(void *) ICM20602_Detect,
                .SelfTest =(void *) ICM20602_SelfTest,
                .Report =(void *) ICM20602_Report,
                .DataReady =(void *) ICM20602_DataReady,
                .EnableDataReadyInt =(void *) ICM20602_EnableDataReadyInt,
                .SoftReset =(void *) ICM20602_SoftReset,
                .ReadSensorBlocking =(void *) ICM20602_ReadSensorBlocking,
                .ReadSensorNonBlocking =(void *) ICM20602_ReadSensorNonBlocking,
                .Convert =(void *) ICM20602_Convert,
                .ConvertRaw =(void *) ICM20602_ConvertRaw,
                .ConvertTemp =(void *) ICM20602_ConvertTemp,
                .IsOpen =(void *) _IMU_IsOpen,
                .Destruct = (void *) ICM20602_Destruct
        };


inv_icm20602_handle_t ICM20602_ConstructI2C(inv_i2c_t _i2c, uint8_t _addr) {
    inv_icm20602_handle_t rtv = (void *) INV_MALLOC(sizeof(inv_icm20602_t));
    inv_imu_handle_t p2parent = _IMU_ConstructI2C(_i2c, _addr);
    memset(rtv, 0, sizeof(inv_icm20602_t));
    rtv->parents = *p2parent;
    _IMU_Destruct(p2parent);

    rtv->parents.vtable = &icm20602_VectorTable;
    rtv->buf = rtv->rxbuf + 1;
    return rtv;
}
inv_icm20602_handle_t ICM20602_ConstructSPI(inv_spi_t _spi) {
    inv_icm20602_handle_t rtv = (void *) INV_MALLOC(sizeof(inv_icm20602_t));
    inv_imu_handle_t p2parent = _IMU_ConstructSPI(_spi);
    memset(rtv, 0, sizeof(inv_icm20602_t));
    rtv->parents = *p2parent;
    _IMU_Destruct(p2parent);

    rtv->parents.vtable = &icm20602_VectorTable;
    rtv->buf = rtv->rxbuf + 1;
    return rtv;
}
int ICM20602_Init(inv_icm20602_handle_t _this, inv_imu_config_t _cfg) {
    SMARTCAR_ASSERT(_this);
    SMARTCAR_ASSERT(_this->parents.vtable);
    _this->parents.cfg = _cfg;
    _this->parents.isOpen = false;
    int res = 0;
    int bw;
    int fs;
    float unit;
    float unit_from;
    if (!IMU_Detect((inv_imu_handle_t) _this)) { return -1; }
    //杞浣�
    res |= IMU_SoftReset((inv_imu_handle_t) _this);

    //鎵撳紑鎵�鏈変紶鎰熷櫒
    res |= IMU_WriteRegVerified((inv_imu_handle_t) _this, (uint8_t) ICM20602_PWR_MGMT_2, 0);

    //1khz閲囨牱鐜�
    res |= IMU_WriteRegVerified((inv_imu_handle_t) _this, (uint8_t) ICM20602_SMPLRT_DIV, 0);

    //閰嶇疆闄�铻轰华lpf
    _InvGetMapVal(ICM20602_GBW_MAP, _this->parents.cfg.gyroBandwidth, bw);
    res |= IMU_WriteRegVerified((inv_imu_handle_t) _this, (uint8_t) ICM20602_CONFIG, bw);

    //閰嶇疆闄�铻轰华閲忕▼鍜屽崟浣�
    _InvGetMapVal(mpu_gyro_unit_dps_map, _this->parents.cfg.gyroFullScale, unit);
    _InvGetMapVal(mpu_gyro_unit_from_dps_map, _this->parents.cfg.gyroUnit, unit_from);
    _this->gyroUnit = unit * unit_from;

    _InvGetMapVal(mpu_gyro_fs_map, _this->parents.cfg.gyroFullScale, fs);
    res |= IMU_WriteRegVerified((inv_imu_handle_t) _this, (uint8_t) ICM20602_GYRO_CONFIG, fs << 3u);

    //閰嶇疆鍔犻�熷害璁￠噺绋嬪拰鍗曚綅
    _InvGetMapVal(mpu_accel_unit_G_map, _this->parents.cfg.accelFullScale, unit);
    _InvGetMapVal(mpu_accel_unit_from_G_map, _this->parents.cfg.accelUnit, unit_from);
    _this->accelUnit = unit * unit_from;

    _InvGetMapVal(mpu_accel_fs_map, _this->parents.cfg.accelFullScale, fs);
    res |= IMU_WriteRegVerified((inv_imu_handle_t) _this, (uint8_t) ICM20602_ACCEL_CONFIG, fs << 3u);

    //閰嶇疆鍔犻�熷害璁pf
    _InvGetMapVal(ICM20602_ABW_MAP, _this->parents.cfg.accelBandwidth, bw);
    res |= IMU_WriteRegVerified((inv_imu_handle_t) _this, (uint8_t) ICM20602_ACCEL_CONFIG2, bw);

    //寮�鍚暟鎹洿鏂颁腑鏂�
    res |= IMU_EnableDataReadyInt((inv_imu_handle_t) _this);

    if (res == 0) {
        _this->parents.isOpen = true;
    }
    return res;
}
bool ICM20602_Detect(inv_icm20602_handle_t _this) {
    uint8_t val = 0;
    if (_this->parents.addrAutoDetect) { _this->parents.i2cTransfer.slaveAddress = 0x68; }
    IMU_ReadReg((inv_imu_handle_t) _this, (uint8_t) ICM20602_WHO_AM_I, &val);
    if (0x12 == val) {
        return true;
    }
    val = 0;
    if (_this->parents.addrAutoDetect) { _this->parents.i2cTransfer.slaveAddress = 0x69; }
    IMU_ReadReg((inv_imu_handle_t) _this, (uint8_t) ICM20602_WHO_AM_I, &val);
    if (0x12 == val) {
        return true;
    }
    return false;
}
int ICM20602_SelfTest(inv_icm20602_handle_t _this) {
    if (!IMU_IsOpen((inv_imu_handle_t) _this)) { return -1; }
    int res = 0;
    inv_imu_config_t backup_cfg = _this->parents.cfg;
    inv_imu_config_t st_cfg = IMU_ConfigDefault();
    st_cfg.gyroFullScale = MPU_FS_250dps;
    st_cfg.accelFullScale = MPU_FS_2G;
    st_cfg.accelBandwidth = MPU_ABW_99;
    st_cfg.gyroBandwidth = MPU_GBW_92;
    if (0 != IMU_Init((inv_imu_handle_t) _this, st_cfg)) {
        IMU_Init((inv_imu_handle_t) _this, backup_cfg);
        return -1;
    }
    int32_t gyro_bias_st[3], gyro_bias_regular[3];
    int32_t accel_bias_st[3], accel_bias_regular[3];
    int16_t abuf[9];
    int16_t *gbuf = &abuf[3];
    int accel_result = 0;
    int gyro_result = 0;
    uint8_t val;
    memset(gyro_bias_st, 0, sizeof(gyro_bias_st));
    memset(gyro_bias_regular, 0, sizeof(gyro_bias_regular));
    memset(accel_bias_st, 0, sizeof(accel_bias_st));
    memset(accel_bias_regular, 0, sizeof(accel_bias_regular));

    int times;
    times = 20;
    while (times--) { while (!IMU_DataReady((inv_imu_handle_t) _this)) {}}//涓㈠純鍓�20涓暟鎹�
    times = 20;
    while (times--) {
        while (!IMU_DataReady((inv_imu_handle_t) _this)) {}
        res |= IMU_ReadSensorBlocking((inv_imu_handle_t) _this);
        IMU_ConvertRaw((inv_imu_handle_t) _this, abuf);
        for (int i = 0; i < 3; ++i) {
            gyro_bias_regular[i] += gbuf[i];
            accel_bias_regular[i] += abuf[i];
        }
    }

    res |= IMU_ReadReg((inv_imu_handle_t) _this, (uint8_t) ICM20602_GYRO_CONFIG, &val);
    res |= IMU_WriteRegVerified((inv_imu_handle_t) _this, (uint8_t) ICM20602_GYRO_CONFIG, val | (0x7/*0b111*/ << 5));//鎵撳紑闄�铻轰华鑷
    res |= IMU_ReadReg((inv_imu_handle_t) _this, (uint8_t) ICM20602_ACCEL_CONFIG, &val);
    res |= IMU_WriteRegVerified((inv_imu_handle_t) _this, (uint8_t) ICM20602_ACCEL_CONFIG, val | (0x7/*0b111*/ << 5));//鎵撳紑鍔犻�熷害璁¤嚜妫�
    times = 20;
    while (times--) { while (!IMU_DataReady((inv_imu_handle_t) _this)) {}}//涓㈠純鍓�20涓暟鎹�
    times = 20;
    while (times--) {
        while (!IMU_DataReady((inv_imu_handle_t) _this)) {}
        res |= IMU_ReadSensorBlocking((inv_imu_handle_t) _this);
        IMU_ConvertRaw((inv_imu_handle_t) _this, abuf);
        for (int i = 0; i < 3; ++i) {
            gyro_bias_st[i] += gbuf[i];
            accel_bias_st[i] += abuf[i];
        }
    }

    for (int i = 0; i < 3; ++i) {
        gyro_bias_regular[i] *= 50;   //(32768/2000)*1000 LSB/mg
        accel_bias_regular[i] *= 50;
        gyro_bias_st[i] *= 50;         //(32768/250)*1000 LSB/dps
        accel_bias_st[i] *= 50;
    }


    //璁＄畻鍔犻�熷害璁¤嚜妫�缁撴灉
    uint8_t regs[3];
    int otp_value_zero = 0;
    int st_shift_prod[3], st_shift_cust[3], st_shift_ratio[3], i;
//    int result;

    res |= IMU_ReadReg((inv_imu_handle_t) _this, (uint8_t) ICM20602_SELF_TEST_X_ACCEL, regs);
    res |= IMU_ReadReg((inv_imu_handle_t) _this, (uint8_t) ICM20602_SELF_TEST_Y_ACCEL, regs + 1);
    res |= IMU_ReadReg((inv_imu_handle_t) _this, (uint8_t) ICM20602_SELF_TEST_Z_ACCEL, regs + 2);
    for (i = 0; i < 3; i++) {
        if (regs[i] != 0) {
            st_shift_prod[i] = sSelfTestEquation[regs[i] - 1];
        } else {
            st_shift_prod[i] = 0;
            otp_value_zero = 1;
        }
    }

    if (!otp_value_zero) {
        /* Self Test Pass/Fail Criteria A */
        for (i = 0; i < 3; i++) {
            st_shift_cust[i] = accel_bias_st[i] - accel_bias_regular[i];
            st_shift_ratio[i] = abs(st_shift_cust[i] / st_shift_prod[i] - DEF_ST_PRECISION);
            if (st_shift_ratio[i] > DEF_ACCEL_ST_SHIFT_DELTA) {
                //鍔犻�熷害璁¤嚜妫�鏈�氳繃
                accel_result = 1;
                SYSLOG_D("accel[%d] st fail,result = %d,it demands less than %d", i, st_shift_ratio[i],
                         DEF_ACCEL_ST_SHIFT_DELTA);
            } else {
                SYSLOG_I("accel[%d] st result = %d,it demands less than %d", i, st_shift_ratio[i],
                         DEF_ACCEL_ST_SHIFT_DELTA);
            }
        }
    } else {
        /* Self Test Pass/Fail Criteria B */
        for (i = 0; i < 3; i++) {
            st_shift_cust[i] = abs(accel_bias_st[i] - accel_bias_regular[i]);
            if (st_shift_cust[i] < DEF_ACCEL_ST_AL_MIN * (32768 / 2000) * 1000
                || st_shift_cust[i] > DEF_ACCEL_ST_AL_MAX * (32768 / 2000) * 1000) {
                //鍔犻�熷害璁¤嚜妫�鏈�氳繃
                accel_result = 1;
                SYSLOG_D("accel[%d] st fail,result = %d,it demands <%d && >%d", i, st_shift_cust[i],
                         DEF_ACCEL_ST_AL_MAX * (32768 / 2000) * 1000, DEF_ACCEL_ST_AL_MIN * (32768 / 2000) * 1000);
            } else {
                SYSLOG_I("accel[%d] st result = %d,it demands <%d && >%d", i, st_shift_cust[i],
                         DEF_ACCEL_ST_AL_MAX * (32768 / 2000) * 1000, DEF_ACCEL_ST_AL_MIN * (32768 / 2000) * 1000);
            }
        }
    }

    //璁＄畻闄�铻轰华鑷缁撴灉
    res |= IMU_ReadReg((inv_imu_handle_t) _this, (uint8_t) ICM20602_SELF_TEST_X_GYRO, regs);
    res |= IMU_ReadReg((inv_imu_handle_t) _this, (uint8_t) ICM20602_SELF_TEST_Y_GYRO, regs + 1);
    res |= IMU_ReadReg((inv_imu_handle_t) _this, (uint8_t) ICM20602_SELF_TEST_Z_GYRO, regs + 2);
    for (i = 0; i < 3; i++) {
        if (regs[i] != 0) {
            st_shift_prod[i] = sSelfTestEquation[regs[i] - 1];
        } else {
            st_shift_prod[i] = 0;
            otp_value_zero = 1;
        }
    }

    for (i = 0; i < 3; i++) {
        st_shift_cust[i] = gyro_bias_st[i] - gyro_bias_regular[i];
        if (!otp_value_zero) {
            /* Self Test Pass/Fail Criteria A */
            if (st_shift_cust[i] < DEF_GYRO_CT_SHIFT_DELTA * st_shift_prod[i]) {
                //闄�铻轰华鑷娌¤繃
                gyro_result = 1;
                SYSLOG_D("gyro[%d] st fail,result = %d,it demands greater than %d", i, st_shift_cust[i],
                         DEF_GYRO_CT_SHIFT_DELTA * st_shift_prod[i]);
            } else {
                SYSLOG_I("gyro[%d] st result = %d,it demands greater than %d", i, st_shift_cust[i],
                         DEF_GYRO_CT_SHIFT_DELTA * st_shift_prod[i]);
            }
        } else {
            /* Self Test Pass/Fail Criteria B */
            if (st_shift_cust[i] < DEF_GYRO_ST_AL * (32768 / 250) * DEF_ST_PRECISION) {
                //闄�铻轰华鑷娌¤繃
                gyro_result = 1;
                SYSLOG_D("gyro[%d] st fail,result = %d,it demands greater than %d", i, st_shift_cust[i],
                         DEF_GYRO_ST_AL * (32768 / 250) * DEF_ST_PRECISION);
            } else {
                SYSLOG_I("gyro[%d] st result = %d,it demands greater than %d", i, st_shift_cust[i],
                         DEF_GYRO_ST_AL * (32768 / 250) * DEF_ST_PRECISION);
            }
        }
    }

    if (gyro_result == 0) {
        /* Self Test Pass/Fail Criteria C */
        for (i = 0; i < 3; i++) {
            if (abs(gyro_bias_regular[i]) > DEF_GYRO_OFFSET_MAX * (32768 / 250) * DEF_ST_PRECISION)
                //闄�铻轰华鑷娌¤繃
            {
                gyro_result = 1;
                SYSLOG_D("gyro[%d] st fail,result = %d,ift demands less than %d", i, (int) abs(gyro_bias_regular[i]),
                         DEF_GYRO_OFFSET_MAX * (32768 / 250) * DEF_ST_PRECISION);
            } else {
                SYSLOG_I("gyro[%d] st result = %d,it demands less than %d", i, (int) abs(gyro_bias_regular[i]),
                         DEF_GYRO_OFFSET_MAX * (32768 / 250) * DEF_ST_PRECISION);
            }
        }
    }

    //鎭㈠鍘熸潵鐨勯厤缃�
    res |= IMU_Init((inv_imu_handle_t) _this, backup_cfg);
    return res | (gyro_result << 1) | accel_result;
}
bool ICM20602_DataReady(inv_icm20602_handle_t _this) {
    uint8_t val = 0;
    IMU_ReadReg((inv_imu_handle_t) _this, (uint8_t) ICM20602_INT_STATUS, &val);
    return (val & 0x01) == 0x01;
}
int ICM20602_EnableDataReadyInt(inv_icm20602_handle_t _this) {
    return IMU_ModifyReg((inv_imu_handle_t) _this, (uint8_t) ICM20602_INT_ENABLE, 0x01, 0x01);
}
int ICM20602_SoftReset(inv_icm20602_handle_t _this) {
    if (!IMU_Detect((inv_imu_handle_t) _this)) { return -1; }
    int res = 0;
    int times;
    uint8_t val;
    //澶嶄綅
    res |= IMU_WriteReg((inv_imu_handle_t) _this, (uint8_t) ICM20602_PWR_MGMT_1, 0x80);
    //绛夊緟澶嶄綅鎴愬姛
    times = 100;//灏濊瘯100娆�
    do {
        INV_DELAY(5);
        res |= IMU_ReadReg((inv_imu_handle_t) _this, (uint8_t) ICM20602_PWR_MGMT_1, &val);
    } while (val != 0x41 && res == 0 && --times);
    if (times == 0) {
        SYSLOG_I("Time out!! 0x%x at PWR_MGMT_1,when waiting it get 0x41", val);
        return -1;
    }
    //鍞よ捣鐫＄湢
    res |= IMU_WriteReg((inv_imu_handle_t) _this, (uint8_t) ICM20602_PWR_MGMT_1, 0x1);
    //绛夊緟鍞よ捣
    times = 100;//灏濊瘯100娆�
    do {
        INV_DELAY(5);
        res |= IMU_ReadReg((inv_imu_handle_t) _this, (uint8_t) ICM20602_PWR_MGMT_1, &val);
    } while (val != 0x1 && res == 0 && --times);
    if (times == 0) {
        SYSLOG_I("Time out!! 0x%x at PWR_MGMT_1,when waiting it get 0x1", val);
        return -1;
    }
    return res;
}
int ICM20602_ReadSensorBlocking(inv_icm20602_handle_t _this) {
    int res;
    if (!_this->parents.isSPI) {
        _this->parents.i2cTransfer.subAddress = (uint8_t) ICM20602_ACCEL_XOUT_H;
        _this->parents.i2cTransfer.data = _this->buf;
        _this->parents.i2cTransfer.dataSize = 14;
        _this->parents.i2cTransfer.direction = inv_i2c_direction_Read;
        res = _this->parents.i2c.masterTransferBlocking(&_this->parents.i2cTransfer);
        if (res != 0) {
            SYSLOG_D("i2c read return code = %d", res);
        }
    } else {
        _this->txbuf[0] = (1U << 7U) | ((uint8_t) ICM20602_ACCEL_XOUT_H & 0x7fU);
        _this->parents.spiTransfer.dataSize = 15;
        _this->parents.spiTransfer.rxData = _this->rxbuf;
        _this->parents.spiTransfer.txData = _this->txbuf;
        res = _this->parents.spi.masterTransferBlocking(&_this->parents.spiTransfer);
        if (res != 0) {
            SYSLOG_D("spi read return code = %d", res);
        }
    }
    return res;
}
int ICM20602_ReadSensorNonBlocking(inv_icm20602_handle_t _this) {
    int res;
    if (!_this->parents.isSPI) {
        _this->parents.i2cTransfer.subAddress = (uint8_t) ICM20602_ACCEL_XOUT_H;
        _this->parents.i2cTransfer.data = _this->buf;
        _this->parents.i2cTransfer.dataSize = 14;
        _this->parents.i2cTransfer.direction = inv_i2c_direction_Read;
        res = _this->parents.i2c.masterTransferNonBlocking(&_this->parents.i2cTransfer);
        if (res != 0) {
            SYSLOG_D("i2c read return code = %d", res);
        }
    } else {
        _this->txbuf[0] = (1U << 7U) | ((uint8_t) ICM20602_ACCEL_XOUT_H & 0x7fU);
        _this->parents.spiTransfer.dataSize = 15;
        _this->parents.spiTransfer.rxData = _this->rxbuf;
        _this->parents.spiTransfer.txData = _this->txbuf;
        res = _this->parents.spi.masterTransferNonBlocking(&_this->parents.spiTransfer);
        if (res != 0) {
            SYSLOG_D("spi read return code = %d", res);
        }
    }
    return res;
}
int ICM20602_Convert(inv_icm20602_handle_t _this, float *array) {
    uint8_t *buf = _this->buf;
    array[0] = _this->accelUnit * ((int16_t) ((buf[0] << 8) | buf[1]));
    array[1] = _this->accelUnit * ((int16_t) ((buf[2] << 8) | buf[3]));
    array[2] = _this->accelUnit * ((int16_t) ((buf[4] << 8) | buf[5]));
    array[3] = _this->gyroUnit * ((int16_t) ((buf[8] << 8) | buf[9]));
    array[4] = _this->gyroUnit * ((int16_t) ((buf[10] << 8) | buf[11]));
    array[5] = _this->gyroUnit * ((int16_t) ((buf[12] << 8) | buf[13]));
    return 0;
}
int ICM20602_ConvertRaw(inv_icm20602_handle_t _this, int16_t *raw) {
    uint8_t *buf = _this->buf;
    raw[0] = ((int16_t) ((buf[0] << 8) | buf[1]));
    raw[1] = ((int16_t) ((buf[2] << 8) | buf[3]));
    raw[2] = ((int16_t) ((buf[4] << 8) | buf[5]));
    raw[3] = ((int16_t) ((buf[8] << 8) | buf[9]));
    raw[4] = ((int16_t) ((buf[10] << 8) | buf[11]));
    raw[5] = ((int16_t) ((buf[12] << 8) | buf[13]));
    return 0;
}
int ICM20602_ConvertTemp(inv_icm20602_handle_t _this, float *temp) {
    if (temp) { *temp = (float) ((int16_t) (_this->buf[6] << 8) | _this->buf[7]) / 326.8f + 25.0f; }
    return 0;
}

