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
 * @file    :   drv_imu_invensense_port.c
 * @author  :   17616&C.M.
 *
 * @version :   v1.0
 *
 * @date        v1.0      2021.04.17
 */

#include "drv_imu_invensense_port.h"
//#include "drv_imu_inv_icm20948.h"
#include "drv_imu_inv_icm20602.h"

inv_imu_config_t icm20602_config;
Quene* inv_imuGyroyQuene;
float inv_imuOffset[6];
float inv_accl[3] = {0.0f, 0.0f, 0.0f};
float inv_gyro[3] = {0.0f, 0.0f, 0.0f};
float inv_imuGyroyAvg;
float inv_imuGyroyOutput;
td_t inv_imuGyroyTD =
{
        .fh = 0,
        .h = 0.005f,
        .r = 20000,
        .v = &inv_imuGyroyAvg,
        .x1 = 0,
        .x2 = 0
};

void INV_ICM20602_Init(void)
{
    spi_init(IMU_SPI_NUM,IMU_SPI_SCLK_PIN,IMU_SPI_MOSI_PIN,IMU_SPI_MISO_PIN,IMU_SPI_CS_PIN,0,1*1000*1000);
    boolean if_init = TRUE;
    inv_spi_t spi_icm20602;
    spi_icm20602.masterTransferBlocking = IMU_INV_SPITransferBlocking;
    spi_icm20602.masterTransferNonBlocking = NULL;
    icm20602 = ICM20602_ConstructSPI(spi_icm20602);
    icm20602_config = IMU_ConfigDefault();
    if (icm20602 != NULL)
    {
        if (0 == ICM20602_Init(icm20602, icm20602_config))
        {
            if(0 != ICM20602_SelfTest(icm20602))
            {
                if_init = FALSE;
            }
        }
        else
        {
            if_init = FALSE;
        }
    }
    else
    {
        if_init = FALSE;
    }
    if (if_init)
    {
        INV_ImuOffsetUpdate();
        return;
    }
    else
    {
        while(1);
    }
}

void INV_ImuUpdate(void)
{
    float data[6];
    ICM20602_ReadSensorBlocking(icm20602);
    ICM20602_Convert(icm20602, data);
    inv_accl[0] = data[0] - inv_imuOffset[0];  ///< 读数并减去零飘
    inv_accl[1] = data[1] - inv_imuOffset[1];
    inv_accl[2] = data[2];
    inv_gyro[0] = INV_ImuDeathZone(data[3]-inv_imuOffset[3], 1);  ///< 角速度设置死区
    inv_gyro[1] = INV_ImuDeathZone(data[4]-inv_imuOffset[4], 1);
    inv_gyro[2] = INV_ImuDeathZone(data[5]-inv_imuOffset[5], 0.25);
    /** ������� */
    inv_imuGyroyAvg = inv_gyro[1] > 0 ? inv_gyro[1] : -inv_gyro[1]; ///< y轴角速度取绝对值ֵ
    INV_DataQueueUpdate(inv_imuGyroyQuene, inv_imuGyroyAvg);        ///< 更新队列
    inv_imuGyroyAvg = INV_ImuGetAverage(inv_imuGyroyQuene);         ///< 均值滤波
    TD_Update(&inv_imuGyroyTD);                                     ///< TD滤波（或许对于坡道可以直接TD）
    inv_imuGyroyOutput = inv_imuGyroyTD.x1;                         ///< 最终输出（上下坡道时该数据将大于某一阈值）
}

void INV_ImuOffsetUpdate(void)
{
    memset(inv_imuOffset, 0, sizeof(inv_imuOffset));
    float data[6];
    for(int i = 0; i < 2000; i++)    ///< 读取2000次取平均以消除零飘
    {
        memset(data, 0, sizeof(data));
        ICM20602_ReadSensorBlocking(icm20602);
        ICM20602_Convert(icm20602, data);
        for(uint8_t j = 0; j < 6; j++)
        {
            inv_imuOffset[j] += data[j];
        }
    }
    for(uint8_t k = 0; k < 6; k++)
    {
        inv_imuOffset[k] /= 2000;
    }
}

float INV_ImuDeathZone(float _raw, float _limit)    ///< 设置读数死区
{
    if(_raw > 0)
    {
        return _raw > _limit ? _raw : 0;
    }
    else
    {
        return _raw < -_limit ? _raw : 0;
    }
}

float INV_ImuGetAverage(Quene *_q)  ///< 均值滤波
{
    float data[CTRL_IMU_QUENE_SIZE];
    float avg = 0.0f;
    INV_GetQueneData(_q, data);
    for(uint8_t i = 0; i < CTRL_IMU_QUENE_SIZE; i++)
    {
        avg += data[i];
    }
    return (avg/CTRL_IMU_QUENE_SIZE);
}

/* ******************** QUENE ******************** */
Quene* INV_DataQueueInit(uint8_t _num)
{
    Node* curnode, *prenode;
    Quene* q = (Quene*)malloc(sizeof(Quene));
    q->num = _num;
    for(uint8_t i = 0; i < _num; i++)
    {
        curnode = (Node *)malloc(sizeof(Node));
        SMARTCAR_ASSERT(curnode);
        curnode->item = 0.0f;
        if(i==0)
        {
            q->head = curnode;
        }
        else if(i == _num-1)
        {
            curnode->pre = prenode;
            prenode->next = curnode;
            q->rear = curnode;
            q->rear->next = q->head;
            q->head->pre = q->rear;
        }
        else
        {
            prenode->next = curnode;
            curnode->pre = prenode;
        }
        prenode = curnode;
    }
    return q;
}

void INV_DataQueueUpdate(Quene* _q, float newdata)
{
    _q->rear = _q->rear->next;
    _q->head = _q->head->next;
    _q->rear->item = newdata;
}

void INV_GetQueneData(Quene* _q, float *_data)
{
    Node* cur = _q->head;
    for(uint8_t i = 0; i < _q->num; i++)
    {
        _data[i] = cur->item;
        cur = cur->next;
    }
}

/* *********************************************** */
