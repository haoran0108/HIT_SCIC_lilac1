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
 * @file    :   drv_imu_invensense_port.h
 * @author  :   17616&C.M.
 *
 * @version :   v1.0
 *
 * @date        v1.0      2021.04.17
 */

#ifndef EMT1_DRV_IMU_INVENSENSE_PORT_H_
#define EMT1_DRV_IMU_INVENSENSE_PORT_H_

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "common.h"
#include "SmartCar_SPI.h"
#include "SmartCar_Systick.h"
#include "lib_algo_td.h"

#define INV_MALLOC malloc
#define INV_FREE free

#define INV_PRINTF(...)

#define INV_DELAY(millisecond)  Delay_ms(STM0, millisecond)

#define IMU_SPI_CS_PIN SPI0_CS0_P20_8
#define IMU_SPI_MISO_PIN SPI0_MISO_P20_12
#define IMU_SPI_MOSI_PIN SPI0_MOSI_P20_14
#define IMU_SPI_SCLK_PIN SPI0_SCLK_P20_13
#define IMU_SPI_NUM SPI_0

typedef enum __inv_i2c_direction
{
    inv_i2c_direction_Write = 0U, /*!< Master transmit. */
    inv_i2c_direction_Read = 1U  /*!< Master receive. */
} inv_i2c_direction_t;

typedef struct __inv_i2c_transfer
{
    uint8_t slaveAddress;      /*!< 7-bit slave address. */
    uint8_t subAddressSize;     /*!< A size of the command buffer. */
    uint32_t subAddress;        /*!< A sub address. Transferred MSB first. */
    void *volatile data;        /*!< A transfer buffer. */
    volatile uint32_t dataSize;          /*!< A transfer size. */
    inv_i2c_direction_t direction; /*!< A transfer direction, read or write. */
    /*******************************************************/
    //you like
} inv_i2c_transfer_t;

typedef struct __inv_i2c
{
    int (*masterTransferBlocking)(const inv_i2c_transfer_t *);
    int (*masterTransferNonBlocking)(const inv_i2c_transfer_t *);
} inv_i2c_t;

typedef struct __inv_spi_transfer
{
    uint8_t *volatile txData;          /*!< Send buffer. */
    uint8_t *volatile rxData;          /*!< Receive buffer. */
    volatile uint32_t dataSize; /*!< Transfer bytes. */
    /*******************************************************/
    //you like
} inv_spi_transfer_t;

typedef struct __inv_spi
{
    int (*masterTransferBlocking)(const inv_spi_transfer_t *);
    int (*masterTransferNonBlocking)(const inv_spi_transfer_t *);
} inv_spi_t;

inline int IMU_INV_SPITransferBlocking(const inv_spi_transfer_t * spi)
{
    spi_mosi(IMU_SPI_NUM, IMU_SPI_CS_PIN, spi->txData, spi->rxData, spi->dataSize, 1);
    return 0;
}

#define CTRL_IMU_QUENE_SIZE     (25U)
typedef struct node
{
    float item;
    struct node * next;
    struct node * pre;
}Node;

typedef struct quene
{
        Node* head;
        Node* rear;
        uint8_t num;
}Quene;

//void INV_ICM20948_Init(void);
extern Quene* inv_imuGyroyQuene;
extern float inv_imuOffset[6];
extern float inv_accl[3];
extern float inv_gyro[3];
extern float inv_imuGyroyAvg;
extern float inv_imuGyroyOutput;
extern td_t inv_imuGyroyTD;
void INV_ICM20602_Init(void);
void INV_ImuUpdate(void);
void INV_ImuOffsetUpdate(void);
float INV_ImuDeathZone(float _raw, float _limit);
float INV_ImuGetAverage(Quene *_q);


Quene* INV_DataQueueInit(uint8_t _num);
void INV_DataQueueUpdate(Quene* _q, float newdata);
void INV_GetQueneData(Quene* _q, float *_data);

#endif /* EMT1_DRV_IMU_INVENSENSE_PORT_H_ */
