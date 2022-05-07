/*
 * SmartCar_TFMINI.h
 *
 *  Created on: 2022��3��8��
 *      Author: ����һ
 */

#include "ifxAsclin_Asc.h"
#include "SmartCar_Uart.h"
#ifndef SMARTCAR_TFMINI_H_
#define SMARTCAR_TFMINI_H_
#define SmartCar_TFMINT_DATA_HEAD 0X59
#define SmartCar_TFMINI_DATA_LEN 9

//��ʼ��TFMINI
#define TFMINI_UARTNUM      0
#define TFMINI_TX_PIN       IfxAsclin0_TX_P14_0_OUT
#define TFMINI_RX_PIN       IfxAsclin0_RXA_P14_1_IN
#define TFMINI_BAUDRATE     115200

extern uint16 TFMINI_Distance;
extern uint16 TFMINI_Strength;
//typedef struct Information
//{
//    uint16 Distance;
//    uint16 Strength;
//    int Iscredible;
//}TFMINI_Information;
//

/*
 * brief ��ʼ��TFminiMi
 * para tx_pin ���ͽ�
 * para rx_pin ���ս�
 * para baudrate ������
 * para uart_num uartģ��
 * sample         SmartCar_TFMINI_Init(IfxAsclin0_TX_P14_0_OUT,IfxAsclin0_RXA_P14_1_IN,115200,0)
 * */
void SmartCar_TFMINI_Init(void);
//uint16 SmartCar_TFMINI_Receive(uint8 uart_num);
//void SmartCar_TFMINI_Receive(void);
void SmartCar_TFMINI_UARTCallBack(void);
uint8 SmartCar_TFMINI_Uart_Queue(uint8* rxData);


#endif /* SMARTCAR_TFMINI_H_ */

