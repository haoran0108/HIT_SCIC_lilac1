/*
 * SmartCar_TFMINI.c
 *
 *  Created on: 2022年3月8日
 *      Author: 潘修一
 */
#include"SmartCar_TFMINI.h"

uint8 TFMINI_RxData[SmartCar_TFMINI_DATA_LEN] = { 0 };
uint8 Receive_Lenth = 0;
//typedef struct
//{
//    uint16 Distance;
//    uint16 Strength;
//    int Iscredible;
//}TFMINI_Information;
uint16 TFMINI_Distance=0;
uint16 TFMINI_Strength=0;
int TFMINI_Iscredible=0;
int TFMINI_receive_flag=0;
void SmartCar_TFMINI_Init(void)
{
    SmartCar_Uart_Init(TFMINI_TX_PIN,TFMINI_RX_PIN,TFMINI_BAUDRATE,TFMINI_UARTNUM);
}

//uint16 SmartCar_TFMINI_Receive(uint8 uart_num)
//{
//    uint8 rxData=0;
//    uint8 pBuffer[9]={0};
//    uint8 lenth=0;
//    uint8 flag=0;
//    while(IfxAsclin_Asc_getReadCount(&uart[uart_num]) > 0 && lenth<=SmartCar_TFMINI_DATA_LEN)
//    {
//        rxData = IfxAsclin_Asc_blockingRead(&uart[uart_num]);
//        if(rxData==SmartCar_TFMINT_DATA_HEAD && flag==0) flag=1;
//        if(flag==1)
//        {
//            pBuffer[lenth]=rxData;
//            lenth++;
//        }
//        if(lenth==SmartCar_TFMINI_DATA_LEN && pBuffer[1]==SmartCar_TFMINT_DATA_HEAD)
//        {
//            uint8 chk_cal=0;
//            uint16 cordist=0;
//            for(uint32 i = 0; i < (SmartCar_TFMINI_DATA_LEN - 1); i++)
//            {
//                chk_cal += pBuffer[i];
//            }
//            if(chk_cal == pBuffer[SmartCar_TFMINI_DATA_LEN - 1])
//            {
//                cordist = pBuffer[3]*256 + pBuffer[2];
//                return cordist;
//            }
//            else return 0;
//        }
//    }
//    return 0;
//}

//void SmartCar_TFMINI_Receive(void)
//{
//    if(TFMINI_receive_flag==1)
//    {
//        TFMINI_Distance = TFMINI_RxData[3]*256 + TFMINI_RxData[2];
//        TFMINI_Strength = TFMINI_RxData[5]*256 + TFMINI_RxData[4];
//        if(TFMINI_Strength<100||TFMINI_Strength==65535) TFMINI_Iscredible=0;
//        else TFMINI_Iscredible=1;
//        TFMINI_receive_flag=0;
//    }
//}

void SmartCar_TFMINI_UARTCallBack(void)
{

    while(SmartCar_TFMINI_Uart_Queue(&TFMINI_RxData[Receive_Lenth])==1)
    {
        Receive_Lenth++;
        if((Receive_Lenth==1 && TFMINI_RxData[0] != SmartCar_TFMINT_DATA_HEAD) ||
           (Receive_Lenth==2 && TFMINI_RxData[1] != SmartCar_TFMINT_DATA_HEAD))  Receive_Lenth=0;
        if(Receive_Lenth == SmartCar_TFMINI_DATA_LEN)
        {
            uint8 chk_cal=0;
            for(uint32 i = 0; i < (SmartCar_TFMINI_DATA_LEN - 1); i++)
            {
                chk_cal += TFMINI_RxData[i];
            }
            if(chk_cal == TFMINI_RxData[SmartCar_TFMINI_DATA_LEN - 1])
            {
                TFMINI_receive_flag=1;
                TFMINI_Distance = TFMINI_RxData[3]*256 + TFMINI_RxData[2];
                TFMINI_Strength = TFMINI_RxData[5]*256 + TFMINI_RxData[4];
                if(TFMINI_Strength<100||TFMINI_Strength==65535)
                {
                    TFMINI_Distance=300;
                    TFMINI_Iscredible=0;
                }
                else TFMINI_Iscredible=1;
//                Receive_Lenth=0;
//                IfxAsclin_Asc_clearRx(&uart[TFMINI_UARTNUM]);
            }
            else
            {
                TFMINI_receive_flag=0;
            }
            Receive_Lenth=0;
            IfxAsclin_Asc_clearRx(&uart[TFMINI_UARTNUM]);
        }
    }
}

uint8 SmartCar_TFMINI_Uart_Queue(uint8* rxData)
{
    if(IfxAsclin_Asc_getReadCount(&uart[TFMINI_UARTNUM]) > 0)
    {
        *rxData = IfxAsclin_Asc_blockingRead(&uart[TFMINI_UARTNUM]);
        //PRINFT("Data received %d",rxData[0]);
        return 1;
    }
    return 0;
}
