#include "SmartCar_Isr.h"
#include "SmartCar_Eru.h"
#include "SmartCar_MT9V034.h"
#include "image.h"
#include "SmartCar_ctrl.h"

            /*pwm�й��ж�*/


            /*����ͷ�й��ж�*/
IFX_INTERRUPT(eru_ch3_ch7_isr, 0, ERU_CH3_CH7_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    //GPIO_Set(P22,2, 1);
    if(GET_GPIO_FLAG(CH3_P02_0))//ͨ��3�ж�,����ͷ���ж�
    {
        CLEAR_GPIO_FLAG(CH3_P02_0);
        if      (3 == camera_type)  MT9V034_VSYNC();
        //else if (1 == camera_type)  ;

    }
    if(GET_GPIO_FLAG(CH7_P15_1))//ͨ��7�ж�
    {
        CLEAR_GPIO_FLAG(CH7_P15_1);
    }
    //GPIO_Set(P22,2, 0);

}


//uint32 img_i;
//uint32 servoPwm;
IFX_INTERRUPT(dma_ch5_isr, 0, ERU_DMA_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��

    if      (3 == camera_type)
    {
        MT9V034_DMA();
    }
    if(mt9v034_finish_flag == 1)
    {
        if(GPIO_Read(P13, 2) || GPIO_Read(P11, 3)){

            if(parkStart == 0)
            {
                image_main();
            }

            CTRL_servoMain();
//
        }

//        if(GPIO_Read(P13, 2) || GPIO_Read(P11, 3)){
//            image_main();
//        }
        mt9v034_finish_flag = 0;

    }

    //else if (1 == camera_type)  ov7725_dma();
}

IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)//����ж�
{
    enableInterrupts();//�����ж�Ƕ��
    PIT_CLEAR_FLAG(CCU6_0, PIT_CH0);

    CTRL_motorMain();
}

            /*spi�й�dma�ж�*/
IFX_INTERRUPT(qspi0DmaTxISR, 0, IFX_INTPRIO_DMA_CH1 )
{
    //IfxCpu_enableInterrupts();//�����ж�Ƕ��
    IfxQspi_SpiMaster_isrDmaTransmit(&oled_spi);
}

IFX_INTERRUPT(qspi0DmaRxISR, 0, IFX_INTPRIO_DMA_CH2)
{
    //IfxCpu_enableInterrupts();//�����ж�Ƕ��
    IfxQspi_SpiMaster_isrDmaReceive(&oled_spi);
}

IFX_INTERRUPT(qspi0ErISR, 0, IFX_INTPRIO_QSPI0_ER)
{
    IfxCpu_enableInterrupts();//�����ж�Ƕ��
    IfxQspi_SpiMaster_isrError(&oled_spi);

    // Process errors. Eg: parity Error is checked below
    //IfxQspi_SpiMaster_Channel *chHandle   = IfxQspi_SpiMaster_activeChannel(&my_spi);
//    if( chHandle->errorFlags.parityError == 1)
//    {
//        // Parity Error
//    }
}


            /*asc�й��ж�*/
    /*�����жϷ�������asclin0TxISR asclin0RxISR asclin0ErISR*/
/*uart0��������λ���Լ��ض����printf*/
IFX_INTERRUPT(asclin0TxISR, 0, IFX_INTPRIO_ASCLIN0_TX)
{
    IfxCpu_enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart[0]);
}

IFX_INTERRUPT(asclin0RxISR, 0, IFX_INTPRIO_ASCLIN0_RX)
{
    IfxCpu_enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart[0]);
}

IFX_INTERRUPT(asclin0ErISR, 0, IFX_INTPRIO_ASCLIN0_ER)
{
    IfxCpu_enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart[0]);
}

IFX_INTERRUPT(asclin1TxISR, 0, IFX_INTPRIO_ASCLIN1_TX)
{
    IfxCpu_enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart[1]);
}

IFX_INTERRUPT(asclin1RxISR, 0, IFX_INTPRIO_ASCLIN1_RX)
{
    IfxCpu_enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart[1]);
    MT9V034_Uart_Callback();
}

IFX_INTERRUPT(asclin1ErISR, 0, IFX_INTPRIO_ASCLIN1_ER)
{
    IfxCpu_enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart[1]);
}

IFX_INTERRUPT(asclin2TxISR, 0, IFX_INTPRIO_ASCLIN2_TX)
{
    IfxCpu_enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart[2]);
}

IFX_INTERRUPT(asclin2RxISR, 0, IFX_INTPRIO_ASCLIN2_RX)
{
    IfxCpu_enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart[2]);
}

IFX_INTERRUPT(asclin2ErISR, 0, IFX_INTPRIO_ASCLIN2_ER)
{
    IfxCpu_enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart[2]);
}

IFX_INTERRUPT(asclin3TxISR, 0, IFX_INTPRIO_ASCLIN3_TX)
{
    IfxCpu_enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart[3]);
}

IFX_INTERRUPT(asclin3RxISR, 0, IFX_INTPRIO_ASCLIN3_RX)
{
    IfxCpu_enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart[3]);
}

IFX_INTERRUPT(asclin3ErISR, 0, IFX_INTPRIO_ASCLIN3_ER)
{
    IfxCpu_enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart[3]);
}
