/*
 * common.c
 *
 *  Created on: 2021��1��22��
 *      Author: ��Ψ
 */

#include "switch.h"
int delayFlag = 0;
void SW_readSwitch()
{

    if(!GPIO_Read(P13, 2))
    {
        motorLFKI = 0;
        motorLFKP = 0;
        motorRTKI = 0;
        motorRTKP = 0;
        mySpeedL = 0;
        mySpeedL = 0;
        zebraCircle = 0;
        flagStop = 0;
        delayFlag = 0;
//        zebraCount = 0;
    }
    else
    {
        if(delayFlag == 0)
        {
            Delay_ms(STM0,1000);
            delayFlag = 1;
        }
        motorLFKP = display1.intValue;
        motorLFKI = display2.intValue;
        motorRTKP = display3.intValue;
        motorRTKI = display4.intValue;
    }
}
