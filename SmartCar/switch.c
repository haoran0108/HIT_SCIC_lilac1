/*
 * common.c
 *
 *  Created on: 2021Äê1ÔÂ22ÈÕ
 *      Author: ËïÎ¨
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
        motorRTKD = 0;
        currentKP_R = 0;
        currentKI_R = 0;
        currentKI_L = 0;
        currentKP_L = 0;
        mySpeedL = 0;
        mySpeedR = 0;
        zebraCircle = 0;
        flagStop = 0;
        delayFlag = 0;
//        zebraCount = 0;
//        zFlag = 0;
        delayStop = 0;
        currentGyro = 0;
//        parkStart = 1;
//        flagStop = 0;
        parkPosition = 0;
        parkStart = 0;
        state = 0;
//        CTRL_directionAngleClean();
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

        currentKP_R = currentRTKP.floatValue;
        currentKI_R = currentRTKI.floatValue;
        currentKI_L = currentLFKI.floatValue;
        currentKP_L = currentLFKP.floatValue;

    }
}
