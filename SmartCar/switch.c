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

    if(GPIO_Read(P11, 3))
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
        straightFlag = 0;
//        zebraCount = 0;
//        zFlag = 0;
        delayStop = 0;
        currentGyro = 0;
//        parkStart = 1;
//        flagStop = 0;
        parkPosition = 0;

        parkStart = startWay.intValue;
        parkType = startWay.intValue;

        state = 0;

        flagCircleForsee = 1;
        stopFlag = 0;
        carParkTimes = 0;
        carParkDelay = 0;
        servoPwm = 630;

        crossCircleCount = 0;
        rampWayCount = 0;
        startCount = 0;
        lastState[8] = 0;
        parkJudgeCount = 0;

        slowFlag = 0;

        lastRampGyro = 0;
        rampGyro = 0;
        rampGyroMax = 0;
        rampFlag1 = 0;
        rampFlag2 = 0;
        rampFlag3 = 0;
        GPIO_Set(P22, 0, 0);
//        CTRL_directionAngleClean();
    }
    else
    {
        if(delayFlag == 0)
        {
            Delay_ms(STM0,1000);
            delayFlag = 1;
        }


    }
}
