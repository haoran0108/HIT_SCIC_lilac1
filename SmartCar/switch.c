/*
 * common.c
 *
 *  Created on: 2021年1月22日
 *      Author: 孙唯
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

        parkStart = file1.intVal;
        if(parkStart == -1)
        {
            leftPark = 0;
            rightPark = 1;
        }

        else if(parkStart == 0)
        {
            leftPark = 1;
            rightPark = 0;
        }

        else if(parkStart == 1)
        {
            leftPark = 1;
            rightPark = 0;
        }
//        parkType = startWay.intVal;

        state = 0;

        flagCircleForsee = 1;
        stopFlag = 0;
        carParkTimes = 0;
        carParkDelay = 0;
        servoPwm = servoMidValue;

        crossCircleCount = 0;
        rampWayCount = 0;
        startCount = 0;
//        lastState[8] = 0;
        parkJudgeCount = 0;
        parkStraightCount = 0;

        slowFlag = 0;

        lastRampGyro = 0;
        rampGyro = 0;
        rampGyroMax = 0;
        rampFlag1 = 0;
        rampFlag2 = 0;
        rampFlag3 = 0;
        GPIO_Set(P22, 0, 0);//蜂鸣器
        GPIO_Set(P00, 8, 0);//电机开关

        GPIO_Set(P02, 8, 1);//图传SD

        testFlag = 1;//赛道测试
        testStateTimes = 0;
        flagStopCount1 = 0;
        flagStop1 = 0;
        memoryFlag = 0;
        memoryState[0] = 0;
        memoryState[1] = 0;
        memoryState[2] = 0;
        memoryState[3] = 0;
        memoryState[4] = 0;
        memoryState[5] = 0;
        memoryState[6] = 0;
        memoryState[7] = 0;
        memoryState[8] = 0;
        memoryState[9] = 0;
        memoryState[10] = 0;
        memoryState[11] = 0;
        memoryState[12] = 0;
        memoryState[13] = 0;
        memoryState[14] = 0;
        memoryState[15] = 0;
        memoryState[16] = 0;
        memoryState[17] = 0;
        memoryState[18] = 0;
        memoryState[19] = 0;

        folkTimes = 0;
        tCrossTimes = 0;
        sRoadFlag = 0;

        duzhuanFlag = 0;
        duzhuanTime = 0;
        duzhuanCount = 0;

        startCount = 0;
        startFlag = 0;
        stopCount = 0;

        islandTimes = 0;
        islandCircleCount = 0;
        tInCount = 0;

        if(file1.intVal == -1)//左 环岛
        {
            wayIT = 1;
        }

        else if(file1.intVal == 0)
        {
            wayIT = -1;
        }

        else if(file1.intVal == 1)
        {
            wayIT = -1;
        }
    }
    else
    {
        if(delayFlag == 0)
        {
            Delay_ms(STM0,1000);
            delayFlag = 1;


        }



        GPIO_Set(P00, 8, 1);//电机


        if(stopFlag == 0 && flagStop == 0)
        {
            GPIO_Set(P02, 8, 0);

        }
        else GPIO_Set(P02, 8, 1);

    }


    if(GPIO_Read(P13, 3))
    {
        GPIO_Set(P02, 7, 1);

    }
    else
    {
        GPIO_Set(P02, 7, 0);

    }
}
