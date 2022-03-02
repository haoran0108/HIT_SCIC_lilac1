/*
 * SmartCar_ctrl.c
 *
 *  Created on: 2021年10月28日
 *      Author: windows
 */
#include "SmartCar_ctrl.h"
/*.c文件中只放自己的.h，其他函数中要用的变量要用全局定义，在.h中用extern+类型+变量名，需要用时包含.h*/
/*传数据*/
float test_varible[20] = {1,2,3,4,5,6,7,8,9,10};//发送数据的   //数组

/*标志位*/
int zebraFlag = 0, zebraCircle = 0;
int flagStop = 0, delayStop = 0;

/*pid相关*/
error servoError = {1, 1, 1};
error errorML = {1, 1, 1}, errorMR = {1, 1, 1};
int sumErrorRT = 0, sumErrorLF = 0;
error currentErrorL = {1, 1, 1}, currentErrorR = {1, 1, 1};
uint32 servoPwm;
uint32 motorPwm;
int32 expectL, expectR;//预期速度
int32 speedL, speedR;//实际速度
int32 mySpeedL = 0, mySpeedR = 0;
float motorLFKP, motorLFKI, motorLFKD, motorRTKP, motorRTKI, motorRTKD;
float currentKP_L, currentKI_L, currentKP_R, currentKI_R;

/*陀螺仪相关*/
float deltaGyro = 0, currentGyro = 0, directionAngle = 0, lastGyro = 0;
float averageSpeed = 0;
float position = 0; positionX = 0, positionY = 0;
float expectGyro = 0;
float gyroK = 20;
float dx = 0, dy = 0;
int parkStart = 0;//陀螺仪进出车库
//float currentAngle;

/*电流环参数*/
int currentFlag = 0, currentTime = 0;
uint16 currentLF = 0, currentRT = 0;
int currentExpectLF = 2110, currentExpectRT = 2110;


void CTRL_gyroInit()
{
    inv_imuGyroyQuene = INV_DataQueueInit(CTRL_IMU_QUENE_SIZE);
    INV_ICM20602_Init();
    currentGyro = 0;

}

void CTRL_gyroUpdate()
{
    INV_ImuUpdate();
    deltaGyro = inv_gyro[2] * 0.05;
    test_varible[9] = inv_gyro[2];

}

void CTRL_directionAngleGet()
{
    currentGyro += deltaGyro * 0.1;
    test_varible[10] = currentGyro;
//    if(currentGyro > 180) currentGyro -= 360;
//    else if(currentGyro < -180) currentGyro += 360;
}



void CTRL_positionGet()
{

    averageSpeed = (float)(speedL - speedR) / 2;
    dx = averageSpeed * (float)(sin(currentGyro * pi / 180)) * 0.05;
    dy = averageSpeed * (float)(cos(currentGyro * pi / 180)) * 0.05;

    positionX += dx;
    positionY += dy;

    if(positionX < 0.0001 && positionX > -0.0001)
    {
        directionAngle = 0;

    }
    else
    {
        directionAngle = (float)(atan2(positionY,positionX) / pi * 180);

    }

    if(currentGyro > 360)
    {
        currentGyro = 0;
        expectGyro = 0;
    }
    else if(currentGyro < -360)
    {
        currentGyro = 0;
        expectGyro = 0;
    }


}

void CTRL_circleControl(int direction, float r)
{
    float delta = 0;
    position = sqrt(positionX * positionX + positionY * positionY);
    delta = r - position;
    if(direction == 1)//1为顺时针
    {
        expectGyro = directionAngle - 90;

        expectGyro += delta * neihuanK.floatValue;
    }
    else//0为逆时针
    {
        expectGyro = directionAngle + 90;

        expectGyro -= delta * neihuanK.floatValue;
    }


}

void CTRL_waihuan()
{
    servoError.currentError = expectGyro - currentGyro;
    servoError.delta = servoError.currentError - servoError.lastError;
    servoPwm = (uint32)(675 + gyroD.floatValue * servoError.delta + gyroP.floatValue * servoError.currentError);
    if(servoPwm > 750)
        servoPwm = 750;
    else if(servoPwm < 600)
        servoPwm = 600;
    servoError.lastError = servoError.currentError;
}

void CTRL_directionAngleClean()
{
    deltaGyro = 0;
    currentGyro = 0;
    expectGyro = 0;
    directionAngle = 0;
    dx = 0;
    dy = 0;
    positionX = 0;
    positionY = 0;
    servoPwm = 675;
}

void CTRL_gyroCircle()
{
    CTRL_gyroUpdate();
    CTRL_positionGet();
    CTRL_directionAngleGet();
    CTRL_circleControl(file3.intValue,presentTHRE.intValue);

}

//void CTRL_currentTIME()
//{
//    currentTime ++;
//    if(currentTime < 400)
//    {
//        currentExpect = 2300;
//    }
//    else if(currentTime >= 400 && currentTime <= 800)
//    {
//        currentExpect = 2100;
//    }
//    else if(currentTime > 800)
//        {currentTime = 0;}
//
//}

void CTRL_speedLoopPID()
{
    if(flagStop == 0)
    {
        expectL = presentSpeed.intValue;//左轮
        expectR = presentSpeed.intValue;//右轮
    }

    speedL = CTRL_speedGetLeft();
    test_varible[6] = speedL;
    errorML.currentError = expectL + speedL;//取偏差
    sumErrorLF += errorML.currentError;
    errorML.delta = errorML.currentError - errorML.lastError;
    currentExpectLF = (int32)(2210 + motorLFKP * errorML.currentError + motorLFKI * errorML.delta);
//    currentExpectLF = (int32)(currentExpectLF + motorLFKI * errorML.currentError + motorLFKP * errorML.delta);
//    if(currentExpectLF < 0) currentExpectLF = -currentExpectLF;
    test_varible[7] = (float)(currentExpectLF);
    errorML.lastError = errorML.currentError;//更新上一次误差
    if(currentExpectLF < 1000) currentExpectLF = 1000;
    else if(currentExpectLF > 3000) currentExpectLF = 3000;


    speedR = CTRL_speedGetRight();
    test_varible[5] = speedR;
    errorMR.currentError = expectR - speedR;//取偏差
    sumErrorRT += errorMR.currentError;
    errorMR.delta = errorMR.currentError - errorMR.lastError;
//    currentExpectRT = (int32)(2210 + motorRTKP * errorMR.currentError + motorRTKI * sumErrorRT + motorRTKD * errorMR.delta);
    currentExpectRT = (int32)(currentExpectRT + motorRTKI * errorMR.currentError + motorRTKP * errorMR.delta);
    errorMR.lastError = errorMR.currentError;//更新上一次误差
    test_varible[4] = currentExpectRT;
    if(currentExpectRT < 1000) currentExpectRT = 1000;
    else if(currentExpectRT > 3000) currentExpectRT = 3000;

}


void CTRL_curLoopPID()
{
    int currentLF_real = 0;

    currentRT = ADC_Get(ADC_0, ADC0_CH5_A5, ADC_12BIT);//右轮
    currentLF = ADC_Get(ADC_0, ADC0_CH7_A7, ADC_12BIT);//左轮

    test_varible[0] = (float)(currentRT);
    test_varible[1] = (float)(currentLF);
    currentLF_real = 4420 - currentLF;

    currentErrorR.currentError = currentExpectRT - currentRT;
    currentErrorR.delta = currentErrorR.currentError - currentErrorR.lastError;
    mySpeedR = (int32)(mySpeedR + currentErrorR.currentError * currentKI_R + currentErrorR.delta * currentKP_R);
    currentErrorR.lastError = currentErrorR.currentError;


    currentErrorL.currentError = currentExpectLF - currentLF_real;
//    currentErrorL.currentError = 2250 - currentLF;
    currentErrorL.delta = currentErrorL.currentError - currentErrorL.lastError;
    mySpeedL = (int32)(mySpeedL + currentErrorL.currentError * currentKI_L + currentErrorL.delta * currentKP_L);
    currentErrorL.lastError = currentErrorL.currentError;


}

void CTRL_Init()
{
//    presentSpeedL.intValue = 1800;
//    presentSpeedR.intValue = 1800;

    Pit_Init_ms(CCU6_0, PIT_CH0, 5);//定时器中断，给电机使用（0模块0通道）
    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM0_0_TOUT48_P22_1_OUT, 50, 720);//舵机
    /*B、D给pwm，左右轮正转，AB右轮，CD左轮*/
    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM0_2_TOUT33_P33_11_OUT, 20000, 2500);//B右正
    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM3_4_TOUT34_P33_12_OUT, 20000, 0);//D左正
    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM1_1_TOUT31_P33_9_OUT, 20000, 0);//A右反
    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM0_5_TOUT40_P32_4_OUT, 20000, 2500);//C左负

    SmartCar_Encoder_Init(GPT12_T5,IfxGpt120_T5INA_P21_7_IN,IfxGpt120_T5EUDA_P21_6_IN);
    SmartCar_Encoder_Init(GPT12_T6,IfxGpt120_T6INA_P20_3_IN,IfxGpt120_T6EUDA_P20_0_IN);

    ADC_Init(ADC_0, ADC0_CH5_A5);
    ADC_Init(ADC_0, ADC0_CH7_A7);

    CTRL_gyroInit();


}


void CTRL_servoPID()
{

    servoError.currentError = 88 - mid_line[presentVision.intValue];
    servoError.delta = servoError.currentError - servoError.lastError;
    servoPwm = (uint32)(820 + presentServoD.floatValue * servoError.delta + fuzzyPB.floatValue * servoError.currentError);
    if(servoPwm > 890)
        servoPwm = 890;
    else if(servoPwm < 720)
        servoPwm = 720;
    servoError.lastError = servoError.currentError;
    test_varible[11] = servoPwm;
}

float CTRL_FuzzyMemberShip(int midError)
{
    float membership[2] = {1, 0};
    float servoKP = 0;



    if(midError < PB && midError > PM)
    {
        membership[0] = (midError - PM) / fuzzyWidth;
        membership[1] = (midError - PB) / fuzzyWidth;

        servoKP = fuzzyPB.floatValue * membership[0] + fuzzyPM.floatValue * membership[1];
    }
    else if(midError < PM && midError > PS)
    {
        membership[0] = (midError - PS) / fuzzyWidth;
        membership[1] = (midError - PM) / fuzzyWidth;

        servoKP = fuzzyPM.floatValue * membership[0] + fuzzyPS.floatValue * membership[1];
    }
    else if(midError < PS && midError > ZO)
    {
        membership[0] = (midError - ZO) / fuzzyWidth;
        membership[1] = (midError - PS) / fuzzyWidth;

        servoKP = fuzzyPS.floatValue * membership[0] + fuzzyZO.floatValue * membership[1];
    }
    else if(midError < ZO && midError > NS)
    {
        membership[0] = (midError - NS) / fuzzyWidth;
        membership[1] = (midError - ZO) / fuzzyWidth;

        servoKP = fuzzyZO.floatValue * membership[0] + fuzzyNS.floatValue * membership[1];
    }
    else if(midError < NS && midError > NM)
    {
        membership[0] = (midError - NM) / fuzzyWidth;
        membership[1] = (midError - NS) / fuzzyWidth;

        servoKP = fuzzyNS.floatValue * membership[0] + fuzzyNM.floatValue * membership[1];
    }
    else if(midError < NM && midError > NB)
    {
        membership[0] = (midError - NB) / fuzzyWidth;
        membership[1] = (midError - NM) / fuzzyWidth;

        servoKP = fuzzyNM.floatValue * membership[0] + fuzzyNB.floatValue * membership[1];
    }

    return servoKP;
}

void CTRL_fuzzyPID()
{
    float fuzzyKP = 1;
    servoError.currentError = 88 - mid_line[presentVision.intValue];
    servoError.delta = servoError.currentError - servoError.lastError;

    fuzzyKP = CTRL_FuzzyMemberShip(servoError.currentError);
    servoPwm = (uint32)(780 + presentServoD.floatValue * servoError.delta + fuzzyKP * servoError.currentError);


    servoError.lastError = servoError.currentError;

}


void CTRL_motorPID()
{

    currentRT = ADC_Get(ADC_0, ADC0_CH5_A5, ADC_12BIT);//右轮
    currentLF = ADC_Get(ADC_0, ADC0_CH7_A7, ADC_12BIT);//左轮

    test_varible[0] = currentRT;
    test_varible[1] = currentLF;

    expectL = presentSpeed.intValue;//左轮
    speedL = CTRL_speedGetLeft();
    test_varible[8] = speedL;

    errorML.currentError = expectL + speedL;//取偏差
    errorML.delta = errorML.currentError - errorML.lastError;
    mySpeedL = (int32)(mySpeedL + motorLFKI * errorML.currentError + motorLFKP * errorML.delta);
    errorML.lastError = errorML.currentError;//更新上一次误差


    expectR = presentSpeed.intValue;//右轮
    speedR = CTRL_speedGetRight();
//    test_varible[3] = speedR;
    errorMR.currentError = expectR - speedR;//取偏差
    errorMR.delta = errorMR.currentError - errorMR.lastError;
    mySpeedR = (int32)(mySpeedR + motorRTKI * errorMR.currentError + motorRTKP * errorMR.delta);
    errorMR.lastError = errorMR.currentError;//更新上一次误差

}


void CTRL_servoMain()
{
    if(GPIO_Read(P13, 2) && parkStart == 1)
    {
        CTRL_ParkStartServo(currentGyro);

    }
    else if(GPIO_Read(P13, 2) && parkStart == 0 && flagStop == 0)
    {
        CTRL_servoPID();
    }

    if(flagStop == 1)
    {
//        CTRL_ParkStartServo(currentGyro);

        if(parkPosition == leftPark)
        {
            servoPwm = 890;
        }
        else if(parkPosition == rightPark)
        {
            servoPwm = 750;
        }
        else CTRL_servoPID();
    }
    SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_0_TOUT48_P22_1_OUT, servoPwm);//舵机控制

//    if(zebraCircle == display6.intValue || flagStop == 1)
//    {
//        delayStop ++;
//        if(delayStop > display5.intValue)
//        {
//            expectL = 0;
//            expectR = 0;
//
//        }
//
//    }
//
//    else
//    {
//        CTRL_motorDiffer();

//    }


}


void CTRL_motor()
{
    int pwmL, pwmR;
    if(mySpeedL > PWM_MAX)
    {
        mySpeedL = PWM_MAX;
    }
    if(mySpeedR > PWM_MAX)
    {
        mySpeedR = PWM_MAX;
    }
    if(mySpeedL < PWM_MAX_N)
    {
        mySpeedL = PWM_MAX_N;
    }
    if(mySpeedR < PWM_MAX_N)
    {
        mySpeedR = PWM_MAX_N;
    }

    if(mySpeedL >= 0 && mySpeedR >= 0)
    {
        pwmR = mySpeedR;
        pwmL = mySpeedL;
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_2_TOUT33_P33_11_OUT, pwmR);//B右正
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM3_4_TOUT34_P33_12_OUT, pwmL);//D左正
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM1_1_TOUT31_P33_9_OUT, 0);//A右负
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_5_TOUT40_P32_4_OUT, 0);//C左负
    }
    else if(mySpeedL >= 0 && mySpeedR < 0)
    {
        pwmR = -mySpeedR;
        pwmL = mySpeedL;
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_2_TOUT33_P33_11_OUT, 0);//B
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM3_4_TOUT34_P33_12_OUT, pwmL);//D
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM1_1_TOUT31_P33_9_OUT, pwmR);//A
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_5_TOUT40_P32_4_OUT, 0);//C
    }
    else if(mySpeedL < 0 && mySpeedR >= 0)
    {
        pwmL = -mySpeedL;
        pwmR = mySpeedR;
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_2_TOUT33_P33_11_OUT, pwmR);//B
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM3_4_TOUT34_P33_12_OUT, 0);//D
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM1_1_TOUT31_P33_9_OUT, 0);//A
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_5_TOUT40_P32_4_OUT, pwmL);//C
    }
    else if(mySpeedL < 0 && mySpeedR < 0)
    {
        pwmL = -mySpeedL;
        pwmR = -mySpeedR;
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_2_TOUT33_P33_11_OUT, 0);//B
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM3_4_TOUT34_P33_12_OUT, 0);//D
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM1_1_TOUT31_P33_9_OUT, pwmR);//A
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_5_TOUT40_P32_4_OUT, pwmL);//C
    }

    test_varible[2] = pwmR;
    test_varible[3] = pwmL;
}


void CTRL_motorMain()
{
//    if(straightSpeedUp() && zebraCircle < display6.intValue)
//    {
//        expectL = (int32)(display7.intValue);
//        expectR = (int32)(display7.intValue);
//    }
//

    CTRL_CarParkStart();
    CTRL_CarParkStop();


    CTRL_speedLoopPID();
    CTRL_curLoopPID();
//         CTRL_motorPID();
    CTRL_motor();


}

void CTRL_ParkStartServo(float gyro)
{
//    servoPwm = (uint32)(gyro * (-0.8) + 760);
    servoPwm = (uint32)(gyro * (-0.8) + 880);

    if(servoPwm > 890)
        servoPwm = 890;
    else if(servoPwm < 750)
        servoPwm = 750;
    test_varible[11] = servoPwm;

}

void CTRL_motorDiffer()
{

    int delta;
    delta = 790 - servoPwm;

    /*内轮减速*/
    float k;
    if(delta > 0)
    {
        k = gap.floatValue * (1.1209 - 0.0061 * delta);
        if(k > 1) k = 1;
        expectL = (int32)(presentSpeed.intValue);
        expectR = (int32)(presentSpeed.intValue * k);
    }
    else if(delta < 0)
    {
        k = gap.floatValue * (1.1209 + 0.0061 * delta);
        if(k > 1) k = 1;
        expectL = (int32)(presentSpeed.intValue * k);
        expectR = (int32)(presentSpeed.intValue);
    }
    else if(delta == 0)
    {

        expectL = (int32)(presentSpeed.intValue);
        expectR = (int32)(presentSpeed.intValue);
    }

    /*内减外加*/
//    float kIN, kOUT;
//    if(delta > 0)
//    {
//        kIN = gap.floatValue * (1.0886 - 0.004 * delta);
//        kOUT = gap.floatValue * (0.9114 + 0.004 * delta);
//        if(kIN > 1) kIN = 1;
//        if(kOUT < 1) kOUT = 1;
//
//        expectL = (int32)(presentSpeed.intValue * kOUT);
//        expectR = (int32)(presentSpeed.intValue * kIN);
//    }
//    else if(delta < 0)
//    {
//        kIN = gap.floatValue * (1.0886 + 0.004 * delta);
//        kOUT = gap.floatValue * (0.9114 - 0.004 * delta);
//        if(kIN > 1) kIN = 1;
//        if(kOUT < 1) kOUT = 1;
//
//        expectL = (int32)(presentSpeed.intValue * kIN);
//        expectR = (int32)(presentSpeed.intValue * kOUT);
//    }
//    else if(delta == 0)
//    {
//        expectL = (int32)(presentSpeed.intValue);
//        expectR = (int32)(presentSpeed.intValue);
//    }
}

void CTRL_CarParkStart()
{
    if(GPIO_Read(P13, 2) && parkStart == 1)
    {
        CTRL_gyroUpdate();
        CTRL_directionAngleGet();

    //
    //        if(currentGyro < -75)//正为左转 负为右转
    //        {
    //            parkStart = 0;
    //        }
        if(currentGyro > 75)
        {
            parkStart = 0;
            currentGyro = 0;
        }
    }
}

void CTRL_CarParkStop()
{
//    delayStop++;
    if(flagStop == 1)
    {
        if(currentGyro > 50 || currentGyro < -50)
        {
            expectL = 0;
            expectR = 0;
//            currentGyro = 80;
        }

        else if(currentGyro <= 50 && currentGyro >= -50)
        {
            expectL = presentSpeed.intValue;
            expectR = presentSpeed.intValue;

            CTRL_gyroUpdate();
            CTRL_directionAngleGet();
        }
   //        CTRL_speedLoopPID();
   //        CTRL_curLoopPID();
   //       CTRL_motorPID();
   //        CTRL_motor();

    }
}

int16_t CTRL_speedGetRight()//左轮编码器1 引脚20.3和20.0对应T6   右轮编码器2 引脚21.6和21.7对应T5
{
    int16_t ctrl_speedR = 0;
    ctrl_speedR = SmartCar_Encoder_Get(GPT12_T5);
    SmartCar_Encoder_Clear(GPT12_T5);
    return ctrl_speedR;
}

int16_t CTRL_speedGetLeft()//左轮编码器1 引脚20.3和20.0对应T6   右轮编码器2 引脚21.6和21.7对应T5
{
    int16_t ctrl_speedL = 0;
    ctrl_speedL = SmartCar_Encoder_Get(GPT12_T6);
    SmartCar_Encoder_Clear(GPT12_T6);
    return ctrl_speedL;
}

