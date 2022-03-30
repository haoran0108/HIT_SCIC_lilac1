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
double my_sin;//圆形前瞻
int flagCircleForsee = 1;

float fuzzy_PB = 3.2, fuzzy_PM = 3.0, fuzzy_PS = 2.9, fuzzy_ZO = 2.7, fuzzy_NS = 2.9, fuzzy_NM = 3.0, fuzzy_NB = 3.2, fuzzy_Dbig = 6.6, fuzzy_Dsmall = 5.8;

/*陀螺仪相关*/
float deltaGyro = 0, currentGyro = 0, directionAngle = 0, lastGyro = 0;
float averageSpeed = 0;
float position = 0; positionX = 0, positionY = 0;
float expectGyro = 0;
float gyroK = 20;
float dx = 0, dy = 0;
int parkStart = 1;//陀螺仪进出车库
//float currentAngle;

/*电流环参数*/
int currentFlag = 0, currentTime = 0;
uint16 currentLF = 0, currentRT = 0;
int currentExpectLF = 2110, currentExpectRT = 2110;

int testFlag = 0;

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

}

void CTRL_directionAngleGet()
{
    currentGyro += deltaGyro * 0.1;

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

//        expectGyro += delta * neihuanK.floatValue;
    }
    else//0为逆时针
    {
        expectGyro = directionAngle + 90;

//        expectGyro -= delta * neihuanK.floatValue;
    }


}

void CTRL_waihuan()
{
//    servoError.currentError = expectGyro - currentGyro;
//    servoError.delta = servoError.currentError - servoError.lastError;
//    servoPwm = (uint32)(675 + IslandPD.floatValue * servoError.delta + CrossCircle.floatValue * servoError.currentError);
//    if(servoPwm > 750)
//        servoPwm = 750;
//    else if(servoPwm < 600)
//        servoPwm = 600;
//    servoError.lastError = servoError.currentError;
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
    if(flagStop == 0 || stopFlag == 0)
    {
        expectL = presentSpeed.intValue;//左轮
        expectR = presentSpeed.intValue;//右轮
    }

    speedL = CTRL_speedGetLeft();
    errorML.currentError = expectL + speedL;//取偏差
    sumErrorLF += errorML.currentError;
    errorML.delta = errorML.currentError - errorML.lastError;
    currentExpectLF = (int32)(2210 + motorLFKP * errorML.currentError + motorLFKI * errorML.delta);
//    currentExpectLF = (int32)(currentExpectLF + motorLFKI * errorML.currentError + motorLFKP * errorML.delta);
    errorML.lastError = errorML.currentError;//更新上一次误差
    if(currentExpectLF < 1000) currentExpectLF = 1000;
    else if(currentExpectLF > 4000) currentExpectLF = 4000;


    speedR = CTRL_speedGetRight();
    errorMR.currentError = expectR - speedR;//取偏差
    sumErrorRT += errorMR.currentError;
    errorMR.delta = errorMR.currentError - errorMR.lastError;
    currentExpectRT = (int32)(2210 + motorRTKP * errorMR.currentError + motorRTKI * errorMR.delta);
//    currentExpectRT = (int32)(currentExpectRT + motorRTKI * errorMR.currentError + motorRTKP * errorMR.delta);
    errorMR.lastError = errorMR.currentError;//更新上一次误差
    if(currentExpectRT < 1000) currentExpectRT = 1000;
    else if(currentExpectRT > 4000) currentExpectRT = 4000;

//    test_varible[0] = speedL;
//    test_varible[1] = speedR;
//
//    test_varible[4] = currentExpectLF;
//    test_varible[5] = currentExpectRT;


}


void CTRL_curLoopPID()
{
    int currentLF_real = 0;

    currentRT = ADC_Get(ADC_0, ADC0_CH5_A5, ADC_12BIT);//右轮
    currentLF = ADC_Get(ADC_0, ADC0_CH7_A7, ADC_12BIT);//左轮

//    test_varible[2] = currentLF;
//    test_varible[3] = currentRT;


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
    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM0_0_TOUT48_P22_1_OUT, 50, 700);//舵机
    /*B、D给pwm，左右轮正转，AB右轮，CD左轮*/
    GPIO_Init(P33, 9,PUSHPULL, 0);//左正
    GPIO_Init(P33, 11,PUSHPULL, 1);//右正

//    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM0_2_TOUT33_P33_11_OUT, 20000, 2500);//B右正
    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM3_4_TOUT34_P33_12_OUT, 20000, 2500);//D左正
//    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM1_1_TOUT31_P33_9_OUT, 20000, 0);//A右反
    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM0_5_TOUT40_P32_4_OUT, 20000, 2500);//C左负
//
    SmartCar_Encoder_Init(GPT12_T5,IfxGpt120_T5INA_P21_7_IN,IfxGpt120_T5EUDA_P21_6_IN);
    SmartCar_Encoder_Init(GPT12_T6,IfxGpt120_T6INA_P20_3_IN,IfxGpt120_T6EUDA_P20_0_IN);

    ADC_Init(ADC_0, ADC0_CH5_A5);
    ADC_Init(ADC_0, ADC0_CH7_A7);

    CTRL_gyroInit();


}


void CTRL_servoPID()
{

    servoError.currentError = 94 - mid_line[presentVision.intValue];
    test_varible[10] =  mid_line[presentVision.intValue];

    servoError.delta = servoError.currentError - servoError.lastError;
    servoPwm = (uint32)(700 + presentServoD.floatValue * servoError.delta + fuzzyPB.floatValue * servoError.currentError);
    if(servoPwm > 780)
        servoPwm = 780;
    else if(servoPwm < 620)
        servoPwm = 620;
    servoError.lastError = servoError.currentError;

}

float CTRL_FuzzyMemberShip(int midError)
{
    float membership[2] = {1, 0};
    float servoKP = 0;
    float fuzzyWidth = 10;
    test_varible[6] = midError;

    if(midError >= PB)
    {
        servoKP = fuzzyPB.floatValue;
    }
    else if(midError < PB && midError > PM)
    {
        membership[0] = fabs((midError - PM) / fuzzyWidth);
        membership[1] = fabs((midError - PB) / fuzzyWidth);

        servoKP = fuzzyPB.floatValue * membership[0] + fuzzyPM.floatValue * membership[1];
    }
    else if(midError == PM)
    {
        servoKP = fuzzyPM.floatValue;
    }
    else if(midError < PM && midError > PS)
    {
        membership[0] = fabs((midError - PS) / fuzzyWidth);
        membership[1] = fabs((midError - PM) / fuzzyWidth);

        servoKP = fuzzyPM.floatValue * membership[0] + fuzzyPS.floatValue * membership[1];
    }
    else if(midError == PS)
    {
        servoKP = fuzzyPS.floatValue;
    }
    else if(midError < PS && midError > ZO)
    {
        membership[0] = fabs((midError - ZO) / fuzzyWidth);
        membership[1] = fabs((midError - PS) / fuzzyWidth);

        servoKP = fuzzyPS.floatValue * membership[0] + fuzzyZO.floatValue * membership[1];
    }
    else if(midError == ZO)
    {
        servoKP = fuzzyZO.floatValue;
    }
    else if(midError < ZO && midError > NS)
    {
        membership[0] = fabs((midError - NS) / fuzzyWidth);
        membership[1] = fabs((midError - ZO) / fuzzyWidth);

        servoKP = fuzzyZO.floatValue * membership[0] + fuzzyNS.floatValue * membership[1];
    }
    else if(midError == NS)
    {
        servoKP = fuzzyNS.floatValue;
    }
    else if(midError < NS && midError > NM)
    {
        membership[0] = fabs((midError - NM) / fuzzyWidth);
        membership[1] = fabs((midError - NS) / fuzzyWidth);

        servoKP = fuzzyNS.floatValue * membership[0] + fuzzyNM.floatValue * membership[1];
    }
    else if(midError == NM)
    {
        servoKP = fuzzyNM.floatValue;
    }
    else if(midError < NM && midError > NB)
    {
        membership[0] = fabs((midError - NB) / fuzzyWidth);
        membership[1] = fabs((midError - NM) / fuzzyWidth);

        servoKP = fuzzyNM.floatValue * membership[0] + fuzzyNB.floatValue * membership[1];
    }
    else if(midError <= NB)
    {
        servoKP = fuzzyNB.floatValue;
    }
//    membership[0] = fabs(membership[0]);
//    membership[1] = fabs(membership[1]);

    test_varible[7] = membership[0];
    test_varible[8] = membership[1];
    test_varible[9] = servoKP;

    return servoKP;
}

void CTRL_fuzzyPID()
{
    float fuzzyKP = 1;
    int servo_error = 0;
    servoError.currentError = 93 - mid_line[presentVision.intValue];
    servoError.delta = servoError.currentError - servoError.lastError;
//    servo_error = servoError.currentError;
    fuzzyKP = CTRL_FuzzyMemberShip(servoError.currentError);
    test_varible[11] = fuzzyKP;
    servoPwm = (uint32)(700 + presentServoD.floatValue * servoError.delta + fuzzyKP * servoError.currentError);
    if(servoPwm > 770)
        servoPwm = 770;
    else if(servoPwm < 630)
        servoPwm = 630;

//    test_varible[10] = servoPwm;
    servoError.lastError = servoError.currentError;

}


void CTRL_motorPID()
{

//    if(testFlag < 400)
//    {
//           expectL = presentSpeed.intValue;//左轮
//           expectR = presentSpeed.intValue;//右轮
//    }
//
//    else if(testFlag >= 400)
//    {
//        expectL = 0;
//        expectR = 0;
//    }
//    if(stopFlag == 0 && flagStop == 0)
//    {
//    expectL = presentSpeed.intValue;//左轮
//    expectR = presentSpeed.intValue;//右轮
//    }

    speedL = CTRL_speedGetLeft();
    speedR = CTRL_speedGetRight();

    errorML.currentError = expectL + speedL;//取偏差
    errorML.delta = errorML.currentError - errorML.lastError;
    mySpeedL = (int32)(mySpeedL + motorLFKI * errorML.currentError + motorLFKP * errorML.delta);
    errorML.lastError = errorML.currentError;//更新上一次误差


    errorMR.currentError = expectR - speedR;//取偏差
    errorMR.delta = errorMR.currentError - errorMR.lastError;
    mySpeedR = (int32)(mySpeedR + motorRTKI * errorMR.currentError + motorRTKP * errorMR.delta);
    errorMR.lastError = errorMR.currentError;//更新上一次误差

    test_varible[0] = speedL;
    test_varible[1] = speedR;


}


void CTRL_servoMain()
{
    if(GPIO_Read(P13, 2) && parkStart == 1)
    {
//        CTRL_ParkStartServo(currentGyro);
        servoPwm = 630;
    }
    else if(GPIO_Read(P13, 2) && parkStart == 0 && flagStop == 0)
    {
        CTRL_fuzzyPID();
    }
////
    else if(flagStop == 1)
    {
//        CTRL_ParkStartServo(currentGyro);

//        if(parkPosition == 1)
//        {
//            servoPwm = 780;
//        }
//        else if(parkPosition == 2)
//        {
            servoPwm = 630;
//        }

    }
//    else CTRL_fuzzyPID();
//    else CTRL_servoPID();

//    CTRL_CircleForsee(presentVision.intValue);
//    if(flagCircleForsee == 0)
//    {
//        CTRL_servoPID();

//        flagCircleForsee = 1;
//
//    }
//    else{
//        CTRL_CircleServoPID();

//    }

//    CTRL_fuzzyPID();
    test_varible[10] = servoPwm;
    SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_0_TOUT48_P22_1_OUT, servoPwm);//舵机控制




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
        GPIO_Init(P33, 9,PUSHPULL, 0);//左正
        GPIO_Init(P33, 11,PUSHPULL, 1);//右正
//        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_2_TOUT33_P33_11_OUT, pwmR);//B右正
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM3_4_TOUT34_P33_12_OUT, pwmR);//D左正
//        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM1_1_TOUT31_P33_9_OUT, 0);//A右负
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_5_TOUT40_P32_4_OUT, pwmL);//C左负
    }
    else if(mySpeedL >= 0 && mySpeedR < 0)
    {
        pwmR = -mySpeedR;
        pwmL = mySpeedL;
        GPIO_Init(P33, 9,PUSHPULL, 0);//左正
        GPIO_Init(P33, 11,PUSHPULL, 0);//右正
//        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_2_TOUT33_P33_11_OUT, 0);//B
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM3_4_TOUT34_P33_12_OUT, pwmR);//D
//        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM1_1_TOUT31_P33_9_OUT, pwmR);//A
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_5_TOUT40_P32_4_OUT, pwmL);//C
    }
    else if(mySpeedL < 0 && mySpeedR >= 0)
    {
        pwmL = -mySpeedL;
        pwmR = mySpeedR;
        GPIO_Init(P33, 9,PUSHPULL, 1);//左正
        GPIO_Init(P33, 11,PUSHPULL, 1);//右正
//        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_2_TOUT33_P33_11_OUT, pwmR);//B
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM3_4_TOUT34_P33_12_OUT, pwmR);//D
//        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM1_1_TOUT31_P33_9_OUT, 0);//A
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_5_TOUT40_P32_4_OUT, pwmL);//C
    }
    else if(mySpeedL < 0 && mySpeedR < 0)
    {
        pwmL = -mySpeedL;
        pwmR = -mySpeedR;
        GPIO_Init(P33, 9,PUSHPULL, 1);//左正
        GPIO_Init(P33, 11,PUSHPULL, 0);//右正
//        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_2_TOUT33_P33_11_OUT, 0);//B
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM3_4_TOUT34_P33_12_OUT, pwmR);//D
//        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM1_1_TOUT31_P33_9_OUT, pwmR);//A
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_5_TOUT40_P32_4_OUT, pwmL);//C
    }



}


void CTRL_motorMain()
{
//    if(straightSpeedUp() && zebraCircle < display6.intValue)
//    {
//        expectL = (int32)(display7.intValue);
//        expectR = (int32)(display7.intValue);
//    }
//
//    testFlag++;

    if(stopFlag == 0 && flagStop == 0)//flagStop=1为车库停车，stopFlag=1为出赛道停车
    {
        CTRL_CarParkStart();


        CTRL_motorDiffer();

    }
    else if(flagStop == 1)
    {
        CTRL_CarParkStop();

//        expectL = 0;
//        expectR= 0;
//        CTRL_speedLoopPID();
//        CTRL_curLoopPID();

    }
    else if(stopFlag == 1)
    {
        expectL = 0;
        expectR = 0;
    }

    CTRL_motorPID();
    CTRL_motor();


}

void CTRL_ParkStartServo(float gyro)
{
//    servoPwm = (uint32)(gyro * (-0.8) + 760);
    servoPwm = (uint32)(gyro * (-0.8) + 780);

    if(servoPwm > 780)
        servoPwm = 780;
    else if(servoPwm < 620)
        servoPwm = 620;

}

void CTRL_motorDiffer()
{

    int delta;
    delta = 700 - servoPwm;

    /*内轮减速*/
    float k;
    if(delta > 0)
    {
        k = gap.floatValue * (0.9935 - 0.0047 * delta);
        if(k > 1) k = 1;
        if(speedFlag == 1)
        {
            expectL = (int32)(presentSpeed.intValue * display6.floatValue);
            expectR = (int32)(presentSpeed.intValue * display6.floatValue * k);
            GPIO_Set(P22, 0, 1);
        }
        else if(speedFlag == 0)
        {
            if(state == 11)
            {
                expectL = (int32)(presentSpeed.intValue * display7.floatValue);
                expectR = (int32)(presentSpeed.intValue * display7.floatValue * k);
            }
            else
            {
                expectL = (int32)(presentSpeed.intValue);
                expectR = (int32)(presentSpeed.intValue * k);
            }
            GPIO_Set(P22, 0, 0);
        }

    }
    else if(delta < 0)
    {
        k = gap.floatValue * (0.9935 + 0.0047 * delta);
        if(k > 1) k = 1;
        if(speedFlag == 1)
        {
            expectL = (int32)(presentSpeed.intValue * display6.floatValue * k);
            expectR = (int32)(presentSpeed.intValue * display6.floatValue);
            GPIO_Set(P22, 0, 1);
        }
        else if(speedFlag == 0)
        {
            if(state == 11)
            {
                expectL = (int32)(presentSpeed.intValue * display7.floatValue * k);
                expectR = (int32)(presentSpeed.intValue * display7.floatValue);
            }
            else
            {
                expectL = (int32)(presentSpeed.intValue * k);
                expectR = (int32)(presentSpeed.intValue);
            }
            GPIO_Set(P22, 0, 0);
        }
    }
    else if(delta == 0)
    {

        if(speedFlag == 1)
        {
            expectL = (int32)(presentSpeed.intValue * display6.floatValue);
            expectR = (int32)(presentSpeed.intValue * display6.floatValue);
            GPIO_Set(P22, 0, 1);
        }
        else if(speedFlag == 0)
        {
            if(state == 11)
            {
                expectL = (int32)(presentSpeed.intValue * display7.floatValue);
                expectR = (int32)(presentSpeed.intValue * display7.floatValue);
            }
            else
            {
                expectL = (int32)(presentSpeed.intValue);
                expectR = (int32)(presentSpeed.intValue);
            }
            GPIO_Set(P22, 0, 0);
        }

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
        if(currentGyro < -30)
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
        if(currentGyro > 40 || currentGyro < -40)
        {
            expectL = 0;
            expectR = 0;
//            currentGyro = 80;
        }

        else if(currentGyro <= 40 && currentGyro >= -40)
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

void CTRL_CircleForsee(int radius)
{
    uint32 radius_square;
    uint32 bigger_radius_square;
    uint32 distance;
    double delta_x;
    radius_square = radius * radius;
    bigger_radius_square = (radius + 1) * (radius + 1);
    for(int forsee = 85; forsee > 20; forsee--)
    {
        distance = (center_x - mid_line[forsee]) * (center_x - mid_line[forsee]) + (center_y - forsee) * (center_y - forsee);
        if(distance >= radius_square && distance <= bigger_radius_square)
        {
            delta_x = (double)(center_x - mid_line[forsee]);
            my_sin = delta_x / radius;

            break;

        }
    }
//    flagCircleForsee = 0;


}

void CTRL_CircleServoPID()
{

    servoError.currentError = my_sin;
    servoError.delta = servoError.lastError - servoError.currentError;
    servoPwm = (uint32)(740 + presentServoD.floatValue * servoError.delta + fuzzyPB.floatValue * servoError.currentError);
    if(servoPwm > 820)
        servoPwm = 820;
    else if(servoPwm < 670)
        servoPwm = 670;
    servoError.lastError = servoError.currentError;
}

void CTRL_ServoPID_Determine()
{
    if(state == stateStart)
    {
        fuzzy_PB = fuzzyPB.floatValue;
        fuzzy_PM = fuzzyPM.floatValue;
        fuzzy_PS = fuzzyPS.floatValue;
        fuzzy_ZO = fuzzyZO.floatValue;
        fuzzy_NS = fuzzyNS.floatValue;
        fuzzy_NM = fuzzyNM.floatValue;
        fuzzy_NB = fuzzyNB.floatValue;
        fuzzy_Dsmall = presentServoD.floatValue;
        fuzzy_Dbig = presentServoD.floatValue;

    }

    else if(state == 1 || state == 2)//cross
    {
        fuzzy_PB = Cross_PB.floatValue;
        fuzzy_PM = Cross_PM.floatValue;
        fuzzy_PS = Cross_PS.floatValue;
        fuzzy_ZO = Cross_ZO.floatValue;
        fuzzy_NS = Cross_NS.floatValue;
        fuzzy_NM = Cross_NM.floatValue;
        fuzzy_NB = Cross_NB.floatValue;
        fuzzy_Dsmall = Cross_DS.floatValue;
        fuzzy_Dbig = Cross_DB.floatValue;

    }

    else if(state == 4 || state == 5)//crossCircle
    {
        fuzzy_PB = circle_PB.floatValue;
        fuzzy_PM = circle_PM.floatValue;
        fuzzy_PS = circle_PS.floatValue;
        fuzzy_ZO = circle_ZO.floatValue;
        fuzzy_NS = circle_NS.floatValue;
        fuzzy_NM = circle_NM.floatValue;
        fuzzy_NB = circle_NB.floatValue;
        fuzzy_Dsmall = circle_DS.floatValue;
        fuzzy_Dbig = circle_DB.floatValue;

    }

    else if(state == 4)//island-45678
    {
        fuzzy_PB = Island_PB.floatValue;
        fuzzy_PM = Island_PM.floatValue;
        fuzzy_PS = Island_PS.floatValue;
        fuzzy_ZO = Island_ZO.floatValue;
        fuzzy_NS = Island_NS.floatValue;
        fuzzy_NM = Island_NM.floatValue;
        fuzzy_NB = Island_NB.floatValue;
        fuzzy_Dsmall = Island_DS.floatValue;
        fuzzy_Dbig = Island_DB.floatValue;

    }

    else if(state == 11)//folk
    {
        fuzzy_PB = Folk_PB.floatValue;
        fuzzy_PM = Folk_PM.floatValue;
        fuzzy_PS = Folk_PS.floatValue;
        fuzzy_ZO = Folk_ZO.floatValue;
        fuzzy_NS = Folk_NS.floatValue;
        fuzzy_NM = Folk_NM.floatValue;
        fuzzy_NB = Folk_NB.floatValue;
        fuzzy_Dsmall = Folk_DS.floatValue;
        fuzzy_Dbig = Folk_DB.floatValue;

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

