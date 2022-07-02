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
error gyroError = {1, 1, 1};
int sumErrorRT = 0, sumErrorLF = 0;
error currentErrorL = {1, 1, 1}, currentErrorR = {1, 1, 1};
uint32 pwmFix;
uint32 servoPwm;
uint32 servoGyroPwm;
uint32 motorPwm;
int32 expectL, expectR;//预期速度
int32 speedL, speedR, lastSpeedL, lastSpeedR;//实际速度
int32 mySpeedL = 0, mySpeedR = 0;
uint8_t present_speed, present_vision;
float motorLFKP, motorLFKI, motorLFKD, motorRTKP, motorRTKI, motorRTKD;
float currentKP_L, currentKI_L, currentKP_R, currentKI_R;
double my_sin;//圆形前瞻
int flagCircleForsee = 1;

float fuzzy_PB = 3.2, fuzzy_PM = 3.0, fuzzy_PS = 2.9, fuzzy_ZO = 2.7, fuzzy_NS = 2.9, fuzzy_NM = 3.0, fuzzy_NB = 3.2, fuzzy_D = 6.6;

/*陀螺仪相关*/
float deltaGyro[10] = {0}, currentGyro = 0, directionAngle = 0, lastGyro = 0;
float lastRampGyro = 0, rampGyro = 0, rampGyroMax = 0;
float deltaAcc = 0, currentAcc = 0;
float averageSpeed = 0;
float position = 0; positionX = 0, positionY = 0;
float expectGyro = 0;
float gyroK = 20;
float dx = 0, dy = 0;
int parkStart = 1;//陀螺仪进出车库
//int parkType = 1;//保存parkStart的值
uint8_t parkStraightCount = 0;
float expectGyro;//期望角速度

/*电流环参数*/
int currentFlag = 0, currentTime = 0;
uint16 currentLF[5] = {2250}, currentRT[5] = {2250};
int currentExpectLF = 2280, currentExpectRT = 2280;

uint8_t testFlag = 1;
uint8_t testStateTimes = 0;
uint8_t duzhuanCount = 0, duzhuanFlag = 0, duzhuanTime = 0;
uint32_t startCount = 0;
uint8_t startFlag = 0;
int islandPwmMax, islandPwmMin;

void CTRL_gyroInit()
{
    inv_imuGyroyQuene = INV_DataQueueInit(CTRL_IMU_QUENE_SIZE);
    INV_ICM20602_Init();
    currentGyro = 0;

}

void CTRL_gyroUpdate()
{
    INV_ImuUpdate();
    deltaGyro[0] = inv_gyro[2];// 1俯仰 0左右翻转 2左右转
    CTRL_gyroAverageFilter();

    deltaAcc = inv_accl[0];//2俯仰 1左右反转 0左右转


}

void CTRL_directionAngleGet()
{
    currentGyro += deltaGyro[0] * 0.005;

//    if(currentGyro > 180) currentGyro -= 360;
//    else if(currentGyro < -180) currentGyro += 360;
}

void CTRL_rampGyroUpdate()
{
    lastRampGyro = rampGyro;
    INV_ImuUpdate();
    rampGyro += inv_gyro[1];// 1俯仰 0左右翻转 2左右转
    if(rampGyro < lastRampGyro)
    {
        rampGyroMax = rampGyro;
    }

}

uint8_t CTRL_fuzzySpeedKp(int speedError)
{
    float membership[2] = {1, 0};
    uint8_t motorKP = 0;
    float fuzzyWidth = 10;

    if(speedError >= speedPB)
    {
        motorKP = LFKP.intVal;
    }
    else if(speedError < speedPB && speedError > speedPM)
    {
        membership[0] = fabs((speedError - speedPM) / fuzzyWidth);
        membership[1] = fabs((speedError - speedPB) / fuzzyWidth);

        motorKP = LFKP.intVal * membership[0] + LFKI.intVal * membership[1];
    }
    else if(speedError == PM)
    {
        motorKP = LFKI.intVal;
    }

    else if(speedError < speedPM && speedError > speedPS)
    {
        membership[0] = fabs((speedError - speedPS) / fuzzyWidth);
        membership[1] = fabs((speedError - speedPM) / fuzzyWidth);

        motorKP = LFKI.intVal * membership[0] + RTKP.intVal * membership[1];
    }
    else if(speedError == PS)
    {
        motorKP = RTKP.intVal;
    }

    else if(speedError < speedPS && speedError > speedZO)
    {
        membership[0] = fabs((speedError - speedZO) / fuzzyWidth);
        membership[1] = fabs((speedError - speedPM) / fuzzyWidth);

        motorKP = RTKP.intVal * membership[0] + RTKI.intVal * membership[1];
    }
    else if(speedError == ZO)
    {
        motorKP = RTKI.intVal;
    }
    else if(speedError < speedZO && speedError > speedNM)
    {
        membership[0] = fabs((speedError - speedNM) / fuzzyWidth);
        membership[1] = fabs((speedError - speedZO) / fuzzyWidth);

        motorKP = RTKI.intVal * membership[0] + fastLFKP.intVal * membership[1];
    }
    else if(speedError == speedNM)
    {
        motorKP = fastLFKP.intVal;
    }
    else if(speedError < speedNM && speedError > speedNB)
    {
        membership[0] = fabs((speedError - speedNB) / fuzzyWidth);
        membership[1] = fabs((speedError - speedNM) / fuzzyWidth);

        motorKP = fastLFKP.intVal * membership[0] + fastLFKI.intVal * membership[1];
    }

    else if(speedError <= speedNB)
    {
        motorKP = fastLFKI.intVal;
    }

    return motorKP;
}

void CTRL_speedLoopPID()
{


    speedL = CTRL_speedGetLeft();
    speedR = CTRL_speedGetRight();
    CTRL_lowpassFilter();

    test_varible[9] = expectL;
    test_varible[10] = expectR;

    errorML.currentError = expectL + speedL;//取偏差
    errorMR.currentError = expectR - speedR;//取偏差
    motorLFKI = CTRL_fuzzySpeedKp(errorML.currentError);
    motorRTKI = CTRL_fuzzySpeedKp(errorMR.currentError);

//    sumErrorLF += errorML.currentError;
//    errorML.delta = errorML.currentError - errorML.lastError;
    errorML.delta = (errorML.currentError - errorML.lastError) * speedKdLpf.floatVal + errorML.delta * (1 - speedKdLpf.floatVal);
//    currentExpectLF = (int32)(2210 + motorLFKP * errorML.currentError + motorLFKI * errorML.delta);
    currentExpectLF = (int32)(currentExpectLF + LFKI.intVal * errorML.currentError + LFKP.intVal * errorML.delta);
    errorML.lastError = errorML.currentError;//更新上一次误差
    if(currentExpectLF < 1000) currentExpectLF = 1000;
    else if(currentExpectLF > 4095) currentExpectLF = 4095;


//    sumErrorRT += errorMR.currentError;
//    errorMR.delta = errorMR.currentError - errorMR.lastError;
    errorMR.delta = (errorMR.currentError - errorMR.lastError) * speedKdLpf.floatVal + errorMR.delta * (1 - speedKdLpf.floatVal);

//    currentExpectRT = (int32)(2210 + motorRTKP * errorMR.currentError + motorRTKI * errorMR.delta);
    currentExpectRT = (int32)(currentExpectRT + LFKI.intVal * errorMR.currentError + LFKP.intVal * errorMR.delta);
    errorMR.lastError = errorMR.currentError;//更新上一次误差
    if(currentExpectRT < 1000) currentExpectRT = 1000;
    else if(currentExpectRT > 4095) currentExpectRT = 4095;

    test_varible[0] = -speedL;
    test_varible[1] = speedR;
//
//    test_varible[4] = currentExpectLF;
//    test_varible[5] = currentExpectRT;


}


void CTRL_curLoopPID()
{
    int currentLF_real = 0;

    currentRT[0] = ADC_Get(ADC_0, ADC0_CH5_A5, ADC_12BIT);//右轮
    currentLF[0] = ADC_Get(ADC_0, ADC0_CH7_A7, ADC_12BIT);//左轮

    CTRL_currentAverageFilter();

//    test_varible[7] = currentLF[0];
//    test_varible[8] = currentRT[0];


//    currentLF_real = 4420 - currentLF;

    int currentR = 0, currentL = 0;
    if(state == stateRampway)
    {
        currentR = currentExpectRT + 200;
        currentL = currentExpectLF + 200;

        currentErrorR.currentError = currentR - currentRT[0];
        currentErrorL.currentError = currentL - currentLF[0];

    }

    else
    {
        currentErrorR.currentError = currentExpectRT - currentRT[0];
        currentErrorL.currentError = currentExpectLF - currentLF[0];

    }
//    currentErrorR.currentError = currentExpectRT - currentRT[0];
    currentErrorR.delta = currentErrorR.currentError - currentErrorR.lastError;
    mySpeedR = (int32)(mySpeedR + currentErrorR.currentError * currentKI_R + currentErrorR.delta * currentKP_R);
    currentErrorR.lastError = currentErrorR.currentError;

    currentErrorL.delta = currentErrorL.currentError - currentErrorL.lastError;
    mySpeedL = (int32)(mySpeedL + currentErrorL.currentError * currentKI_L + currentErrorL.delta * currentKP_L);
    currentErrorL.lastError = currentErrorL.currentError;


}

void CTRL_Init()
{
//    presentSpeedL.intVal = 1800;
//    presentSpeedR.intVal = 1800;

    Pit_Init_ms(CCU6_0, PIT_CH0, 5);//定时器中断，给电机使用（0模块0通道）
    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM0_0_TOUT48_P22_1_OUT, 50, 730);//舵机
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
    ADC_Init(ADC_1, ADC1_CH5_A21);

    CTRL_gyroInit();


}


void CTRL_servoPID()
{

    servoError.currentError = 94 - mid_line[presentVision.intVal];

    servoError.delta = servoError.currentError - servoError.lastError;
    servoPwm = (uint32)(servoMidValue + presentServoD.floatVal * servoError.delta + fuzzyPB.floatVal * servoError.currentError);
    if(servoPwm > servoMax)
        servoPwm = servoMax;
    else if(servoPwm < servoMin)
        servoPwm = servoMin;
    servoError.lastError = servoError.currentError;

}

float CTRL_FuzzyMemberShip(int midError)
{
    float membership[2] = {1, 0};
    float servoKP = 0;
    float fuzzyWidth = 10;

    if(midError >= PB)
    {
        servoKP = fuzzy_PB;
    }
    else if(midError < PB && midError > PM)
    {
        membership[0] = fabs((midError - PM) / fuzzyWidth);
        membership[1] = fabs((midError - PB) / fuzzyWidth);

        servoKP = fuzzy_PB * membership[0] + fuzzy_PM * membership[1];
    }
    else if(midError == PM)
    {
        servoKP = fuzzy_PM;
    }
    else if(midError < PM && midError > PS)
    {
        membership[0] = fabs((midError - PS) / fuzzyWidth);
        membership[1] = fabs((midError - PM) / fuzzyWidth);

        servoKP = fuzzy_PM * membership[0] + fuzzy_PS * membership[1];
    }
    else if(midError == PS)
    {
        servoKP = fuzzy_PS;
    }
    else if(midError < PS && midError > ZO)
    {
        membership[0] = fabs((midError - ZO) / fuzzyWidth);
        membership[1] = fabs((midError - PS) / fuzzyWidth);

        servoKP = fuzzy_PS * membership[0] + fuzzy_ZO * membership[1];
    }
    else if(midError == ZO)
    {
        servoKP = fuzzy_ZO;
    }
    else if(midError < ZO && midError > NS)
    {
        membership[0] = fabs((midError - NS) / fuzzyWidth);
        membership[1] = fabs((midError - ZO) / fuzzyWidth);

        servoKP = fuzzy_ZO * membership[0] + fuzzy_NS * membership[1];
    }
    else if(midError == NS)
    {
        servoKP = fuzzy_NS;
    }
    else if(midError < NS && midError > NM)
    {
        membership[0] = fabs((midError - NM) / fuzzyWidth);
        membership[1] = fabs((midError - NS) / fuzzyWidth);

        servoKP = fuzzy_NS * membership[0] + fuzzy_NM * membership[1];
    }
    else if(midError == NM)
    {
        servoKP = fuzzy_NM;
    }
    else if(midError < NM && midError > NB)
    {
        membership[0] = fabs((midError - NB) / fuzzyWidth);
        membership[1] = fabs((midError - NM) / fuzzyWidth);

        servoKP = fuzzy_NM * membership[0] + fuzzy_NB * membership[1];
    }
    else if(midError <= NB)
    {
        servoKP = fuzzy_NB;
    }


    return servoKP;
}

void CTRL_fuzzyPID()
{
    float fuzzyKP = 1;
//    if(memoryFlag == 1)
//    {
//        present_vision = memoryVision1.intVal;
//    }
//    else if(memoryFlag == 0)
//    {
//        present_vision = presentVision.intVal;
//    }
//    int realVision;
//    realVision = foresee();
//    servoError.currentError = 92 - mid_line[presentVision.intVal];
    uint8_t myMidLine;
    myMidLine = aver_mid_line_foresee();
    if(state == stateRampway)
    {
        servoError.currentError = 93 - mid_line[present_vision+10];

    }
    else servoError.currentError = 93 - myMidLine;
    test_varible[12] = myMidLine;
//    servoError.currentError = 94 - mid_line[realVision];
    servoError.delta = servoError.currentError - servoError.lastError;
    fuzzyKP = CTRL_FuzzyMemberShip(servoError.currentError);
    servoPwm = (uint32)(servoMidValue + fuzzy_D * servoError.delta + fuzzyKP * servoError.currentError);

    if(state == stateTIn)
    {
        servoPwm += pwmFix;
    }

//    else if(state == stateIslandCircle)
//    {
//        servoPwm += pwmFix;

//    }

    if(servoPwm > servoMax)
        servoPwm = servoMax;
    else if(servoPwm < servoMin)
        servoPwm = servoMin;

    servoError.lastError = servoError.currentError;

}

//void CTRL_midLineLoopPID()
//{
//    int averageSpeed;
//    float fuzzyKP = 1;
//
//    averageSpeed = (speedR - speedL) / 2;
//    servoError.currentError = 92 - mid_line[presentVision.intVal];
//    test_varible[12] = servoError.currentError;
//    expectGyro = servoError.currentError * midLineKP.floatVal * averageSpeed;
//
//    servoError.delta = servoError.currentError - servoError.lastError;
//    fuzzyKP = CTRL_FuzzyMemberShip(servoError.currentError);
//    servoPwm = (uint32)(servoMidValue + presentServoD.floatVal * servoError.delta + fuzzyKP * servoError.currentError);
//    if(servoPwm > servoMax)
//        servoPwm = servoMax;
//    else if(servoPwm < servoMin)
//        servoPwm = servoMin;
//
//    servoError.lastError = servoError.currentError;
////    test_varible[13] = servoPwm;
////    test_varible[15] = expectGyro;
//
//}
//
//void CTRL_gyroLoopPID()
//{
//    CTRL_gyroUpdate();
////    expectGyro = midLineKP.floatVal;
//    gyroError.currentError = expectGyro - deltaGyro[0];
//    gyroError.delta = gyroError.currentError - gyroError.lastError;
//    if(straightFlag == 0)
//    {
//        if(abs(servoError.currentError) > 10 || (state == 5 || state == 6 || state == 7 || state == 9 || state == 12 || state == 16))
//        {
//            servoGyroPwm = (uint32)(servoPwm + gyroKP.floatVal * gyroError.currentError + gyroKD.floatVal * gyroError.delta);
//
//            if(servoGyroPwm > servoPwm + 5)
//            {
//                servoGyroPwm = servoPwm + 5;
//            }
//            else if(servoGyroPwm < servoPwm - 5)
//            {
//                servoGyroPwm = servoPwm - 5;
//
//            }
//        }
//
//        else
//        {
//            servoGyroPwm = servoPwm;
//        }
//    }
//
//    else
//    {
//        servoGyroPwm = servoPwm;
//    }
//    if(servoGyroPwm > servoMax)
//        servoGyroPwm = servoMax;
//    else if(servoGyroPwm < servoMin)
//        servoGyroPwm = servoMin;
//
//    gyroError.lastError = gyroError.currentError;
//}

void CTRL_motorPID()
{

    currentRT[0] = ADC_Get(ADC_0, ADC0_CH5_A5, ADC_12BIT);//右轮
    currentLF[0] = ADC_Get(ADC_0, ADC0_CH7_A7, ADC_12BIT);//左轮
    CTRL_currentAverageFilter();
//    test_varible[7] = currentLF[0];
//    test_varible[8] = currentRT[0];


    speedL = CTRL_speedGetLeft();
    speedR = CTRL_speedGetRight();
    speedL = speedL / 8;

    CTRL_lowpassFilter();



    errorML.currentError = expectL + speedL;//取偏差
    errorML.delta = errorML.currentError - errorML.lastError;
    mySpeedL = (int32)(mySpeedL + motorLFKI * errorML.currentError + motorLFKP * errorML.delta);
    errorML.lastError = errorML.currentError;//更新上一次误差

    errorMR.currentError = expectR - speedR;//取偏差
    errorMR.delta = errorMR.currentError - errorMR.lastError;
    mySpeedR = (int32)(mySpeedR + motorRTKI * errorMR.currentError + motorRTKP * errorMR.delta);
    errorMR.lastError = errorMR.currentError;//更新上一次误差

    lastSpeedL = speedL;
    lastSpeedR = speedR;

    test_varible[0] = speedL;
    test_varible[1] = speedR;


}


void CTRL_servoMain()
{

    if(GPIO_Read(P13, 2))
    {
        if(parkStart == 1 || parkStart == 2)
        {
            parkStraightCount += 1;
            if(parkStraightCount <= parkCount.intVal)
            {
                servoPwm = servoMidValue;
            }
            else
            {
                servoPwm = servoMin;//630

            }
        }
        else if(parkStart == -1 || parkStart == -2)
        {
            parkStraightCount += 1;
            if(parkStraightCount <= parkCount.intVal)
            {
                servoPwm = servoMidValue;
            }
            else
            {
                servoPwm = servoMax;//770

            }
        }
        else if(parkStart == 0 && flagStop == 0)
        {
            if(duzhuanFlag == 0)
            {

                CTRL_ServoPID_Determine();

                CTRL_fuzzyPID();

            }
            else if(duzhuanFlag == 1)
            {
                CTRL_duzhuanZhuanWan();

            }
//
        }
        else if(flagStop == 1 && leftPark == 0 && rightPark == 1)
        {
            servoPwm = servoMin;//630
        }
        else if(flagStop == 1 && leftPark == 1 && rightPark == 0)
        {
            servoPwm = servoMax;//770
        }
    }
    CTRL_islandPwmCount();
    CTRL_rampPwmxianfu();
    CTRL_carParkPwmxianfu();


    if(servoPwm > servoMax)
        servoPwm = servoMax;
    else if(servoPwm < servoMin)
        servoPwm = servoMin;
//    servoPwm = display8.intVal;
    test_varible[11] = servoPwm;

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

//    test_varible[2] = mySpeedL;
//    test_varible[3] = mySpeedR;

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
//    CTRL_gyroUpdate();
    if(stopFlag == 0 && flagStop == 0 && testFlag == 1 && duzhuanFlag == 0)//flagStop=1为车库停车，stopFlag=1为出赛道停车,testFlag=1为赛道测试
    {
        CTRL_CarParkStart();

        CTRL_duzhuanTest();

        speedDetermine();
        CTRL_motorDiffer();
//        CTRL_visionDecision();

        CTRL_RoadTest();

    }
    else if(flagStop == 1 && duzhuanFlag == 0)
    {
        CTRL_CarParkStop();

    }
    else if(stopFlag == 1 && duzhuanFlag == 0)
    {
        expectL = 0;
        expectR = 0;
    }
    else if(testFlag == 0)
    {
        expectL = 0;
        expectR = 0;
    }

    if(duzhuanFlag == 1)
    {
        CTRL_duzhuan();
    }

    motorParamDefine();

//    CTRL_motorPID();
    CTRL_speedLoopPID();
    CTRL_curLoopPID();
    CTRL_motor();


}

void CTRL_motorDiffer()
{

    double delta, FabsDelta;
    int32 speedDelta;
    delta = (double)(servoMidValue - servoPwm);
    FabsDelta = fabs(delta);
    FabsDelta = FabsDelta / 100;

    /*内轮减速*/
    float k;
    if(delta > 0)
    {
//        k = 0.9459 - 0.2679 * FabsDelta * FabsDelta - 0.1511 * FabsDelta;
        k = 0.9997 - 0.4404 * FabsDelta;
        expectL = (int32)(present_speed);
        expectR = (int32)(present_speed * k);
//        if(straightFlag == 1)
//        {
//            expectL = (int32)(present_speed * display6.floatVal);
//            expectR = (int32)(present_speed * display6.floatVal * k);
//            GPIO_Set(P22, 0, 1);
//        }
//        else if(straightFlag == 0)
//        {
//            if(state == 20 || state == 50)
//            {
//                expectL = (int32)(present_speed * display7.floatVal);
//                expectR = (int32)(present_speed * display7.floatVal * k);
//            }
//            else if(state == 130)
//            {
//                expectL = rampSpeed.intVal;
//                expectR = rampSpeed.intVal;
//            }
//            else
//            {
//                expectL = (int32)(present_speed);
//                expectR = (int32)(present_speed * k);
//            }
//            GPIO_Set(P22, 0, 0);
//        }

        speedDelta = (int32)((expectL - expectR) * gap.floatVal);
        if(speedDelta >= 0)
        {
            expectR = expectL - speedDelta;

        }
    }
    else if(delta < 0)
    {
        k = 0.9927 - 0.4257 * FabsDelta;

        expectL = (int32)(present_speed * k);
        expectR = (int32)(present_speed);
//        if(straightFlag == 1)
//        {
//            expectL = (int32)(present_speed * display6.floatVal * k);
//            expectR = (int32)(present_speed * display6.floatVal);
//            GPIO_Set(P22, 0, 1);
//        }
//        else if(straightFlag == 0)
//        {
//            if(state == 20 || state == 50)
//            {
//                expectL = (int32)(present_speed * display7.floatVal * k);
//                expectR = (int32)(present_speed * display7.floatVal);
//            }
//            else if(state == 130)
//            {
//                expectL = rampSpeed.intVal;
//                expectR = rampSpeed.intVal;
//            }
//            else
//            {
//                expectL = (int32)(present_speed * k);
//                expectR = (int32)(present_speed);
//            }
//            GPIO_Set(P22, 0, 0);
//        }

        speedDelta = (int32)((expectR - expectL) * gap.floatVal);
        if(speedDelta >= 0)
        {
            expectL = expectR - speedDelta;

        }
    }
    else if(delta == 0)
    {
        expectL = (int32)(present_speed);
        expectR = (int32)(present_speed);
//        if(straightFlag == 1)
//        {
//            expectL = (int32)(present_speed * display6.floatVal);
//            expectR = (int32)(present_speed * display6.floatVal);
//            GPIO_Set(P22, 0, 1);
//        }
//        else if(straightFlag == 0)
//        {
//            if(state == 20 || state == 50)
//            {
//                expectL = (int32)(present_speed * display7.floatVal);
//                expectR = (int32)(present_speed * display7.floatVal);
//            }
//            else if(state == 130)
//            {
//                expectL = rampSpeed.intVal;
//                expectR = rampSpeed.intVal;
//            }
//            else
//            {
//                expectL = (int32)(present_speed);
//                expectR = (int32)(present_speed);
//            }
//            GPIO_Set(P22, 0, 0);
//        }

    }


    /*内减外加*/
//    double kIN, kOUT;
//
//    if(delta > 0)//右转
//    {
//        kIN = gap.floatVal * (0.9925 - 0.0885 * FabsDelta * FabsDelta - 0.2881 * FabsDelta);
//        kOUT = gap.floatVal * (1.0027 + 0.0265 * FabsDelta * FabsDelta + 0.067 * FabsDelta);
//
//        if(straightFlag == 1)
//        {
//            expectL = (int32)(present_speed * display6.floatVal * kOUT);
//            expectR = (int32)(present_speed * display6.floatVal * kIN);
//            GPIO_Set(P22, 0, 1);
//        }
//        else if(straightFlag == 0)
//        {
//            if(state == 20 || state == 60)
//            {
//                expectL = (int32)(present_speed * display7.floatVal * kOUT);
//                expectR = (int32)(present_speed * display7.floatVal * kIN);
//            }
//            else if(state == 130)
//            {
//                expectL = rampSpeed.intVal;
//                expectR = rampSpeed.intVal;
//            }
//            else
//            {
//                expectL = (int32)(present_speed * kOUT);
//                expectR = (int32)(present_speed * kIN);
//            }
//            GPIO_Set(P22, 0, 0);
//        }
//
//    }
//    else if(delta < 0)
//    {
//        kIN = gap.floatVal * (0.9808 - 0.0803 * FabsDelta * FabsDelta - 0.2924 * FabsDelta);
//        kOUT = gap.floatVal * (1.0042 + 0.0157 * FabsDelta * FabsDelta + 0.0771 * FabsDelta);
//
//
//        if(straightFlag == 1)
//        {
//            expectL = (int32)(present_speed * display6.floatVal * kIN);
//            expectR = (int32)(present_speed * display6.floatVal *kOUT);
//                GPIO_Set(P22, 0, 1);
//        }
//        else if(straightFlag == 0)
//        {
//            if(state == 20 || state == 60)
//            {
//                expectL = (int32)(present_speed * display7.floatVal * kIN);
//                expectR = (int32)(present_speed * display7.floatVal * kOUT);
//            }
//            else if(state == 130)
//            {
//                expectL = rampSpeed.intVal;
//                expectR = rampSpeed.intVal;
//            }
//            else
//            {
//                expectL = (int32)(present_speed * kIN);
//                expectR = (int32)(present_speed * kOUT);
//            }
//            GPIO_Set(P22, 0, 0);
//        }
//    }
//    else if(delta == 0)
//    {
//
//        if(straightFlag == 1)
//        {
//            expectL = (int32)(present_speed * display6.floatVal);
//            expectR = (int32)(present_speed * display6.floatVal);
//            GPIO_Set(P22, 0, 1);
//        }
//        else if(straightFlag == 0)
//        {
//            if(state == 20 || state == 60)
//            {
//                expectL = (int32)(present_speed * display7.floatVal);
//                expectR = (int32)(present_speed * display7.floatVal);
//            }
//            else if(state == 130)
//            {
//                expectL = rampSpeed.intVal;
//                expectR = rampSpeed.intVal;
//            }
//            else
//            {
//                expectL = (int32)(present_speed);
//                expectR = (int32)(present_speed);
//            }
//            GPIO_Set(P22, 0, 0);
//        }
//
//    }


}

void CTRL_CarParkStart()
{
    if(GPIO_Read(P13, 2) && (parkStart == 1 || parkStart == -1 || parkStart == 2 || parkStart == -2) && parkStraightCount > parkCount.intVal)
    {
        CTRL_gyroUpdate();
        CTRL_directionAngleGet();

    //
    //        if(currentGyro < -75)//正为左转 负为右转
    //        {
    //            parkStart = 0;
    //        }


        if(currentGyro < (-startGyro.intVal) || currentGyro > startGyro.intVal)
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
        if(currentGyro > endGyro.intVal || currentGyro < (-endGyro.intVal))
        {
            expectL = 0;
            expectR = 0;
//            currentGyro = 80;
        }

        else if(currentGyro <= endGyro.intVal && currentGyro >= (-endGyro.intVal))
        {
            expectL = present_speed;
            expectR = present_speed;

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

void CTRL_ServoPID_Determine()
{


    if((state == stateTIslandIn || state == stateTIn || state == stateTOut || state == stateSTIsland) && CrossCircle.intVal == 1)//crossCircle
    {
        fuzzy_PB = circle_PB.floatVal;
        fuzzy_PM = circle_PB.floatVal;
        fuzzy_PS = circle_PB.floatVal;
        fuzzy_ZO = circle_PB.floatVal;
        fuzzy_NS = circle_PB.floatVal;
        fuzzy_NM = circle_PB.floatVal;
        fuzzy_NB = circle_PB.floatVal;
        fuzzy_D = circle_DS.floatVal;
//        fuzzy_Dbig = circle_DB.floatVal;

    }

    else if((state == stateTIslandIn || state == stateIslandIng || state == stateIslandTurn || state == stateIslandCircle || state == stateIslandOut || state == stateIslandFinal) && IslandPD.intVal == 1)//island-45678
    {
        if(IslandRadius == 50)
        {
            fuzzy_PB = Island_PS.floatVal;
            fuzzy_PM = Island_PS.floatVal;
            fuzzy_PS = Island_PS.floatVal;
            fuzzy_ZO = Island_PS.floatVal;
            fuzzy_NS = Island_PS.floatVal;
            fuzzy_NM = Island_PS.floatVal;
            fuzzy_NB = Island_PS.floatVal;
            fuzzy_D = Island_DS.floatVal;
        }

        else if(IslandRadius >= 60 && IslandRadius <= 70)
        {
            fuzzy_PB = Island_PM.floatVal;
            fuzzy_PM = Island_PM.floatVal;
            fuzzy_PS = Island_PM.floatVal;
            fuzzy_ZO = Island_PM.floatVal;
            fuzzy_NS = Island_PM.floatVal;
            fuzzy_NM = Island_PM.floatVal;
            fuzzy_NB = Island_PM.floatVal;
            fuzzy_D = Island_DS.floatVal;
        }

        else if(IslandRadius >= 70 && IslandRadius <= 100)
        {
            fuzzy_PB = Island_PB.floatVal;
            fuzzy_PM = Island_PB.floatVal;
            fuzzy_PS = Island_PB.floatVal;
            fuzzy_ZO = Island_PB.floatVal;
            fuzzy_NS = Island_PB.floatVal;
            fuzzy_NM = Island_PB.floatVal;
            fuzzy_NB = Island_PB.floatVal;
            fuzzy_D = Island_DS.floatVal;
        }

//        fuzzy_Dbig = Island_DB.floatVal;

    }

    else if(FolkPD.intVal == 1 && (folkTimes == 1 || folkTimes == 3))
    {
        fuzzy_PB = Folk_PB.floatVal;
        fuzzy_PM = Folk_PM.floatVal;
        fuzzy_PS = Folk_PS.floatVal;
        fuzzy_ZO = Folk_ZO.floatVal;
        fuzzy_NS = Folk_NS.floatVal;
        fuzzy_NM = Folk_NM.floatVal;
        fuzzy_NB = Folk_NB.floatVal;
        fuzzy_D = Folk_DS.floatVal;
    }

    else if(straightPD.intVal == 1 && straightFlag == 1)
    {
        fuzzy_PB = straight_KP.floatVal;
        fuzzy_PM = straight_KP.floatVal;
        fuzzy_PS = straight_KP.floatVal;
        fuzzy_ZO = straight_KP.floatVal;
        fuzzy_NS = straight_KP.floatVal;
        fuzzy_NM = straight_KP.floatVal;
        fuzzy_NB = straight_KP.floatVal;
        fuzzy_D = straight_KD.floatVal;
    }

    else
    {
        fuzzy_PB = fuzzyPB.floatVal;
        fuzzy_PM = fuzzyPM.floatVal;
        fuzzy_PS = fuzzyPS.floatVal;
        fuzzy_ZO = fuzzyZO.floatVal;
        fuzzy_NS = fuzzyNS.floatVal;
        fuzzy_NM = fuzzyNM.floatVal;
        fuzzy_NB = fuzzyNB.floatVal;
        fuzzy_D = presentServoD.floatVal;
    //        fuzzy_Dbig = presentServoD.floatVal;

    }

}

int foresee()
{
    int speedAve;
    int speedDelta;
    int realForesee;
    speedAve = (expectL + expectR) / 2;
    speedDelta = speedAve - present_speed;
    if(speedDelta < 10 && speedDelta > -10)
    {
        realForesee = present_vision;
    }
    else if(speedDelta >= 10 && speedDelta < 20)
    {
        realForesee = present_vision - 1;
    }

    else if(speedDelta >= 20)
    {
        realForesee = present_vision - 2;
    }
    else if(speedDelta <= -10 && speedDelta > -20)
    {
        realForesee = present_vision + 1;
    }

    else if(speedDelta <= -20)
    {
        realForesee = present_vision + 2;
    }

    return realForesee;
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


void CTRL_lowpassFilter()
{
    float alpha;
    alpha = speedFilter.floatVal;
    speedL = speedL * alpha + lastSpeedL * (1 - alpha);
    speedR = speedR * alpha + lastSpeedR * (1 - alpha);


}

void CTRL_currentAverageFilter()
{
//    currentRT[0] = (currentRT[4] + currentRT[3] + currentRT[2] + currentRT[1] + currentRT[0]) / 5;
//    currentLF[0] = (currentLF[4] + currentLF[3] + currentLF[2] + currentLF[1] + currentLF[0]) / 5;
//    currentRT[0] = (currentRT[2] + currentRT[1] + currentRT[0]) / 3;
//    currentLF[0] = (currentLF[2] + currentLF[1] + currentLF[0]) / 3;
//    if((currentFilter1.floatVal + currentFilter2.floatVal) < 1)
//    {
//        currentRT[0] = currentRT[2] * (1 - currentFilter1.floatVal - currentFilter2.floatVal) + currentRT[1] * currentFilter2.floatVal + currentRT[0] * currentFilter1.floatVal;
//        currentLF[0] = currentLF[2] * (1 - currentFilter1.floatVal - currentFilter2.floatVal) + currentLF[1] * currentFilter2.floatVal + currentLF[0] * currentFilter1.floatVal;
//
//    }
//
//    else
//    {
//        currentRT[0] = (currentRT[2] + currentRT[1] + currentRT[0]) / 3;
//        currentLF[0] = (currentLF[2] + currentLF[1] + currentLF[0]) / 3;
//    }
    currentRT[0] = currentRT[1] * (1 - currentFilter1.floatVal) + currentRT[0] * currentFilter1.floatVal;
    currentLF[0] = currentLF[1] * (1 - currentFilter1.floatVal) + currentLF[0] * currentFilter1.floatVal;
//    currentRT[4] = currentRT[3];
//    currentRT[3] = currentRT[2];
    currentRT[2] = currentRT[1];
    currentRT[1] = currentRT[0];

//    currentLF[4] = currentLF[3];
//    currentLF[3] = currentLF[2];
    currentLF[2] = currentLF[1];
    currentLF[1] = currentLF[0];
}

void CTRL_gyroAverageFilter()
{
    deltaGyro[0] = (deltaGyro[0] + deltaGyro[1] + deltaGyro[2]) / 3;
//    deltaGyro[4] = deltaGyro[3];
    deltaGyro[6] = deltaGyro[5];
    deltaGyro[5] = deltaGyro[4];
    deltaGyro[4] = deltaGyro[3];
    deltaGyro[3] = deltaGyro[2];
    deltaGyro[2] = deltaGyro[1];
    deltaGyro[1] = deltaGyro[0];
}

void motorParamDefine()
{
    if(delayFlag == 1)
    {
//        if(straightFlag == 2)
//        {
//            motorLFKP = fastLFKP.intVal;
//            motorLFKI = fastLFKI.intVal;
//            motorRTKP = fastRTKP.intVal;
//            motorRTKI = fastRTKI.intVal;
//        }
//        else if(slowFlag == 1)
//        {
//            motorLFKP = slowLFKP.intVal;
//            motorLFKI = slowLFKI.intVal;
//            motorRTKP = slowRTKP.intVal;
//            motorRTKI = slowRTKI.intVal;
//        }
//        else
//        {
//            motorLFKP = LFKP.intVal;
//            motorLFKI = LFKI.intVal;
//            motorRTKP = RTKP.intVal;
//            motorRTKI = RTKI.intVal;
//        }


        currentKP_R = currentRTKP.floatVal;
        currentKI_R = currentRTKI.floatVal;
        currentKI_L = currentLFKI.floatVal;
        currentKP_L = currentLFKP.floatVal;
    }

}

void speedDetermine()
{


    if(memoryState[0] == 1)
    {
        present_speed = memorySpeed1.intVal;
        present_vision = memoryVision1.intVal;
    }
    else if(memoryState[1] == 1)
    {
        present_speed = memorySpeed2.intVal;
        present_vision = memoryVision2.intVal;
    }
    else if(memoryState[2] == 1)
    {
        present_speed = memorySpeed3.intVal;
        present_vision = memoryVision3.intVal;
    }
    else if(memoryState[3] == 1)
    {
        present_speed = memorySpeed4.intVal;
        present_vision = memoryVision4.intVal;
    }
    else if(memoryState[4] == 1)
    {
        present_speed = memorySpeed5.intVal;
        present_vision = memoryVision5.intVal;
    }
    else if(memoryState[5] == 1)
    {
        present_speed = memorySpeed6.intVal;
        present_vision = memoryVision6.intVal;
    }
    else if(memoryState[6] == 1)
    {
        present_speed = memorySpeed7.intVal;
        present_vision = memoryVision7.intVal;
    }
    else if(memoryState[7] == 1)
    {
        present_speed = memorySpeed8.intVal;
        present_vision = memoryVision8.intVal;
    }
    else if(memoryState[8] == 1)
    {
        present_speed = memorySpeed9.intVal;
        present_vision = memoryVision9.intVal;
    }
    else if(memoryState[9] == 1)
    {
        present_speed = memorySpeed10.intVal;
        present_vision = memoryVision10.intVal;
    }
    else if(memoryState[10] == 1)
    {
        present_speed = memorySpeed11.intVal;
        present_vision = memoryVision11.intVal;
    }
    else if(memoryState[11] == 1)
    {
        present_speed = memorySpeed12.intVal;
        present_vision = memoryVision12.intVal;
    }
    else if(memoryState[12] == 1)
    {
        present_speed = memorySpeed13.intVal;
        present_vision = memoryVision13.intVal;
    }
    else if(memoryState[13] == 1)
    {
        present_speed = memorySpeed14.intVal;
        present_vision = memoryVision14.intVal;
    }
    else if(memoryState[14] == 1)
    {
        present_speed = memorySpeed15.intVal;
        present_vision = memoryVision15.intVal;
    }
//    else if(memoryState[15] == 1)
//    {
//        present_speed = memorySpeed2.intVal;
//        present_vision = memoryVision2.intVal;
//    }
    else if(parkStart == 0)
    {
        present_speed = presentSpeed.intVal;
        present_vision = presentVision.intVal;
    }

    else if(parkStart != 0)
    {
        present_speed = 60;
//        present_vision = 60;
    }

//    if(state == stateTIslandIn || state == stateIslandIng || state == stateIslandTurn || state == stateIslandCircle || state == stateIslandOut || state == stateIslandFinal)
//    {
//        present_vision = islandParam2.intVal;
//    }

    if(state == stateTIslandIn || state == stateTIn || state == stateTOut)
    {
        present_vision = cross_circle_param1.intVal;
    }

    if(straightFlag == 1)
    {
        present_speed = present_speed * display6.floatVal;
    }
    else if(straightFlag == 0)
    {
        if(state == 20 || state == 50)
        {
            present_speed = (uint8_t)(present_speed * display7.floatVal);
//            present_speed = (uint8_t)(present_speed * display7.floatVal);
        }
        else if(state == 130)
        {
            present_speed = rampSpeed.intVal;
//            present_speed = rampSpeed.intVal;
        }
        else
        {
            present_speed = presentSpeed.intVal;
//            present_speed = present_speed;
        }
//        GPIO_Set(P22, 0, 0);
    }

    CTRL_speedDecision(speedLow.intVal, present_speed);
//    present_vision = foresee();
}
void CTRL_RoadTest()
{
    if(file1.intVal == 0)
    {
        if(laststate != 0 && state == 0)
        {
            testStateTimes += 1;
            laststate = 0;

        }
        if(testStateTimes == display9.intVal)
        {
            testFlag = 0;
        }
    }
}


void CTRL_speedDecision(int32 speedHigh, int32 speedLow)
{
    float speedDelta;
    float param;
    int error;
    double speedRatio;
    int32 lowSpeed;
    lowSpeed = speedLow;

    if(speedHigh - speedLow >= 5)
    {
        error = 93 - averMidLine;
;
        speedDelta = (float)(speedHigh - speedLow);
        param = speedDelta / (25*25);

        present_speed = (uint8_t)(speedHigh - (int)(param * error * error));
        if(present_speed <= lowSpeed)
        {
            present_speed = lowSpeed;
        }
    }
    else
    {
        present_speed = lowSpeed;
    }


//    speedRatio = (double)(present_speed / lowSpeed) - 1;
//
//    if(speedRatio > 1e-5)
//    {
//        present_vision = present_vision - (uint8_t)(speedRatio * speedRatio * 100);
//    }
}

void CTRL_visionDecision()
{
    float averSpeed;
    float speed;
    float speedRatio;
    speed = (float)(presentSpeed.intVal);
    averSpeed = (float)(expectL + expectR) / 2;

    speedRatio = (averSpeed / speed) - 1;

    if(speedRatio < -1e-5)
    {
        present_vision = present_vision + (uint8_t)(speedRatio * speedRatio * 60);
    }

    else if(speedRatio > 1e-5)
    {
        present_vision = present_vision - (uint8_t)(speedRatio * speedRatio * 80);
    }

//    test_varible[6] = present_vision;
}

void CTRL_duzhuanTest()
{
    if(startCount < 1000)
    {
        startCount += 1;
    }

    else if(startCount >= 1000)
    {
        startFlag = 1;
    }


    if(delayFlag == 1)//延迟发车1s后变为1
    {
        if(parkStart == 0 && startFlag == 1)//转出车库
        {
            if(abs(speedL) < 30 || abs(speedR) < 30)
            {
                duzhuanCount += 1;
            }

            else if(abs(speedL) < 20 && abs(speedR) < 20 && mySpeedL > 5000 && mySpeedR > 5000)
            {
                duzhuanCount += 1;

            }
//            else
//            {
//                duzhuanCount = 0;
//            }

        }

    }

    if(duzhuanCount > 15)
    {
        duzhuanFlag = 1;
    }

    else
    {
        duzhuanFlag = 0;
    }

}


void CTRL_duzhuan()
{
    duzhuanTime += 1;

    expectR = -20;
    expectL = -20;


}

void CTRL_duzhuanZhuanWan()
{
    float fuzzyKP = 1;

    uint8_t myMidLine;
    myMidLine = aver_mid_line_foresee();

    if(myMidLine >= 83 && myMidLine <= 103)
    {
//        servoPwm = servoMidValue;
        duzhuanFlag = 0;
        duzhuanTime = 0;
        duzhuanCount = 0;
    }

    else if(myMidLine > 103) //偏差在右
    {
        servoPwm = servoMax;//轮子左转
    }

    else if(myMidLine < 103)
    {
        servoPwm = servoMin;
    }

}


void CTRL_islandPwmCount()
{
    if(state == stateIslandCircle)
    {
        if(islandWhere == LEFT)
        {
            if(servoPwm > islandPwmMax)
            {
                islandPwmMax = servoPwm;
            }

            if(servoPwm < islandPwmMin && servoPwm > servoMidValue)
            {
                islandPwmMin = servoPwm;
            }

            else if(servoPwm <= servoMidValue)
            {
                servoPwm = islandPwmMax;
            }
        }
        else if(islandWhere == RIGHT)
        {
            if(servoPwm > islandPwmMax && servoPwm < servoMidValue)
            {
                islandPwmMax = servoPwm;
            }

            if(servoPwm < islandPwmMin)
            {
                islandPwmMin = servoPwm;
            }

            else if(servoPwm >= servoMidValue)
            {
                servoPwm = islandPwmMin;
            }
        }

//        test_varible[14] = islandPwmMax;
//        test_varible[15] = islandPwmMin;

    }

    else if(state == stateIslandOut)
    {
        if(islandWhere == LEFT)
        {
            if(servoPwm > islandPwmMax)
            {
                servoPwm = islandPwmMax;
            }

            if(servoPwm < islandPwmMin)
            {
                servoPwm = islandPwmMin;
            }
        }
        else if(islandWhere == RIGHT)
        {
            if(servoPwm > islandPwmMax)
            {
                servoPwm = islandPwmMax;
            }

            if(servoPwm < islandPwmMin)
            {
                servoPwm = islandPwmMin;
            }
        }
    }

    else if(state == stateIslandTurn)
    {
        if(islandWhere == LEFT)
        {
            islandPwmMax = servoMidValue;
            islandPwmMin = servoMax;
        }
        else if(islandWhere == RIGHT)
        {
            islandPwmMax = servoMin;
            islandPwmMin = servoMidValue;
        }

//        test_varible[14] = islandPwmMax;
//        test_varible[15] = islandPwmMin;
    }
//    else
//    {
//        islandPwmMax = servoMidValue;
//        islandPwmMin = servoMax;
//    }
}

void CTRL_carParkPwmxianfu()
{
   if(state == stateParkIn)
   {
       if(servoPwm > servoParkMax)
       {
           servoPwm = servoParkMax;
       }

       else if(servoPwm < servoParkMin)
       {
           servoPwm = servoParkMin;
       }
   }

}


void CTRL_rampPwmxianfu()
{
   if(state == stateRampway)
   {
       if(servoPwm > servoRampMax)
       {
           servoPwm = servoRampMax;
       }

       else if(servoPwm < servoRampMin)
       {
           servoPwm = servoRampMin;
       }
   }

}

