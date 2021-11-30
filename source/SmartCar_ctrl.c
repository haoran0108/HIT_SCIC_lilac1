/*
 * SmartCar_ctrl.c
 *
 *  Created on: 2021��10��28��
 *      Author: windows
 */
#include "SmartCar_ctrl.h"
/*.c�ļ���ֻ���Լ���.h������������Ҫ�õı���Ҫ��ȫ�ֶ��壬��.h����extern+����+����������Ҫ��ʱ����.h*/

int zebraFlag = 0, zebraCircle = 0;
float test_varible[20] = {1,2,3,4,5,6,7,8,9,10};//�������ݵ�   //����
int flagStop = 0, delayStop = 0;
error servoError = {1, 1, 1};
error errorML = {1, 1, 1}, errorMR = {1, 1, 1};
uint32 servoPwm;
uint32 motorPwm;

int32 expectL, expectR;//Ԥ���ٶ�
int32 speedL, speedR;//ʵ���ٶ�
int32 mySpeedL = 0, mySpeedR = 0;

float motorLFKP, motorLFKI, motorRTKP, motorRTKI;
void CTRL_Init()
{
//    presentSpeedL.intValue = 1800;
//    presentSpeedR.intValue = 1800;

    Pit_Init_ms(CCU6_0, PIT_CH0, 5);//��ʱ���жϣ������ʹ�ã�0ģ��0ͨ����
    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM0_0_TOUT48_P22_1_OUT, 50, 790);//���
    /*B��D��pwm����������ת��AB���֣�CD����*/
    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM0_2_TOUT33_P33_11_OUT, 20000, 3000);//B
    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM3_4_TOUT34_P33_12_OUT, 20000, 3000);//D
    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM1_1_TOUT31_P33_9_OUT, 20000, 0);//A
    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM0_5_TOUT40_P32_4_OUT, 20000, 0);//C

    SmartCar_Encoder_Init(GPT12_T5,IfxGpt120_T5INA_P21_7_IN,IfxGpt120_T5EUDA_P21_6_IN);
    SmartCar_Encoder_Init(GPT12_T6,IfxGpt120_T6INA_P20_3_IN,IfxGpt120_T6EUDA_P20_0_IN);

}


void CTRL_servoPID()
{

    servoError.currentError = 94 - mid_line[presentVision.intValue];
    servoError.delta = servoError.currentError - servoError.lastError;
    servoPwm = (uint32)(790 + presentServoD.floatValue * servoError.delta + presentServoP.floatValue * servoError.currentError);
    if(servoPwm > 865)
        servoPwm = 865;
    else if(servoPwm < 715)
        servoPwm = 715;
    servoError.lastError = servoError.currentError;
}

void CTRL_motorPID()//expectL��pwm��Ӧ��ϵ�� 70-2500  80-2800  90-3150
{

//    expectL = presentSpeed.intValue;
    speedL = CTRL_speedGetLeft();
    errorML.currentError = expectL - speedL;//ȡƫ��
    errorML.delta = errorML.currentError - errorML.lastError;
    mySpeedL = (int32)(mySpeedL + motorLFKI * errorML.currentError + motorLFKP * errorML.delta);
    errorML.lastError = errorML.currentError;//������һ�����


//    expectR = presentSpeed.intValue;
    speedR = CTRL_speedGetRight();
    errorMR.currentError = expectR + speedR;//ȡƫ��
    errorMR.delta = errorMR.currentError - errorMR.lastError;
    mySpeedR = (int32)(mySpeedR + motorRTKI * errorMR.currentError + motorRTKP * errorMR.delta);
    errorMR.lastError = errorMR.currentError;//������һ�����

}


void CTRL_servoMain()//�Ż�����װpid���㣬�����ʱ����pwm�����༶��pid��
{
    CTRL_servoPID();
    test_varible[4] = 790 - (float)(servoPwm);

    if(zebraCircle == display6.intValue || flagStop == 1)
    {
        delayStop ++;
        if(delayStop > display5.intValue)
        {
            expectL = 0;
            expectR = 0;

        }

    }
    else
    {
//        if(straightSpeedUp())
//        {
//            test_varible[5] = straightSpeedUp();
//            expectL = (int32)(presentSpeed.intValue * display7.floatValue);
//            expectR = (int32)(presentSpeed.intValue * display7.floatValue);
//        }
//        else
//        {
            CTRL_motorDiffer();
//        }

    }
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_0_TOUT48_P22_1_OUT, servoPwm);//�������

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
    test_varible[5] = mySpeedL;
    test_varible[6] = mySpeedR;
    if(mySpeedL >= 0 && mySpeedR >= 0)
    {
        pwmR = mySpeedR;
        pwmL = mySpeedL;
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_2_TOUT33_P33_11_OUT, pwmR);//B
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM3_4_TOUT34_P33_12_OUT, pwmL);//D
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM1_1_TOUT31_P33_9_OUT, 0);//A
        SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_5_TOUT40_P32_4_OUT, 0);//C
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


}


void CTRL_motorMain()
{
        CTRL_motorPID();
        CTRL_motor();

        test_varible[0] = -expectR;
        test_varible[1] = expectL;

        test_varible[2] = speedL;
        test_varible[3] = speedR;
}


void CTRL_motorDiffer()
{

    int delta;
    delta = 790 - servoPwm;

    /*���ּ���*/
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

    /*�ڼ����*/
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

int16_t CTRL_speedGetRight()//���ֱ�����1 ����20.3��20.0��ӦT6   ���ֱ�����2 ����21.6��21.7��ӦT5
{
    int16_t ctrl_speedR = 0;
    ctrl_speedR = SmartCar_Encoder_Get(GPT12_T5);
    SmartCar_Encoder_Clear(GPT12_T5);
    return ctrl_speedR;
}

int16_t CTRL_speedGetLeft()//���ֱ�����1 ����20.3��20.0��ӦT6   ���ֱ�����2 ����21.6��21.7��ӦT5
{
    int16_t ctrl_speedL = 0;
    ctrl_speedL = SmartCar_Encoder_Get(GPT12_T6);
    SmartCar_Encoder_Clear(GPT12_T6);
    return ctrl_speedL;
}

