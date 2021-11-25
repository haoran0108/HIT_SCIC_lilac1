/*
 * SmartCar_ctrl.c
 *
 *  Created on: 2021��10��28��
 *      Author: windows
 */
#include "SmartCar_ctrl.h"
/*.c�ļ���ֻ���Լ���.h������������Ҫ�õı���Ҫ��ȫ�ֶ��壬��.h����extern+����+����������Ҫ��ʱ����.h*/



void CTRL_Init()
{
//    presentSpeedL.intValue = 1800;
//    presentSpeedR.intValue = 1800;

//    //Pit_Init_ms(CCU6_0, PIT_CH0, 5);//��ʱ���жϣ������ʹ�ã�0ģ��0ͨ����
    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM0_0_TOUT48_P22_1_OUT, 50, 785);//���
    /*B��D��pwm����������ת��AB���֣�CD����*/
    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM0_2_TOUT33_P33_11_OUT, 20000,  presentSpeedR.intValue);//B
    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM3_4_TOUT34_P33_12_OUT, 20000, presentSpeedL.intValue);//D
    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM1_1_TOUT31_P33_9_OUT, 20000, 0);//A
    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM0_5_TOUT40_P32_4_OUT, 20000, 0);//C

}

float kp = 1.8;
float kd = 2;
error servoError = {1, 1, 1};
uint32 servoPwm;

void CTRL_calculatePID()
{

    servoError.currentError = 94 - mid_line[presentVision.intValue];
    servoPwm = (uint32)(785 + kp * servoError.currentError);
    if(servoPwm > 865)
        servoPwm = 865;
    else if(servoPwm < 705)
        servoPwm = 705;

    //    servoError.delta = servoError.currentError - servoError.lastError;
//    servoPwm = (uint32)(785 + kd * servoError.delta + kp * servoError.currentError);
//    if(servoPwm > 865)
//        servoPwm = 865;
//    else if(servoPwm < 705)
//        servoPwm = 705;
//    servoError.lastError = servoError.currentError;
}

void CTRL_motor()
{
    SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_2_TOUT33_P33_11_OUT, presentSpeedR.intValue);//B
    SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM3_4_TOUT34_P33_12_OUT, presentSpeedR.intValue);//D
    SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM1_1_TOUT31_P33_9_OUT, 0);//A
    SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_5_TOUT40_P32_4_OUT, 0);//C
}

void CTRL_Main()//�Ż�����װpid���㣬�����ʱ����pwm�����༶��pid��
{
    CTRL_calculatePID();
    SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_0_TOUT48_P22_1_OUT, servoPwm);//�������
}
