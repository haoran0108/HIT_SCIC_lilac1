/*
 * SmartCar_ctrl.c
 *
 *  Created on: 2021年10月28日
 *      Author: windows
 */
#include "SmartCar_ctrl.h"
/*.c文件中只放自己的.h，其他函数中要用的变量要用全局定义，在.h中用extern+类型+变量名，需要用时包含.h*/



void CTRL_Init()
{
//    presentSpeedL.intValue = 1800;
//    presentSpeedR.intValue = 1800;

//    //Pit_Init_ms(CCU6_0, PIT_CH0, 5);//定时器中断，给电机使用（0模块0通道）
    SmartCar_Gtm_Pwm_Init(&IfxGtm_ATOM0_0_TOUT48_P22_1_OUT, 50, 785);//舵机
    /*B、D给pwm，左右轮正转，AB右轮，CD左轮*/
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

void CTRL_Main()//优化：封装pid计算，电机定时设置pwm，（多级环pid）
{
    CTRL_calculatePID();
    SmartCar_Gtm_Pwm_Setduty(&IfxGtm_ATOM0_0_TOUT48_P22_1_OUT, servoPwm);//舵机控制
}
