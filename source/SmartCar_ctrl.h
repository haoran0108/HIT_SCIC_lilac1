/*
 * SmartCar_ctrl.h
 *
 *  Created on: 2021年10月28日
 *      Author: windows
 */

#ifndef SOURCE_SMARTCAR_CTRL_H_
#define SOURCE_SMARTCAR_CTRL_H_

#include "common.h"
#include "SmartCar_PIT.h"
#include "SmartCar_Pwm.h"
#include "Image.h"
#include "menu.h"
#include "SmartCar_Encoder.h"
#include "switch.h"

#define PWM_MAX 9500
#define PWM_MAX_N -8500

extern uint32 servoPwm;
extern int32 mySpeedL, mySpeedR;
extern int32 speedL, speedR;
extern float test_varible[20];
extern float motorLFKP, motorLFKI, motorRTKP, motorRTKI;
extern int zebraFlag, zebraCircle;
extern int flagStop, delayStop;;
struct error
{
    int delta;
    int currentError;
    int lastError;
};
typedef struct error error;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      控制初始化
//  @param
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void CTRL_Init();

//-------------------------------------------------------------------------------------------------------------------
//  @brief      计算pid
//  @param
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void CTRL_servoPID();

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机控制
//  @param
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void CTRL_motor();

//-------------------------------------------------------------------------------------------------------------------
//  @brief      舵机控制
//  @param
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void CTRL_servoMain();

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机控制
//  @param
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void CTRL_motorMain();

//-------------------------------------------------------------------------------------------------------------------
//  @brief      转弯差速
//  @param
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void CTRL_motorDiffer();

//-------------------------------------------------------------------------------------------------------------------
//  @brief      编码器计数获取速度，分左右轮
//  @param
//  @return     int16_t
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
int16_t CTRL_speedGetLeft();
int16_t CTRL_speedGetRight();

#endif /* SOURCE_SMARTCAR_CTRL_H_ */
