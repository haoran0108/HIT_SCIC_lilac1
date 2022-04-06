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
#include "drv_imu_invensense_port.h"
#include "drv_imu_inv_icm20602.h"
#include "math.h"
#include "SmartCar_ADC.h"
#include"SmartCar_TFMINI.h"

#define PWM_MAX 9900
#define PWM_MAX_N -9900
#define pi 3.1415927

/*圆形前瞻*/
#define center_x 94
#define center_y 106

/*模糊PID*/
#define PB 30
#define PM 20
#define PS 10
#define ZO 0
#define NS -10
#define NM -20
#define NB -30

extern uint32 servoPwm;
extern int32 mySpeedL, mySpeedR;
extern int32 speedL, speedR;
extern float test_varible[20];
extern float motorLFKP, motorLFKI, motorRTKP, motorRTKI, motorRTKD;
extern float currentKP_R, currentKI_R, currentKI_L, currentKP_L;
extern int zebraFlag, zebraCircle;

extern float currentGyro;

extern int parkStart;
extern int parkType;

extern int flagCircleForsee;

extern int flagStop, delayStop;
extern int testFlag;
extern int speedFlag;

struct error
{
    double delta;
    double currentError;
    double lastError;
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
void CTRL_gyroInit();
void CTRL_gyroUpdate();
void CTRL_directionAngleGet();
void CTRL_directionAngleClean();
void CTRL_positionGet();
void CTRL_circleControl(int direction, float r);
void CTRL_waihuan();
void CTRL_gyroCircle();
void CTRL_ParkStartServo(float gyro);

/*电流环*/
void CTRL_curLoopPID();
void CTRL_speedLoopPID();
//-------------------------------------------------------------------------------------------------------------------
//  @brief      计算pid
//  @param
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void CTRL_servoPID();

/*模糊PID*/
float CTRL_FuzzyMemberShip(int midError);
void CTRL_fuzzyPID();
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

/*出入车库*/
void CTRL_CarParkStart();
void CTRL_CarParkStop();

/*圆形前瞻*/
void CTRL_CircleForsee(int radius);
void CTRL_CircleServoPID();


//-------------------------------------------------------------------------------------------------------------------
//  @brief      编码器计数获取速度，分左右轮
//  @param
//  @return     int16_t
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
int16_t CTRL_speedGetLeft();
int16_t CTRL_speedGetRight();

void speedUP_define();

#endif /* SOURCE_SMARTCAR_CTRL_H_ */
