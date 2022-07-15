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

/*速度环模糊pid*/
#define speedPB 30
#define speedPM 20
#define speedPS 10
#define speedZO 0
#define speedNM 10
#define speedNB 20

/*舵机*/
#define servoMidValue 721
#define servoMin 642
#define servoMax 805

#define servoParkMax 761
#define servoParkMin 681

#define servoRampMax 741
#define servoRampMin 701

#define parkStopStraightTime 2


extern uint32 servoPwm;
extern int32 mySpeedL, mySpeedR;
extern int32 speedL, speedR;
extern float test_varible[20];
extern float motorLFKP, motorLFKI, motorRTKP, motorRTKI, motorRTKD;
extern float currentKP_R, currentKI_R, currentKI_L, currentKP_L;
extern int zebraFlag, zebraCircle;

extern float currentGyro;
extern float lastRampGyro, rampGyro, rampGyroMax;

extern int parkStart;
extern int parkType;

extern int flagCircleForsee;

extern int flagStop, delayStop;
extern uint8_t testFlag;
extern uint8_t testStateTimes;
extern int speedFlag;
extern uint8_t parkStraightCount;
extern uint32 pwmFix;
extern uint8_t present_speed, present_vision;
extern uint8_t duzhuanCount, duzhuanFlag, duzhuanTime;
extern uint32_t startCount;
extern uint8_t startFlag;
extern uint8_t flagStop1, flagStopCount1;
extern uint8_t lastMyMidLine;
struct error
{
    double delta;
    double currentError;
    double lastError;
};
typedef struct error error;
extern error servoError;

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
void CTRL_rampGyroUpdate();
void CTRL_directionAngleGet();


/*电流环*/
void CTRL_curLoopPID();
void CTRL_speedLoopPID();
uint8_t CTRL_fuzzySpeedKp(int speedError);

//-------------------------------------------------------------------------------------------------------------------
//  @brief      计算pid
//  @param
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void CTRL_servoPID();
void CTRL_midLineLoopPID();
void CTRL_gyroLoopPID();

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
int foresee();
void CTRL_lowpassFilter();//低通滤波
void CTRL_currentAverageFilter();//电流环平均滑动滤波
void CTRL_gyroAverageFilter();
void CTRL_RoadTest();
void speedDetermine();
void CTRL_speedDecision(int32 speedHigh, int32 speedLow);
void CTRL_visionDecision();

void motorParamDefine();
void CTRL_ServoPID_Determine();
void CTRL_islandPwmCount();

/*撞车倒退*/
void CTRL_duzhuan();
void CTRL_duzhuanTest();
void CTRL_duzhuanZhuanWan();//堵转转弯

/*元素打角限幅*/
void CTRL_carParkPwmxianfu();
void CTRL_rampPwmxianfu();


#endif /* SOURCE_SMARTCAR_CTRL_H_ */
