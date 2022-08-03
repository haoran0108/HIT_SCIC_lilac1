/*
 * Image.h
 *
 *  Created on: 2021年10月28日
 *      Author: windows
 */

#ifndef SOURCE_IMAGE_H_
#define SOURCE_IMAGE_H_


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "SmartCar_MT9V034.h"
#include "SmartCar_ctrl.h"
#include "menu.h"
 //#include<string>
 //#ifndef _IMAGE_H
 //#define _IMAGE_H
 //#include <stdio.h>
 //#include <stdlib.h>
 //#include <math.h>
 //#include "image.h"

#define MISS 255
//#define CAMERA_H  120                            //图片高度
//#define CAMERA_W  188                            //图片宽度
//#define FAR_LINE 5//图像处理上边界
//#define NEAR_LINE 80//图像处理下边界
//#define LEFT_SIDE 0//图像处理左边界
//#define RIGHT_SIDE 187//图像处理右边界
//#define MISS 255
//#define white_num_MAX 10//每行最多允许白条数
//
///////////////////////////////
//#define black 0
//#define white 1
//#define blue  2
//#define green 3
//#define red   4
//#define gray  5
//#define purple 6
/////////////////////////////
//
//extern uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组
//extern uint8_t image_Buffer_0[CAMERA_H][CAMERA_W];
//extern uint8_t* fullBuffer;//指向灰度图的首地址
//
//void head_clear(void);
//void THRE(void);
//int find_f(int a);
//void search_white_range();
//void find_all_connect();
//void find_road();
//uint8_t find_continue(uint8_t i_start, uint8_t j_start);
//void ordinary_two_line(void);
//void image_main();
//void get_mid_line(void);
//
//void my_memset(uint8_t* ptr, uint8_t num, uint8_t size);
//#endif //
#define CHANGED_H 120
#define CHANGED_W 188

//输出图片尺寸
#define OUT_H  120
#define OUT_W  188

#define uint8 int
#define uint16 int
#define uint32 int

#define MISS 255
#define CAMERA_H  120                            //图片高度
#define CAMERA_W  188                            //图片宽度
#define FAR_LINE 1//图像处理上边界
#define NEAR_LINE 110//图像处理下边界
#define LEFT_SIDE 0//图像处理左边界
#define RIGHT_SIDE 187//图像处理右边界
#define MISS 255
#define white_num_MAX 12//每行最多允许白条数

#define white_straight 37 //正视直道的格数
#define LEFT 1
#define RIGHT -1
#define UP 2
#define DOWN -2

#define MISSHAPPEN 296775

/////////////////////////////
#define black 0
#define white 1
#define blue  2
#define green 3
#define red   4
#define gray  5
#define purple 6
///////////////////////////

//////////////////////////////状态机的标志
#define stateStart 0
#define stateCrossIn 10//十字

#define stateFolkRoadIn 20//三叉

#define stateTIslandIn 30//环岛 回环
#define stateTIn 40
#define stateTOut 50
#define stateSTIsland 60//30 40 50 60回环

#define stateIslandIng 70//30 70 80 90 100 110环岛
#define stateIslandTurn 80
#define stateIslandCircle 90
#define stateIslandOut 100
#define stateIslandFinal 110

#define stateParkIn 120 //车库
#define stateRampway 130//坡道
/////////////////////////////
#define white_straight 43 //正视直道的格数
#define lim_white_straight 55 //斜直道最大像素点数



extern uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组
extern uint8_t IMG_distor[CAMERA_H][CAMERA_W];
extern uint8_t image_Buffer_0[CAMERA_H][CAMERA_W];
extern uint8_t* fullBuffer;//指向灰度图的首地址
extern uint8_t mid_line[CAMERA_H];
extern int state;
extern int parkPosition;
extern int stopFlag;
extern int carParkTimes;
extern int carParkDelay;
extern int straightFlag, lastStraightFlag;
extern int slowFlag;
extern int rampFlag1, rampFlag2,rampFlag3;
extern int rampWayCount;
extern int rampJudgeCount;

extern int leftPark,rightPark;
extern int parkJudgeCount;
extern int crossCircleCount;

extern int laststate;
extern uint8_t memoryFlag;
extern uint16_t memoryState[20];
extern uint8_t folkTimes;
extern uint8_t averMidLine;
extern uint8_t sRoadFlag;
extern uint8_t stopCount;

extern uint8_t thresholdUp;
extern uint8_t thresholdDown;
extern uint8_t islandTimes, IslandRadius;

extern uint8_t minThre, maxThre;
extern int islandWhere;
extern uint8_t islandCircleCount;
extern uint8_t tInCount;
extern int threOriginal;
extern int wayIT;
extern uint8_t tCrossTimes;
extern uint8_t tCrossStatus;
extern uint8_t rampTimes;
extern uint8_t parkSlowDownCount;
extern uint8_t car_stop;
extern int flagsee;
extern int TWhere;
extern uint8_t afterRampFlag;
extern uint8_t folkCNT;
extern uint8_t islandTimesCNT;
extern uint8_t folkOutTimes;
extern uint8_t speedUpPhase;
extern uint8_t islandFinalTimes;
extern int FolkRoadWhere;
extern uint8_t count_num_IT;
extern int lastState;
extern uint8_t longStrFlag, shortStrFlag;

typedef struct {
    int x;
    int y;
    int type;
    int direction;// 左,右，  上,下 = 10,01,  10,01
}jump_point;

//每个白条子属性
typedef struct {
    uint8_t left;//左边界
    uint8_t right;//右边界
    int connect_num;//连通标记
}range;

//每行的所有白条子
typedef struct {
    uint8_t num;//每行白条数量
    range area[white_num_MAX];//该行各白条区域
}all_range;

//属于赛道的每个白条子属性
typedef struct {
    uint8_t left;//左边界
    uint8_t right;//右边界
    uint8_t width;//宽度
}road_range;

//每行属于赛道的每个白条子
typedef struct {
    uint8_t white_num;
    road_range connected[white_num_MAX];
}road;

typedef struct {
    uint8_t origin_x;
    uint8_t origin_y;

}road_noise;

typedef struct {
    int x;
    int y;
}coordinate;

extern road my_road[CAMERA_H];//赛道


void head_clear(void);
void OTSU();
void part_OUST();
void iteration();
void THRE(int num);
int find_f(int a);
void search_white_range();
void find_all_connect();
void find_road();
uint8_t find_continue(uint8_t i_start, uint8_t j_start);
void ordinary_two_line(void);
void image_main();
void get_mid_line(void);
void IPM_map();
void distortion();
void my_memset(uint8_t* ptr, uint8_t num, uint8_t size);
void transform(int wayThre);
void transform_sd(int wayThre);
void orinary_two_line_history();
void adapt_threshold();
uint8_t mid_aver(int way);
void ZHAO_THRE();
void image_Preprocess(void);
int image_GetOtsuThre(void);
void image_GetHistGram(uint16_t* ptrHistGram, uint8_t startLine, uint8_t endLine);
void Entropy();
int My_Max(signed int i, signed int j);
int My_Min(signed int i, signed int j);

void roof();
int strenghen_contrast_ratio(int oldThre, int threA, int threNewA, int threB, int threNewB);
void XueSong();
int IMAGE_ThreConfig();
void Set_thre(int Foreshold, int Backshold, int ThreLine);
void protection();
//道路规划
void judge_type_road();

//数据处理函数
double calculate_slope(int start, int end, int side[CAMERA_H]);
double calculate_slope_uint(int start, int end, uint8_t side[CAMERA_H]);
double calculate_two_point_slope(int start, int xStart, int end, int xEnd);
double calculate_slope_struct(int start, int end, uint8_t j_mid[CAMERA_H], int type);
double variance(int yStart, int yEnd, int side[CAMERA_H]);
double correlation_coefficient(int start, int end, int side[CAMERA_W]);
double linear_judgement(int start, int end, uint8_t side[CAMERA_H]);
double linear_judgement_struct(int start, int end, uint8_t j[CAMERA_H], int direction);

int midMaxColumn(int istart, int iend, int param, int leftMid, int rightMid);
int straight_delta(int istart, int iend, int deltaThre);

//十字
void cross_in();
void design_cross_ing();
void cross_over();


//T与环岛共用函数
void T_island_in_start();
void design_T_island_in();
void T_or_island();
void straightT_or_island();
//环岛
void design_island_ing();
void island_turn();
void design_island_turn();
void island_circle();
void design_island_circle();
void island_out();
void design_island_out();
void island_straight();
void design_island_straight();
void island_final();
void island_radius();
int param_island(int state);
int memory_IT();
//三叉
void folk_road_in();
void design_folk_road();
void folk_road_out();

//T字口

void cross_T_in_over();
void design_cross_T_circle();
void cross_T_out_start();
void design_cross_T_out();
void cross_T_out_over();

//上下坡

//入库
void carpark_in();
void carpark_out();
void design_carpark();
void carPark_main();
void carpark_stop();
void design_carpark_turn();
int sign_carPark_in();


//停车

//上下坡
void rampwayOn();
void rampwayDown();
//滤波
void mid_line_filter();
void folkTimesCNT();
void straight_define();
uint8_t valid_row();



#endif //
