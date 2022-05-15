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
#define NEAR_LINE 113//图像处理下边界
#define LEFT_SIDE 0//图像处理左边界
#define RIGHT_SIDE 187//图像处理右边界
#define MISS 255
#define white_num_MAX 10//每行最多允许白条数

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
#define stateCrossIn 10
#define stateFolkRoadIn 20
#define stateTIslandIn 30
#define stateTIn 40
#define stateTOut 50
#define stateTover 60
#define stateIslandIng 70
#define stateIslandTurn 80
#define stateIslandCircle 90
#define stateIslandOut 100
#define stateIslandFinal 110
#define stateParkIn 120
#define stateRampway 130
/////////////////////////////

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
extern int startCount;
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
void transform();
void orinary_two_line_history();

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

//十字
void cross_in();
void design_cross_ing();
void cross_over();


//T与环岛共用函数
void T_island_in_start();
void design_T_island_in();
void T_or_island();
//环岛
void design_island_ing();
void island_turn();
void design_island_turn();
void island_circle();
void island_out();
void design_island_out();
void island_straight();
void design_island_straight();
void island_final();

//三叉
void folk_road_in();
void design_folk_road();
void folk_road_out();

//T字口

void cross_T_in_over();
void cross_T_out_start();
void design_cross_T_out();
void cross_T_out_over();

//上下坡

//入库
void carpark_in();
void carpark_out();
void design_carpark();
void searchParkLine();

//停车

//上下坡
void rampwayOn();
void rampwayDown();

//直道
int straight_variance(int istart, int iend,float varThreshold);
int midMaxColumn(int istart, int iend, int param);
void straight_define();


//废案
void cross_T_in_start();
void design_cross_T_in();
void island_start();
void design_island_start();

#endif //
