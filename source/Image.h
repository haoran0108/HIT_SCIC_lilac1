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
#define NEAR_LINE 106//图像处理下边界
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

//////////////////////////////
#define stateStart 0      //初始状态或者直线

#define stateInCrossStraight 1    //直入十字
#define stateOutCrossStraight 2   //直出十字

#define stateIslandTurn 3
#define stateIsland 4       //进入环岛
#define stateIsland1 5
#define stateIsland2 6
#define stateIsland3 7
#define stateIsland4 8
#define stateIsland5 9
#define stateIsland6 10

#define stateFolkRoadIn 11

#define stateCrossCircleIn 12
#define stateCrossCircleIng 13

#define stateCarPark 14

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


void circle_foresee();
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
void sharpen();
void IPM_map();
void distortion();
//void show_dis(cv::Mat image_undisor);
void my_memset(uint8_t* ptr, uint8_t num, uint8_t size);

void transform();
void orinary_two_line_history();
//道路规划
void find_part_jumppoint(int yStart, int yEnd, int type);
double calculate_slope(int start, int end, int type);
double calculate_slope_two_point(int yStart, int startType, int yEnd, int endType);
void find_jump_point();
void judge_type_road();
int circle_360();
double cos_angle();

//十字
void cross_in();
void cross_out();
void cross_over();
void design_cross_in();
void design_cross_out();
void design_straight_route(int y_start, int y_end, int type);
double variance(int y_start, int y_end, int type);
void folk_or_cross();


//环岛
void decide_miss();
void island_start(int type);
void design_island_start(int type);
void design_island_in(int type);
void island_ing(int type);
void design_island_ing(int type);
void islandOrcross_circle(int type);
void design_island_turn(int type);
void island_turn(int type);
void design_island_out(int type);
//void island_out_turn(int type);
//void design_island_out_turn(int type);
void island_out_straight(int type);
void design_island_final(int type);
void island_straight(int type);
//int check_state();
void folk_or_cross();

//三叉
void folk_road_in();
void design_folkroad_in();
void folk_road_out();
double cos_angle(int x1,int x2,int x3,int y1,int y2,int y3);
//十字回环
//void cross_circle_in();
//void design_cross_circle_in();
void cross_circle_out();
void cross_circle_out_patch();
void design_cross_circle_out();
//上下坡

//入库
void carPark_in();
void searchParkLine();
void design_park();

//停车
void protection();
//上下坡

#endif //
