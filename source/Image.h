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
#include <math.h>
#include "SmartCar_ctrl.h"
#include "SmartCar_MT9V034.h"
#include "menu.h"
#define MISS 255
#define CAMERA_H  120                            //图片高度
#define CAMERA_W  188                            //图片宽度
#define FAR_LINE 1//图像处理上边界
#define NEAR_LINE 85//图像处理下边界
#define LEFT_SIDE 0//图像处理左边界
#define RIGHT_SIDE 187//图像处理右边界
#define MISS 255
#define white_num_MAX 10//每行最多允许白条数
#define MIDLINE_DELTA 8
#define SPEEDUP_COUNT1 55
#define SPEEDUP_COUNT2 14
/////////////////////////////
#define black 0
#define white 1
#define blue  2
#define green 3
#define red   4
#define gray  5
#define purple 6
///////////////////////////

extern uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组
extern uint8_t image_Buffer_0[CAMERA_H][CAMERA_W];
extern uint8_t* fullBuffer;//指向灰度图的首地址
extern uint8_t mid_line[CAMERA_H];
extern int zebraCount, zFlag;
extern int flag3;

void head_clear(void);
void THRE(void);
int find_f(int a);
void search_white_range();
void find_all_connect();
void find_road();
uint8_t find_continue(uint8_t i_start, uint8_t j_start);
void ordinary_two_line(void);
void image_main();
void get_mid_line(void);
void find_point_of_inflection(uint8_t j_continue[CAMERA_H]);
void stra_cross_road(uint8_t j_continue[CAMERA_H]);
void find_type_road(uint8_t j_continue[CAMERA_H]);
//float calculate_curvature(int x1, int x2, int x3, int y1, int y2, int y3);
float calculate_k(int x1, int x2, int y1, int y2);
void stra_cross_road2(uint8_t j_continue[CAMERA_H]);
void zebra_cross(uint8_t j_continue[CAMERA_H]);
void zebra_count();
void make_unmain_black(uint8_t j_continue[CAMERA_H]);
//void stra_cross_left_down(uint8_t j_continue[CAMERA_H]);
//void stra_cross_right_down(uint8_t j_continue[CAMERA_H]);
//void stra_cross_left_up(uint8_t j_continue[CAMERA_H]);
//void stra_cross_right_up(uint8_t j_continue[CAMERA_H]);
int zebraPanduan();
void my_memset(uint8_t* ptr, uint8_t num, uint8_t size);
int straightSpeedUp();
void stra_cross_left1(uint8_t j_continue[CAMERA_H]);
void stra_cross_left2(uint8_t j_continue[CAMERA_H]);
void stra_cross_left3(uint8_t j_continue[CAMERA_H]);
void stra_cross_right1(uint8_t j_continue[CAMERA_H]);
void stra_cross_right2(uint8_t j_continue[CAMERA_H]);
void stra_cross_right3(uint8_t j_continue[CAMERA_H]);
void stop();
void quasilinear(uint8_t x[], int n1, uint8_t y[], int n2, int n);
#endif //
