/*
 * Image.h
 *
 *  Created on: 2021��10��28��
 *      Author: windows
 */

#ifndef SOURCE_IMAGE_H_
#define SOURCE_IMAGE_H_


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Image.h"
#include "SmartCar_MT9V034.h"

#define MISS 255
#define CAMERA_H  120                            //ͼƬ�߶�
#define CAMERA_W  188                            //ͼƬ���
#define FAR_LINE 1//ͼ�����ϱ߽�
#define NEAR_LINE 113//ͼ�����±߽�
#define LEFT_SIDE 0//ͼ������߽�
#define RIGHT_SIDE 187//ͼ�����ұ߽�
#define MISS 255
#define white_num_MAX 10//ÿ��������������

/////////////////////////////
#define black 0
#define white 1
#define blue  2
#define green 3
#define red   4
#define gray  5
#define purple 6
///////////////////////////

extern uint8_t IMG[CAMERA_H][CAMERA_W];//��ֵ����ͼ������
extern uint8_t image_Buffer_0[CAMERA_H][CAMERA_W];
extern uint8_t* fullBuffer;//ָ��Ҷ�ͼ���׵�ַ
extern uint8_t mid_line[CAMERA_H];

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

void my_memset(uint8_t* ptr, uint8_t num, uint8_t size);



#endif /* SOURCE_IMAGE_H_ */
