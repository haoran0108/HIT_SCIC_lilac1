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
//#define CAMERA_H  120                            //ͼƬ�߶�
//#define CAMERA_W  188                            //ͼƬ���
//#define FAR_LINE 5//ͼ�����ϱ߽�
//#define NEAR_LINE 80//ͼ�����±߽�
//#define LEFT_SIDE 0//ͼ������߽�
//#define RIGHT_SIDE 187//ͼ�����ұ߽�
//#define MISS 255
//#define white_num_MAX 10//ÿ��������������
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
//extern uint8_t IMG[CAMERA_H][CAMERA_W];//��ֵ����ͼ������
//extern uint8_t image_Buffer_0[CAMERA_H][CAMERA_W];
//extern uint8_t* fullBuffer;//ָ��Ҷ�ͼ���׵�ַ
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

//���ͼƬ�ߴ�
#define OUT_H  120
#define OUT_W  188

#define uint8 int
#define uint16 int
#define uint32 int

#define MISS 255
#define CAMERA_H  120                            //ͼƬ�߶�
#define CAMERA_W  188                            //ͼƬ���
#define FAR_LINE 1//ͼ�����ϱ߽�
#define NEAR_LINE 110//ͼ�����±߽�
#define LEFT_SIDE 0//ͼ������߽�
#define RIGHT_SIDE 187//ͼ�����ұ߽�
#define MISS 255
#define white_num_MAX 12//ÿ��������������

#define white_straight 37 //����ֱ���ĸ���
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

//////////////////////////////״̬���ı�־
#define stateStart 0
#define stateCrossIn 10//ʮ��

#define stateFolkRoadIn 20//����

#define stateTIslandIn 30//���� �ػ�
#define stateTIn 40
#define stateTOut 50
#define stateSTIsland 60//30 40 50 60�ػ�

#define stateIslandIng 70//30 70 80 90 100 110����
#define stateIslandTurn 80
#define stateIslandCircle 90
#define stateIslandOut 100
#define stateIslandFinal 110

#define stateParkIn 120 //����
#define stateRampway 130//�µ�
/////////////////////////////
#define white_straight 43 //����ֱ���ĸ���
#define lim_white_straight 55 //бֱ��������ص���



extern uint8_t IMG[CAMERA_H][CAMERA_W];//��ֵ����ͼ������
extern uint8_t IMG_distor[CAMERA_H][CAMERA_W];
extern uint8_t image_Buffer_0[CAMERA_H][CAMERA_W];
extern uint8_t* fullBuffer;//ָ��Ҷ�ͼ���׵�ַ
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
    int direction;// ��,�ң�  ��,�� = 10,01,  10,01
}jump_point;

//ÿ������������
typedef struct {
    uint8_t left;//��߽�
    uint8_t right;//�ұ߽�
    int connect_num;//��ͨ���
}range;

//ÿ�е����а�����
typedef struct {
    uint8_t num;//ÿ�а�������
    range area[white_num_MAX];//���и���������
}all_range;

//����������ÿ������������
typedef struct {
    uint8_t left;//��߽�
    uint8_t right;//�ұ߽�
    uint8_t width;//���
}road_range;

//ÿ������������ÿ��������
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

extern road my_road[CAMERA_H];//����


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
//��·�滮
void judge_type_road();

//���ݴ�����
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

//ʮ��
void cross_in();
void design_cross_ing();
void cross_over();


//T�뻷�����ú���
void T_island_in_start();
void design_T_island_in();
void T_or_island();
void straightT_or_island();
//����
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
//����
void folk_road_in();
void design_folk_road();
void folk_road_out();

//T�ֿ�

void cross_T_in_over();
void design_cross_T_circle();
void cross_T_out_start();
void design_cross_T_out();
void cross_T_out_over();

//������

//���
void carpark_in();
void carpark_out();
void design_carpark();
void carPark_main();
void carpark_stop();
void design_carpark_turn();
int sign_carPark_in();


//ͣ��

//������
void rampwayOn();
void rampwayDown();
//�˲�
void mid_line_filter();
void folkTimesCNT();
void straight_define();
uint8_t valid_row();



#endif //
