/*
 * file_name.h
 *
 *  Created on: 2021��10��16��
 *      Author: windows
 */
#ifndef SMARTCAR_MENU_H_
#define SMARTCAR_MENU_H_

#include "SmartCar_Oled.h"
#include "SmartCar_Systick.h"
#include "SmartCar_PIT.h"
#include "SmartCar_Flash.h"
#include "common.h"
#include "image.h"
#include "menu.h"


typedef enum data//�о�
{
    none,
    dataint,
    datafloat
}menuData;


struct menuNode
{
//    int intValue;
//    float floatValue;
    union
    {
            int16 intVal;
            float floatVal;
    };
    char name[10];//�ļ���
    uint8_t pos;//�ļ���λ�ã���������Ļ�ĵڼ��У�
    menuData i;//�������ͱ�־
    struct menuNode* prior; //ͬһ�ļ��µ���һ���ļ�
    struct menuNode* next;//ͬһ���ļ��µ���һ���ļ�
    struct menuNode* backward;//���ļ�
    struct menuNode* forward;//���ļ�
};
typedef struct menuNode node_t;
typedef struct menuNode* nodeptr_t;

extern float voltage;
extern nodeptr_t tempFile;
extern node_t file1, file3;
extern uint32 leftSpeed;
extern uint32 rightSpeed;
extern node_t presentSpeed, presentTHRE, presentVision;
extern node_t fuzzyPB, fuzzyPM, fuzzyPS, fuzzyZO, fuzzyNS, fuzzyNM, fuzzyNB, presentServoD;
extern node_t presentMotorP, presentMotorI;
extern node_t gap;
extern node_t midLineKP, gyroKP, gyroKD;
extern node_t presentTHRE;
extern node_t display1, display2, display3, display4, display5, display6, display7, display8, display9, display10;
//extern node_t CrossPD, CrossCircle, IslandPD;
extern node_t LFKP, LFKI, RTKP, RTKI, slowLFKP, slowLFKI, slowRTKP, slowRTKI, fastLFKP, fastLFKI, fastRTKP, fastRTKI;

extern node_t currentRTKP, currentRTKI, currentLFKP, currentLFKI, expectC;

extern node_t CrossCircle, IslandPD;
extern node_t Cross_PB, Cross_PM, Cross_PS, Cross_ZO, Cross_NS, Cross_NM, Cross_NB, Cross_DS, Cross_DB;
extern node_t circle_PB, circle_PM, circle_PS, circle_ZO, circle_NS, circle_NM, circle_NB, circle_DS, circle_DB;
extern node_t Island_PB, Island_PM, Island_PS, Island_ZO, Island_NS, Island_NM, Island_NB, Island_DS, Island_DB;
extern node_t Folk_PB, Folk_PM, Folk_PS, Folk_ZO, Folk_NS, Folk_NM, Folk_NB, Folk_DS, Folk_DB;


extern node_t islandout_up, design_island_k, islandParam1, islandParam2, islandParam3, islandParam4, islandParam5;
extern node_t cross_circle_param1, cross_circle_param2, cross_circle_param3, cross_circle_param4, cross_circle_param5, cross_circle_param6;
extern node_t parkCount, startGyro, endGyro, search_line, parkDelay;
extern node_t rampCount, rampDistance, rampSpeed,rampMax, rampMin;
extern node_t folkWay;
extern node_t folkParam1, folkParam2, folkParam3, folkParam4, folkParam5, folkParam6, folkParam7, folkParam8, folkParam9, folkParam10, folkBottom;
extern node_t speedFilter, currentFilter1, currentFilter2, currentKdLpf, speedKdLpf;

extern node_t startWay;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ļ����������г�ʼ����ֵ
//  @param      file Ҫ��ֵ���ļ�  ���ͣ�node_t
//  @param      intVal ���ݵ�����ֵ
//  @param      floatVal ���ݵĸ�����ֵ
//  @param      name    �ļ�����
//  @param      pos �ļ����ֵ����� ��Χ2--                      7
//  @param      i �ж���������   none������������    dataint:����Ϊ����ֵ    datafloat:����Ϊ����ֵ
//  @param      prior ָ���ļ�����һ���ļ�
//  @param      next  ָ���ļ�����һ���ļ�
//  @param      backward  ָ���ļ��ĸ��ļ�
//  @param      forward   ָ���ļ������ļ�
//  @return     node_t  ��ֵ�õ��ļ�
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
node_t MENU_fileInit(node_t file, int16 valuei, float valuef, char name[10], uint8_t pos,menuData i,struct menuNode* prior,struct menuNode* next,struct menuNode* backward,struct menuNode* forward);

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ʵ�ʴ����ļ�
//  @param      ��
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void MENU_Init();

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ӡͬһ���ļ�������
//  @param      allFile   ����nodeptr_t ��ǰ���ļ���ָ�����ͣ�
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void MENU_namePrintf(nodeptr_t allFile);

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ�������״̬��������Ӧ�Ĳ���
//  @param      temp      ��ǰ���ļ���ָ�룩
//  @return     ����nodeptr_t  ���ز������temp
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
nodeptr_t MENU_curPosition(nodeptr_t temp);

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ӡ���ݵ�ֵ
//  @param      temp     �ļ���ָ�룩
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void MENU_valuePrintf(nodeptr_t temp);

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ������д��
//  @param      sectorNum ������     gearData Ҫд��ĵ�һ������
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void MENU_sectorSave(uint32 sectorNum, nodeptr_t gearData);

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʾ����ͷͼ��
//  @param
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void MENU_showIMG();

#endif
