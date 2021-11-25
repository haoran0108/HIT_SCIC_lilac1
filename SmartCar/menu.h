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
}data;

struct node
{
    int intValue;
    float floatValue;
    char name[10];//�ļ���
    int pos;//�ļ���λ�ã���������Ļ�ĵڼ��У�
    data i;//�������ͱ�־
    struct node* prior; //ͬһ�ļ��µ���һ���ļ�
    struct node* next;//ͬһ���ļ��µ���һ���ļ�
    struct node* backward;//���ļ�
    struct node* forward;//���ļ�
};
typedef struct node node_t;
typedef struct node* nodeptr_t;

extern nodeptr_t tempFile;
extern node_t file1;
extern uint32 leftSpeed;
extern uint32 rightSpeed;
extern node_t presentSpeed, presentTHRE, presentVision;
extern node_t presentServoP, presentServoD;
extern node_t presentMotorP, presentMotorI;
extern node_t gap;
extern node_t presentTHRE;
extern node_t display1, display2, display3, display4, display5, display6;
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
node_t MENU_fileInit(node_t file, int intVal, float floatVal, char name[10], uint8 pos,data i,struct node* prior,struct node* next,struct node* backward,struct node* forward);

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
