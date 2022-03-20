/*
 * file_name.c
 *
 *  Created on: 2021��10��20��
 *      Author: ���Ȼ
 */

#include "menu.h"

node_t file1;
node_t file2, file3;
node_t CrossPD, CrossCircle, IslandPD, FolkPD;
node_t gear1, g1_Data1, g1_Data2, g1_Data3, g1_Data4, g1_Data5, g1_Data6, g1_Data7;
node_t gear2, g2_Data1, g2_Data2, g2_Data3, g2_Data4, g2_Data5, g2_Data6, g2_Data7;
node_t gear3, g3_Data1, g3_Data2, g3_Data3, g3_Data4, g3_Data5, g3_Data6, g3_Data7;
node_t currentK, currentRTKP, currentRTKI, currentLFKP, currentLFKI, expectC;
node_t Threshold, wayThre, OTSU1, OTSU_Klow, OTSU_Khigh, partOTSU, part_klow1, part_khigh1, part_klow2, part_khigh2;
node_t Cross_PB, Cross_PM, Cross_PS, Cross_ZO, Cross_NS, Cross_NM, Cross_NB, Cross_DS, Cross_DB;
node_t circle_PB, circle_PM, circle_PS, circle_ZO, circle_NS, circle_NM, circle_NB, circle_DS, circle_DB;
node_t Island_PB, Island_PM, Island_PS, Island_ZO, Island_NS, Island_NM, Island_NB, Island_DS, Island_DB;
node_t Folk_PB, Folk_PM, Folk_PS, Folk_ZO, Folk_NS, Folk_NM, Folk_NB, Folk_DS, Folk_DB;

node_t dataEnd;//���һ������ָ��dataEnd
node_t preGear1, preGear2, preGear3;
/*���ڵĵ�λ����,bottomDataΪ���һ�����ݣ���ʵ�����壬Ϊ�˵����ڶ��������ܸ���ֵ*/
node_t presentSpeed, presentTHRE, presentVision, fuzzyPB, fuzzyPM, fuzzyPS, fuzzyZO, fuzzyNS, fuzzyNM, fuzzyNB, presentServoD, presentMotorP, presentMotorI, bottomData;
node_t gap;//����Ҫ�˵ı���
node_t display, display1, display2, display3, display4, display5, display6, display7, display8, display9;//չʾ��ҳЧ��
node_t fileSave, saveGear1, saveGear2, saveGear3;//����д��flash
node_t image;//��ʾ����ͷͼ��
node_t list;
nodeptr_t filePrint;
nodeptr_t tempFile;

uint32 y = 1;
uint32 pageNum = 1;

node_t MENU_fileInit(node_t file, int intVal, float floatVal, char name[10],int pos,menuData i,
        struct menuNode* prior,struct menuNode* next,struct menuNode* backward,struct menuNode* forward)
{
    file.floatValue = floatVal;
    file.intValue = intVal;
    snprintf(file.name,12*sizeof(char), name);
    file.pos = pos;
    file.i = i;
    file.prior = prior;
    file.next = next;
    file.backward = backward;
    file.forward = forward;

    return file;
}

void MENU_Init()//��ȡ����ʱ���һ�����ݲ��ܲ����������
{
    //���ú���fileInit()��ʼ���ļ�����
    file1 = MENU_fileInit(file1, 1, 1.0, "GEAR", 2, none, NULL, &file2, NULL, &gear1);
    file2 = MENU_fileInit(file2, 1, 1.0, "PRESENT", 3, none, &file1, &file3, NULL, &presentSpeed);
    file3 = MENU_fileInit(file3, 1, 1.0, "SERVO-PD", 4, none, &file2, &display, NULL, &CrossPD);
    CrossPD = MENU_fileInit(CrossPD, 1, 0.5, "cross", 2, none, NULL, &CrossCircle, &file3, &Cross_PB);
    CrossCircle = MENU_fileInit(CrossCircle, 1, 0.5, "crs-circle", 3, none, &CrossPD, &IslandPD, NULL, &circle_PB);
    IslandPD = MENU_fileInit(IslandPD, 1, 0, "island", 4, none, &CrossCircle, &FolkPD, NULL, &Island_PB);
    FolkPD = MENU_fileInit(FolkPD, 1, 1.0, "folk", 5, none, &IslandPD, NULL, NULL, &Folk_PB);

    Cross_PB = MENU_fileInit(Cross_PB, 1, 0.5, "crossPB", 2, datafloat, NULL, &Cross_PM, &CrossPD, NULL);
    Cross_PM = MENU_fileInit(Cross_PM, 1, 0.5, "crossPM", 3, datafloat, &Cross_PB, &Cross_PS, NULL, NULL);
    Cross_PS = MENU_fileInit(Cross_PS, 1, 0.5, "crossPS", 4, datafloat, &Cross_PM, &Cross_ZO, NULL, NULL);
    Cross_ZO = MENU_fileInit(Cross_ZO, 1, 0.5, "crossZO", 5, datafloat, &Cross_PS, &Cross_NS, NULL, NULL);
    Cross_NS = MENU_fileInit(Cross_NS, 1, 0.5, "crossNS", 6, datafloat, &Cross_ZO, &Cross_NM, NULL, NULL);
    Cross_NM = MENU_fileInit(Cross_NM, 1, 0.5, "crossNM", 7, datafloat, &Cross_NS, &Cross_NB, NULL, NULL);
    Cross_NB = MENU_fileInit(Cross_NB, 1, 0.5, "crossNB", 2, datafloat, &Cross_NM, &Cross_DS, NULL, NULL);
    Cross_DS = MENU_fileInit(Cross_DS, 1, 0.5, "D-SMALL", 3, datafloat, &Cross_NB, &Cross_DB, NULL, NULL);
    Cross_DB = MENU_fileInit(Cross_DB, 1, 0.5, "D-BIG", 4, datafloat, &Cross_DS, NULL, NULL, NULL);

    circle_PB = MENU_fileInit(circle_PB, 1, 0.5, "circlePB", 2, datafloat, NULL, &circle_PM, &CrossCircle, NULL);
    circle_PM = MENU_fileInit(circle_PM, 1, 0.5, "circlePM", 3, datafloat, &circle_PB, &circle_PS, NULL, NULL);
    circle_PS = MENU_fileInit(circle_PS, 1, 0.5, "circlePS", 4, datafloat, &circle_PM, &circle_ZO, NULL, NULL);
    circle_ZO = MENU_fileInit(circle_ZO, 1, 0.5, "circleZO", 5, datafloat, &circle_PS, &circle_NS, NULL, NULL);
    circle_NS = MENU_fileInit(circle_NS, 1, 0.5, "circleNS", 6, datafloat, &circle_ZO, &circle_NM, NULL, NULL);
    circle_NM = MENU_fileInit(circle_NM, 1, 0.5, "circleNM", 7, datafloat, &circle_NS, &circle_NB, NULL, NULL);
    circle_NB = MENU_fileInit(circle_NB, 1, 0.5, "circleNB", 2, datafloat, &circle_NM, &circle_DS, NULL, NULL);
    circle_DS = MENU_fileInit(circle_DS, 1, 0.5, "D-SMALL", 3, datafloat, &circle_NB, &circle_DB, NULL, NULL);
    circle_DB = MENU_fileInit(circle_DB, 1, 0.5, "D-BIG", 4, datafloat, &circle_DS, NULL, NULL, NULL);

    Island_PB = MENU_fileInit(Island_PB, 1, 0.5, "IslandPB", 2, datafloat, NULL, &Island_PM, &IslandPD, NULL);
    Island_PM = MENU_fileInit(Island_PM, 1, 0.5, "IslandPM", 3, datafloat, &Island_PB, &Island_PS, NULL, NULL);
    Island_PS = MENU_fileInit(Island_PS, 1, 0.5, "IslandPS", 4, datafloat, &Island_PM, &Island_ZO, NULL, NULL);
    Island_ZO = MENU_fileInit(Island_ZO, 1, 0.5, "IslandZO", 5, datafloat, &Island_PS, &Island_NS, NULL, NULL);
    Island_NS = MENU_fileInit(Island_NS, 1, 0.5, "IslandNS", 6, datafloat, &Island_ZO, &Island_NM, NULL, NULL);
    Island_NM = MENU_fileInit(Island_NM, 1, 0.5, "IslandNM", 7, datafloat, &Island_NS, &Island_NB, NULL, NULL);
    Island_NB = MENU_fileInit(Island_NB, 1, 0.5, "IslandNB", 2, datafloat, &Island_NM, &Island_DS, NULL, NULL);
    Island_DS = MENU_fileInit(Island_DS, 1, 0.5, "D-SMALL", 3, datafloat, &Island_NB, &Island_DB, NULL, NULL);
    Island_DB = MENU_fileInit(Island_DB, 1, 0.5, "D-BIG", 4, datafloat, &Island_DS, NULL, NULL, NULL);

    Folk_PB = MENU_fileInit(Folk_PB, 1, 0.5, "FolkPB", 2, datafloat, NULL, &Folk_PM, &FolkPD, NULL);
    Folk_PM = MENU_fileInit(Folk_PM, 1, 0.5, "FolkPM", 3, datafloat, &Folk_PB, &Folk_PS, NULL, NULL);
    Folk_PS = MENU_fileInit(Folk_PS, 1, 0.5, "FolkPS", 4, datafloat, &Folk_PM, &Folk_ZO, NULL, NULL);
    Folk_ZO = MENU_fileInit(Folk_ZO, 1, 0.5, "FolkZO", 5, datafloat, &Folk_PS, &Folk_NS, NULL, NULL);
    Folk_NS = MENU_fileInit(Folk_NS, 1, 0.5, "FolkNS", 6, datafloat, &Folk_ZO, &Folk_NM, NULL, NULL);
    Folk_NM = MENU_fileInit(Folk_NM, 1, 0.5, "FolkNM", 7, datafloat, &Folk_NS, &Folk_NB, NULL, NULL);
    Folk_NB = MENU_fileInit(Folk_NB, 1, 0.5, "FolkNB", 2, datafloat, &Folk_NM, &Folk_DS, NULL, NULL);
    Folk_DS = MENU_fileInit(Folk_DS, 1, 0.5, "D-SMALL", 3, datafloat, &Folk_NB, &Folk_DB, NULL, NULL);
    Folk_DB = MENU_fileInit(Folk_DB, 1, 0.5, "D-BIG", 4, datafloat, &Folk_DS, NULL, NULL, NULL);






    gear1 = MENU_fileInit(gear1, 1, 1.0, "GearSlow", 2, none, NULL, &gear2, &file1, &g1_Data1);//���ٵ�
    g1_Data1 = MENU_fileInit(g1_Data1, 75, 32.3, "speed1", 2, dataint, NULL, &g1_Data2, &gear1, NULL);
    g1_Data2 = MENU_fileInit(g1_Data2, 140, 70.22, "THRE1", 3, dataint, &g1_Data1, &g1_Data3, NULL, NULL);
    g1_Data3 = MENU_fileInit(g1_Data3, 31, 42.09, "Vision1", 4, dataint, &g1_Data2, &g1_Data4, NULL, NULL);
    g1_Data4 = MENU_fileInit(g1_Data4, 1, 1.8, "servo-P", 5, datafloat, &g1_Data3, &g1_Data5, NULL, NULL);
    g1_Data5 = MENU_fileInit(g1_Data5, 1, 6.0, "servo-D", 6, datafloat, &g1_Data4, &g1_Data6, NULL, NULL);
    g1_Data6 = MENU_fileInit(g1_Data6, 1, 0.9, "GAP1", 7, datafloat, &g1_Data5, &g1_Data7, NULL, NULL);
    g1_Data7 = MENU_fileInit(g1_Data7, 1, 1.3, "motor-I", 2, datafloat, &g1_Data6, &preGear1, NULL, NULL);

    gear2 = MENU_fileInit(gear2, 1, 1.0, "GearFast", 3, none, &gear1, &gear3, NULL, &g2_Data1);//���ٵ�
    g2_Data1 = MENU_fileInit(g2_Data1, 85, 38.27, "speed2", 2, dataint, NULL, &g2_Data2, &gear2, NULL);
    g2_Data2 = MENU_fileInit(g2_Data2, 140, 75.32, "THRE2", 3, dataint, &g2_Data1, &g2_Data3, NULL, NULL);
    g2_Data3 = MENU_fileInit(g2_Data3, 30, 11.62, "Vision2", 4, dataint, &g2_Data2, &g2_Data4, NULL, NULL);
    g2_Data4 = MENU_fileInit(g2_Data4, 119, 1.9, "servo-P", 5, datafloat, &g2_Data3, &g2_Data5, NULL, NULL);
    g2_Data5 = MENU_fileInit(g2_Data5, 119, 6.0, "servo-D", 6, datafloat, &g2_Data4, &g2_Data6, NULL, NULL);
    g2_Data6 = MENU_fileInit(g2_Data6, 119, 0.9, "GAP2", 7, datafloat, &g2_Data5, &g2_Data7, NULL, NULL);
    g2_Data7 = MENU_fileInit(g2_Data7, 119, 1.1, "motor-I", 2, datafloat, &g2_Data6, &preGear2, NULL, NULL);

    gear3 = MENU_fileInit(gear3, 1, 1.0, "GearFast2", 4, none, &gear2, &currentK, NULL, &g3_Data1);//���ٵ�����g2�ĸ�����
    g3_Data1 = MENU_fileInit(g3_Data1, 90, 38.27, "speed3", 2, dataint, NULL, &g3_Data2, &gear3, NULL);
    g3_Data2 = MENU_fileInit(g3_Data2, 140, 75.32, "THRE3", 3, dataint, &g3_Data1, &g3_Data3, NULL, NULL);
    g3_Data3 = MENU_fileInit(g3_Data3, 29, 11.62, "Vision3", 4, dataint, &g3_Data2, &g3_Data4, NULL, NULL);
    g3_Data4 = MENU_fileInit(g3_Data4, 119, 1.9, "servo-P", 5, datafloat, &g3_Data3, &g3_Data5, NULL, NULL);
    g3_Data5 = MENU_fileInit(g3_Data5, 119, 6.8, "servo-D", 6, datafloat, &g3_Data4, &g3_Data6, NULL, NULL);
    g3_Data6 = MENU_fileInit(g3_Data6, 119, 0.9, "GAP3", 7, datafloat, &g3_Data5, &g3_Data7, NULL, NULL);
    g3_Data7 = MENU_fileInit(g3_Data7, 119, 1.1, "motor-I", 2, datafloat, &g3_Data6, &preGear3, NULL, NULL);

    currentK = MENU_fileInit(currentK, 1, 1.0, "currentK", 5, none, &gear3, &Threshold, NULL, &currentRTKP);
    currentRTKP = MENU_fileInit(currentRTKP, 3, 4.0, "curRTKP", 2, datafloat, NULL, &currentRTKI, &currentK, NULL);
    currentRTKI = MENU_fileInit(currentRTKI, 2, 2.0, "curRTKI", 3, datafloat, &currentRTKP, &currentLFKP, NULL, NULL);
    currentLFKP = MENU_fileInit(currentLFKP, 3, 4.0, "curLFKP", 4, datafloat, &currentRTKI, &currentLFKI, &currentK, NULL);
    currentLFKI = MENU_fileInit(currentLFKI, 2, 2.0, "curLFKI", 5, datafloat, &currentLFKP, &expectC, NULL, NULL);
//    currentKD = MENU_fileInit(currentKD, 2, 3.0, "motorKD", 4, datafloat, &currentRTKI, &expectC, NULL, NULL);
    expectC = MENU_fileInit(expectC, 2200, 9.5, "expect", 6, dataint, &currentLFKI, NULL, NULL, NULL);

    Threshold = MENU_fileInit(Threshold, 1, 1.0, "THRE", 6, none, &currentK, NULL, NULL, &wayThre);//��������
    wayThre = MENU_fileInit(wayThre, 1, 1.0, "THREway", 2, dataint, NULL, &OTSU1, &Threshold, NULL);
    OTSU1 = MENU_fileInit(OTSU1, 1, 1.0, "OTSU", 3, none, &wayThre, &partOTSU, NULL, &OTSU_Klow);
    OTSU_Klow = MENU_fileInit(OTSU_Klow, 1, 1.0, "K-low", 2, dataint, NULL, &OTSU_Khigh, &OTSU1, NULL);
    OTSU_Khigh = MENU_fileInit(OTSU_Khigh, 1, 1.0, "K-higt", 3, dataint, &OTSU_Klow, NULL, NULL, NULL);

    partOTSU = MENU_fileInit(partOTSU, 1, 1.0, "partOTSU", 4, none, &OTSU1, NULL, NULL, &part_klow1);
    part_klow1 = MENU_fileInit(part_klow1, 1, 1.0, "partlow1", 2, dataint, NULL, &part_khigh1, &partOTSU, NULL);
    part_khigh1 = MENU_fileInit(part_khigh1, 1, 1.0, "parthigh1", 3, dataint, &part_klow1, &part_klow2, NULL, NULL);
    part_klow2 = MENU_fileInit(part_klow2, 1, 1.0, "partlow2", 4, dataint, &part_khigh1, &part_khigh2, NULL, NULL);
    part_khigh2 = MENU_fileInit(part_khigh2, 1, 1.0, "parthigh2", 5, dataint, &part_khigh2, NULL, NULL, NULL);


    /* ���µĵ��pwmֵ��speedL/R��������ͷǰհvision */
    presentSpeed = MENU_fileInit(presentSpeed, 75, 1.1, "speed", 2, dataint, NULL, &presentTHRE, &file2, NULL);
    presentTHRE = MENU_fileInit(presentTHRE, 127, 2.2, "THRE", 3, dataint, &presentSpeed, &presentVision, NULL, NULL);
    presentVision = MENU_fileInit(presentVision, 71, 3.3, "VISION", 4, dataint, &presentTHRE, &fuzzyPB, NULL, NULL);
    fuzzyPB = MENU_fileInit(fuzzyPB, 1, 1.9, "fuzzyPB", 5, datafloat, &presentVision, &fuzzyPM, NULL, NULL);
    fuzzyPM = MENU_fileInit(fuzzyPB, 1, 1.8, "fuzzyPM", 6, datafloat, &fuzzyPB, &fuzzyPS, NULL, NULL);
    fuzzyPS = MENU_fileInit(fuzzyPB, 1, 1.6, "fuzzyPS", 7, datafloat, &fuzzyPM, &fuzzyZO, NULL, NULL);
    fuzzyZO = MENU_fileInit(fuzzyPB, 1, 1.5, "fuzzyZO", 2, datafloat, &fuzzyPS, &fuzzyNS, NULL, NULL);
    fuzzyNS = MENU_fileInit(fuzzyPB, 1, 1.6, "fuzzyNS", 3, datafloat, &fuzzyZO, &fuzzyNM, NULL, NULL);
    fuzzyNM = MENU_fileInit(fuzzyPB, 1, 1.8, "fuzzyNM", 4, datafloat, &fuzzyNS, &fuzzyNB, NULL, NULL);
    fuzzyNB = MENU_fileInit(fuzzyPB, 1, 1.9, "fuzzyNB", 5, datafloat, &fuzzyNM, &presentServoD, NULL, NULL);
    presentServoD = MENU_fileInit(presentServoD, 1, 3.7, "preServoD", 6, datafloat, &fuzzyNB, &gap, NULL, NULL);
    gap = MENU_fileInit(presentMotorP, 1, 0.9, "GAP", 7, datafloat, &presentServoD, &bottomData, NULL, NULL);
//    presentMotorI = MENU_fileInit(presentMotorI, 1, 190.0, "preMotorI", 2, datafloat, &gap, &bottomData, NULL, NULL);
//    presentMotorP = MENU_fileInit(gap, 8, 250, "preMotorP", 3, datafloat, &presentMotorI, &bottomData, NULL, NULL);
    bottomData = MENU_fileInit(bottomData, 1, 1.0, "bottom", 2, none, &gap, NULL, NULL, NULL);
    preGear1 = MENU_fileInit(preGear1, 1, 1.0, "Present1", 3, none, &g1_Data7, NULL, NULL, &presentSpeed);
    preGear2 = MENU_fileInit(preGear2, 1, 1.0, "Present2", 3, none, &g2_Data7, NULL, NULL, &presentSpeed);
    preGear3 = MENU_fileInit(preGear3, 1, 1.0, "Present3", 3, none, &g3_Data7, NULL, NULL, &presentSpeed);

    /*չʾ��ҳЧ��*/
    display = MENU_fileInit(display, 1, 1.0, "motor", 5, none, &file3, &fileSave, NULL, &display1);
    display1 = MENU_fileInit(display1, 50, 133.03, "LFKP", 2, dataint, NULL, &display2, &display, NULL);
    display2 = MENU_fileInit(display2, 40, 33.71, "LFKI", 3, dataint, &display1, &display3, NULL, NULL);
    display3 = MENU_fileInit(display3, 60, 2, "RTKP", 4, dataint, &display2, &display4, NULL, NULL);
    display4 = MENU_fileInit(display4, 50, 1, "RTKI", 5, dataint, &display3, &display5, NULL, NULL);
    display5 = MENU_fileInit(display5, 80, 2.701, "ParkDelay", 6, dataint, &display4, &display6, NULL, NULL);
    display6 = MENU_fileInit(display6, 2, 91.881, "round", 7, dataint, &display5, &display7, NULL, NULL);
    display7 = MENU_fileInit(display7, 100, 1.1, "SPEEDUP", 2, dataint, &display6, &display8, NULL, NULL);
    display8 = MENU_fileInit(display8, 55, 7.91, "count1", 3, dataint, &display7, &display9, NULL, NULL);
    display9 = MENU_fileInit(display9, 14, 9.02, "count2", 4, dataint, &display8, NULL, NULL, NULL);

    //����д��flash
    fileSave = MENU_fileInit(fileSave, 1, 1.0, "SAVE", 6, none, &display, NULL, NULL, &saveGear1);
    saveGear1 = MENU_fileInit(saveGear1, 1, 1.0, "SaveGear1", 2, none, NULL, &saveGear2, &fileSave, &g1_Data1);
    saveGear2 = MENU_fileInit(saveGear2, 1, 1.0, "SaveGear2", 3, none, &saveGear1, &saveGear3, NULL, &g2_Data1);
    saveGear3 = MENU_fileInit(saveGear3, 1, 1.0, "SaveGear3", 4, none, &saveGear2, NULL, NULL, &g3_Data1);

    //����ͷ��ʾ��oled��
//    image = MENU_fileInit(image, 1, 1.0, "IMAGE", 7, none, &fileSave, NULL, NULL, NULL);
   // list = MENU_fileInit(list, 1, 1.0, "list", 2, none, &image, NULL, NULL, NULL);
}


void MENU_namePrintf(nodeptr_t printFile)
{

    uint32 flag;
    uint32 count = 0;
    SmartCar_OLED_Fill(0);

    SmartCar_OLED_Printf6x8(0, 0, "%s", printFile->name);

    SmartCar_OLED_P6x8Str(0, printFile->pos, ">> ");
    SmartCar_OLED_P6x8Str(65, 0, "Page");
    SmartCar_OLED_Printf6x8(90, 0, "%d", pageNum);

    SmartCar_OLED_P6x8Str(0, 1, "======================");

    while(printFile->prior != NULL)
    {
        printFile = printFile->prior;
        //num--;
    }
    if(y > 6 && y < 13)
    {
        flag = 6;
        while((flag--) && (printFile->next != NULL))
        {
            printFile = printFile->next;
        }
    }

    do
    {

        SmartCar_OLED_Printf6x8(20, 2+count, "%s", printFile->name);

        printFile=printFile->next;
        count++;
        if(printFile->next==NULL)
        {
            SmartCar_OLED_Printf6x8(20, 2+count, "%s", printFile->name);

            count=0;
        }

    }while(printFile->next != NULL);

}


void MENU_valuePrintf(nodeptr_t temp)
{
    uint32 flag1;
    while(temp->prior != NULL)
    {
        temp = temp->prior;
    }
    if(y > 6 && y < 13)
    {
        flag1 = 6;
        while((flag1--) && (temp->next != NULL))
        {
            temp = temp->next;
        }
    }
    uint32 count1 = 0;
    do
    {
        if(temp->i == dataint)
        {
             SmartCar_OLED_Printf6x8(80, 2+count1, "%d", temp->intValue);
        }
        else if(temp->i == datafloat)
        {
            SmartCar_OLED_Printf6x8(80, 2+count1, "%0.3f", temp->floatValue);
        }

        temp = temp->next;
        count1++;
        if(temp->next == NULL)
        {
            if(temp->i == dataint)
            {
                SmartCar_OLED_Printf6x8(80, 2+count1, "%d", temp->intValue);
            }
            else if(temp->i == datafloat)
            {
                SmartCar_OLED_Printf6x8(80, 2+count1, "%0.3f", temp->floatValue);
            }
            count1 = 0;
        }

    }while(temp->next != NULL);
}



nodeptr_t MENU_curPosition(nodeptr_t temp)
{
    nodeptr_t printTemp;
    nodeptr_t dataRead;

    if(GPIO_Read(P13, 2))
    {
        SmartCar_OLED_Fill(0);
        MENU_showIMG();
    }
    else if(!GPIO_Read(P13, 2)){
    if (!GPIO_Read(P11, 2))//��
    {
        SmartCar_OLED_Fill(0);

        if(temp->prior != NULL)
        {
            y--;
            if(y < 1) {y = 1;}
            temp = temp->prior;
        }
        printTemp = temp;
        MENU_namePrintf(printTemp);
        MENU_valuePrintf(printTemp);
        SmartCar_OLED_P6x8Str(0, temp->pos, ">> ");

    }

    else if (!GPIO_Read(P11, 9))//��
    {
        SmartCar_OLED_Fill(0);

        if(temp->next != NULL)
        {
            y++;
            temp = temp->next;
        }
        printTemp = temp;
        MENU_namePrintf(printTemp);
        MENU_valuePrintf(printTemp);

        SmartCar_OLED_P6x8Str(0, temp->pos, ">> ");
    }


    else if (!GPIO_Read(P11, 10))//��
    {
        SmartCar_OLED_Fill(0);

        while(temp->prior != NULL)
        {
            temp = temp->prior;
        }
        if(temp->backward != NULL)
        {
            temp = temp->backward;
            pageNum--;
        }

        y = temp->pos;
        printTemp = temp;
        MENU_namePrintf(printTemp);
        //SmartCar_OLED_Printf6x8(20, 0, "%s", temp->name);
    }


    else if (!GPIO_Read(P11, 11))//��
    {


        if((strcmp(temp->name, "SaveGear1")) == 0)//��"SaveGear1"����������sector0��д������
        {
            MENU_sectorSave(0, temp->forward);
        }

        else if((strcmp(temp->name, "SaveGear2")) == 0)//��"SaveGear2"����������sector1��д������
        {
            MENU_sectorSave(1, temp->forward);
        }
        else if((strcmp(temp->name, "SaveGear3")) == 0)//��"SaveGear3"����������sector2��д������
        {
            MENU_sectorSave(2, temp->forward);
        }
//        else if((strcmp(temp->name, "IMAGE")) == 0)
//        {
//            SmartCar_OLED_Fill(0);
//            MENU_showIMG();
//        }
        else if(temp->forward != NULL)
        {
            y = 1;
            temp = temp->forward;
            pageNum++;
        }

        if((strcmp(temp->name, "IMAGE")) != 0)
        {
            printTemp = temp;
            MENU_namePrintf(printTemp);
            MENU_valuePrintf(printTemp);
        }

    }


    else if (!GPIO_Read(P11, 12))//ok
    {
        float times = 1;//�ı���ֵ�ı���

        float tempFloat;//��δ����ʱ�ĸ���ֵ
        int tempInt;//��δ���ĵ�����ֵ

        if(temp->i != none)
        {
            SmartCar_OLED_Fill(0);

            tempInt = temp->intValue;
            tempFloat = temp->floatValue;

            while(TRUE)
            {
                if (!GPIO_Read(P11, 2) || !GPIO_Read(P11, 9) || !GPIO_Read(P11, 10) ||
                                   !GPIO_Read(P11, 11) || !GPIO_Read(P11, 12))
                {
                    Delay_ms(STM0,100);

                    if(temp->i == dataint)/*�ı���������*/
                    {

                        if(!GPIO_Read(P11, 2))//ֵ���� ��
                        {
                            SmartCar_OLED_Fill(0);
                            temp->intValue += 1 * times;
                        }
                        else if(!GPIO_Read(P11, 9))//ֵ��С ��
                        {
                            SmartCar_OLED_Fill(0);
                            temp->intValue -= 1 * times;
                        }
                        else if(!GPIO_Read(P11, 10))//λ������ ��
                        {
                            SmartCar_OLED_Fill(0);
                            times = times * 10;
                            //intCursor -= 5;

                            SmartCar_OLED_Printf6x8(55, 3, "%2d", temp->intValue);
                            SmartCar_OLED_Printf6x8(55, 4, "%2d", tempInt);


                        }
                        else if (!GPIO_Read(P11, 11))//λ������ ��
                        {
                            SmartCar_OLED_Fill(0);
                            if(times != 1)
                            {
                                times = times / 10.0;
                            }

                            SmartCar_OLED_Printf6x8(55, 4, "%2d", tempInt);
                            SmartCar_OLED_Printf6x8(55, 3, "%2d", temp->intValue);

                        }
                        else if(!GPIO_Read(P11, 12))//ȷ�� ok���ص���һ��
                        {
                            SmartCar_OLED_Fill(0);
                            printTemp = temp;
                            MENU_namePrintf(printTemp);
                            MENU_valuePrintf(printTemp);
                            break;
                        }
                        SmartCar_OLED_P6x8Str(40, 5, "times:");
                        SmartCar_OLED_Printf6x8(80, 5, "%2.1f", times);
                        SmartCar_OLED_Printf6x8(55, 4, "%2d", tempInt);
                        SmartCar_OLED_Printf6x8(55, 3, "%2d", temp->intValue);
                    }

                    else if(temp->i == datafloat)
                    {

                        if(!GPIO_Read(P11, 2))//ֵ���� ��
                        {
                            SmartCar_OLED_Fill(0);
                            temp->floatValue += 0.1 * times;
                        }
                        else if(!GPIO_Read(P11, 9))//ֵ��С ��
                        {
                            SmartCar_OLED_Fill(0);
                            temp->floatValue -= 0.1 * times;
                        }
                        else if(!GPIO_Read(P11, 10))//λ������ ��
                        {
                            SmartCar_OLED_Fill(0);
                            times = times * 10;

                            SmartCar_OLED_Printf6x8(40, 4, "%0.3f", tempFloat);
                            SmartCar_OLED_Printf6x8(40, 3, "%0.3f", temp->floatValue);

                        }
                        else if (!GPIO_Read(P11, 11))//λ������ ��
                        {
                            SmartCar_OLED_Fill(0);
                            times = times / 10.0;

                            SmartCar_OLED_Printf6x8(40, 4, "%0.3f", tempFloat);
                            SmartCar_OLED_Printf6x8(40, 3, "%0.3f", temp->floatValue);

                        }
                        else if(!GPIO_Read(P11, 12))//ȷ�� ok���ص���һ��
                        {
                            SmartCar_OLED_Fill(0);
                            printTemp = temp;
                            MENU_namePrintf(printTemp);
                            MENU_valuePrintf(printTemp);
                            break;
                        }
                        SmartCar_OLED_P6x8Str(40, 5, "times:");
                        SmartCar_OLED_Printf6x8(80, 5, "%0.3f", times / 10.0);
                        SmartCar_OLED_Printf6x8(40, 4, "%3.3f", tempFloat);
                        SmartCar_OLED_Printf6x8(40, 3, "%3.3f", temp->floatValue);
                    }

                }

            }

        }
        else if(temp->i == none)
                {
                    int page = 0;
                    if((strcmp(temp->name, "GearSlow")) == 0)/*��ȡflash��gear1������*/
                    {
                        dataRead = temp;
                        dataRead = dataRead->forward;
                        do
                        {
                            if(dataRead->i == datafloat)
                            {
                                dataRead->floatValue = Page_Read(0, page, float);
                            }
                            else if(dataRead->i == dataint)
                            {
                                dataRead->intValue = Page_Read(0, page, uint32);
                            }

                            page ++;
                            dataRead = dataRead->next;
                        }while(dataRead->next != NULL);
                        //page = 0;
                    }
                    else if((strcmp(temp->name, "GearFast")) == 0)
                    {
                        page = 0;
                        dataRead = temp;
                        dataRead = dataRead->forward;
                        do
                        {
                            if(dataRead->i == datafloat)
                            {
                                dataRead->floatValue = Page_Read(1, page, float);
                            }
                            else if(dataRead->i == dataint)
                            {
                                dataRead->intValue = Page_Read(1, page, uint32);
                            }

                            page ++;
                            dataRead = dataRead->next;
                        }while(dataRead->next != NULL);
                        //page = 0;
                    }

                    else if((strcmp(temp->name, "GearFast2")) == 0)
                    {
                        page = 0;
                        dataRead = temp;
                        dataRead = dataRead->forward;
                        do
                        {
                            if(dataRead->i == datafloat)
                            {
                                dataRead->floatValue = Page_Read(2, page, float);
                            }
                            else if(dataRead->i == dataint)
                            {
                                dataRead->intValue = Page_Read(2, page, uint32);
                            }

                            page ++;
                            dataRead = dataRead->next;
                        }while(dataRead->next != NULL);
                        //page = 0;
                    }

                    else if((strcmp(temp->name, "Present1")) == 0)
                    {
                        dataRead = temp;
                        dataRead = dataRead->forward;
                        do
                        {
                            if(dataRead->i == datafloat)
                            {
                                dataRead->floatValue = Page_Read(0, page, float);
                            }
                            else if(dataRead->i == dataint)
                            {
                                dataRead->intValue = Page_Read(0, page, uint32);
                            }

                            page ++;
                            dataRead = dataRead->next;
                        }while(dataRead->next != NULL);
                       // page = 0;
                    }

                    else if((strcmp(temp->name, "Present2")) == 0)
                    {
                        dataRead = temp;
                        dataRead = dataRead->forward;
                        while(dataRead->next != NULL)
                        {
                            if(dataRead->i == datafloat)
                            {
                                dataRead->floatValue = Page_Read(1, page, float);
                            }
                            else if(dataRead->i == dataint)
                            {
                                dataRead->intValue = Page_Read(1, page, uint32);
                            }

                            page ++;
                            dataRead = dataRead->next;
                        }
                       // page = 0;
                    }

                    else if((strcmp(temp->name, "Present3")) == 0)
                    {
                        dataRead = temp;
                        dataRead = dataRead->forward;
                        while(dataRead->next != NULL)
                        {
                            if(dataRead->i == datafloat)
                            {
                                dataRead->floatValue = Page_Read(2, page, float);
                            }
                            else if(dataRead->i == dataint)
                            {
                                dataRead->intValue = Page_Read(2, page, uint32);
                            }

                            page ++;
                            dataRead = dataRead->next;
                        }
                        // page = 0;
                    }

                }

    }
    }
    return temp;
}


void MENU_sectorSave(uint32 sectorNum, nodeptr_t gearData)//������д��
{
    uint32_t buf[1024];

    uint32 count = 0;
    Sector_Erase(sectorNum);

    do
    {
        if(gearData->i == dataint)
        {
            memcpy(buf + count, &(gearData->intValue), sizeof(uint32_t));
        }
        else if(gearData->i == datafloat)
        {
            memcpy(buf + count, &(gearData->floatValue), sizeof(uint32_t));
        }

        if(gearData->i != none)
        {
            count ++;
        }

        gearData = gearData->next;
    }while(gearData->next != NULL);

    Sector_Program(sectorNum, buf);
}

void MENU_showIMG()
{

    while(TRUE)
    {

         if(fullBuffer != NULL)
         {

#define USEUART
#define DISBUF
#ifndef USEOLED
#ifndef DISIMG
        SmartCar_Show_IMG_WITHOUT_TH(&IMG[0][0],120,188);
#else
        SmartCar_Show_IMG(fullBuffer,120,188);
#endif
#endif
#ifdef USEUART
#ifdef DISIMG
        SmartCar_Show_IMG(fullBuffer,120,188);
#endif
#endif
        }
        if (!GPIO_Read(P11, 12) || !GPIO_Read(P11, 9) || !GPIO_Read(P11, 10) ||
                     !GPIO_Read(P11, 11) || !GPIO_Read(P11, 2) )
        {
            if(!GPIO_Read(P13, 2))
            {

                while(tempFile->prior != NULL)
                {
                    tempFile = tempFile->prior;
                    //num--;

                }
                break;
            }
        }
    }

}
