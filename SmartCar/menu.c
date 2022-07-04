

/*
 * file_name.c
 *
 *  Created on: 2021年10月20日
 *      Author: 李灏然
 */

#include "menu.h"

node_t file1;
node_t file2, file3;
node_t PD, CrossPD, CrossCircle, IslandPD, FolkPD, straightPD;
node_t gear1, g1_Data1, g1_Data2, g1_Data3, g1_Data4, g1_Data5, g1_Data6, g1_Data7, g1_Data8, g1_Data9, g1_Data10, g1_Data11, g1_Data12;
node_t gear2, g2_Data1, g2_Data2, g2_Data3, g2_Data4, g2_Data5, g2_Data6, g2_Data7;
node_t gear3, g3_Data1, g3_Data2, g3_Data3, g3_Data4, g3_Data5, g3_Data6, g3_Data7;
node_t currentK, currentRTKP, currentRTKI, currentLFKP, currentLFKI, expectC;
node_t Threshold, wayThre, OTSU1, OTSU_Klow, OTSU_Khigh, partOTSU, part_klow1, part_khigh1, part_klow2, part_khigh2;
node_t expTime;
node_t Cross_PB, Cross_PM, Cross_PS, Cross_ZO, Cross_NS, Cross_NM, Cross_NB, Cross_DS, Cross_DB;
node_t circle_PB, circle_PM, circle_PS, circle_ZO, circle_NS, circle_NM, circle_NB, circle_DS, circle_DB;
node_t Island_PB, Island_PM, Island_PS, Island_ZO, Island_NS, Island_NM, Island_NB, Island_DS, Island_DB;
node_t Folk_PB, Folk_PM, Folk_PS, Folk_ZO, Folk_NS, Folk_NM, Folk_NB, Folk_DS, Folk_DB;
node_t straight_KP, straight_KD;

node_t dataEnd;//最后一个数据指向dataEnd
node_t preGear1, preGear2, preGear3;
/*现在的档位参数,bottomData为最后一个数据，无实际意义，为了倒数第二个数据能附上值*/
node_t presentSpeed, presentTHRE, presentVision, fuzzyPB, fuzzyPM, fuzzyPS, fuzzyZO, fuzzyNS, fuzzyNM, fuzzyNB, presentServoD, presentMotorP, presentMotorI, bottomData;
node_t gap;//差速要乘的倍数
//node_t midLineKP, gyroKP, gyroKD;
node_t speedLow;
node_t display, display1, display2, display3, display4, display5, display6, display7, display8, display9, display10;//展示翻页效果
node_t LFKP, LFKI, RTKP, RTKI, slowLFKP, slowLFKI, slowRTKP, slowRTKI, fastLFKP, fastLFKI, fastRTKP, fastRTKI;
node_t fileSave, saveGear1, saveGear2, saveGear3;//用于写入flash
node_t image;//显示摄像头图像
node_t list;

node_t param;
node_t island,cross_circle, carPark, ramp, folkWay;
node_t islandout_up, design_island_k, islandParam1, islandParam2, islandParam3, islandParam4, islandParam5, islandParam6, islandBottom;
node_t cross_circle_param1, cross_circle_param2, cross_circle_param3, cross_circle_param4, cross_circle_param5, cross_circle_param6, cross_circle_param7, cross_circle_param8, ccBottom;
node_t parkCount, startGyro, endGyro, search_line, parkDelay;
node_t paramBottom;

node_t raceMemory, memory1, memory2, memory3, memory4, memory5, memory6, memory7, memory8, memory9, memory10, memory11, memory12, memory13, memory14, memory15, memoryBottom;
node_t memorySpeed1, memoryVision1;
node_t memorySpeed2, memoryVision2;
node_t memorySpeed3, memoryVision3;
node_t memorySpeed4, memoryVision4;
node_t memorySpeed5, memoryVision5;
node_t memorySpeed6, memoryVision6;
node_t memorySpeed7, memoryVision7;
node_t memorySpeed8, memoryVision8;
node_t memorySpeed9, memoryVision9;
node_t memorySpeed10, memoryVision10;
node_t memorySpeed11, memoryVision11;
node_t memorySpeed12, memoryVision12;
node_t memorySpeed13, memoryVision13;
node_t memorySpeed14, memoryVision14;
node_t memorySpeed15, memoryVision15;

node_t rampCount, rampDistance, rampSpeed, rampMax, rampMin, rampBottom;
node_t folkParam1, folkParam2, folkParam3, folkParam4, folkParam5, folkParam6, folkParam7, folkParam8, folkParam9, folkParam10, folkBottom;
node_t startWay;

node_t speedFilter, currentFilter1, currentFilter2, currentKdLpf, speedKdLpf, filterBottom;
node_t bugFix1;
//node_t roadTest;
nodeptr_t filePrint;
nodeptr_t tempFile;

uint32 y = 1;
uint32 pageNum = 1;
float voltage;

node_t MENU_fileInit(node_t file, int16 valuei, float valuef, char name[10],uint8_t pos,menuData type,
        struct menuNode* prior,struct menuNode* next,struct menuNode* backward,struct menuNode* forward)
{
//    file.floatValue = floatVal;
//    file.intValue = intVal;
    file.i = type;
    if(file.i == dataint)
    {
        file.intVal = valuei;
    }
    else if(file.i == datafloat)
    {
        file.floatVal = valuef;
    }
//    else if()
    snprintf(file.name,12*sizeof(char), name);
    file.pos = pos;

    file.prior = prior;
    file.next = next;
    file.backward = backward;
    file.forward = forward;

    return file;
}

void MENU_Init()//存取数据时最后一个数据不能操作，待解决
{
    //利用函数fileInit()初始化文件参数
    file1 = MENU_fileInit(file1, 0, 1.0, "GEAR", 2, dataint, NULL, &file2, NULL, &Threshold);
    file2 = MENU_fileInit(file2, 1, 1.0, "PRESENT", 3, none, &file1, &file3, NULL, &presentSpeed);
    file3 = MENU_fileInit(file3, 1, 1.0, "param", 4, none, &file2, &display, NULL, &PD);
    PD = MENU_fileInit(PD, 1, 0, "PD", 2, none, NULL, &param, &file3, &IslandPD);

    IslandPD = MENU_fileInit(IslandPD, 1, 0, "islandPD", 2, dataint, NULL, &CrossCircle, &PD, &Island_PB);

//    CrossPD = MENU_fileInit(CrossPD, 1, 0.5, "cross", 2, none, NULL, &CrossCircle, &file3, &Cross_PB);
    CrossCircle = MENU_fileInit(CrossCircle, 1, 0.5, "circlePD", 3, dataint, &IslandPD, &FolkPD, NULL, &circle_PB);
    param = MENU_fileInit(param, 1, 1.0, "Param", 3, none, &PD, &raceMemory, NULL, &island);
    raceMemory = MENU_fileInit(raceMemory, 0, 1.0, "memory", 4, dataint, &param, NULL, NULL, &memory1);//0关闭 1打开
    FolkPD = MENU_fileInit(FolkPD, 1, 1.0, "folk", 4, dataint, &CrossCircle, &straightPD, NULL, &Folk_PB);
    straightPD = MENU_fileInit(straightPD, 0, 1.0, "straight", 5, dataint, &FolkPD, NULL, NULL, &straight_KP);

//    bugFix1 = MENU_fileInit(bugFix1, 1, 1.0, "nomean", 2, none, &straightPD, NULL, NULL, NULL);

    memory1 = MENU_fileInit(memory1, 0, 1.0, "memory1", 2, dataint, NULL, &memory2, &raceMemory, &memorySpeed1);
    memorySpeed1 = MENU_fileInit(memorySpeed1, 85, 1.0, "speed1", 2, dataint, NULL, &memoryVision1, &memory1, NULL);
    memoryVision1 = MENU_fileInit(memoryVision1, 92, 1.0, "vision1", 3, dataint, &memorySpeed1, NULL, NULL, NULL);

    memory2 = MENU_fileInit(memory2, 110, 1.0, "memory2", 3, dataint, &memory1, &memory3, NULL, &memorySpeed2);
    memorySpeed2 = MENU_fileInit(memorySpeed2, 100, 1.0, "speed2", 2, dataint, NULL, &memoryVision2, &memory2, NULL);
    memoryVision2 = MENU_fileInit(memoryVision2, 88, 1.0, "vision2", 3, dataint, &memorySpeed2, NULL, NULL, NULL);

    memory3 = MENU_fileInit(memory3, 40, 1.0, "memory3", 4, dataint, &memory2, &memory4, NULL, &memorySpeed3);
    memorySpeed3 = MENU_fileInit(memorySpeed3, 95, 1.0, "speed3", 2, dataint, NULL, &memoryVision3, &memory3, NULL);
    memoryVision3 = MENU_fileInit(memoryVision3, 90, 1.0, "vision3", 3, dataint, &memorySpeed3, NULL, NULL, NULL);

    memory4 = MENU_fileInit(memory4, 50, 1.0, "memory4", 5, dataint, &memory3, &memory5, NULL, &memorySpeed4);
    memorySpeed4 = MENU_fileInit(memorySpeed4, 100, 1.0, "speed4", 2, dataint, NULL, &memoryVision4, &memory4, NULL);
    memoryVision4 = MENU_fileInit(memoryVision4, 90, 1.0, "vision4", 3, dataint, &memorySpeed4, NULL, NULL, NULL);

    memory5 = MENU_fileInit(memory5, 30, 1.0, "memory5", 6, dataint, &memory4, &memory6, NULL, &memorySpeed5);
    memorySpeed5 = MENU_fileInit(memorySpeed5, 90, 1.0, "speed5", 2, dataint, NULL, &memoryVision5, &memory5, NULL);
    memoryVision5 = MENU_fileInit(memoryVision5, 92, 1.0, "vision5", 3, dataint, &memorySpeed5, NULL, NULL, NULL);

    memory6 = MENU_fileInit(memory6, 110, 1.0, "memory6", 7, dataint, &memory5, &memory7, NULL, &memorySpeed6);
    memorySpeed6 = MENU_fileInit(memorySpeed6, 95, 1.0, "speed6", 2, dataint, NULL, &memoryVision6, &memory6, NULL);
    memoryVision6 = MENU_fileInit(memoryVision6, 90, 1.0, "vision6", 3, dataint, &memorySpeed6, NULL, NULL, NULL);

    memory7 = MENU_fileInit(memory7, 10, 1.0, "memory7", 2, dataint, &memory6, &memory8, NULL, &memorySpeed7);
    memorySpeed7 = MENU_fileInit(memorySpeed7, 90, 1.0, "speed7", 2, dataint, NULL, &memoryVision7, &memory7, NULL);
    memoryVision7 = MENU_fileInit(memoryVision7, 86, 1.0, "vision7", 3, dataint, &memorySpeed7, NULL, NULL, NULL);

    memory8 = MENU_fileInit(memory8, 20, 1.0, "memory8", 3, dataint, &memory7, &memory9, NULL, &memorySpeed8);
    memorySpeed8 = MENU_fileInit(memorySpeed8, 90, 1.0, "speed8", 2, dataint, NULL, &memoryVision8, &memory8, NULL);
    memoryVision8 = MENU_fileInit(memoryVision8, 90, 1.0, "vision8", 3, dataint, &memorySpeed8, NULL, NULL, NULL);

    memory9 = MENU_fileInit(memory9, 20, 1.0, "memory9", 4, dataint, &memory8, &memory10, NULL, &memorySpeed9);
    memorySpeed9 = MENU_fileInit(memorySpeed9, 95, 1.0, "speed9", 2, dataint, NULL, &memoryVision9, &memory9, NULL);
    memoryVision9 = MENU_fileInit(memoryVision9, 89, 1.0, "vision9", 3, dataint, &memorySpeed9, NULL, NULL, NULL);

    memory10 = MENU_fileInit(memory10, 40, 1.0, "memory10", 5, dataint, &memory9, &memory11, NULL, &memorySpeed10);
    memorySpeed10 = MENU_fileInit(memorySpeed10, 95, 1.0, "speed10", 2, dataint, NULL, &memoryVision10, &memory10, NULL);
    memoryVision10 = MENU_fileInit(memoryVision10, 90, 1.0, "vision10", 3, dataint, &memorySpeed10, NULL, NULL, NULL);

    memory11 = MENU_fileInit(memory11, 50, 1.0, "memory11", 6, dataint, &memory10, &memory12, NULL, &memorySpeed11);
    memorySpeed11 = MENU_fileInit(memorySpeed11, 90, 1.0, "speed11", 2, dataint, NULL, &memoryVision11, &memory11, NULL);
    memoryVision11 = MENU_fileInit(memoryVision11, 90, 1.0, "vision11", 3, dataint, &memorySpeed11, NULL, NULL, NULL);

    memory12 = MENU_fileInit(memory12, 100, 1.0, "memory12", 7, dataint, &memory11, &memory13, NULL, &memorySpeed12);
    memorySpeed12 = MENU_fileInit(memorySpeed12, 95, 1.0, "speed12", 2, dataint, NULL, &memoryVision12, &memory12, NULL);
    memoryVision12 = MENU_fileInit(memoryVision12, 90, 1.0, "vision12", 3, dataint, &memorySpeed12, NULL, NULL, NULL);

    memory13 = MENU_fileInit(memory13, 130, 1.0, "memory13", 2, dataint, &memory12, &memory14, NULL, &memorySpeed13);
    memorySpeed13 = MENU_fileInit(memorySpeed13, 95, 1.0, "speed13", 2, dataint, NULL, &memoryVision13, &memory13, NULL);
    memoryVision13 = MENU_fileInit(memoryVision13, 90, 1.0, "vision13", 3, dataint, &memorySpeed13, NULL, NULL, NULL);

    memory14 = MENU_fileInit(memory14, 20, 1.0, "memory14", 3, dataint, &memory13, &memory15, NULL, &memorySpeed14);
    memorySpeed14 = MENU_fileInit(memorySpeed14, 90, 1.0, "speed14", 2, dataint, NULL, &memoryVision14, &memory14, NULL);
    memoryVision14 = MENU_fileInit(memoryVision14, 92, 1.0, "vision14", 3, dataint, &memorySpeed14, NULL, NULL, NULL);

    memory15 = MENU_fileInit(memory15, 0, 1.0, "memory15", 4, dataint, &memory14, &memoryBottom, NULL, &memorySpeed15);
    memorySpeed15 = MENU_fileInit(memorySpeed15, 90, 1.0, "speed15", 2, dataint, NULL, &memoryVision15, &memory15, NULL);
    memoryVision15 = MENU_fileInit(memoryVision15, 92, 1.0, "vision15", 3, dataint, &memorySpeed15, NULL, NULL, NULL);

    memoryBottom = MENU_fileInit(memoryBottom, 0, 1.0, "bottom", 5, none, &memory15, NULL, NULL, NULL);


    island = MENU_fileInit(island, 1, 1.0, "island", 2, none, NULL, &cross_circle, &param, &islandout_up);
    islandout_up = MENU_fileInit(islandout_up, 0, 1.0, "upPoint", 2, dataint, NULL, &design_island_k, &island, NULL);
    design_island_k = MENU_fileInit(design_island_k, 1, 0, "left-dk", 3, datafloat, &islandout_up, &islandParam1, NULL, NULL);
    islandParam1 = MENU_fileInit(islandParam1, 1, 0, "right-dk", 4, datafloat, &design_island_k, &islandParam2, NULL, NULL);
    islandParam2 = MENU_fileInit(islandParam2, 70, 1.8, "radius", 5, dataint, &islandParam1, &islandParam3, NULL, NULL);
    islandParam3 = MENU_fileInit(islandParam3, 0, 0.35, "xleft", 6, dataint, &islandParam2, &islandParam4, NULL, NULL);
    islandParam4 = MENU_fileInit(islandParam4, 0, 0.35, "xright", 7, dataint, &islandParam3, &islandParam5, NULL, NULL);
    islandParam5 = MENU_fileInit(islandParam5, 64, 0, "dk-out", 2, datafloat, &islandParam4, &islandParam6, NULL, NULL);
    islandParam6 = MENU_fileInit(islandParam6, 0, 0, "width", 3, dataint, &islandParam5, &islandBottom, NULL, NULL);
    islandBottom = MENU_fileInit(islandBottom, 1, 0.35, "bottom", 4, none, &islandParam6, NULL, NULL, NULL);


    cross_circle = MENU_fileInit(cross_circle, 1, 1.0, "crocircle", 3, none, &island, &carPark, NULL, &cross_circle_param1);
    cross_circle_param1 = MENU_fileInit(cross_circle_param1, 90, 1.0, "vision", 2, dataint, NULL, &cross_circle_param2, &cross_circle, NULL);
    cross_circle_param2 = MENU_fileInit(cross_circle_param2, 90, 1.0, "rightX", 3, dataint, &cross_circle_param1, &cross_circle_param3, NULL, NULL);
    cross_circle_param3 = MENU_fileInit(cross_circle_param3, 84, 1.0, "leftX", 4, dataint, &cross_circle_param2, &cross_circle_param4, NULL, NULL);
    cross_circle_param4 = MENU_fileInit(cross_circle_param4, 89, 1.0, "outwidth", 5, dataint, &cross_circle_param3, &cross_circle_param5, NULL, NULL);
    cross_circle_param5 = MENU_fileInit(cross_circle_param5, 55, 0, "kfix", 6, datafloat, &cross_circle_param4, &cross_circle_param6, NULL, NULL);
    cross_circle_param6 = MENU_fileInit(cross_circle_param6, 30, 1.0, "turnwidth", 7, dataint, &cross_circle_param5, &cross_circle_param7, NULL, NULL);
    cross_circle_param7 = MENU_fileInit(cross_circle_param7, 55, 0, "rowKP", 2, datafloat, &cross_circle_param6, &cross_circle_param8, NULL, NULL);
    cross_circle_param8 = MENU_fileInit(cross_circle_param8, 82, 1.0, "tgt-row", 3, dataint, &cross_circle_param7, &ccBottom, NULL, NULL);
    ccBottom = MENU_fileInit(ccBottom, 55, 1.0, "bottom", 4, none, &cross_circle_param8, NULL, NULL, NULL);

    carPark = MENU_fileInit(carPark, 1, 1.0, "park", 4, none, &cross_circle, &ramp, NULL, &parkCount);
    parkCount = MENU_fileInit(parkCount, 58, 1.0, "parkCount", 2, dataint, NULL, &startGyro, &carPark, NULL);
    startGyro = MENU_fileInit(startGyro, 30, 1.0, "st-gyro", 3, dataint, &parkCount, &endGyro, NULL, NULL);
    endGyro = MENU_fileInit(endGyro, 40, 1.0, "end-gyro", 4, dataint, &startGyro, &search_line, NULL, NULL);
    search_line = MENU_fileInit(search_line, 80, 1.0, "line", 5, dataint, &endGyro, &parkDelay, NULL, NULL);
    parkDelay = MENU_fileInit(parkDelay, 5, 1.0, "delay", 6, dataint, &search_line, &paramBottom, NULL, NULL);

    paramBottom = MENU_fileInit(paramBottom, 1, 1.0, "bottom", 5, none, &parkDelay, NULL, NULL, NULL);

    ramp = MENU_fileInit(ramp, 1, 1.0, "ramp", 5, none, &carPark, &folkWay, NULL, &rampCount);
    rampCount = MENU_fileInit(rampCount, 80, 1.0, "rampCount", 2, dataint, NULL, &rampDistance, &ramp, NULL);
    rampDistance = MENU_fileInit(rampDistance, 40, 1.0, "distance", 3, dataint, &rampCount, &rampSpeed, NULL, NULL);
    rampSpeed = MENU_fileInit(rampSpeed, 70, 1.0, "rampSpeed", 4, dataint, &rampDistance, &rampMax, NULL, NULL);
    rampMax = MENU_fileInit(rampMax, 60, 1.0, "Max", 5, dataint, &rampSpeed, &rampMin, NULL, NULL);
    rampMin = MENU_fileInit(rampMin, 15, 1.0, "Min", 6, dataint, &rampMax, &rampBottom, NULL, NULL);

    rampBottom = MENU_fileInit(rampBottom, 50, 1.0, "Rbottom", 7, none, &rampMin, NULL, NULL, NULL);


    folkWay = MENU_fileInit(folkWay, 1, 1.0, "folk", 6, dataint, &ramp, NULL, NULL, &folkParam1);//1-left   -1-right
    folkParam1 = MENU_fileInit(folkParam1, 30, 1.0, "param1", 2, dataint, NULL, &folkParam2, &folkWay, NULL);
    folkParam2 = MENU_fileInit(folkParam2, 25, 0.1, "param2", 3, datafloat, &folkParam1, NULL, NULL, NULL);
//    folkParam3 = MENU_fileInit(folkParam3, 95, 1.0, "param3", 4, dataint, &folkParam2, &folkParam4, NULL, NULL);
//    folkParam4 = MENU_fileInit(folkParam4, 1, 1.0, "param4", 5, dataint, &folkParam3, &folkBottom, NULL, NULL);
//    folkParam5 = MENU_fileInit(folkParam5, 1, 1.0, "param5", 6, dataint, &folkParam4, &folkParam6, NULL, NULL);
//    folkParam6 = MENU_fileInit(folkParam6, 1, 1.0, "param6", 7, dataint, &folkParam5, &folkParam7, NULL, NULL);
//    folkParam7 = MENU_fileInit(folkParam7, 1, 1.0, "param7", 2, dataint, &folkParam6, &folkParam8, NULL, NULL);
//    folkParam8 = MENU_fileInit(folkParam8, 1, 1.0, "param8", 3, dataint, &folkParam7, &folkParam9, NULL, NULL);
//    folkParam9 = MENU_fileInit(folkParam9, 1, 1.0, "param9", 4, dataint, &folkParam8, &folkParam10, NULL, NULL);
//    folkParam10 = MENU_fileInit(folkParam10, 1, 1.0, "param10", 5, dataint, &folkParam9, &folkBottom, NULL, NULL);
//    folkBottom = MENU_fileInit(folkBottom, 1, 1.0, "bottom", 6, none, &folkParam4, NULL, NULL, NULL);

//    Cross_PB = MENU_fileInit(Cross_PB, 1, 0.5, "crossPB", 2, datafloat, NULL, &Cross_PM, &CrossPD, NULL);
//    Cross_PM = MENU_fileInit(Cross_PM, 1, 0.5, "crossPM", 3, datafloat, &Cross_PB, &Cross_PS, NULL, NULL);
//    Cross_PS = MENU_fileInit(Cross_PS, 1, 0.5, "crossPS", 4, datafloat, &Cross_PM, &Cross_ZO, NULL, NULL);
//    Cross_ZO = MENU_fileInit(Cross_ZO, 1, 0.5, "crossZO", 5, datafloat, &Cross_PS, &Cross_NS, NULL, NULL);
//    Cross_NS = MENU_fileInit(Cross_NS, 1, 0.5, "crossNS", 6, datafloat, &Cross_ZO, &Cross_NM, NULL, NULL);
//    Cross_NM = MENU_fileInit(Cross_NM, 1, 0.5, "crossNM", 7, datafloat, &Cross_NS, &Cross_NB, NULL, NULL);
//    Cross_NB = MENU_fileInit(Cross_NB, 1, 0.5, "crossNB", 2, datafloat, &Cross_NM, &Cross_DS, NULL, NULL);
//    Cross_DS = MENU_fileInit(Cross_DS, 1, 0.5, "D-SMALL", 3, datafloat, &Cross_NB, &Cross_DB, NULL, NULL);
//    Cross_DB = MENU_fileInit(Cross_DB, 1, 0.5, "D-BIG", 4, datafloat, &Cross_DS, NULL, NULL, NULL);

    circle_PB = MENU_fileInit(circle_PB, 1, 3, "circlePB", 2, datafloat, NULL, &circle_DS, &CrossCircle, NULL);
//    circle_PM = MENU_fileInit(circle_PM, 1, 3, "circlePM", 3, datafloat, &circle_PB, &circle_PS, NULL, NULL);
//    circle_PS = MENU_fileInit(circle_PS, 1, 2.2, "circlePS", 4, datafloat, &circle_PM, &circle_ZO, NULL, NULL);
//    circle_ZO = MENU_fileInit(circle_ZO, 1, 1.4, "circleZO", 5, datafloat, &circle_PS, &circle_NS, NULL, NULL);
//    circle_NS = MENU_fileInit(circle_NS, 1, 2.2, "circleNS", 6, datafloat, &circle_ZO, &circle_NM, NULL, NULL);
//    circle_NM = MENU_fileInit(circle_NM, 1, 3, "circleNM", 7, datafloat, &circle_NS, &circle_NB, NULL, NULL);
//    circle_NB = MENU_fileInit(circle_NB, 1, 3.9, "circleNB", 2, datafloat, &circle_NM, &circle_DS, NULL, NULL);
    circle_DS = MENU_fileInit(circle_DS, 1, 2.5, "D-SMALL", 3, datafloat, &circle_PB, NULL, NULL, NULL);
//    circle_DB = MENU_fileInit(circle_DB, 1, 0.5, "D-BIG", 4, datafloat, &circle_DS, NULL, NULL, NULL);

    Island_PB = MENU_fileInit(Island_PB, 1, 2.4, "BIG-I", 2, datafloat, NULL, &Island_PM, &IslandPD, NULL);
    Island_PM = MENU_fileInit(Island_PM, 1, 2.7, "MID-I", 3, datafloat, &Island_PB, &Island_PS, NULL, NULL);
    Island_PS = MENU_fileInit(Island_PS, 1, 2.8, "SMALL-I", 4, datafloat, &Island_PM, &Island_DS, NULL, NULL);
//    Island_ZO = MENU_fileInit(Island_ZO, 1, 2.8, "IslandZO", 5, datafloat, &Island_PS, &Island_NS, NULL, NULL);
//    Island_NS = MENU_fileInit(Island_NS, 1, 3, "IslandNS", 6, datafloat, &Island_ZO, &Island_NM, NULL, NULL);
//    Island_NM = MENU_fileInit(Island_NM, 1, 3.3, "IslandNM", 7, datafloat, &Island_NS, &Island_NB, NULL, NULL);
//    Island_NB = MENU_fileInit(Island_NB, 1, 3.7, "IslandNB", 3, datafloat, &Island_PB, &Island_DS, NULL, NULL);
    Island_DS = MENU_fileInit(Island_DS, 1, 2.5, "D-SMALL", 5, datafloat, &Island_PS, NULL, NULL, NULL);
//    Island_DB = MENU_fileInit(Island_DB, 1, 0.5, "D-BIG", 4, datafloat, &Island_DS, NULL, NULL, NULL);

    Folk_PB = MENU_fileInit(Folk_PB, 1, 3.6, "FolkPB", 2, datafloat, NULL, &Folk_PM, &FolkPD, NULL);
    Folk_PM = MENU_fileInit(Folk_PM, 1, 2.6, "FolkPM", 3, datafloat, &Folk_PB, &Folk_PS, NULL, NULL);
    Folk_PS = MENU_fileInit(Folk_PS, 1, 1.6, "FolkPS", 4, datafloat, &Folk_PM, &Folk_ZO, NULL, NULL);
    Folk_ZO = MENU_fileInit(Folk_ZO, 1, 1, "FolkZO", 5, datafloat, &Folk_PS, &Folk_NS, NULL, NULL);
    Folk_NS = MENU_fileInit(Folk_NS, 1, 1.6, "FolkNS", 6, datafloat, &Folk_ZO, &Folk_NM, NULL, NULL);
    Folk_NM = MENU_fileInit(Folk_NM, 1, 2.6, "FolkNM", 7, datafloat, &Folk_NS, &Folk_NB, NULL, NULL);
    Folk_NB = MENU_fileInit(Folk_NB, 1, 3.6, "FolkNB", 2, datafloat, &Folk_NM, &Folk_DS, NULL, NULL);
    Folk_DS = MENU_fileInit(Folk_DS, 1, 3, "D-SMALL", 3, datafloat, &Folk_NB, &Folk_DB, NULL, NULL);
//    Folk_DB = MENU_fileInit(Folk_DB, 1, 0.5, "D-BIG", 4, datafloat, &Folk_DS, NULL, NULL, NULL);

    straight_KP = MENU_fileInit(straight_KP, 1, 1.6, "str-KP", 2, datafloat, NULL, &straight_KD, &straightPD, NULL);
    straight_KD = MENU_fileInit(straight_KD, 1, 1.4, "str-KD", 3, datafloat, &straight_KP, NULL, NULL, NULL);


    gear1 = MENU_fileInit(gear1, 1, 1.0, "GearSlow", 4, none, &currentK, &gear2, NULL, &g1_Data1);//慢速档
//    g1_Data1 = MENU_fileInit(g1_Data1, 75, 32.3, "speed1", 2, dataint, NULL, &g1_Data2, &gear1, NULL);
//    g1_Data2 = MENU_fileInit(g1_Data2, 170, 70.22, "THRE1", 3, dataint, &g1_Data1, &g1_Data3, NULL, NULL);
//    g1_Data3 = MENU_fileInit(g1_Data3, 90, 42.09, "Vision1", 4, dataint, &g1_Data2, &g1_Data4, NULL, NULL);
//    g1_Data4 = MENU_fileInit(g1_Data4, 1, 2.9, "PB1", 5, datafloat, &g1_Data3, &g1_Data5, NULL, NULL);
//    g1_Data5 = MENU_fileInit(g1_Data5, 1, 2.6, "PM1", 6, datafloat, &g1_Data4, &g1_Data6, NULL, NULL);
//    g1_Data6 = MENU_fileInit(g1_Data6, 1, 2.2, "PS1", 7, datafloat, &g1_Data5, &g1_Data7, NULL, NULL);
//    g1_Data7 = MENU_fileInit(g1_Data7, 1, 2, "ZO1", 2, datafloat, &g1_Data6, &g1_Data8, NULL, NULL);
//    g1_Data8 = MENU_fileInit(g1_Data8, 1, 2.2, "NS1", 3, datafloat, &g1_Data7, &g1_Data9, NULL, NULL);
//    g1_Data9 = MENU_fileInit(g1_Data9, 1, 2.6, "NM1", 4, datafloat, &g1_Data8, &g1_Data10, NULL, NULL);
//    g1_Data10 = MENU_fileInit(g1_Data10, 1, 2.9, "NB1", 5, datafloat, &g1_Data9, &g1_Data11, NULL, NULL);
//    g1_Data11 = MENU_fileInit(g1_Data11, 1, 4.8, "servoKD", 6, datafloat, &g1_Data10, &preGear1, NULL, NULL);

    gear2 = MENU_fileInit(gear2, 1, 1.0, "GearFast", 5, none, &gear1, &gear3, NULL, &g2_Data1);//快速档
//    g2_Data1 = MENU_fileInit(g2_Data1, 85, 38.27, "speed2", 2, dataint, NULL, &g2_Data2, &gear2, NULL);
//    g2_Data2 = MENU_fileInit(g2_Data2, 140, 75.32, "THRE2", 3, dataint, &g2_Data1, &g2_Data3, NULL, NULL);
//    g2_Data3 = MENU_fileInit(g2_Data3, 30, 11.62, "Vision2", 4, dataint, &g2_Data2, &g2_Data4, NULL, NULL);
//    g2_Data4 = MENU_fileInit(g2_Data4, 119, 1.9, "servo-P", 5, datafloat, &g2_Data3, &g2_Data5, NULL, NULL);
//    g2_Data5 = MENU_fileInit(g2_Data5, 119, 6.0, "servo-D", 6, datafloat, &g2_Data4, &g2_Data6, NULL, NULL);
//    g2_Data6 = MENU_fileInit(g2_Data6, 119, 0.9, "GAP2", 7, datafloat, &g2_Data5, &g2_Data7, NULL, NULL);
//    g2_Data7 = MENU_fileInit(g2_Data7, 119, 1.1, "motor-I", 2, datafloat, &g2_Data6, &preGear2, NULL, NULL);

    gear3 = MENU_fileInit(gear3, 1, 1.0, "GearFast2", 6, none, &gear2, NULL, NULL, &g3_Data1);//快速档，含g2四个数据
//    g3_Data1 = MENU_fileInit(g3_Data1, 90, 38.27, "speed3", 2, dataint, NULL, &g3_Data2, &gear3, NULL);
//    g3_Data2 = MENU_fileInit(g3_Data2, 140, 75.32, "THRE3", 3, dataint, &g3_Data1, &g3_Data3, NULL, NULL);
//    g3_Data3 = MENU_fileInit(g3_Data3, 29, 11.62, "Vision3", 4, dataint, &g3_Data2, &g3_Data4, NULL, NULL);
//    g3_Data4 = MENU_fileInit(g3_Data4, 119, 1.9, "servo-P", 5, datafloat, &g3_Data3, &g3_Data5, NULL, NULL);
//    g3_Data5 = MENU_fileInit(g3_Data5, 119, 6.8, "servo-D", 6, datafloat, &g3_Data4, &g3_Data6, NULL, NULL);
//    g3_Data6 = MENU_fileInit(g3_Data6, 119, 0.9, "GAP3", 7, datafloat, &g3_Data5, &g3_Data7, NULL, NULL);
//    g3_Data7 = MENU_fileInit(g3_Data7, 119, 1.1, "motor-I", 2, datafloat, &g3_Data6, &preGear3, NULL, NULL);

    currentK = MENU_fileInit(currentK, 1, 1.0, "currentK", 3, none, &Threshold, &gear1, NULL, &currentRTKP);
    currentRTKP = MENU_fileInit(currentRTKP, 3, 3.6, "curRTKP", 2, datafloat, NULL, &currentRTKI, &currentK, NULL);
    currentRTKI = MENU_fileInit(currentRTKI, 2, 4, "curRTKI", 3, datafloat, &currentRTKP, &currentLFKP, NULL, NULL);
    currentLFKP = MENU_fileInit(currentLFKP, 3, 3.6, "curLFKP", 4, datafloat, &currentRTKI, &currentLFKI, &currentK, NULL);
    currentLFKI = MENU_fileInit(currentLFKI, 2, 4, "curLFKI", 5, datafloat, &currentLFKP, &expectC, NULL, NULL);
//    currentKD = MENU_fileInit(currentKD, 2, 3.0, "motorKD", 4, datafloat, &currentRTKI, &expectC, NULL, NULL);
    expectC = MENU_fileInit(expectC, 2200, 9.5, "expect", 6, dataint, &currentLFKI, NULL, NULL, NULL);

    Threshold = MENU_fileInit(Threshold, 0, 1.0, "THRE", 2, none, NULL, &currentK, &file1, &wayThre);//上下左右
    wayThre = MENU_fileInit(wayThre, 2, 1.0, "THREway", 2, dataint, NULL, &OTSU1, &Threshold, NULL);
    OTSU1 = MENU_fileInit(OTSU1, 1, 1.0, "OTSU", 3, none, &wayThre, &partOTSU, NULL, &OTSU_Klow);
    OTSU_Klow = MENU_fileInit(OTSU_Klow, 110, 1.0, "K-low", 2, dataint, NULL, &OTSU_Khigh, &OTSU1, NULL);
    OTSU_Khigh = MENU_fileInit(OTSU_Khigh, 140, 1.0, "K-higt", 3, dataint, &OTSU_Klow, NULL, NULL, NULL);

    partOTSU = MENU_fileInit(partOTSU, 1, 1.0, "partOTSU", 4, none, &OTSU1, &expTime, NULL, &part_klow1);
    part_klow1 = MENU_fileInit(part_klow1, 115, 0.81, "fixup", 2, datafloat, NULL, &part_khigh1, &partOTSU, NULL);
    part_khigh1 = MENU_fileInit(part_khigh1, 140, 0.75, "fixdown", 3, datafloat, &part_klow1, &part_klow2, NULL, NULL);
    part_klow2 = MENU_fileInit(part_klow2, 50, 0.9, "minthre", 4, dataint, &part_khigh1, &part_khigh2, NULL, NULL);
    part_khigh2 = MENU_fileInit(part_khigh2, 100, 1.0, "maxthre", 5, dataint, &part_klow2, NULL, NULL, NULL);

    expTime = MENU_fileInit(expTime, 300, 1.0, "exptime", 5, dataint, &partOTSU, NULL, NULL, NULL);

    /* 当下的电机pwm值（speedL/R）和摄像头前瞻vision */
    presentSpeed = MENU_fileInit(presentSpeed, 85, 1.1, "speed", 2, dataint, NULL, &presentTHRE, &file2, NULL);
    presentTHRE = MENU_fileInit(presentTHRE, 110, 2.2, "THRE", 3, dataint, &presentSpeed, &presentVision, NULL, NULL);
    presentVision = MENU_fileInit(presentVision, 91, 3.3, "VISION", 4, dataint, &presentTHRE, &fuzzyPB, NULL, NULL);
    fuzzyPB = MENU_fileInit(fuzzyPB, 1, 3.6, "fuzzyPB", 5, datafloat, &presentVision, &fuzzyPM, NULL, NULL);
    fuzzyPM = MENU_fileInit(fuzzyPM, 1, 3.3, "fuzzyPM", 6, datafloat, &fuzzyPB, &fuzzyPS, NULL, NULL);
    fuzzyPS = MENU_fileInit(fuzzyPS, 1, 2.4, "fuzzyPS", 7, datafloat, &fuzzyPM, &fuzzyZO, NULL, NULL);
    fuzzyZO = MENU_fileInit(fuzzyZO, 1, 1.8, "fuzzyZO", 2, datafloat, &fuzzyPS, &fuzzyNS, NULL, NULL);
    fuzzyNS = MENU_fileInit(fuzzyNS, 1, 2.4, "fuzzyNS", 3, datafloat, &fuzzyZO, &fuzzyNM, NULL, NULL);
    fuzzyNM = MENU_fileInit(fuzzyNM, 1, 3.3, "fuzzyNM", 4, datafloat, &fuzzyNS, &fuzzyNB, NULL, NULL);
    fuzzyNB = MENU_fileInit(fuzzyNB, 1, 4, "fuzzyNB", 5, datafloat, &fuzzyNM, &presentServoD, NULL, NULL);
    presentServoD = MENU_fileInit(presentServoD, 1, 3.6, "preServoD", 6, datafloat, &fuzzyNB, &gap, NULL, NULL);
    gap = MENU_fileInit(gap, 1, 1, "GAP", 7, datafloat, &presentServoD, &speedLow, NULL, NULL);
    speedLow = MENU_fileInit(speedLow, 85, 0.14, "midlineKP", 2, dataint, &gap, &bottomData, NULL, NULL);
//    gyroKP = MENU_fileInit(gyroKP, 1, 0.12, "gyroKP", 3, datafloat, &midLineKP, &gyroKD, NULL, NULL);
//    gyroKD = MENU_fileInit(gyroKD, 1, 0.22, "gyroKD", 4, datafloat, &gyroKP, &bottomData, NULL, NULL);
//    presentMotorI = MENU_fileInit(presentMotorI, 1, 190.0, "preMotorI", 2, datafloat, &gap, &bottomData, NULL, NULL);
//    presentMotorP = MENU_fileInit(gap, 8, 250, "preMotorP", 3, datafloat, &presentMotorI, &bottomData, NULL, NULL);
    bottomData = MENU_fileInit(bottomData, 1, 1.0, "bottom", 5, none, &speedLow, NULL, NULL, NULL);
//    preGear1 = MENU_fileInit(preGear1, 1, 1.0, "Present1", 3, none, &g1_Data11, NULL, NULL, &presentSpeed);
//    preGear2 = MENU_fileInit(preGear2, 1, 1.0, "Present2", 3, none, &g2_Data7, NULL, NULL, &presentSpeed);
//    preGear3 = MENU_fileInit(preGear3, 1, 1.0, "Present3", 3, none, &g3_Data7, NULL, NULL, &presentSpeed);


    display = MENU_fileInit(display, 1, 1.0, "motor", 5, none, &file3, &fileSave, NULL, &display1);
    display1 = MENU_fileInit(display1, 40, 133.03, "motor", 2, none, NULL, &display2, &display, &LFKP);
    display2 = MENU_fileInit(display2, 25, 0.1, "rowKp", 3, datafloat, &display1, &display3, NULL, NULL);

    LFKP = MENU_fileInit(LFKP, 34, 3.8, "KP", 2, dataint, NULL, &LFKI, &display1, NULL);
    LFKI = MENU_fileInit(LFKI, 18, 1.5, "KI", 3, dataint, &LFKP, &RTKP, NULL, NULL);
    RTKP = MENU_fileInit(RTKP, 8, 3.8, "motorPS", 4, dataint, &LFKI, &RTKI, NULL, NULL);
    RTKI = MENU_fileInit(RTKI, 4, 1.5, "motorZO", 5, dataint, &RTKP, &fastLFKP, NULL, NULL);
    fastLFKP = MENU_fileInit(fastLFKP, 5, 133.03, "motorNM", 6, dataint, &RTKI, &fastLFKI, NULL, NULL);
    fastLFKI = MENU_fileInit(fastLFKI, 20, 133.03, "motorNB", 7, dataint, &fastLFKP, &fastRTKP, NULL, NULL);
    fastRTKP = MENU_fileInit(fastRTKP, 20, 133.03, "motorKD", 2, dataint, &fastLFKI, &fastRTKI, NULL, NULL);
    fastRTKI = MENU_fileInit(fastRTKI, 50, 133.03, "BOTTOM", 3, dataint, &fastRTKP, NULL, NULL, NULL);
//
//    slowLFKP = MENU_fileInit(slowLFKP, 25, 133.03, "slowLFKP", 2, dataint, NULL, &slowLFKI, &display2, NULL);
//    slowLFKI = MENU_fileInit(slowLFKI, 15, 133.03, "slowLFKI", 3, dataint, &slowLFKP, &slowRTKP, NULL, NULL);
//    slowRTKP = MENU_fileInit(slowRTKP, 25, 133.03, "slowRTKP", 4, dataint, &slowLFKI, &slowRTKI, NULL, NULL);
//    slowRTKI = MENU_fileInit(slowRTKI, 15, 133.03, "slowRTKI", 5, dataint, &slowRTKP, NULL, NULL, NULL);

    display3 = MENU_fileInit(display3, 40, 0.95, "filter", 4, none, &display2, &display4, NULL, &speedFilter);

    speedFilter = MENU_fileInit(speedFilter, 40, 0.95, "alpha", 2, datafloat, NULL, &currentFilter1, &display3, NULL);
    currentFilter1 = MENU_fileInit(currentFilter1, 40, 0.8, "beta1", 3, datafloat, &speedFilter, &currentFilter2, NULL, NULL);
    currentFilter2 = MENU_fileInit(currentFilter2, 40, 0.2, "beta2", 4, datafloat, &currentFilter1, &currentKdLpf, NULL, NULL);
    currentKdLpf = MENU_fileInit(currentKdLpf, 40, 0.9, "curDfilter", 5, datafloat, &currentFilter2, &speedKdLpf, NULL, NULL);
    speedKdLpf = MENU_fileInit(speedKdLpf, 40, 0.9, "speDfilter", 6, datafloat, &currentKdLpf, &filterBottom, NULL, NULL);
    filterBottom = MENU_fileInit(filterBottom, 40, 0.9, "bottom", 7, none, &speedKdLpf, NULL, NULL, NULL);

    display4 = MENU_fileInit(display4, 10, 0.5, "beta1", 5, datafloat, &display3, &display5, NULL, NULL);
    display5 = MENU_fileInit(display5, 10, 0.3, "beta2", 6, datafloat, &display4, &display6, NULL, NULL);
    display6 = MENU_fileInit(display6, 2, 1, "speedUP", 7, datafloat, &display5, &display7, NULL, NULL);
    display7 = MENU_fileInit(display7, 100, 1, "speedDOWN", 2, datafloat, &display6, &display8, NULL, NULL);
    display8 = MENU_fileInit(display8, 130, 7.91, "stopTHRE", 3, dataint, &display7, &display9, NULL, NULL);
    display9 = MENU_fileInit(display9, 1, 9.02, "testTimes", 4, dataint, &display8, NULL, NULL, NULL);
//    display10 = MENU_fileInit(display10, 70, 9.02, "param2", 5, dataint, &display9, NULL, NULL, NULL);

    //数据写入flash
    fileSave = MENU_fileInit(fileSave, 1, 1.0, "SAVE", 6, none, &display, NULL, NULL, &saveGear1);
    saveGear1 = MENU_fileInit(saveGear1, 1, 1.0, "SaveGear1", 2, none, NULL, &saveGear2, &fileSave, &g1_Data1);
    saveGear2 = MENU_fileInit(saveGear2, 1, 1.0, "SaveGear2", 3, none, &saveGear1, &saveGear3, NULL, &g2_Data1);
    saveGear3 = MENU_fileInit(saveGear3, 1, 1.0, "SaveGear3", 4, none, &saveGear2, &startWay, NULL, &g3_Data1);

    startWay = MENU_fileInit(startWay, 0, 1.0, "start-way", 5, dataint, &saveGear3, NULL, NULL, NULL);

    //摄像头显示在oled上
//    image = MENU_fileInit(image, 1, 1.0, "IMAGE", 7, none, &fileSave, NULL, NULL, NULL);
   // list = MENU_fileInit(list, 1, 1.0, "list", 2, none, &image, NULL, NULL, NULL);

    minThre = part_klow2.intVal;
    maxThre = part_khigh2.intVal;
}


void MENU_namePrintf(nodeptr_t printFile)
{

    uint32 flag;
    uint32 count = 0;
    SmartCar_OLED_Fill(0);

    SmartCar_OLED_Printf6x8(0, 0, "%s", printFile->name);

    SmartCar_OLED_P6x8Str(0, printFile->pos, ">> ");
//    SmartCar_OLED_P6x8Str(65, 0, "Page");
    SmartCar_OLED_Printf6x8(65, 0, "%.2f", voltage);

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

    else if(y >= 13 && y < 19)
    {
        flag = 12;
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
//            break;
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
    else if(y >= 13 && y < 19)
    {
        flag1 = 12;
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
             SmartCar_OLED_Printf6x8(80, 2+count1, "%d", temp->intVal);
        }
        else if(temp->i == datafloat)
        {
            SmartCar_OLED_Printf6x8(80, 2+count1, "%0.3f", temp->floatVal);
        }

        temp = temp->next;
        count1++;
        if(temp->next == NULL)
        {
            if(temp->i == dataint)
            {
                SmartCar_OLED_Printf6x8(80, 2+count1, "%d", temp->intVal);
            }
            else if(temp->i == datafloat)
            {
                SmartCar_OLED_Printf6x8(80, 2+count1, "%0.3f", temp->floatVal);
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
    if (!GPIO_Read(P11, 2))//上
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

    else if (!GPIO_Read(P11, 9))//下
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


    else if (!GPIO_Read(P11, 10))//左
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


    else if (!GPIO_Read(P11, 11))//右
    {


        if((strcmp(temp->name, "SaveGear1")) == 0)//在"SaveGear1"处按扇区（sector0）写入数据
        {
            MENU_sectorSave(0, temp->forward);
        }

        else if((strcmp(temp->name, "SaveGear2")) == 0)//在"SaveGear2"处按扇区（sector1）写入数据
        {
            MENU_sectorSave(1, temp->forward);
        }
        else if((strcmp(temp->name, "SaveGear3")) == 0)//在"SaveGear3"处按扇区（sector2）写入数据
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
        float times = 1;//改变数值的倍数

        float tempFloat;//还未更改时的浮点值
        int tempInt;//还未更改的整型值

        if(temp->i != none)
        {
            SmartCar_OLED_Fill(0);

            tempInt = temp->intVal;
            tempFloat = temp->floatVal;

            while(TRUE)
            {
                if (!GPIO_Read(P11, 2) || !GPIO_Read(P11, 9) || !GPIO_Read(P11, 10) ||
                                   !GPIO_Read(P11, 11) || !GPIO_Read(P11, 12))
                {
                    Delay_ms(STM0,100);

                    if(temp->i == dataint)/*改变整型数据*/
                    {

                        if(!GPIO_Read(P11, 2))//值增加 上
                        {
                            SmartCar_OLED_Fill(0);
                            temp->intVal += 1 * times;
                        }
                        else if(!GPIO_Read(P11, 9))//值减小 下
                        {
                            SmartCar_OLED_Fill(0);
                            temp->intVal -= 1 * times;
                        }
                        else if(!GPIO_Read(P11, 10))//位数向左 左
                        {
                            SmartCar_OLED_Fill(0);
                            times = times * 10;
                            //intCursor -= 5;

                            SmartCar_OLED_Printf6x8(55, 3, "%2d", temp->intVal);
                            SmartCar_OLED_Printf6x8(55, 4, "%2d", tempInt);


                        }
                        else if (!GPIO_Read(P11, 11))//位数向右 右
                        {
                            SmartCar_OLED_Fill(0);
                            if(times != 1)
                            {
                                times = times / 10.0;
                            }

                            SmartCar_OLED_Printf6x8(55, 4, "%2d", tempInt);
                            SmartCar_OLED_Printf6x8(55, 3, "%2d", temp->intVal);

                        }
                        else if(!GPIO_Read(P11, 12))//确认 ok并回到上一级
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
                        SmartCar_OLED_Printf6x8(55, 3, "%2d", temp->intVal);
                    }

                    else if(temp->i == datafloat)
                    {

                        if(!GPIO_Read(P11, 2))//值增加 上
                        {
                            SmartCar_OLED_Fill(0);
                            temp->floatVal += 0.1 * times;
                        }
                        else if(!GPIO_Read(P11, 9))//值减小 下
                        {
                            SmartCar_OLED_Fill(0);
                            temp->floatVal -= 0.1 * times;
                        }
                        else if(!GPIO_Read(P11, 10))//位数向左 左
                        {
                            SmartCar_OLED_Fill(0);
                            times = times * 10;

                            SmartCar_OLED_Printf6x8(40, 4, "%0.3f", tempFloat);
                            SmartCar_OLED_Printf6x8(40, 3, "%0.3f", temp->floatVal);

                        }
                        else if (!GPIO_Read(P11, 11))//位数向右 右
                        {
                            SmartCar_OLED_Fill(0);
                            times = times / 10.0;

                            SmartCar_OLED_Printf6x8(40, 4, "%0.3f", tempFloat);
                            SmartCar_OLED_Printf6x8(40, 3, "%0.3f", temp->floatVal);

                        }
                        else if(!GPIO_Read(P11, 12))//确认 ok并回到上一级
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
                        SmartCar_OLED_Printf6x8(40, 3, "%3.3f", temp->floatVal);
                    }

                }

            }

        }
        else if(temp->i == none)
                {
                    int page = 0;
                    if((strcmp(temp->name, "GearSlow")) == 0)/*读取flash中gear1的数据*/
                    {
                        dataRead = temp;
                        dataRead = dataRead->forward;
                        do
                        {
                            if(dataRead->i == datafloat)
                            {
                                dataRead->floatVal = Page_Read(0, page, float);
                            }
                            else if(dataRead->i == dataint)
                            {
                                dataRead->intVal = Page_Read(0, page, uint32);
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
                                dataRead->floatVal = Page_Read(1, page, float);
                            }
                            else if(dataRead->i == dataint)
                            {
                                dataRead->intVal = Page_Read(1, page, uint32);
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
                                dataRead->floatVal = Page_Read(2, page, float);
                            }
                            else if(dataRead->i == dataint)
                            {
                                dataRead->intVal = Page_Read(2, page, uint32);
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
                                dataRead->floatVal = Page_Read(0, page, float);
                            }
                            else if(dataRead->i == dataint)
                            {
                                dataRead->intVal = Page_Read(0, page, uint32);
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
                                dataRead->floatVal = Page_Read(1, page, float);
                            }
                            else if(dataRead->i == dataint)
                            {
                                dataRead->intVal = Page_Read(1, page, uint32);
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
                                dataRead->floatVal = Page_Read(2, page, float);
                            }
                            else if(dataRead->i == dataint)
                            {
                                dataRead->intVal = Page_Read(2, page, uint32);
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


void MENU_sectorSave(uint32 sectorNum, nodeptr_t gearData)//按扇区写入
{
    uint32_t buf[1024];

    uint32 count = 0;
    Sector_Erase(sectorNum);

    do
    {
        if(gearData->i == dataint)
        {
            memcpy(buf + count, &(gearData->intVal), sizeof(uint32_t));
        }
        else if(gearData->i == datafloat)
        {
            memcpy(buf + count, &(gearData->floatVal), sizeof(uint32_t));
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
//        SmartCar_OLED_Fillpart(0);
        if(wayThre.intVal == 2)
        {
//             SmartCar_OLED_P6x8Str(110, 0, "up");
             SmartCar_OLED_Printf6x8(110, 0, "%d", thresholdUp);
//             SmartCar_OLED_P6x8Str(100, 2, "down");
             SmartCar_OLED_Printf6x8(110, 1, "%d", thresholdDown);
        }

        else if(wayThre.intVal == 3)
        {
            SmartCar_OLED_P6x8Str(100, 0, "origin");

            SmartCar_OLED_Printf6x8(110, 1, "%d", threOriginal);

        }
        SmartCar_OLED_Printf6x8(110, 4, "%d", mid_line[present_vision]);

//        SmartCar_OLED_P6x8Str(100, 6, "state");
        SmartCar_OLED_Printf6x8(110, 5, "%d", state);

        SmartCar_OLED_Printf6x8(110, 6, "%d", minThre);
        SmartCar_OLED_Printf6x8(110, 7, "%d", maxThre);


        if (!GPIO_Read(P11, 2) || !GPIO_Read(P11, 9) || !GPIO_Read(P11, 10) ||
                !GPIO_Read(P11, 11) || !GPIO_Read(P11, 12) )
        {
            Delay_ms(STM0,100);

            if (!GPIO_Read(P11, 2))//上
            {
                minThre += 1;
            }

            else if (!GPIO_Read(P11, 9))//下
            {
                minThre -= 1;
            }


            else if (!GPIO_Read(P11, 10))//左
            {
                maxThre -= 1;
            }


            else if (!GPIO_Read(P11, 11))//右
            {
                maxThre += 1;
            }


//            else if (!GPIO_Read(P11, 12))//ok
//            {
//
//            }
        }
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

        if (!GPIO_Read(P13, 2))
        {
            if(!GPIO_Read(P11, 12) || !GPIO_Read(P11, 9) || !GPIO_Read(P11, 10) ||
                    !GPIO_Read(P11, 11) || !GPIO_Read(P11, 2))
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
