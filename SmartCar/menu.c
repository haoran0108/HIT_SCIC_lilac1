/*
 * file_name.c
 *
 *  Created on: 2021年10月20日
 *      Author: 李灏然
 */

#include "menu.h"

node_t file1;
node_t file2, file3;
node_t neihuanK, gyroP, gyroD, gyrodatabottom;
node_t gear1, g1_Data1, g1_Data2, g1_Data3, g1_Data4, g1_Data5, g1_Data6, g1_Data7;
node_t gear2, g2_Data1, g2_Data2, g2_Data3, g2_Data4, g2_Data5, g2_Data6, g2_Data7;
node_t gear3, g3_Data1, g3_Data2, g3_Data3, g3_Data4, g3_Data5, g3_Data6, g3_Data7;
node_t currentK, currentRTKP, currentRTKI, currentLFKP, currentLFKI, expectC;

node_t dataEnd;//最后一个数据指向dataEnd
node_t preGear1, preGear2, preGear3;
/*现在的档位参数,bottomData为最后一个数据，无实际意义，为了倒数第二个数据能附上值*/
node_t presentSpeed, presentTHRE, presentVision, fuzzyPB, fuzzyPM, fuzzyPS, fuzzyZO, fuzzyNS, fuzzyNM, fuzzyNB, presentServoD, presentMotorP, presentMotorI, bottomData;
node_t gap;//差速要乘的倍数
node_t display, display1, display2, display3, display4, display5, display6, display7, display8, display9;//展示翻页效果
node_t fileSave, saveGear1, saveGear2, saveGear3;//用于写入flash
node_t image;//显示摄像头图像
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

void MENU_Init()//存取数据时最后一个数据不能操作，待解决
{
    //利用函数fileInit()初始化文件参数
    file1 = MENU_fileInit(file1, 1, 1.0, "GEAR", 2, none, NULL, &file2, NULL, &gear1);
    file2 = MENU_fileInit(file2, 1, 1.0, "PRESENT", 3, none, &file1, &file3, NULL, &presentSpeed);
    file3 = MENU_fileInit(file3, 1, 1.0, "Gyro1", 4, dataint, &file2, &display, NULL, &neihuanK);
    neihuanK = MENU_fileInit(neihuanK, 1, 0.5, "K", 2, datafloat, NULL, &gyroP, &file3, NULL);
    gyroP = MENU_fileInit(gyroP, 1, 0.5, "GyroKP", 3, datafloat, &neihuanK, &gyroD, NULL, NULL);
    gyroD = MENU_fileInit(gyroD, 1, 0, "GyroKD", 4, datafloat, &gyroP, &gyrodatabottom, NULL, NULL);
    gyrodatabottom = MENU_fileInit(gyrodatabottom, 1, 1.0, "bottom", 5, datafloat, &gyroD, NULL, NULL, NULL);

    gear1 = MENU_fileInit(gear1, 1, 1.0, "GearSlow", 2, none, NULL, &gear2, &file1, &g1_Data1);//慢速档
    g1_Data1 = MENU_fileInit(g1_Data1, 75, 32.3, "speed1", 2, dataint, NULL, &g1_Data2, &gear1, NULL);
    g1_Data2 = MENU_fileInit(g1_Data2, 140, 70.22, "THRE1", 3, dataint, &g1_Data1, &g1_Data3, NULL, NULL);
    g1_Data3 = MENU_fileInit(g1_Data3, 31, 42.09, "Vision1", 4, dataint, &g1_Data2, &g1_Data4, NULL, NULL);
    g1_Data4 = MENU_fileInit(g1_Data4, 1, 1.8, "servo-P", 5, datafloat, &g1_Data3, &g1_Data5, NULL, NULL);
    g1_Data5 = MENU_fileInit(g1_Data5, 1, 6.0, "servo-D", 6, datafloat, &g1_Data4, &g1_Data6, NULL, NULL);
    g1_Data6 = MENU_fileInit(g1_Data6, 1, 0.9, "GAP1", 7, datafloat, &g1_Data5, &g1_Data7, NULL, NULL);
    g1_Data7 = MENU_fileInit(g1_Data7, 1, 1.3, "motor-I", 2, datafloat, &g1_Data6, &preGear1, NULL, NULL);

    gear2 = MENU_fileInit(gear2, 1, 1.0, "GearFast", 3, none, &gear1, &gear3, NULL, &g2_Data1);//快速档
    g2_Data1 = MENU_fileInit(g2_Data1, 85, 38.27, "speed2", 2, dataint, NULL, &g2_Data2, &gear2, NULL);
    g2_Data2 = MENU_fileInit(g2_Data2, 140, 75.32, "THRE2", 3, dataint, &g2_Data1, &g2_Data3, NULL, NULL);
    g2_Data3 = MENU_fileInit(g2_Data3, 30, 11.62, "Vision2", 4, dataint, &g2_Data2, &g2_Data4, NULL, NULL);
    g2_Data4 = MENU_fileInit(g2_Data4, 119, 1.9, "servo-P", 5, datafloat, &g2_Data3, &g2_Data5, NULL, NULL);
    g2_Data5 = MENU_fileInit(g2_Data5, 119, 6.0, "servo-D", 6, datafloat, &g2_Data4, &g2_Data6, NULL, NULL);
    g2_Data6 = MENU_fileInit(g2_Data6, 119, 0.9, "GAP2", 7, datafloat, &g2_Data5, &g2_Data7, NULL, NULL);
    g2_Data7 = MENU_fileInit(g2_Data7, 119, 1.1, "motor-I", 2, datafloat, &g2_Data6, &preGear2, NULL, NULL);

    gear3 = MENU_fileInit(gear3, 1, 1.0, "GearFast2", 4, none, &gear2, &currentK, NULL, &g3_Data1);//快速档，含g2四个数据
    g3_Data1 = MENU_fileInit(g3_Data1, 90, 38.27, "speed3", 2, dataint, NULL, &g3_Data2, &gear3, NULL);
    g3_Data2 = MENU_fileInit(g3_Data2, 140, 75.32, "THRE3", 3, dataint, &g3_Data1, &g3_Data3, NULL, NULL);
    g3_Data3 = MENU_fileInit(g3_Data3, 29, 11.62, "Vision3", 4, dataint, &g3_Data2, &g3_Data4, NULL, NULL);
    g3_Data4 = MENU_fileInit(g3_Data4, 119, 1.9, "servo-P", 5, datafloat, &g3_Data3, &g3_Data5, NULL, NULL);
    g3_Data5 = MENU_fileInit(g3_Data5, 119, 6.8, "servo-D", 6, datafloat, &g3_Data4, &g3_Data6, NULL, NULL);
    g3_Data6 = MENU_fileInit(g3_Data6, 119, 0.9, "GAP3", 7, datafloat, &g3_Data5, &g3_Data7, NULL, NULL);
    g3_Data7 = MENU_fileInit(g3_Data7, 119, 1.1, "motor-I", 2, datafloat, &g3_Data6, &preGear3, NULL, NULL);

    currentK = MENU_fileInit(currentK, 1, 1.0, "currentK", 5, none, &gear3, NULL, NULL, &currentRTKP);
    currentRTKP = MENU_fileInit(currentRTKP, 3, 4.0, "curRTKP", 2, datafloat, NULL, &currentRTKI, &currentK, NULL);
    currentRTKI = MENU_fileInit(currentRTKI, 2, 2.0, "curRTKI", 3, datafloat, &currentRTKP, &currentLFKP, NULL, NULL);
    currentLFKP = MENU_fileInit(currentLFKP, 3, 4.0, "curLFKP", 4, datafloat, &currentRTKI, &currentLFKI, &currentK, NULL);
    currentLFKI = MENU_fileInit(currentLFKI, 2, 2.0, "curLFKI", 5, datafloat, &currentLFKP, &expectC, NULL, NULL);
//    currentKD = MENU_fileInit(currentKD, 2, 3.0, "motorKD", 4, datafloat, &currentRTKI, &expectC, NULL, NULL);
    expectC = MENU_fileInit(expectC, 2200, 9.5, "expect", 6, dataint, &currentLFKI, NULL, NULL, NULL);

    /* 当下的电机pwm值（speedL/R）和摄像头前瞻vision */
    presentSpeed = MENU_fileInit(presentSpeed, 30, 1.1, "speed", 2, dataint, NULL, &presentTHRE, &file2, NULL);
    presentTHRE = MENU_fileInit(presentTHRE, 160, 2.2, "THRE", 3, dataint, &presentSpeed, &presentVision, NULL, NULL);
    presentVision = MENU_fileInit(presentVision, 80, 3.3, "VISION", 4, dataint, &presentTHRE, &fuzzyPB, NULL, NULL);
    fuzzyPB = MENU_fileInit(fuzzyPB, 1, 2.0, "fuzzyPB", 5, datafloat, &presentVision, &fuzzyPM, NULL, NULL);
    fuzzyPM = MENU_fileInit(fuzzyPB, 1, 1.8, "fuzzyPM", 6, datafloat, &fuzzyPB, &fuzzyPS, NULL, NULL);
    fuzzyPS = MENU_fileInit(fuzzyPB, 1, 1.8, "fuzzyPS", 7, datafloat, &fuzzyPM, &fuzzyZO, NULL, NULL);
    fuzzyZO = MENU_fileInit(fuzzyPB, 1, 1.8, "fuzzyZO", 2, datafloat, &fuzzyPS, &fuzzyNS, NULL, NULL);
    fuzzyNS = MENU_fileInit(fuzzyPB, 1, 1.8, "fuzzyNS", 3, datafloat, &fuzzyZO, &fuzzyNM, NULL, NULL);
    fuzzyNM = MENU_fileInit(fuzzyPB, 1, 1.8, "fuzzyNM", 4, datafloat, &fuzzyNS, &fuzzyNB, NULL, NULL);
    fuzzyNB = MENU_fileInit(fuzzyPB, 1, 1.8, "fuzzyNB", 5, datafloat, &fuzzyNM, &presentServoD, NULL, NULL);
    presentServoD = MENU_fileInit(presentServoD, 1, 3.1, "preServoD", 6, datafloat, &fuzzyNB, &gap, NULL, NULL);
    gap = MENU_fileInit(presentMotorP, 1, 0.9, "GAP", 7, datafloat, &presentServoD, &bottomData, NULL, NULL);
//    presentMotorI = MENU_fileInit(presentMotorI, 1, 190.0, "preMotorI", 2, datafloat, &gap, &bottomData, NULL, NULL);
//    presentMotorP = MENU_fileInit(gap, 8, 250, "preMotorP", 3, datafloat, &presentMotorI, &bottomData, NULL, NULL);
    bottomData = MENU_fileInit(bottomData, 1, 1.0, "bottom", 2, none, &gap, NULL, NULL, NULL);
    preGear1 = MENU_fileInit(preGear1, 1, 1.0, "Present1", 3, none, &g1_Data7, NULL, NULL, &presentSpeed);
    preGear2 = MENU_fileInit(preGear2, 1, 1.0, "Present2", 3, none, &g2_Data7, NULL, NULL, &presentSpeed);
    preGear3 = MENU_fileInit(preGear3, 1, 1.0, "Present3", 3, none, &g3_Data7, NULL, NULL, &presentSpeed);

    /*展示翻页效果*/
    display = MENU_fileInit(display, 1, 1.0, "motor", 5, none, &file3, &fileSave, NULL, &display1);
    display1 = MENU_fileInit(display1, 50, 133.03, "LFKP", 2, dataint, NULL, &display2, &display, NULL);
    display2 = MENU_fileInit(display2, 40, 33.71, "LFKI", 3, dataint, &display1, &display3, NULL, NULL);
    display3 = MENU_fileInit(display3, 60, 2, "RTKP", 4, dataint, &display2, &display4, NULL, NULL);
    display4 = MENU_fileInit(display4, 50, 1, "RTKI", 5, dataint, &display3, &display5, NULL, NULL);
    display5 = MENU_fileInit(display5, 5, 2.701, "delaytime", 6, dataint, &display4, &display6, NULL, NULL);
    display6 = MENU_fileInit(display6, 2, 91.881, "round", 7, dataint, &display5, &display7, NULL, NULL);
    display7 = MENU_fileInit(display7, 100, 1.1, "SPEEDUP", 2, dataint, &display6, &display8, NULL, NULL);
    display8 = MENU_fileInit(display8, 55, 7.91, "count1", 3, dataint, &display7, &display9, NULL, NULL);
    display9 = MENU_fileInit(display9, 14, 9.02, "count2", 4, dataint, &display8, NULL, NULL, NULL);

    //数据写入flash
    fileSave = MENU_fileInit(fileSave, 1, 1.0, "SAVE", 6, none, &display, NULL, NULL, &saveGear1);
    saveGear1 = MENU_fileInit(saveGear1, 1, 1.0, "SaveGear1", 2, none, NULL, &saveGear2, &fileSave, &g1_Data1);
    saveGear2 = MENU_fileInit(saveGear2, 1, 1.0, "SaveGear2", 3, none, &saveGear1, &saveGear3, NULL, &g2_Data1);
    saveGear3 = MENU_fileInit(saveGear3, 1, 1.0, "SaveGear3", 4, none, &saveGear2, NULL, NULL, &g3_Data1);

    //摄像头显示在oled上
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

    if(GPIO_Read(P11, 3))
    {
        SmartCar_OLED_Fill(0);
        MENU_showIMG();
    }
    else if(!GPIO_Read(P11, 3)){
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

            tempInt = temp->intValue;
            tempFloat = temp->floatValue;

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
                            temp->intValue += 1 * times;
                        }
                        else if(!GPIO_Read(P11, 9))//值减小 下
                        {
                            SmartCar_OLED_Fill(0);
                            temp->intValue -= 1 * times;
                        }
                        else if(!GPIO_Read(P11, 10))//位数向左 左
                        {
                            SmartCar_OLED_Fill(0);
                            times = times * 10;
                            //intCursor -= 5;

                            SmartCar_OLED_Printf6x8(55, 3, "%2d", temp->intValue);
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
                            SmartCar_OLED_Printf6x8(55, 3, "%2d", temp->intValue);

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
                        SmartCar_OLED_Printf6x8(55, 3, "%2d", temp->intValue);
                    }

                    else if(temp->i == datafloat)
                    {

                        if(!GPIO_Read(P11, 2))//值增加 上
                        {
                            SmartCar_OLED_Fill(0);
                            temp->floatValue += 0.1 * times;
                        }
                        else if(!GPIO_Read(P11, 9))//值减小 下
                        {
                            SmartCar_OLED_Fill(0);
                            temp->floatValue -= 0.1 * times;
                        }
                        else if(!GPIO_Read(P11, 10))//位数向左 左
                        {
                            SmartCar_OLED_Fill(0);
                            times = times * 10;

                            SmartCar_OLED_Printf6x8(40, 4, "%0.3f", tempFloat);
                            SmartCar_OLED_Printf6x8(40, 3, "%0.3f", temp->floatValue);

                        }
                        else if (!GPIO_Read(P11, 11))//位数向右 右
                        {
                            SmartCar_OLED_Fill(0);
                            times = times / 10.0;

                            SmartCar_OLED_Printf6x8(40, 4, "%0.3f", tempFloat);
                            SmartCar_OLED_Printf6x8(40, 3, "%0.3f", temp->floatValue);

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
                        SmartCar_OLED_Printf6x8(40, 3, "%3.3f", temp->floatValue);
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


void MENU_sectorSave(uint32 sectorNum, nodeptr_t gearData)//按扇区写入
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
            if(!GPIO_Read(P11, 10))
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
