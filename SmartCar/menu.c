/*
 * file_name.c
 *
 *  Created on: 2021年10月20日
 *      Author: 李灏然
 */

#include "menu.h"

node_t file1;
node_t file2, file3;
node_t gear1, g1_Data1, g1_Data2, g1_Data3, g1_Data4;
node_t gear2, g2_Data1, g2_Data2, g2_Data3, g2_Data4;
node_t gear3;
node_t dataEnd;//最后一个数据指向dataEnd
node_t preGear1, preGear2;
node_t presentSpeedL, presentSpeedR, presentVision, present4;//现在的档位参数
node_t display, display1, display2, display3, display4, display5, display6, display7, display8, display9;//展示翻页效果
node_t fileSave, saveGear1, saveGear2;//用于写入flash
node_t image;//显示摄像头图像
node_t list;
nodeptr_t filePrint;
nodeptr_t tempFile;

uint32 y = 1;
uint32 pageNum = 1;
node_t MENU_fileInit(node_t file, int intVal, float floatVal, char name[10], uint8 pos,data i,
                struct node* prior,struct node* next,struct node* backward,struct node* forward)
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
    file2 = MENU_fileInit(file2, 1, 1.0, "PRESENT", 3, none, &file1, &display, NULL, &presentSpeedL);
    //file3 = MENU_fileInit(file3, 1, 1.0, "CONTROL", 4, none, &file2, &display, NULL, NULL);

    gear1 = MENU_fileInit(gear1, 1, 1.0, "GearSlow", 2, none, NULL, &gear2, &file1, &g1_Data1);//慢速档，含g1四个数据
    g1_Data1 = MENU_fileInit(g1_Data1, 1800, 32.3, "speed-L1", 2, dataint, NULL, &g1_Data2, &gear1, NULL);
    g1_Data2 = MENU_fileInit(g1_Data2, 1800, 70.22, "speed-R1", 3, dataint, &g1_Data1, &g1_Data3, NULL, NULL);
    g1_Data3 = MENU_fileInit(g1_Data3, 55, 42.09, "Vision1", 4, dataint, &g1_Data2, &g1_Data4, NULL, NULL);
    g1_Data4 = MENU_fileInit(g1_Data4, 19, 28.16, "data4", 5, dataint, &g1_Data3, &preGear1, NULL, NULL);

    gear2 = MENU_fileInit(gear2, 1, 1.0, "GearFast", 3, none, &gear1, NULL, NULL, &g2_Data1);//快速档，含g2四个数据
    g2_Data1 = MENU_fileInit(g2_Data1, 2300, 38.27, "speed-L2", 2, datafloat, NULL, &g2_Data2, &gear2, NULL);
    g2_Data2 = MENU_fileInit(g2_Data2, 2300, 75.32, "speed-R2", 3, dataint, &g2_Data1, &g2_Data3, NULL, NULL);
    g2_Data3 = MENU_fileInit(g2_Data3, 45, 11.62, "Vision2", 4, datafloat, &g2_Data2, &g2_Data4, NULL, NULL);
    g2_Data4 = MENU_fileInit(g2_Data4, 119, 48.01, "data4", 5, dataint, &g2_Data3, &preGear2, NULL, NULL);

    /* 当下的电机pwm值（speedL/R）和摄像头前瞻vision */
    presentSpeedL = MENU_fileInit(presentSpeedL, 3000, 1.1, "speed-L", 2, dataint, NULL, &presentSpeedR, &file2, NULL);
    presentSpeedR = MENU_fileInit(presentSpeedR, 3000, 2.2, "speed-R", 3, dataint, &presentSpeedL, &presentVision, NULL, NULL);
    presentVision = MENU_fileInit(presentVision, 50, 3.3, "VISION", 4, dataint, &presentSpeedR, &present4, NULL, NULL);
    present4 = MENU_fileInit(present4, 4, 4.4, "preData4", 5, dataint, &presentVision, NULL, NULL, NULL);
    preGear1 = MENU_fileInit(preGear1, 1, 1.0, "Present1", 6, none, &g1_Data4, NULL, NULL, &presentSpeedL);
    preGear2 = MENU_fileInit(preGear2, 1, 1.0, "Present2", 6, none, &g2_Data4, NULL, NULL, &presentSpeedL);

    /*展示翻页效果*/
    display = MENU_fileInit(display, 1, 1.0, "DISPLAY", 4, none, &file2, &fileSave, NULL, &display1);
    display1 = MENU_fileInit(display1, 91, 133.03, "display1", 2, dataint, NULL, &display2, &display, NULL);
    display2 = MENU_fileInit(display2, 210, 33.71, "display2", 3, datafloat, &display1, &display3, NULL, NULL);
    display3 = MENU_fileInit(display3, 25, 37.11, "display3", 4, datafloat, &display2, &display4, NULL, NULL);
    display4 = MENU_fileInit(display4, 20, 63.29, "display4", 5, dataint, &display3, &display5, NULL, NULL);
    display5 = MENU_fileInit(display5, 87, 2.701, "display5", 6, datafloat, &display4, &display6, NULL, NULL);
    display6 = MENU_fileInit(display6, 261, 91.881, "display6", 7, datafloat, &display5, &display7, NULL, NULL);
    display7 = MENU_fileInit(display7, 22, 33.71, "display7", 2, dataint, &display6, &display8, NULL, NULL);
    display8 = MENU_fileInit(display8, 8, 7.91, "display8", 3, datafloat, &display7, &display9, NULL, NULL);
    display9 = MENU_fileInit(display9, 120, 9.02, "display9", 4, dataint, &display8, NULL, NULL, NULL);

    //数据写入flash
    fileSave = MENU_fileInit(fileSave, 1, 1.0, "SAVE", 5, none, &display, &image, NULL, &saveGear1);
    saveGear1 = MENU_fileInit(saveGear1, 1, 1.0, "SaveGear1", 2, none, NULL, &saveGear2, &fileSave, &g1_Data1);
    saveGear2 = MENU_fileInit(saveGear2, 1, 1.0, "SaveGear2", 3, none, &saveGear1, NULL, NULL, &g2_Data1);

    //摄像头显示在oled上
    image = MENU_fileInit(image, 1, 1.0, "IMAGE", 6, none, &fileSave, NULL, NULL, NULL);
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
    if(y > 6 && y < 11)
    {
        flag = 5;
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
    if(y > 6 && y < 11)
    {
        flag1 = 5;
        while((flag1--) && (temp->next != NULL))
        {
            temp = temp->next;
        }
    }
    uint32 count1=0;
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

    if (!GPIO_Read(P11, 12))//上
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
        else if((strcmp(temp->name, "IMAGE")) == 0)
        {
            SmartCar_OLED_Fill(0);
            MENU_showIMG();
        }
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


    else if (!GPIO_Read(P11, 6))//ok
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
                if (!GPIO_Read(P11, 6) || !GPIO_Read(P11, 9) || !GPIO_Read(P11, 10) ||
                                   !GPIO_Read(P11, 11) || !GPIO_Read(P11, 12))
                {
                    Delay_ms(STM0,100);

                    if(temp->i == dataint)/*改变整型数据*/
                    {

                        if(!GPIO_Read(P11, 12))//值增加 上
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
                        else if(!GPIO_Read(P11, 6))//确认 ok并回到上一级
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

                        if(!GPIO_Read(P11, 12))//值增加 上
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
                        else if(!GPIO_Read(P11, 6))//确认 ok并回到上一级
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
                    if((strcmp(temp->name, "Gear1")) == 0)/*读取flash中gear1的数据*/
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
                    else if((strcmp(temp->name, "Gear2")) == 0)
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
        if (!GPIO_Read(P11, 6) || !GPIO_Read(P11, 9) || !GPIO_Read(P11, 10) ||
                     !GPIO_Read(P11, 11) || !GPIO_Read(P11, 12) )
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
