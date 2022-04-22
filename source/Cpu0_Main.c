/**********************************************************************************************************************
 * \file Cpu0_Main.c
 * \copyright Copyright (C) Infineon Technologies AG 2019
 *
 * Use of this file is subject to the terms of use agreed between (i) you or the company in which ordinary course of
 * business you are acting and (ii) Infineon Technologies AG or its licensees. If and as long as no such terms of use
 * are agreed, use of this file is subject to following:
 *
 * Boost Software License - Version 1.0 - August 17th, 2003
 *
 * Permission is hereby granted, free of charge, to any person or organization obtaining a copy of the software and
 * accompanying documentation covered by this license (the "Software") to use, reproduce, display, distribute, execute,
 * and transmit the Software, and to prepare derivative works of the Software, and to permit third-parties to whom the
 * Software is furnished to do so, all subject to the following:
 *
 * The copyright notices in the Software and this entire statement, including the above license grant, this restriction
 * and the following disclaimer, must be included in all copies of the Software, in whole or in part, and all
 * derivative works of the Software, unless such copies or derivative works are solely in the form of
 * machine-executable object code generated by a source language processor.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *********************************************************************************************************************/
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include <ifxCpu_Irq.h>
#include "IfxScuWdt.h"
#include "SmartCar_Uart.h"
#include "SmartCar_Upload.h"
#include "SmartCar_Oled.h"
#include "SmartCar_Pwm.h"
#include "SmartCar_MPU.h"
#include "SmartCar_MT9V034.h"

#include "SmartCar_Systick.h"
#include "SmartCar_PIT.h"
#include "SmartCar_ADC.h"
#include "SmartCar_Flash.h"
#include "Image.h"
#include "common.h"
#include "SmartCar_ctrl.h"
#include "menu.h"
#include "switch.h"
#include "SmartCar_GPIO.h"
#include"SmartCar_TFMINI.h"

#pragma section all "cpu0_dsram"
//IfxCpu_syncEvent g_cpuSyncEvent;

uint32 onPower;//上电次数

int core0_main(void)
{
    /** 关闭总中断*/
    IfxCpu_disableInterrupts();
    /** 初始化时钟*/
    get_clk();
    /* !!WATCHDOG0 AND SAFETY WATCHDOG ARE DISABLED HERE!!
     * Enable the watchdogs and service them periodically if it is required
     */
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
    /* Wait for CPU sync event */
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);
    
    /** 初始化OLED*/

    SmartCar_Oled_Init();
    SmartCar_Buffer_Upload(DISP_image_HSIC_lilac);

    MENU_Init();
    CTRL_Init();//控制初始化
    SmartCar_MT9V034_Init();//摄像头初始化，没连摄像头时不能初始化，不然oled就停留在队标页面
    SmartCar_TFMINI_Init();
    GPIO_Init(P22, 0, PUSHPULL, 0);
    GPIO_Init(P00, 8, PUSHPULL, 0);
//    SmartCar_Uart_Init(IfxAsclin3_TX_P15_6_OUT,IfxAsclin3_RXA_P15_7_IN,1152000,3);

    Delay_ms(STM0, 1000);
    SmartCar_OLED_Fill(0);
    /** 开启总中断*/
    IfxCpu_enableInterrupts();

    SmartCar_OLED_Fill(0);
    SmartCar_OLED_Printf6x8(70, 0, "%d", 1);
    SmartCar_OLED_P6x8Str(0, 1, ">> ");
    //namePrintf(filePrint);
    /** 主循环 */


    /*记录上电次数*/
    uint32_t buf1[1024] = { 0 };
    onPower = Page_Read(3, 0, int32_t);
    onPower ++;
    memcpy(buf1, &onPower, sizeof(uint32_t));
    Sector_Erase(3);//将onPower放在第四个扇区
    Sector_Program(3, buf1);

    tempFile = &file1;
    MENU_namePrintf(tempFile);

//    GPIO_Set(P22, 0, 1);


    while(TRUE)
        {

            SmartCar_OLED_Printf6x8(110, 0, "%d", onPower);
            if (!GPIO_Read(P11, 2) || !GPIO_Read(P11, 9) || !GPIO_Read(P11, 10) ||
                    !GPIO_Read(P11, 11) || !GPIO_Read(P11, 12) )
                {
                    Delay_ms(STM0,100);

                    tempFile = MENU_curPosition(tempFile);
                }
//            if(!GPIO_Read(P13, 2) || !GPIO_Read(P11, 3))
//            {
//                Delay_ms(STM0,100);
//                SW_readSwitch();
//            }

            SW_readSwitch();//发车键

//        SmartCar_ImgUpload(fullBuffer,120,188);//传图像给电脑，和uart一起用
//
        }

}

#pragma section all restore
