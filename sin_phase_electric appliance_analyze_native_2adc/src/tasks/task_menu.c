/*
 * task_menu.c
 *
 *  Created on: 2017-7-25
 *      Author: redchenjs
 */
#include <driverlib.h>
#include <device.h>

#include "inc/device/adc.h"
#include "inc/device/epwm.h"
#include "inc/device/ecap.h"
#include "inc/device/eqep.h"

#include "inc/system/fonts.h"

#include "inc/tasks/task_adc.h"
#include "inc/tasks/task_sci.h"
#include "inc/tasks/task_ecap.h"
#include "inc/tasks/task_eqep.h"
#include "inc/tasks/task_disp.h"
#include "inc/tasks/task_menu.h"
#include "inc/tasks/task_status.h"

const uint16_t menu_item_num = 6;

enum menu_item_t {
    MENU_ITEM_MEASURE   = 0x0,
    MENU_ITEM_SPECTRUM  = 0x1,
    MENU_ITEM_HARMONIC  = 0x2,
    MENU_ITEM_LEARN     = 0x3,
    MENU_ITEM_VIEW      = 0x4,
    MENU_ITEM_RESET     = 0x5
};

uint16_t menu_item    = 0;
uint16_t menu_level   = 0;
uint16_t menu_refresh = 0;

const char menu_item_string[6][24] = {
                                   "Measure      ",
                                   "Spectrum     ",
                                   "Harmonic     ",
                                   "Learn        ",
                                   "View         ",
                                   "Reset        "
};

const int menu_item_fore_color[6] = {White, White, White, White, White, White};
const int menu_item_back_color[6] = {Black, Black, Black, Black, Black, Black};

void menu_run(void)
{
    extern volatile uint16_t adc1_buffer_read;
    extern volatile uint16_t adc2_buffer_read;
    extern volatile uint16_t ecap1_ready;
    extern volatile uint16_t ecap2_ready;
    extern uint32_t eqep1_position;                //编码器按键
    extern uint16_t func_flag;

    eqep1_position_caculate();                     //编码器位置检测

    disp_refresh();                                //显示菜单等级
    switch (menu_level) {                          //菜单等级0:,1:
        case 0:
            menu_item = (uint16_t)eqep1_position % menu_item_num;   //旋转按钮切换菜单项，菜单数为6，eqep1_position累加为清零所以要对6取模。
            disp_menu();                                            //菜单项显示函数，反白表示当前选中项
            disp_time();                                            //显示程序运行时间
            break;
        case 1:
            switch (menu_item) {
                case MENU_ITEM_MEASURE:                            //测量模式。
                    func_flag = 0;
                    adc1_2_start();                                //启动ADC
                    if (adc1_buffer_read == 0) {                    //若读取完毕进入
                        adc1_buffer_read = 1;                       //标志位重置
                        adc1_voltage_caculate();                    //计算均方根值，滤波
                        disp_adc1_voltage();                        //显示电压
                    }
                    if (adc2_buffer_read == 0) {                    //此处采集电流，数据处理同上
                        adc2_buffer_read = 1;
                        adc2_current_caculate();
                        disp_adc2_current();
                    }
                    status_update();                                //状态更新
                    disp_status();                                  //显示状态值

                    sci2_transmit_data();                           //串口数据发送
                    break;
                case MENU_ITEM_SPECTRUM:                            //频谱模式
                    func_flag = 0;
                    adc1_2_start();                                 //启动ADC1,2
//                    if (adc1_buffer_read == 0) {
//                        adc1_buffer_read = 1;
//                        epwm1_stop();
//                        adc1_fft();
//                        adc1_spectrum_caculate();
//                        disp_adc1_spectrum();
//                    }
                    if (adc2_buffer_read == 0) {
                        adc2_buffer_read = 1;
                        adc2_fft();                                 //ADC值做频谱存储
                        adc2_spectrum_caculate();                   //频谱计算
                        disp_adc2_spectrum();                       //***显示频谱***
                    }
                    break;
                case MENU_ITEM_HARMONIC:                            //谐波分析模式
                    func_flag = 0;
                    adc1_2_start();                                 //启动ADC1,2
//                    if (adc1_buffer_read == 0) {
//                        adc1_buffer_read = 1;
//                        epwm1_stop();
//                        adc1_fft();
//                        adc1_spectrum_caculate();
//                        adc1_harmonic_caculate();
//                        disp_adc1_harmonic();
//                    }
                    if (adc2_buffer_read == 0) {
                        adc2_buffer_read = 1;
                        adc2_fft();                                //ADC值做频谱存储
                        adc2_spectrum_caculate();                  //频谱计算
                        adc2_harmonic_caculate();                  //谐波计算
                        disp_adc2_harmonic();                      //谐波显示
                    }
                    break;
                case MENU_ITEM_LEARN:                              //学习模式
                    func_flag = 1;                                 //学习菜单标志
                    adc1_2_start();                                //启动ADC1,2
                    if (adc2_buffer_read == 0) {
                        adc2_buffer_read = 1;
                        adc2_current_caculate();                   //电流计算
//                        adc2_fft();
//                        adc2_spectrum_caculate();
//                        adc2_harmonic_caculate();
                    }
                    status_learn();                                //用电器状态学习
                    break;
                case MENU_ITEM_VIEW:
                    func_flag = 0;
                    status_view();                                 //用电器状态显示
                    break;
                case MENU_ITEM_RESET:                              //用电器状态清空
                    func_flag = 0;
                    status_clear();
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

interrupt void xint1_isr(void)     //按键1进入
{
    extern uint16_t disp_refresh_flag;
    extern uint16_t learn_flag;
    extern uint16_t func_flag;

    if (func_flag && learn_flag == 0) {         //若学习菜单标志位==1，且学习标志位==0
        learn_flag = 1;                         //学习标志位=1
    }

    if (menu_level == MENU_LEVEL_1) {           //菜单切换
        menu_level = MENU_LEVEL_2;
        disp_refresh_flag = 1;                  //刷新界面标志位
    }

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);     //清中断组内所有标志位
}

interrupt void xint2_isr(void)    //按键2返回
{
    extern uint16_t disp_refresh_flag;

    if (menu_level == MENU_LEVEL_2) {
        menu_level = MENU_LEVEL_1;
        disp_refresh_flag = 1;
    }

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
