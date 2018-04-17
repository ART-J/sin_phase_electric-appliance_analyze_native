/*
 * adc.c
 *
 *  Created on: 2017年6月29日
 *      Author: redchenjs
 */
#include "driverlib.h"
#include "device.h"

#include "fpu_vector.h"

#include "inc/device/epwm.h"
//
// Globals
//
int16_t adc1_results[2048];   // Buffer for results
int16_t adc2_results[2048];   // Buffer for results
//int16_t adc3_results[2048];   // Buffer for results
//int16_t adc4_results[2048];   // Buffer for results

uint16_t adc1_index;                          // Index into result buffer
volatile uint16_t adc1_ready;                // Flag to indicate buffer is full
volatile uint16_t adc1_buffer_read;

uint16_t adc2_index;                          // Index into result buffer
volatile uint16_t adc2_ready;                // Flag to indicate buffer is full
volatile uint16_t adc2_buffer_read;

//uint16_t adc3_index;                          // Index into result buffer
//volatile uint16_t adc3_ready;                // Flag to indicate buffer is full
//volatile uint16_t adc3_buffer_read;
//
//uint16_t adc4_index;                          // Index into result buffer
//volatile uint16_t adc4_ready;                // Flag to indicate buffer is full
//volatile uint16_t adc4_buffer_read;

//
// Function Prototypes
//
__interrupt void adc1_isr(void);
__interrupt void adc2_isr(void);
//__interrupt void adc3_isr(void);
//__interrupt void adc4_isr(void);
//
// Main
//
void adc1_init(void)
{
    Interrupt_register(INT_ADCC1, &adc1_isr);

    //
    // Set up the ADC and the ePWM and initialize the SOC
    //
    //
    // Set ADCCLK divider to /4
    //

    ADC_setPrescaler(ADCC_BASE, ADC_CLK_DIV_4_0);

    //
    // Set resolution and signal mode (see #defines above) and load
    // corresponding trims.
    //
    ADC_setMode(ADCC_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_SINGLE_ENDED);

    //
    // Set pulse positions to late
    //
    ADC_setInterruptPulseMode(ADCC_BASE, ADC_PULSE_END_OF_CONV);

    //
    // Power up the ADC and then delay for 1 ms
    //
    ADC_enableConverter(ADCC_BASE);
    DEVICE_DELAY_US(1000);

    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN5, 64);

    //
    // Set SOC0 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCC_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0); //中断源设置
    ADC_enableInterrupt(ADCC_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);

    //
    // Initialize results buffer
    //
    for (adc1_index = 0; adc1_index < 2048; adc1_index++) {
        adc1_results[adc1_index] = 0;
    }

    adc1_index = 0;
    adc1_ready = 1;
    adc1_buffer_read = 1;
}

void adc2_init(void)
{
    Interrupt_register(INT_ADCD1, &adc2_isr);

    //
    // Set up the ADC and the ePWM and initialize the SOC
    //
    //
    // Set ADCCLK divider to /4
    //

    ADC_setPrescaler(ADCD_BASE, ADC_CLK_DIV_4_0);

    //
    // Set resolution and signal mode (see #defines above) and load
    // corresponding trims.
    //
    ADC_setMode(ADCD_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);  //设置差分采样方式

    //
    // Set pulse positions to late
    //
    ADC_setInterruptPulseMode(ADCD_BASE, ADC_PULSE_END_OF_CONV);

    //
    // Power up the ADC and then delay for 1 ms
    //
    ADC_enableConverter(ADCD_BASE);
    DEVICE_DELAY_US(1000);

    ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,   //设置为EPWM触发采样
                 ADC_CH_ADCIN4_ADCIN5, 64);

    //
    // Set SOC0 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCD_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    ADC_enableInterrupt(ADCD_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);

    //
    // Initialize results buffer
    //
    for (adc2_index = 0; adc2_index < 2048; adc2_index++) {
        adc2_results[adc2_index] = 0;
    }

    adc2_index = 0;
    adc2_ready = 1;
    adc2_buffer_read = 1;
}

//void adc3_init(void)
//{
//    Interrupt_register(INT_ADCA1, &adc3_isr);
//
//    //
//    // Set up the ADC and the ePWM and initialize the SOC
//    //
//    //
//    // Set ADCCLK divider to /4
//    //
//
//    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
//
//    //
//    // Set resolution and signal mode (see #defines above) and load
//    // corresponding trims.
//    //
//    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_SINGLE_ENDED);
//
//    //
//    // Set pulse positions to late
//    //
//    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
//
//    //
//    // Power up the ADC and then delay for 1 ms
//    //
//    ADC_enableConverter(ADCA_BASE);
//    DEVICE_DELAY_US(1000);
//
//    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
//                 ADC_CH_ADCIN5, 64);
//
//    //
//    // Set SOC0 to set the interrupt 1 flag. Enable the interrupt and make
//    // sure its flag is cleared.
//    //
//    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
//    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
//    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
//
//    //
//    // Initialize results buffer
//    //
//    for (adc3_index = 0; adc3_index < 2048; adc3_index++) {
//        adc3_results[adc3_index] = 0;
//    }
//
//    adc3_index = 0;
//    adc3_ready = 1;
//    adc3_buffer_read = 1;
//}
//void adc4_init(void)
//{
//    Interrupt_register(INT_ADCB1, &adc4_isr);
//
//    //
//    // Set up the ADC and the ePWM and initialize the SOC
//    //
//    //
//    // Set ADCCLK divider to /4
//    //
//
//    ADC_setPrescaler(ADCB_BASE, ADC_CLK_DIV_4_0);
//
//    //
//    // Set resolution and signal mode (see #defines above) and load
//    // corresponding trims.
//    //
//    ADC_setMode(ADCB_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_SINGLE_ENDED);
//
//    //
//    // Set pulse positions to late
//    //
//    ADC_setInterruptPulseMode(ADCB_BASE, ADC_PULSE_END_OF_CONV);
//
//    //
//    // Power up the ADC and then delay for 1 ms
//    //
//    ADC_enableConverter(ADCB_BASE);
//    DEVICE_DELAY_US(1000);
//
//    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
//                 ADC_CH_ADCIN5, 64);
//
//    //
//    // Set SOC0 to set the interrupt 1 flag. Enable the interrupt and make
//    // sure its flag is cleared.
//    //
//    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
//    ADC_enableInterrupt(ADCB_BASE, ADC_INT_NUMBER1);
//    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);
//
//    //
//    // Initialize results buffer
//    //
//    for (adc4_index = 0; adc4_index < 2048; adc4_index++) {
//        adc4_results[adc1_index] = 0;
//    }
//
//    adc4_index = 0;
//    adc4_ready = 1;
//    adc4_buffer_read = 1;
//}

void adc1_2_start(void)
{
    if (adc1_ready != 1 || adc2_ready != 1) {      //判忙，若ADC1，ADC2等于0表示上次采集未结束
        return;
    } else {
        adc1_ready = 0;                            //清零标志位
        adc2_ready = 0;
        Interrupt_enable(INT_ADCC1);               //使能ADCC1中断
        Interrupt_enable(INT_ADCD1);               //使能ADCD1中断   (C1,D1为一组差分ADC)
        epwm1_start();                             //使能PWM  触发ADC采样
    }
}

//
// ADC A Interrupt 1 ISR
//
__interrupt void adc1_isr(void)
{
    extern int16_t adc1_buffer_sample[2048];
    //
    // Add the latest result to the buffer
    //
    adc1_results[adc1_index] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0) - 31389;       //差分减半
    adc1_index += 2;
    //
    // Set the bufferFull flag if the buffer is full
    //
    if (2048 <= adc1_index) {
        adc1_index = 0;

        epwm1_stop();
        Interrupt_disable(INT_ADCC1);              //取消ADCC1中断

        if (adc1_buffer_read == 1) {                //？？
            adc1_buffer_read = 0;
            memcpy_fast(adc1_buffer_sample, adc1_results, sizeof(adc1_buffer_sample));
        }
        adc1_ready = 1;                            //采集完毕，标志位置一
    }

    //
    // Clear the interrupt flag and issue ACK
    //
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt void adc2_isr(void)
{
    extern int16_t adc2_buffer_sample[2048];
    //
    // Add the latest result to the buffer
    //
    adc2_results[adc2_index] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER0) - 32768; //差分减半
    adc2_index += 2;
    //
    // Set the bufferFull flag if the buffer is full
    //
    if (2048 <= adc2_index) {      //取满值
        adc2_index = 0;

        epwm1_stop();
        Interrupt_disable(INT_ADCD1);

        if (adc2_buffer_read == 1) {     //adc2_buffer_read初始值为1，若ADC采集完值，且adc2_buffer_read为1则复制adc2_buffer_sample值到 adc2_results数组
            adc2_buffer_read = 0;
            memcpy_fast(adc2_buffer_sample, adc2_results, sizeof(adc2_buffer_sample));
        }
        adc2_ready = 1;
    }

    //
    // Clear the interrupt flag and issue ACK
    //
    ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

//__interrupt void adc3_isr(void)
//{
//    extern int16_t adc3_buffer_sample[2048];
//    //
//    // Add the latest result to the buffer
//    //
//    adc3_results[adc3_index] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0) - 32768;
//    adc3_index += 2;
//    //
//    // Set the bufferFull flag if the buffer is full
//    //
//    if (2048 <= adc3_index) {      //取满值
//        adc2_index = 0;
//
//        epwm1_stop();
//        Interrupt_disable(INT_ADCA1);
//
//        if (adc3_buffer_read == 1) {     //adc2_buffer_read初始值为1，若ADC采集完值，且adc2_buffer_read为1则复制adc2_buffer_sample值到 adc2_results数组
//            adc3_buffer_read = 0;
//            memcpy_fast(adc3_buffer_sample, adc3_results, sizeof(adc3_buffer_sample));
//        }
//        adc3_ready = 1;
//    }
//
//    //
//    // Clear the interrupt flag and issue ACK
//    //
//    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
//    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
//}
//__interrupt void adc4_isr(void)
//{
//    extern int16_t adc4_buffer_sample[2048];
//    //
//    // Add the latest result to the buffer
//    //
//    adc4_results[adc2_index] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0) - 32768;
//    adc4_index += 2;
//    //
//    // Set the bufferFull flag if the buffer is full
//    //
//    if (2048 <= adc4_index) {      //取满值
//        adc4_index = 0;
//
//        epwm1_stop();
//        Interrupt_disable(INT_ADCB1);
//
//        if (adc4_buffer_read == 1) {     //adc2_buffer_read初始值为1，若ADC采集完值，且adc2_buffer_read为1则复制adc2_buffer_sample值到 adc2_results数组
//            adc4_buffer_read = 0;
//            memcpy_fast(adc4_buffer_sample, adc4_results, sizeof(adc4_buffer_sample));
//        }
//        adc4_ready = 1;
//    }
//
//    //
//    // Clear the interrupt flag and issue ACK
//    //
//    ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);
//    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
//}
