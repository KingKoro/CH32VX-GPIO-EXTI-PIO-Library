/**
 *  CH32VX GPIO/EXTI Library example
 * 
 *  Example based on: https://github.com/openwch/ch32v20x/tree/main/EVT/EXAM/EXTI/EXTI0
 *  GPIO Library mostly based on: https://github.com/bitbank2/CH32V_Experiments/tree/master
 *
 *  Copyright (c) 2024 Florian Korotschenko aka KingKoro
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 *
 *
 *  file         : main.c
 *  description  : example main code
 *
 */
#if defined(CH32V00X)
#include <ch32v00x.h>
#elif defined(CH32V10X)
#include <ch32v10x.h>
#elif defined(CH32V20X)
#include <ch32v20x.h>
#elif defined(CH32V30X)
#include <ch32v30x.h>
#elif defined(CH32X035) || defined(CH32X033)
#include <ch32x035.h>
#endif

#include "ch32v_gpio.h"
#include "debug.h"

#define TRUE 1
#define FALSE 0

/*********************************************************************
 * @fn      pinB13_interrupt_handler()
 * 
 * @brief   Interrupt Service Routine for Pin B13 event.
 *
 * @return  none
 */
void pinB13_interrupt_handler(void)
{
    // When B13 interrupt, print message and toggle LED state
    printf("Run at EXTI\r\n");
    digitalWrite(0xA0, (1 - digitalRead(0xA0)));
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    // Setup
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("EXTI0 Test\r\n");

    // Enable Interrupt (0xD = 13)
    pinInterrupt(0xBD, INPUT_PULLUP, ENABLE, EXTI_Trigger_Falling, pinB13_interrupt_handler);

    // Set LED pin as output
    pinMode(0xA0, OUTPUT);
    digitalWrite(0xA0, Bit_RESET);

    while(1)
    {
        Delay_Ms(1000);
        printf("Run at main\r\n");
    }
}