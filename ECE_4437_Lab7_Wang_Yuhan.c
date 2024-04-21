//*****************************************************************************
//
// uart_echo.c - Example for reading data from and writing data to the UART in
//               an interrupt driven fashion.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "string.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "inc/hw_ints.h"
#include "math.h"
#include "driverlib/fpu.h"
#include "driverlib/timer.h"
#define RED_LED   GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3
int count = 0;
int count1 = 1;
int red = 0;
int green = 0;
int blue = 0;
int red1 = 0;
int green1 = 0;
int blue1 = 0;
int flag = 0;
int flag1 = 0;
int judge;
int i1 = 0;
int j1 = 0;
char cmd1[4];
char cmd2[4];
char pg[4];
char pg1[4];
int duty = 80;
int width = 0;
int load = 0;
int step = 0;
int green_flag = 0;
int yellow_flag = 0;
float distance;
float distance1;
float mean;
float mean1;
int red_flag = 0;
uint32_t ADC0Val[4];
uint32_t ADC1Val[4];
int f1 = 20;
int ratio = 0;
int target = 11;
float e_current;
float e_sum;
float e_previous;
float P = 0.8;
float P_right = 5;
float D = 0.0046;
float I = 0.000001;
float p_value;
float d_value;
float i_value;
float adj;
float max_duty;
int width1; // ”“¬÷
int width2;
int duty1; // ”“¬÷
int duty2;
int PG13 = 0;
float SB;
int right = 0;
int u_turn = 0;
int working = 0;
//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART Echo (uart_echo)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the USB debug virtual serial port on the evaluation board)
//! will be configured in 115,200 baud, 8-n-1 mode.  All characters received on
//! the UART are transmitted back to the UART.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
typedef struct{
	char cmd[4];
	void (*function)(void);
}lookup;

//
//void
//UART0IntHandler(void)
//{
//    uint32_t ui32Status;
//
//    //
//    // Get the interrrupt status.
//    //
//    ui32Status = UARTIntStatus(UART0_BASE, true);
//
//    //
//    // Clear the asserted interrupts.
//    //
//    UARTIntClear(UART0_BASE, ui32Status);
//
//    //GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|GREEN_LED|BLUE_LED, GREEN_LED);
//    //
//    // Loop while there are characters in the receive FIFO.
//    //
//    while(ROM_UARTCharsAvail(UART0_BASE))
//    {
//        //
//        // Read the next character from the UART and write it back to the UART.
//        //
//    	char spj = UARTCharGetNonBlocking(UART0_BASE);
//
//        ROM_UARTCharPutNonBlocking(UART0_BASE,
//                                   spj);
//
//        UARTCharPutNonBlocking(UART1_BASE, spj);
//        pg[count] = spj;
//
//        //
//        // Blink the LED to show a character transfer is occuring.
//        //
//
//        //
//        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
//        //
//        SysCtlDelay(SysCtlClockGet() / (1000 * 3));
//
//        //
//        // Turn off the LED
//        //
//
//
//                //
//                // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
//                //
//        SysCtlDelay(SysCtlClockGet() / (1000 * 3));
//
//                //
//                // Turn off the LED
//                //
//        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
//        count++;
//     }
//}


void ADC0IntHandler(void)
{
	ADCIntClear(ADC0_BASE, 1);
	ADCSequenceDataGet(ADC0_BASE, 1, ADC0Val);
	int i = 0;
	for(i = 0;i<4;i++)
	{
		 mean+= ADC0Val[i];

	}
	mean = mean / 4;

	distance = -1.858203*pow(10,-9)*pow(mean, 3) + 1.3214630459*pow(10, -5)*pow(mean, 2) - 0.03356491643966*mean + 35.536853696418802;

	SB = target - distance;
	if (distance > 13.967)
	{
		right = 1;
	}
	else
	{
		right = 2;
	}

}
void ADC1IntHandler(void)
{
	ADCIntClear(ADC0_BASE, 2);
	ADCSequenceDataGet(ADC0_BASE, 2, ADC1Val);

	int j = 0;
	for (j = 0; j<4;j++)
	{
		mean1+=ADC1Val[j];
	}
	mean1 = mean1/4;
	distance1 = -1.858203*pow(10,-9)*pow(mean1, 3) + 1.3214630459*pow(10, -5)*pow(mean1, 2) - 0.03356491643966*mean1 + 35.536853696418802;
	if(distance1 < 6.3)
	{
		u_turn = 1;
		GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0);
		GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0);
		GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|GREEN_LED|BLUE_LED, RED_LED);

	}
	else if (distance1 > 10.68)
	{
		u_turn = 2;
	}
}
void UTURN(void)
{
	working = 1;
	GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|GREEN_LED|BLUE_LED, RED_LED);

	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_7);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_3);
	duty1 = 78;
	duty2 = 78;
	width1 = load * duty1*0.01;
	width2 = load * duty2*0.01;
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, width1);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, width2);
}
void TURNRIGHT(void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|GREEN_LED|BLUE_LED, BLUE_LED);
    GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0);
	duty1 = 41;
	duty2 = 95;
	width1 = load * duty1*0.01;
	width2 = load * duty2*0.01;

	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, width1);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, width2);
}
void timer_init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	int a = SysCtlClockGet();
	ratio = a / f1 - 1;
	TimerLoadSet(TIMER0_BASE, TIMER_A, ratio);
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	//GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|GREEN_LED|BLUE_LED, RED_LED);
}
void TIMER0IntHandler(void)
{
	TimerIntClear(TIMER0_BASE, 1);
	ADCProcessorTrigger(ADC0_BASE, 1);

	ADCProcessorTrigger(ADC0_BASE, 2);
}
void start()
{
	PWM_init();
	f1 = 20;
	timer_init();
	TimerEnable(TIMER0_BASE, TIMER_A);

}
lookup table1[] = {{"STR", start}};
int len1 = sizeof(table1)/sizeof(table1[0]);
void
UART1IntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = UARTIntStatus(UART1_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART1_BASE, ui32Status);

    //GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|GREEN_LED|BLUE_LED, GREEN_LED);
    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART1_BASE))
    {
    	char george = UARTCharGetNonBlocking(UART1_BASE);
    	ROM_UARTCharPutNonBlocking(UART1_BASE,
    	                           george);

    	UARTCharPutNonBlocking(UART0_BASE, george);
    	pg1[count1-1] = george;
    	int enter = 0;
    	        //
    	        // Blink the LED to show a character transfer is occuring.
    	        //
    	if((count1%3) == 0)
    	{
    	    int i;
    	    pg1[count1] = '\0';
    	    for (i=0; i<len1;i++)
    	    {
    	        strcpy(cmd1, table1[i].cmd);
    	        judge = strncmp(cmd1, pg1, 3);

    	        if(judge == 0)
    	        {
    	        	flag1 = 1;
    	        	enter = 1;
    	        	i1 = i;
    	        	//table1[i1].function();
    	        }

    	     }
    	    if(enter!=1)
    	    {
    	        flag1 = 2;
    	    }

    	}
        // Read the next character from the UART and write it back to the UART.
        //

        //
        // Blink the LED to show a character transfer is occuring.
        //


        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        SysCtlDelay(SysCtlClockGet() / (1000 * 3));

        //
        // Turn off the LED
        //
        //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
        count1 ++;
     }
}
//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
    }
}
void
UART1Send(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        UARTCharPutNonBlocking(UART1_BASE, *pui8Buffer++);
    }
}

//*****************************************************************************
//
// This example demonstrates how to send a string of data to the UART.
//
//*****************************************************************************



void ADC_init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

	ADCSequenceDisable(ADC0_BASE, 1);
	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 1);
	IntMasterEnable();
	IntEnable(INT_ADC0SS1);
	ADCIntEnable(ADC0_BASE, 1);
}
void ADC1_init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_6);

	ADCSequenceDisable(ADC0_BASE, 2);
	ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 2, 2, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 2, 3, ADC_CTL_CH1|ADC_CTL_IE|ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 2);
	IntMasterEnable();
	IntEnable(INT_ADC0SS2);
	ADCIntEnable(ADC0_BASE, 2);
}
void PWM_init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1));
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));

	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	GPIOPinConfigure(GPIO_PB7_M0PWM1);
	GPIOPinConfigure(GPIO_PB6_M0PWM0);

	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);


	int PWM_clock = SysCtlClockGet() / 64;
	int PWM_freq = 55;
	load = PWM_clock / PWM_freq - 1;

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, load);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    width = load * duty*0.01;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, width);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, width);
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    IntMasterEnable();
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_6);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_3);
}
void PID_CONTROL(void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|GREEN_LED|BLUE_LED, GREEN_LED);
    GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0);
	e_current = SB;
	p_value = P * e_current;
	d_value = D*(e_current - e_previous);
	i_value = I * e_sum;
	adj = p_value + d_value + i_value;
	e_sum = e_sum + e_current;
	e_previous = e_current;
	duty1 = duty + adj;
	duty2 = duty - adj;

	if(duty1 > 95)
	{
		duty1 = 95;
	}
	else if (duty1 < 77)
	{
		duty1 = 77;
	}
	else if(duty2 > 95)
	{
		duty2 = 95;
	}
	else if(duty2 <77)
	{
		duty2 = 77;
	}

	width1 = load * duty1*0.01;
	width2 = load * duty2*0.01;

	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, width1);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, width2);
}
int
main(void)
{
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    FPUEnable();
    FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF2).
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    //
    // Enable the peripherals used by this example.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
//    UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
//                                (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
//                                 UART_CONFIG_PAR_NONE));
    UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 9600,
                                (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                 UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);



    ADC_init();
    ADC1_init();
    start();




    //
    // Prompt for text to be entered.
    //
    //GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|GREEN_LED|BLUE_LED, GREEN_LED);

    //
    // Loop forever echoing data through the UART.
    //
    e_sum = e_current;
    e_previous = 0;


    while(1)
    {
    	SysCtlDelay(SysCtlClockGet() / (1000 * 3));
    	if((u_turn == 1) && (working != 1))
    	{
    		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6|GPIO_PIN_7, 0);
    		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3, 0);
    		GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0);
    		GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0);
    		GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|GREEN_LED|BLUE_LED, RED_LED);
    		SysCtlDelay(SysCtlClockGet() / (1000 * 3));
    		UTURN();
    		u_turn = 0;
    	}
    	else if((u_turn == 1) && (working == 1))
    	{
    		GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0);
    		GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0);
    		GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|GREEN_LED|BLUE_LED, RED_LED);
    		UTURN();
    		u_turn = 0;

    	}
    	else if (u_turn == 2 )
    	{
    		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_6);
    		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_3);
    		duty1 = 50;
    		duty2 = 50;
    		width1 = load * duty1*0.01;
    		width2 = load * duty2*0.01;

    		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, width1);
    		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, width2);
    		PID_CONTROL();
    		u_turn = 0;
    	}

    	else if((right == 1) && (u_turn != 1))
    	{
    		TURNRIGHT();
    		right = 0;
    	}
    	else if((right == 2) &&(u_turn !=1))
    	{

    		PID_CONTROL();
    		right = 0;
    	}


    }
}
