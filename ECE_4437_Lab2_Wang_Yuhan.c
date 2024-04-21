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
void
UART0IntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART0_BASE, ui32Status);

    //GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|GREEN_LED|BLUE_LED, GREEN_LED);
    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
    	char spj = UARTCharGetNonBlocking(UART0_BASE);

        ROM_UARTCharPutNonBlocking(UART0_BASE,
                                   spj);

        UARTCharPutNonBlocking(UART1_BASE, spj);
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

        if((count%3) == 0)

           {

             green = 1;
             blue = 0;
             red = 0;

           }
        else if ((count%3) == 1)
           {
             green = 0;
             blue = 1;
             red = 0;

           }
        else if((count%3) == 2)
           {
             green =0;
             red = 1;
             blue = 0;

           }
                //
                // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
                //
        SysCtlDelay(SysCtlClockGet() / (1000 * 3));

                //
                // Turn off the LED
                //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
        count ++;
     }
}

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
        //
        // Read the next character from the UART and write it back to the UART.
        //
    	char george = UARTCharGetNonBlocking(UART1_BASE);
        ROM_UARTCharPutNonBlocking(UART1_BASE,
                                   george);

        UARTCharPutNonBlocking(UART0_BASE, george);

        //
        // Blink the LED to show a character transfer is occuring.
        //
        if((count1%3) == 0)
        {
        	green1 = 1;
        	blue1 = 0;
        	red1 = 0;

        }
        else if ((count1%3) == 1)
        {
        	green1 = 0;
        	blue1 = 1;
        	red1 = 0;

        }
        else if((count1%3) == 2)
        {
        	green1 =0;
        	red1 = 1;
        	blue1 = 0;

        }
        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        SysCtlDelay(SysCtlClockGet() / (1000 * 3));

        //
        // Turn off the LED
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
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
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
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
    UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
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

    //
    // Prompt for text to be entered.
    //
    GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|GREEN_LED|BLUE_LED, GREEN_LED);
    char str1[] = "Please enter characters in the remote Bluetooth Serial Terminal and see the color change:";

    int x = strlen(str1),iter = 0;
    for(iter;iter<x;iter++)
    {
    	UARTSend((uint8_t *)(str1+iter),1);
    	UART1Send((uint8_t *)(str1+iter),1);
    	SysCtlDelay(SysCtlClockGet() / (10 * 3));
    }

    //
    // Loop forever echoing data through the UART.
    //
    while(1)
    {
    	if (green == 1)
    	{
    		GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|GREEN_LED|BLUE_LED, GREEN_LED);
    	}
    	else if(red == 1)
    	{
    		GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|GREEN_LED|BLUE_LED, RED_LED);
    	}
    	else if(blue == 1)
    	{
    		GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|GREEN_LED|BLUE_LED, BLUE_LED);
    	}
    	else if(green1==1)
    	{
    		GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|GREEN_LED|BLUE_LED, GREEN_LED);
    	}
    	else if(red1 == 1)
    	{
    		GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|GREEN_LED|BLUE_LED, RED_LED);
    	}
    	else if(blue1 == 1)
    	{
    		GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|GREEN_LED|BLUE_LED, BLUE_LED);
    	}


    }
}
