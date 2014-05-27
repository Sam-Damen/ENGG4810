/*
 * UART.c
 *
 *  Created on: 15/04/2014
 *      Author: Sam
 */


#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "Config.h"
#include "LED.h"
#include "UART.h"


extern char SDBuf[100];
extern int GPS_Flag;


//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UARTIntHandler(void)
{
    uint32_t ui32Status;
    static uint32_t i = 0;
    unsigned char c;


    //
    // Get the interrrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART1_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART1_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //

    while(ROM_UARTCharsAvail(UART1_BASE))
    {
        //
        // Read the next character from the UART1 and write it to buffer for printing
        //
    	c = (unsigned char) ROM_UARTCharGetNonBlocking(UART1_BASE);
    	ROM_UARTCharPutNonBlocking(UART0_BASE,c);

    	SDBuf[i] = c;
    	i++;

    	//Got all of the data (GPS appends \r)
    	if (c == '\n') {
    		GPS_Flag = 1;
    		i = 0;

    	}

    }




}


//*****************************************************************************
//
// Configure the UART and its pins.
//
//*****************************************************************************
void
Configure_UART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Enable UART 0 & 1
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
    ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize UART0 & UART1 (8 bit, no parity, 1 stop)
    //
    UARTStdioConfig(0, 115200, 16000000);

#ifdef GPS_EN

    ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 38400,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));


    UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);



    //
    // Enable UART & Master Interrupts
    //
    UARTIntRegister(UART1_BASE, UARTIntHandler);
    ROM_IntEnable(INT_UART1);
    ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

#endif
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
        ROM_UARTCharPut(UART1_BASE, *pui8Buffer++);
    }
}



//*****************************************************************************
//
// Use to perform multiple changes to the GPS module
//
//*****************************************************************************
void
Configure_GPS(uint32_t mode)
{

	switch (mode)
	{

	case HIGH:


		//UARTSend(powerSave, sizeof(powerSave));

		break;

	case MIN:

		break;

	case WAKE:


		break;

	case OFF:
		//Turn off the GPS RF Section

		//Keeps all previous settings
		//No RF so ~5mA

		UARTSend(rfOFF, sizeof(rfOFF));
		ROM_SysCtlDelay(SysCtlClockGet() / 12 );
		break;

	case ON:
		//Turn on the GPS RF Section

		//Call after OFF

		UARTSend(rfON, sizeof(rfON));
		ROM_SysCtlDelay(SysCtlClockGet() / 12 );
		UARTSend(rate1Hz, sizeof(rate1Hz));
		ROM_SysCtlDelay(SysCtlClockGet() / 12 );
		break;

	default:
		//Initial Set-up of GPS

		//1 Sec message Rate
		//No power savings
		//Accurate positioning (pedestrian)

		UARTSend(rate1Hz, sizeof(rate1Hz));
		ROM_SysCtlDelay(SysCtlClockGet() / 12 );
		UARTSend(powerHigh, sizeof(powerHigh));
		ROM_SysCtlDelay(SysCtlClockGet() / 12 );
		UARTSend(pedMode, sizeof(pedMode));
		ROM_SysCtlDelay(SysCtlClockGet() / 12 );
		break;

	}


}



