/*
 * UART.h
 *
 *  Created on: 15/04/2014
 *      Author: Sam
 */

#ifndef UART_H_
#define UART_H_

//*****************************************************************************
//
// Defines
//
//*****************************************************************************



//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************


extern void Configure_UART(void);
extern void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
extern uint32_t GPSCheckSum(char * data);


#endif /* UART_H_ */
