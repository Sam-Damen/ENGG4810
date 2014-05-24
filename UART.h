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
// GPS Constants
//
//*****************************************************************************


    static const uint8_t powerSave[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92 };
    static const uint8_t powerHigh[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91 };
    static const uint8_t rate1Hz[] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xD0, 0x07, 0x01, 0x00, 0x01, 0x00, 0xED, 0xBD };
    static const uint8_t rate1min[] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x60, 0xEA, 0x01, 0x00, 0x01, 0x00, 0x60, 0x8C };


//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************


extern void Configure_UART(void);
extern void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
extern uint32_t GPSCheckSum(char * data);


#endif /* UART_H_ */
