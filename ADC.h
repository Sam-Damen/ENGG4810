/*
 * ADC.h
 *
 *  Created on: 15/04/2014
 *      Author: Sam
 */

#ifndef ADC_H_
#define ADC_H_

//*****************************************************************************
//
// Defines
//
//*****************************************************************************

#define UV_ADC 		ADC0_BASE
#define TEM_ADC 	ADC1_BASE


//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************


extern void Configure_ADC(void);
extern uint32_t ReadADC(unsigned long ADCBase);



#endif /* ADC_H_ */
