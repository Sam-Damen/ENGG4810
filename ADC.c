/*
 * ADC.c
 *
 *  Created on: 15/04/2014
 *      Author: Sam
 */


#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/i2c.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"

#include "ADC.h"


//*****************************************************************************
//
// Initialize the ADC inputs
//
//
//*****************************************************************************
void
Configure_ADC(void)
{

    //
    // Enable the GPIOs and the ADC used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    //
    // Select the external reference for greatest accuracy.
    //
    ROM_ADCReferenceSet(ADC0_BASE, ADC_REF_EXT_3V);

    //
    // Configure the pins which are used as analog inputs.
    //
    ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);


    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);


    ROM_ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
                                                  ADC_CTL_END);
    //
    // Enable the sequence but do not start it yet.
    //
    ROM_ADCSequenceEnable(ADC0_BASE, 3);

    //
    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    //
    ADCIntClear(ADC0_BASE, 3);
}



//*****************************************************************************
//
// Get an ADC Reading
//
//*****************************************************************************
uint32_t
ReadADC(unsigned long ADCBase)
{
	uint32_t ADCVal[1];

	//
	// Trigger ADC, wait, reset
	//
	ADCProcessorTrigger(ADCBase, 3);

	while(!ADCIntStatus(ADCBase, 3, false))
	{
	}

	ADCIntClear(ADCBase, 3);

	ADCSequenceDataGet(ADCBase, 3, ADCVal);

	return ADCVal[0];

}
