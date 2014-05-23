/*
 * LED.c
 *
 *  Created on: 21/04/2014
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

#include "examples/boards/ek-tm4c123gxl/drivers/rgb.h"

#include "LED.h"


//*****************************************************************************
//
// Change the RGB Colour/ Blink Rate
//
//*****************************************************************************

void
Configure_RGB(int Col)
{

	uint32_t Colours[3];


	switch(Col) {

	case RED:
		Colours[0] = 0xFFFF;
		Colours[1] = 0x0000;
		Colours[2] = 0x0000;
		RGBColorSet(Colours);
		break;

	case GREEN:
		Colours[0] = 0x0000;
		Colours[1] = 0xFFFF;
		Colours[2] = 0x0000;
		RGBColorSet(Colours);
		break;

	case BLUE:
		Colours[0] = 0x0000;
		Colours[1] = 0x0000;
		Colours[2] = 0xFFFF;
		RGBColorSet(Colours);
		break;

	case YELLOW:
		Colours[0] = 0xFFFF;
		Colours[1] = 0xFFFF;
		Colours[2] = 0x0000;
		RGBColorSet(Colours);
		break;

	default:
		Colours[0] = 0xFFFF;
		Colours[1] = 0xFFFF;
		Colours[2] = 0xFFFF;
		RGBColorSet(Colours);
		break;
	}



}
