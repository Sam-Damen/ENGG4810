/*
 * I2C.c
 *
 *  Created on: 15/04/2014
 *      Author: Sam
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "I2C.h"



//*****************************************************************************
//
// Configure I2C module
//
//*****************************************************************************

void
Configure_I2C(void)
{

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	ROM_GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	ROM_GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    ROM_GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    ROM_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    //
    // Data rate of 100kbps
    //
    I2CMasterInitExpClk(I2C0_BASE, ROM_SysCtlClockGet(), false);

    //
    // Enable the Master
    //
    I2CMasterEnable(I2C0_BASE);

    //
    // Perform search of all addresses on the line
	//
    //I2CBusScan(I2C0_BASE);

}

//*****************************************************************************
//
// Probe the I2C line for slave addresses
//
//*****************************************************************************

unsigned long
I2CBusScan(unsigned long ulI2CBase)
{
	unsigned char ucProbeAdress;
	unsigned long ucerrorstate;

	//
	// Wait until master module is done transferring.
	//
	while(ROM_I2CMasterBusy(ulI2CBase))
	{
	};

	//
	// I2C Addresses are 7-bit values
	// probe the address range of 0 to 127 to find I2C slave devices on the bus
	//
	for (ucProbeAdress = 0; ucProbeAdress < 127; ucProbeAdress++)
	{
		//
		// Tell the master module what address it will place on the bus when
		// writing to the slave.
		//
		ROM_I2CMasterSlaveAddrSet(ulI2CBase, ucProbeAdress, false);
		ROM_SysCtlDelay(50000);

		//
		// Place the command to be sent in the data register.
		//
		ROM_I2CMasterDataPut(ulI2CBase, 0x00);

		//
		// Initiate send of data from the master.
		//
		ROM_I2CMasterControl(ulI2CBase, I2C_MASTER_CMD_BURST_SEND_START);

		//
		// Make some delay
		//
		ROM_SysCtlDelay(500);

		//
		// Read the I2C Master Control/Status (I2CMCS) Register to a local
		// variable
		//
		ucerrorstate = ROM_I2CMasterErr(ulI2CBase);


		//
		// Examining the content I2C Master Control/Status (I2CMCS) Register
		// to see if the ADRACK-Bit (Acknowledge Address) is TRUE (1)
		// ( 1: The transmitted address was not acknowledged by the slave)
		//
		if(ucerrorstate & I2C_MASTER_ERR_ADDR_ACK)
		{

			//
			// device at selected address did not acknowledge --> there's no device
			// with this address present on the I2C bus
		}

		else
		{
			//
			// device at selected address acknowledged --> there is a device
			// with this address present on the I2C bus

			UARTprintf("Address found: 0x%2x - %3d\n",ucProbeAdress,ucProbeAdress);

			//
			// Make some delay
			//
			//ROM_SysCtlDelay(1500000);
		}
	}

	//
	// End transfer of data from the master.
	//
	ROM_I2CMasterControl(ulI2CBase, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

	//
	// Print a message to Stdio
	//
	UARTprintf("I2C Bus-Scan done...\n");

	//
	// Return 1 if there is no error.
	//
	return 1;
}



//*****************************************************************************
//
// Read from an I2C device in Single Read mode (probs change for lis3dh)
//
//*****************************************************************************

uint32_t
I2CRead(uint8_t ADDR, uint8_t REGADDR)
{
	uint8_t I2CDataRx;

    //
    // Set the device on the i2c bus, false = write, true = read from slave
    //
    ROM_I2CMasterSlaveAddrSet(I2C0_BASE, ADDR, false);

	//
	//Set the Required register address
	//
    ROM_I2CMasterDataPut(I2C0_BASE, REGADDR);

    //
	// Initiate the send of data  (r/w bit/ addr of slave)
	//
	ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);


    while(ROM_I2CMasterBusy(I2C0_BASE))
    {
    }

    //
    // Set to read mode
    //
    ROM_I2CMasterSlaveAddrSet(I2C0_BASE, ADDR, true);

    //I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

	ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);


    while(ROM_I2CMasterBusy(I2C0_BASE))
    {
    }

	//
	// Get Data From Slave
	//
	I2CDataRx = ROM_I2CMasterDataGet(I2C0_BASE);

    //I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);


	return I2CDataRx;
}


//*****************************************************************************
//
// Write to an I2C device in Single Write mode
//
//*****************************************************************************
void
I2CWrite(uint8_t deviceAdd, uint8_t regAdd, uint8_t data)
{

    //
    // Set the device on the i2c bus, false = write, true = read from slave
    //
    ROM_I2CMasterSlaveAddrSet(I2C0_BASE, deviceAdd, false);

	//
	//Set the Required register address
	//
    ROM_I2CMasterDataPut(I2C0_BASE, regAdd);

    //
	// Initiate the send of data  (r/w bit/ addr of slave)
	//
	ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    while(ROM_I2CMasterBusy(I2C0_BASE))
    {
    }

	//
	//Send the data
	//
    ROM_I2CMasterDataPut(I2C0_BASE, data);

    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);


    while(ROM_I2CMasterBusy(I2C0_BASE))
    {
    }


}

//*****************************************************************************
//
// Configure the Accelerometer
//
//*****************************************************************************
void
AccelSetup(void)
{
    uint8_t ctrl;

	if (I2CRead(ACCEL_ADDR, WHO_AM_I) != 0xE5 ) {
	   	 UARTprintf("Could not Connect\r\n");
	}


	//
	// Get Control Register Data
	//
	ctrl = I2CRead(ACCEL_ADDR,CTRL_REG);


	//
	// Setup Click detection (all axis), INT1
	//
	I2CWrite(ACCEL_ADDR, CLICK_MAP, 0xBF);
	I2CWrite(ACCEL_ADDR, CLICK_AXIS, 0x07);
	I2CWrite(ACCEL_ADDR, CLICK_THRESH, 0x38);	//adjust for sensitivity
	I2CWrite(ACCEL_ADDR, CLICK_DUR, 0x20);
	I2CWrite(ACCEL_ADDR, CLICK_INT, 0xC0);



	//
	// Enable Device  & +- 16g & slower output rate
	//
	I2CWrite(ACCEL_ADDR, CTRL_RATE, 0x08);
	I2CWrite(ACCEL_ADDR, CTRL_RANGE, 0x0B);
	I2CWrite(ACCEL_ADDR, CTRL_REG, ctrl  | (0x08) );


	ROM_SysCtlDelay(ROM_SysCtlClockGet() / 12 );

}

//*****************************************************************************
//
// Read from the Pressure Sensor
//
//*****************************************************************************

void
pressRead(uint16_t* data)
{
	//Temp and Press Readings from sensor
	uint32_t Padc_MSB, Padc_LSB, Tadc_MSB, Tadc_LSB;

	//Tell Pressure Sensor to Get Data
	I2CWrite(PRES_ADDR, 0x12, 0x00);

	ROM_SysCtlDelay(ROM_SysCtlClockGet() / 6 );

	//Read the values
	Padc_MSB =  I2CRead(PRES_ADDR, 0x00);
	Padc_LSB =  I2CRead(PRES_ADDR, 0x01);
	Tadc_MSB =  I2CRead(PRES_ADDR, 0x02);
	Tadc_LSB =  I2CRead(PRES_ADDR, 0x03);


	//Combine MSB and LSB to full reading
	data[0] = ((Padc_MSB << 8) + Padc_LSB) >> 6;
	data[1] = ((Tadc_MSB << 8) + Tadc_LSB) >> 6;


/*

 	//Coeffcients
	uint32_t a0_MSB, a0_LSB, b1_MSB, b1_LSB, b2_MSB, b2_LSB, c12_MSB, c12_LSB;

	//Combined Coeffcients
	uint32_t aa0, bb1, bb2, cc12;

	a0_MSB =  I2CRead(PRES_ADDR, 0x04);
	a0_LSB =  I2CRead(PRES_ADDR, 0x05);
	b1_MSB =  I2CRead(PRES_ADDR, 0x06);
	b1_LSB =  I2CRead(PRES_ADDR, 0x07);
	b2_MSB =  I2CRead(PRES_ADDR, 0x08);
	b2_LSB =  I2CRead(PRES_ADDR, 0x09);
	c12_MSB =  I2CRead(PRES_ADDR, 0x0A);
	c12_LSB =  I2CRead(PRES_ADDR, 0x0B);

	aa0 = ((a0_MSB  << 8) + a0_LSB);
    bb1 = ((b1_MSB << 8) + b1_LSB);
    bb2 = ((b2_MSB << 8) + b2_LSB);
	cc12 = ((c12_MSB << 8) + c12_LSB);

	UARTprintf("a0 - %d\n",aa0);
	UARTprintf("b1 - %d\n",bb1);
	UARTprintf("b2 - %d\n",bb2);
	UARTprintf("c12 - %d\n",cc12);

*/

}

//*****************************************************************************
//
// Read from the Accelerometer
//
//*****************************************************************************

void
accelRead(int16_t* data)
{

	int16_t X_L, X_H, Y_L, Y_H, Z_H, Z_L;

    X_L = I2CRead(ACCEL_ADDR, 0x32);
    X_H = I2CRead(ACCEL_ADDR, 0x33);
    Y_L = I2CRead(ACCEL_ADDR, 0x34);
    Y_H = I2CRead(ACCEL_ADDR, 0x35);
    Z_L = I2CRead(ACCEL_ADDR, 0x36);
    Z_H = I2CRead(ACCEL_ADDR, 0x37);

    data[0] = (X_H << 8) | X_L;
    data[1] = (Y_H << 8) | Y_L;
    data[2] = (Z_H << 8) | Z_L;

}




