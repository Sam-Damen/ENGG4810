/*
 * I2C.h
 *
 *  Created on: 15/04/2014
 *      Author: Sam
 */

#ifndef I2C_H_
#define I2C_H_

//*****************************************************************************
//
// Defines
//
//*****************************************************************************

#define ACCEL_ADDR 		0x1D
#define PRESS_ADDR		0x60
#define CTRL_REG 		0x2A
#define WHO_AM_I		0x0D
#define XYZ_CONFIG		0x0E

//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************

extern void Configure_I2C(void);
extern unsigned long I2CBusScan(unsigned long ulI2CBase);
extern uint32_t I2CRead(uint8_t ADDR, uint8_t REGADDR);
extern void I2CWrite(uint8_t deviceAdd, uint8_t regAdd, uint8_t data);
extern void MMA8452QSetup(void);



#endif /* I2C_H_ */
