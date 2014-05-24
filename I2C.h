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

/*
#define ACCEL_ADDR 		0x19
#define WHO_AM_I		0x0F
#define CTRL_REG 		0x20
#define CTRL_REG4		0x23
#define CLICK_CFG		0x38
#define CLICK_THRESH	0x3A
*/

#define ACCEL_ADDR		0x1D
#define WHO_AM_I		0x00
#define CTRL_REG		0x2D
#define CTRL_RATE		0x2C
#define CTRL_RANGE		0x31
#define CLICK_DUR		0x21
#define CLICK_THRESH	0x1D
#define CLICK_INT		0x2E
#define CLICK_MAP		0x2F
#define CLICK_AXIS		0x2A

#define PRES_ADDR		0x60


//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************

extern void Configure_I2C(void);
extern unsigned long I2CBusScan(unsigned long ulI2CBase);
extern uint32_t I2CRead(uint8_t ADDR, uint8_t REGADDR);
extern void I2CWrite(uint8_t deviceAdd, uint8_t regAdd, uint8_t data);
extern void AccelSetup(void);
extern void pressRead(uint16_t*);
extern void accelRead(int16_t*);


#endif /* I2C_H_ */
