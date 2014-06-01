//*****************************************************************************
//
//	ENGG4810 TP2 Firmware
//	Team 22
//
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"

#include "utils/cmdline.h"
#include "utils/uartstdio.h"
#include "fatfs/src/ff.h"
#include "fatfs/src/diskio.h"

#include "examples/boards/ek-tm4c123gxl/drivers/rgb.h"

#include "Config.h"
#include "I2C.h"
#include "ADC.h"
#include "UART.h"
#include "LED.h"


#define GPIO_PB6_M0PWM0         0x00011804

//Variables for Wake on Movement
int ACCEL_Flag = 0;
uint32_t AppState = 0;
int MIN_Flag = 1;
uint32_t minCount = 0;


int highRes = 0;
int highResEnd = 0;
uint32_t sec = 0;
uint32_t secCount = 0;
char GPSData[150] = {0};

uint32_t points[5][4] = {0};
uint32_t point2[4] = {0};
char SDdata[80] = {0};

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


#ifdef SD_EN

//*****************************************************************************
//
// This buffer holds the full path to the current working directory.  Initially
// it is root ("/").
//
//*****************************************************************************
static char Cwd[80] = "/";

//*****************************************************************************
//
// The following are data structures used by FatFs.
//
//*****************************************************************************
static FATFS FatFs;
static DIR DirObj;
static FILINFO FilInfo;
static FIL FilObj;
static FIL FilObj2;

//*****************************************************************************
//
// Write Buffer for SD card
//
//*****************************************************************************
char SDBuf[150] = {0};
int GPS_Flag = 0;

//*****************************************************************************
//
// A structure that holds a mapping between an FRESULT numerical code, and a
// string representation.  FRESULT codes are returned from the FatFs FAT file
// system driver.
//
//*****************************************************************************
typedef struct
{
    FRESULT iFResult;
    char *pcResultStr;
}
tFResultString;

//*****************************************************************************
//
// A macro to make it easy to add result codes to the table.
//
//*****************************************************************************
#define FRESULT_ENTRY(f)        { (f), (#f) }

//*****************************************************************************
//
// A table that holds a mapping between the numerical FRESULT code and it's
// name as a string.  This is used for looking up error codes for printing to
// the console.
//
//*****************************************************************************
tFResultString g_psFResultStrings[] =
{
    FRESULT_ENTRY(FR_OK),
    FRESULT_ENTRY(FR_DISK_ERR),
    FRESULT_ENTRY(FR_INT_ERR),
    FRESULT_ENTRY(FR_NOT_READY),
    FRESULT_ENTRY(FR_NO_FILE),
    FRESULT_ENTRY(FR_NO_PATH),
    FRESULT_ENTRY(FR_INVALID_NAME),
    FRESULT_ENTRY(FR_DENIED),
    FRESULT_ENTRY(FR_EXIST),
    FRESULT_ENTRY(FR_INVALID_OBJECT),
    FRESULT_ENTRY(FR_WRITE_PROTECTED),
    FRESULT_ENTRY(FR_INVALID_DRIVE),
    FRESULT_ENTRY(FR_NOT_ENABLED),
    FRESULT_ENTRY(FR_NO_FILESYSTEM),
    FRESULT_ENTRY(FR_MKFS_ABORTED),
    FRESULT_ENTRY(FR_TIMEOUT),
    FRESULT_ENTRY(FR_LOCKED),
    FRESULT_ENTRY(FR_NOT_ENOUGH_CORE),
    FRESULT_ENTRY(FR_TOO_MANY_OPEN_FILES),
    FRESULT_ENTRY(FR_INVALID_PARAMETER),
};

//*****************************************************************************
//
// A macro that holds the number of result codes.
//
//*****************************************************************************
#define NUM_FRESULT_CODES       (sizeof(g_psFResultStrings) /                 \
                                 sizeof(tFResultString))

//*****************************************************************************
//
// This function returns a string representation of an error code that was
// returned from a function call to FatFs.  It can be used for printing human
// readable error messages.
//
//*****************************************************************************
const char *
StringFromFResult(FRESULT iFResult)
{
    uint_fast8_t ui8Idx;

    //
    // Enter a loop to search the error code table for a matching error code.
    //
    for(ui8Idx = 0; ui8Idx < NUM_FRESULT_CODES; ui8Idx++)
    {
        //
        // If a match is found, then return the string name of the error code.
        //
        if(g_psFResultStrings[ui8Idx].iFResult == iFResult)
        {
            return(g_psFResultStrings[ui8Idx].pcResultStr);
        }
    }

    //
    // At this point no matching code was found, so return a string indicating
    // an unknown error.
    //
    return("UNKNOWN ERROR CODE");
}

//*****************************************************************************
//
// This is the handler for this SysTick interrupt.  FatFs requires a timer tick
// every 10 ms for internal timing purposes.
//
//*****************************************************************************
void
SysTickHandler(void)
{
    //
    // Call the FatFs tick timer.
    //
    disk_timerproc();
}


void
SDerror(void)
{
	Configure_RGB(YELLOW);
	RGBEnable();
	while(1);

}


//*****************************************************************************
//
// SD card write function
//	filename and data are inputs
//
//*****************************************************************************

void
SDWrite(const char * file, char * data)
{
    //
    //Disable Interrupts
    //
    ROM_IntMasterDisable();

    FRESULT iFResult;
    UINT bytesWritten;

	//
	// Open the file
	//
	iFResult = f_open(&FilObj,file, FA_WRITE | FA_OPEN_EXISTING);
	if(iFResult != FR_OK)
	{
		UARTprintf("\n %s", StringFromFResult(iFResult) );
		SDerror();
	}

	//
	//Move r/w pointer
	//
	f_lseek(&FilObj, f_size(&FilObj));

	//
	// Write Data
	//
	iFResult = f_write(&FilObj, data, strlen(data), &bytesWritten);
	if(iFResult != FR_OK)
	{
		UARTprintf("Write Error: %s\n", StringFromFResult(iFResult) );
		SDerror();
	}

	//
	//Close file
	//
	f_close(&FilObj);

    //
    //Enable Interrupts
    //
    ROM_IntMasterEnable();

}


//*****************************************************************************
//
// SD card read function
//	filename & buffer for data
//	return single line of file
//
//*****************************************************************************

int
SDRead(char * data)
{
	//
	//Disable Interrupts
	//
	ROM_IntMasterDisable();

	//Get a Line of Data
	f_gets(data, FilObj2.fsize , &FilObj2);

	//May not work f_eof...?
	if ( data[0] == '\0' ) {
		//EOF
		return 1;
	}

    //
    //Enable Interrupts
    //
    ROM_IntMasterEnable();

    return 0;


}


//*****************************************************************************
//
// Configure the SD Card
//
//*****************************************************************************

const char *
Configure_SD(void)
{

	FRESULT iFResult;
	uint32_t count = 0;
	static char fileName[10];

    //
    // Enable the peripheral.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    //
    // Configure SysTick for a 100Hz interrupt needed for fatfs.
    //
    ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / 100);
    ROM_SysTickEnable();
    ROM_SysTickIntEnable();

    //
    // Mount the file system (set up SPI and Disk I/O)
    //
    iFResult = f_mount(0, &FatFs);
    if(iFResult != FR_OK)
    {
        UARTprintf("f_mount error: %s\n", StringFromFResult(iFResult));
		SDerror();
    }


	//Search for GPSData.txt
	if (f_opendir(&DirObj, Cwd) == FR_OK) {
		while( (f_readdir(&DirObj, &FilInfo) == FR_OK) && FilInfo.fname[0]) {
			if (FilInfo.fname[0] == 'G'){
				//Now in 7 mode, Open the file
				AppState = 2;
				iFResult = f_open(&FilObj2,"GPSData.txt", FA_READ );
			    if(iFResult != FR_OK)
			    {
			        UARTprintf("Create file error: %s\n", StringFromFResult(iFResult));
			        SDerror();
			    }
			}
		}
	}


    //
    // Create New File
    //

    //Open Directory
	if (f_opendir(&DirObj, Cwd) == FR_OK) {
		//Count files that are not directory
		while( (f_readdir(&DirObj, &FilInfo) == FR_OK) && FilInfo.fname[0]) {
			if( !(FilInfo.fattrib & AM_DIR)) {
				count++;
			}
		}
	}

	//Create Filename and file
	sprintf(fileName, "LOG_%d.txt", count + 1);

	iFResult = f_open(&FilObj,fileName, FA_WRITE | FA_CREATE_ALWAYS);
    if(iFResult != FR_OK)
    {
        UARTprintf("Create file error: %s\n", StringFromFResult(iFResult));
        SDerror();
    }
    f_close(&FilObj);

    return fileName;
}

#endif

//*****************************************************************************
//
// Function to Collect & Write Data to SD
//
//*****************************************************************************

void
writeLog(const char * fileName)
{
	char ADCBuf[11];
	char UVBuf[5];
	char ACCLBuf[20];
	char PRESSBuf[10];

	int16_t ACCLdata[3] = {0};
	uint16_t PRESSdata[2] = {0};

	//Temperature
	sprintf(ADCBuf, "%d\n", ReadADC(TEM_ADC));
	ROM_SysCtlDelay(SysCtlClockGet() / 24 );

	//UV Level
	sprintf(UVBuf, "%d\n", ReadADC(UV_ADC));
	ROM_SysCtlDelay(SysCtlClockGet() / 24 );

	//Acceleration
	accelRead(ACCLdata);
	sprintf(ACCLBuf, "%d %d %d\n", ACCLdata[0], ACCLdata[1], ACCLdata[2]);
	ROM_SysCtlDelay(SysCtlClockGet() / 24 );

	//Pressure
	pressRead(PRESSdata);
	sprintf(PRESSBuf, "%d %d\n\n", PRESSdata[0], PRESSdata[1]);
	ROM_SysCtlDelay(SysCtlClockGet() / 24 );

	if (highRes) {
		//Clear SDBuf then load GPSData
		memset(&SDBuf[0], 0, sizeof(SDBuf));
		strcat(SDBuf, GPSData);
	}

	//Combine into one Buffer
	strcat(SDBuf, ACCLBuf);
	strcat(SDBuf, ADCBuf);
	strcat(SDBuf, UVBuf);
	strcat(SDBuf, PRESSBuf);

	//Write to SD Card
	SDWrite(fileName, SDBuf);
}

//*****************************************************************************
//
// Accelerometer Interrupt Handler
//
//*****************************************************************************

void
GPIOEIntHandler(void)
{

	//Clear the interrupt
	GPIOIntClear(GPIO_PORTE_BASE ,GPIO_PIN_0);

	//Clear the Accel Interrupt
	I2CRead(ACCEL_ADDR, 0x30);

	//Set Flag so we know to wake up
	ACCEL_Flag = 1;
}


//*****************************************************************************
//
// PushButton Interrupt Handler
//
//*****************************************************************************

void
GPIOFIntHandler(void)
{
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);

	//Debounce
	ROM_SysCtlDelay(ROM_SysCtlClockGet() / 6);

	if (AppState == 2) {
		AppState = 0;
	}

	AppState ^= 1;

}

//*****************************************************************************
//
// Timer Interrupt Handler
//
//*****************************************************************************

void
Timer0IntHandler(void)
{
	//Let program know 1 min has passed
	ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	MIN_Flag = 1;
	UARTprintf("1MIN\n");
	if (AppState <= 1) {
		minCount++;
	}
	if (AppState == 2) {
		secCount++;
		sec = 1;
		UARTprintf("s\n");
	}
}


//*****************************************************************************
//
// Timer1 Interrupt Handler
//
//*****************************************************************************

void
Timer1IntHandler(void)
{
	//Let program know 1 sec has passed
	ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	sec = 1;
	secCount++;
}


//*****************************************************************************
//
// Check if current Coords are within the bounds
//
//*****************************************************************************
int
checkGPS(uint32_t *p, uint32_t *p2)
{

	//Point = SD card
	//Point2 = GPS

	int32_t sdLatD = p[0];		//Degrees & Minutes
	int32_t sdLatF = p[1];		//Fractional part of minutes
	int32_t sdLonD = p[2];
	int32_t sdLonF = p[3];

	uint32_t gpsLatD = p2[0];
	uint32_t gpsLatF = p2[1];
	uint32_t gpsLonD = p2[2];
	uint32_t gpsLonF = p2[3];

	//Bounding Box variables
	uint32_t tD, tF, bD, bF, lD, lF, rD, rF;

	//Overflow Flags
	uint8_t tO = 0, bO = 0, lO = 0, rO = 0;
	uint8_t top = 0, bot = 0, left = 0, right = 0;

	//Value in decimal minutes to add/ subtract around the GPS point
	int32_t accuracy = 1200;

	//Create the Bounding Box positions
	//Taking into account overflow for each side

	//Top
	if ( (sdLatF - accuracy) < 0) {
		tD = sdLatD - 1;
		tF = 100000 - (accuracy - sdLatF);
		tO = 1;
	} else {
		tD = sdLatD;
		tF = sdLatF - accuracy;
	}

	//Bottom
	if (sdLatF + accuracy > 100000) {
		bD = sdLatD + 1;
		bF = (sdLatF + accuracy) - 1000000;
		bO = 1;
	} else {
		bD = sdLatD;
		bF = sdLatF + accuracy;
	}

	//Left
	if ( (sdLonF - accuracy) < 0) {
		lD = sdLonD -1;
		lF = 100000 - (accuracy - sdLonF);
		lO = 1;
	} else {
		lD = sdLonD;
		lF = sdLonF - accuracy;
	}

	//Right
	if (sdLonF + accuracy > 100000) {
		rD = sdLonD + 1;
		rF = (sdLonF + accuracy) - 100000;
		rO = 1;
	} else {
		rD = sdLonD;
		rF = sdLonF + accuracy;
	}

	//Check if GPS coord is within bounding box

	//Top Check
	if (tO) {
		//Overflow
		if ( (gpsLatD == tD ) && (gpsLatF >= tF) ) {
			top = 1;
		} else if ( (gpsLatD > tD) ) {
			top = 1;
		}
	} else {
		if ( (gpsLatD >= tD) && (gpsLatF >= tF) ) {
			top = 1;
		}
	}

	//Bottom Check
	if (bO) {
		//Overflow
		if ( (gpsLatD == bD ) && (gpsLatF <= bF) ) {
			bot = 1;
		} else if ( (gpsLatD < bD) ) {
			bot = 1;
		}
	} else {
		if ( (gpsLatD <= bD) && (gpsLatF <= bF) ) {
			bot = 1;
		}
	}

	//Left Check
	if (lO) {
		//Overflow
		if ( (gpsLonD == lD ) && (gpsLonF >= lF) ) {
			left = 1;
		} else if ( (gpsLonD > lD) ) {
			left = 1;
		}
	} else {
		if ( (gpsLonD >= lD) && (gpsLonF >= lF) ) {
			left = 1;
		}
	}

	//Right Check
	if (rO) {
		//Overflow
		if ( (gpsLonD == rD ) && (gpsLonF <= rF) ) {
			right = 1;
		} else if ( (gpsLonD < rD) ) {
			right = 1;
		}
	} else {
		if ( (gpsLonD <= rD) && (gpsLonF <= rF) ) {
			right = 1;
		}
	}

	UARTprintf("T: %d\nB: %d\nL: %d\nR: %d\n", top, bot, left, right);
	if (top && bot && left && right) {
		return 1;
	} else {
		return 0;
	}
}

//*****************************************************************************
//
// Enter and Configure Deep Sleep Mode
//
//*****************************************************************************

void
enterSleep(void)
{

	//
	// Keep Key Peripherals Enabled (Timer & Accel Interrupt)
	//


	//Ensure timer & accel can wake the device
	if (!AppState) {
		ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);
		ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOF);
	} else {
		ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);
		ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);
		ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOF);
	}

	//Turn off other features
	ROM_SysCtlPeripheralSleepDisable(SYSCTL_PERIPH_I2C0);
	ROM_SysCtlPeripheralSleepDisable(SYSCTL_PERIPH_PWM0);
	SysTickDisable();

	//
	// 	Enable Clock Gating to turn off everything else
	//
	ROM_SysCtlPeripheralClockGating(true);

	//Ensure LED is off (to stop interrupts)
	RGBBlinkRateSet(0.0f);
	RGBDisable();


	//Turn off GPS RF
	Configure_GPS(OFF);

	//Start the 1min timer
	ROM_TimerEnable(TIMER0_BASE, TIMER_A);

	//Delay before entering sleep
	ROM_SysCtlDelay(ROM_SysCtlClockGet() * 2 );

	//Enter Sleep Mode
	ROM_SysCtlSleep();

}


//*****************************************************************************
//
// Setup Basic System Functions and I/O
//
//*****************************************************************************

void
Setup(void)
{
    //
    // Set the clocking to run from the PLL at 10MHz
    // 200MHz/ SYSDIV_#

	ROM_SysCtlClockSet(SYSCTL_SYSDIV_20 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    //Enable lazy stacking for interrupt handlers
    //
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();


#ifdef LED_EN
    //
    //Enable GIPO for RGB
    //
    RGBInit(0);
    RGBIntensitySet(0.2f);
    Configure_RGB(BLUE);
    RGBBlinkRateSet(0.7f);
    //RGBEnable();

#endif

#ifdef UV_EN
    //
    // Enable GPIO for UV
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, UV_PIN);

    //Enable the sensor
    GPIOPinWrite(GPIO_PORTE_BASE, UV_PIN, UV_PIN);

#endif

#ifdef PR_EN

    //For the RST and SHDN pins
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6);

    //Enable the Pressure Sensor (Active High)
    ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);


#endif

#ifdef SPEAK_EN

    //Set PWM Clock
    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    //Enable PWM and Pins
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //Set PWM
    ROM_GPIOPinConfigure(GPIO_PB6_M0PWM0);
    ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

    //Config Frequency roughly 600Hz
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN |
                       PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 150000);

    //Duty Cycle 50%
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,
                     PWMGenPeriodGet(PWM0_BASE, PWM_OUT_0) / 2);


    //Enable Pin Output
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);

    //Enable PWM
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);


#endif

#ifdef WAKE_MOV

    //Set the pin as input
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0);

    //Setup Pin Interrupt Handler PE0
    ROM_GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_RISING_EDGE);

    // enable interrupt on pins
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_INT_PIN_0);

    // enable interrupts on port B
    ROM_IntEnable(INT_GPIOE);


    //Also Configure the PushButton SW1
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
    ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    ROM_GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);
    ROM_IntEnable(INT_GPIOF);


#endif

    //Setup a Timer for GPS & Wake Handling (1min) 32bit
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet() * 60);	//* 60
    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

#ifdef ADV_FEAT

    /*
    //1 Sec timer for highRes
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, ROM_SysCtlClockGet());
    ROM_IntEnable(INT_TIMER1A);
    ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

*/

#endif


}


//We need this systick handler when the SD card is not running


/*
void
SysTickHandler(void)
{
    //
    // Call the FatFs tick timer.
    //

}
*/



//*****************************************************************************
//
// 	MAIN
//
//*****************************************************************************
int
main(void)
{
	const char * fileName;
	uint8_t fileCreated = 0;

	uint32_t dummy = 0;
	uint32_t sdOne = 1;

	//7 Variables
	char SDdata[80] = {0};
	uint32_t point[4] = {0};

	//Store all SD card points
	int inside = 0;
	uint32_t i = 0, j = 0;


    Setup();

#ifdef ADC_EN
    Configure_ADC();
#endif

#ifdef UART_EN
    Configure_UART();
    UARTprintf("ENGG4810\n");
#endif

#ifdef I2C_EN
    Configure_I2C();
    AccelSetup();
#endif

#ifdef GPS_EN
   Configure_GPS(0);
#endif

#ifdef ADV_FEAT

    //Check for SD Config file
    fileName = Configure_SD();
    //Don't switch to other modes....
    fileCreated = 1;
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet() * 60);
    MIN_Flag = 0;

#endif

    //
    // Enable Interrupts
    //
    ROM_IntMasterEnable();


    while(1)
    {

		ROM_SysCtlDelay(SysCtlClockGet() / 12 );

    	switch (AppState)
    	{

    	// 1min Sample Mode
    	case 0:

    		if (MIN_Flag) {
    			Configure_RGB(GREEN);

    			//Need to Turn on GPS Here (once)
        		if (!dummy) {
        			Configure_GPS(ON);
        			//RGBEnable();
        			RGBBlinkRateSet(0.7f);
        			SysTickEnable();
        			//Reset timer load
        			ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet() * 60);
        			dummy = 1;
        		}

				if (GPS_Flag) {
					ROM_IntMasterDisable();

					if (! fileCreated) {
						//Wait until GPS Fix to create log file
						if ( (SDBuf[17] == 'A')  ) {
							fileName = Configure_SD();
							fileCreated = 1;
							//Know Log file created/ fix found
							UARTprintf("FILEMADE\n");
							Configure_RGB(YELLOW);
							ROM_SysCtlDelay(SysCtlClockGet() / 12 );
						}
					} else {
						//Collect & Log 'good' data, Start Timer again to ensure 1-Min accuracy

						if (SDBuf[17] == 'A') {
							UARTprintf("FILEWROTE\n");
							writeLog(fileName);
							MIN_Flag = 0;
							minCount = 0;
							//Need to still Log if no fix...
						} else if ( (minCount >= 2) && (SDBuf[3] == 'R') ) {
							UARTprintf("NO FIX 2 MIN\n");
							writeLog(fileName);
							MIN_Flag = 0;
							minCount = 0;
						}
					}

					GPS_Flag = 0;
					memset(&SDBuf[0], 0, sizeof(SDBuf));
					ROM_IntMasterEnable();
				}

    		} else {
    			dummy = 0;
    			UARTprintf("SLEEP\n");
    			enterSleep();
    		}
    		break;


    	// Wake on Movement Mode
    	case 1:

 			//Got Movement
    		if (ACCEL_Flag) {
        		Configure_RGB(CYAN);

    			//Turn on GPS here (once)
        		if (!dummy) {
        			Configure_GPS(ON);
        			//RGBEnable();
        			RGBBlinkRateSet(0.7f);
        			SysTickEnable();
					//Reset time to ensure 1min
					ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet() * 30);
					minCount = 0;
        			dummy = 1;
        		}

    			if (! fileCreated) {
    				fileName = Configure_SD();
    				fileCreated = 1;
    				Configure_RGB(YELLOW);
    				ROM_SysCtlDelay(SysCtlClockGet() / 12 );
    			}
    			if (fileCreated) {
					//Ensure full RMC string is included
    				if (GPS_Flag) {
    					ROM_IntMasterDisable();
 						GPS_Flag = 0;

 						if (sdOne) {
 	 	   					//Stop errors in SD log & (only want to write once)
 	 	    				if (SDBuf[3] == 'R') {
 	 	    						writeLog(fileName);
 	 	    						sdOne = 0;
 	 	    				}
 						}

						if ( (SDBuf[17] == 'A')  ) {
							UARTprintf("FIXFOUND\n");
							writeLog(fileName);
							//Got a Fix so go back to sleep
							ACCEL_Flag = 0;
							dummy = 0;
							sdOne = 1;
							enterSleep();
						}
						memset(&SDBuf[0], 0, sizeof(SDBuf));
						ROM_IntMasterEnable();
    				}
    			}
    			//2 Minutes have passed so go back to sleep
    			if (minCount >= 2) {
    				ACCEL_Flag = 0;
    				MIN_Flag = 0;
    				minCount = 0;
    				dummy = 0;
    				sdOne = 1;
    				UARTprintf("2MIN\n");
    				enterSleep();
    			}

    		} else {
    			dummy = 0;
    			enterSleep();
    		}

    		break;

    	//7 Mode
    	case 2:
    		Configure_RGB(BLUE);

			if (!dummy) {

				//Load in all the SD card points
				while ( !(SDRead(SDdata)) ) {
					parseGPS(SDdata, point);
					for (j = 0; j < 4; j++) {
						points[i][j] = point[j];
					}
					i++;
				}
				memset(&SDdata[0], 0, sizeof(SDdata));
				memset(&point[0], 0, sizeof(point));

				dummy = 1;
				Configure_GPS(ON);
				RGBBlinkRateSet(0.7f);
				ROM_SysCtlDelay(SysCtlClockGet() / 12 );
			}

			//Read in Current GPS coordinates
			if (GPS_Flag) {
				ROM_IntMasterDisable();
				GPS_Flag = 0;
				inside = 0;

				if (SDBuf[17] == 'A') {
					memset(&point2[0], 0, sizeof(point2));

					parseGPS(SDBuf, point2);

					for (j = 0; j < 4; j++) {
					//Check if we are within path
						if ( checkGPS(points[j], point2) ) {
							inside = 1;
							UARTprintf("YAY\n");
						}
					}

					if (!inside) {
						//Beep the Buzzer
						//Flash the LED
						PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
						Configure_RGB(GREEN);
						ROM_SysCtlDelay(500000);
						PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
					}

				}
				memset(&SDBuf[0], 0, sizeof(SDBuf));
				ROM_IntMasterEnable();
			}
			break;
    	}
    }
}
