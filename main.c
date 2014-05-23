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
// Enter and Configure Deep Sleep Mode
//
//*****************************************************************************

void
enterSleep(void)
{

	//
	// Keep Key Peripherals Enabled
	//
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART1);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);
	//For accel interrupt
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);

	//
	// 	Enable Clock Gating
	//
	ROM_SysCtlPeripheralClockGating(1);

	//
	// Reduce Voltage & Power
	//
	SysCtlLDOSleepSet(SYSCTL_LDO_0_90V);
	SysCtlSleepPowerSet(SYSCTL_SRAM_LOW_POWER | SYSCTL_FLASH_LOW_POWER);

	//
	// Turn off UV/MPL
	//
	//mpl = PD6
	//uv = PE4

	//Ensure LED is off (to stop interrupts)
	RGBDisable();

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

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 15000);

    //Duty Cycle 50%
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,
                     PWMGenPeriodGet(PWM0_BASE, PWM_OUT_0) / 2);


    //Enable Pin Output
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);

    //Enable PWM
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);


#endif

#ifdef WAKE_MOV

    //Set the pin as input
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0);

    //Setup Pin Interrupt Handler PE0
    ROM_GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_RISING_EDGE);

    // enable interrupt on pins
    GPIOPinIntEnable(GPIO_PORTE_BASE, GPIO_PIN_0);

    // enable interrupts on port B
    IntEnable(INT_GPIOE);


#endif


    //
    //Enable lazy stacking for interrupt handlers
    //
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();

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
	char ADCBuf[11];
	char UVBuf[5];
	char ACCLBuf[20];
	char PRESSBuf[10];

	int16_t ACCLdata[3] = {0};
	uint16_t PRESSdata[2] = {0};

    uint8_t powerSave[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92 };
    uint8_t powerHigh[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91 };
    uint8_t rate1Hz[] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xD0, 0x07, 0x01, 0x00, 0x01, 0x00, 0xED, 0xBD };
    uint8_t rate1min[] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x60, 0xEA, 0x01, 0x00, 0x01, 0x00, 0x60, 0x8C };


	uint8_t fileCreated = 0;

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

	ROM_SysCtlDelay(SysCtlClockGet() / 12 );

	UARTSend(rate1Hz, sizeof(rate1Hz));


    //
    // Enable Interrupts
    //
    ROM_IntMasterEnable();

    ROM_SysCtlDelay(SysCtlClockGet() / 6 );

    //Use to Turn off/on SPEAK
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);

    while(1)
    {


    	if (GPS_Flag) {


    		ROM_IntMasterDisable();



#ifdef SD_EN

    		if (! fileCreated) {
    			//Wait until GPS Fix to create log file
    			if ( (SDBuf[18] == 'A')  ) {
    				fileName = Configure_SD();
    				fileCreated = 1;
    				UARTprintf("FIleMade\n");
    				Configure_RGB(YELLOW);

    				//Turn on Powersave mode
    				//UARTSend(powerSave, sizeof(powerSave));
    			}

    		} else {

    			sprintf(ADCBuf, "%d\n", ReadADC(TEM_ADC));
    			ROM_SysCtlDelay(SysCtlClockGet() / 24 );

    			sprintf(UVBuf, "%d\n", ReadADC(UV_ADC));
    			ROM_SysCtlDelay(SysCtlClockGet() / 24 );

    			accelRead(ACCLdata);
    			sprintf(ACCLBuf, "%d %d %d\n", ACCLdata[0], ACCLdata[1], ACCLdata[2]);
    			ROM_SysCtlDelay(SysCtlClockGet() / 24 );

    			pressRead(PRESSdata);
    			sprintf(PRESSBuf, "%d %d\n", PRESSdata[0], PRESSdata[1]);
    			ROM_SysCtlDelay(SysCtlClockGet() / 24 );

    			strcat(SDBuf, ACCLBuf);
    			ROM_SysCtlDelay(SysCtlClockGet() / 24 );
    			strcat(SDBuf, ADCBuf);
    			ROM_SysCtlDelay(SysCtlClockGet() / 24 );
    			strcat(SDBuf, UVBuf);
    			ROM_SysCtlDelay(SysCtlClockGet() / 24 );
    			strcat(SDBuf, PRESSBuf);
    			ROM_SysCtlDelay(SysCtlClockGet() / 24 );

    			SDWrite(fileName, SDBuf);
    		}

    		GPS_Flag = 0;

    		memset(&SDBuf[0], 0, sizeof(SDBuf));
    		memset(&ACCLBuf[0],0, sizeof(ACCLBuf));
    		memset(&ADCBuf[0],0, sizeof(ADCBuf));
    		memset(&UVBuf[0],0, sizeof(UVBuf));
    		memset(&PRESSBuf[0],0, sizeof(PRESSBuf));

#endif
    		ROM_IntMasterEnable();

    	}


    	//enterSleep();
    	Configure_RGB(GREEN);

		ROM_SysCtlDelay(SysCtlClockGet() / 12 );

    }
}
