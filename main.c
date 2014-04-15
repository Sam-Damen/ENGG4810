//*****************************************************************************
//
//	ENGG4810 TP2 Firmware
//	Team 22
//  By Jack McKinnon
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/cmdline.h"
#include "utils/uartstdio.h"
#include "fatfs/src/ff.h"
#include "fatfs/src/diskio.h"

#include "Config.h"
#include "I2C.h"
#include "ADC.h"
#include "UART.h"
//#include "LED.h"




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


//*****************************************************************************
//
// SD card write function
//	filename and data are inputs
//
//*****************************************************************************

void
SDWrite(char * file, char * data)
{
    //
    //Disable Interrupts
    //
    ROM_IntMasterDisable();

    FRESULT iFResult;
    UINT bytesWritten;

	//
	// Open Directory
	//
	iFResult = f_opendir(&DirObj, Cwd);
	if(iFResult != FR_OK)
	{
		UARTprintf("\n %s", StringFromFResult(iFResult) );
	}

	//
	// Open the file
	//
	iFResult = f_open(&FilObj,file, FA_WRITE | FA_OPEN_ALWAYS);
	if(iFResult != FR_OK)
	{
		UARTprintf("\n %s", StringFromFResult(iFResult) );
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
		UARTprintf("\n %s", StringFromFResult(iFResult) );
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

void
Configure_SD(void) {

	FRESULT iFResult;


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
    }
}


#endif


//*****************************************************************************
//
// Setup Basic System Functions
//
//*****************************************************************************

void
Setup(void)
{
    //
    // Set the clocking to run from the PLL at 50MHz
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);


    //
    //Enable GIPO for LED
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);


    //
    //Enable lazy stacking for interrupt handlers
    //
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();
}



//*****************************************************************************
//
// 	MAIN
//
//*****************************************************************************
int
main(void)
{

    Setup();

#ifdef ADC_EN
    Configure_ADC();
#endif

#ifdef UART_EN
    Configure_UART();
#endif

#ifdef I2C_EN
    Configure_I2C();
    MMA8452QSetup();
#endif

#ifdef SD_EN
    Configure_SD();
#endif

    //
    // Enable Interrupts
    //
    ROM_IntMasterEnable();


    while(1)
    {

    	//UARTprintf("Testing\r\n");
    	SDWrite("test.txt", "hello \r\n");

		//
		// Turn on LED so we know something is happening
		//
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);


		ROM_SysCtlDelay(SysCtlClockGet() / 12 );

    }
}
