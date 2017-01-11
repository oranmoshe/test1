/********************************************************************
FileName:     main.c
Dependencies: See INCLUDES section
Processor:   PIC18 or PIC24 USB Microcontrollers
Hardware:    The code is natively intended to be used on the following
hardware platforms: PICDEM™ FS USB Demo Board,
PIC18F87J50 FS USB Plug-In Module, or
Explorer 16 + PIC24 USB PIM.  The firmware may be
modified for use on other USB platforms by editing the
HardwareProfile.h file.
Complier:    Microchip C18 (for PIC18) or C30 (for PIC24)
Company:   Microchip Technology, Inc.

Software License Agreement:

The software supplied herewith by Microchip Technology Incorporated
(the “Company”) for its PIC® Microcontroller is intended and
supplied to you, the Company’s customer, for use solely and
exclusively on Microchip PIC Microcontroller products. The
software is owned by the Company and/or its supplier, and is
protected under applicable copyright laws. All rights are reserved.
Any use in violation of the foregoing restrictions may subject the
user to criminal sanctions under applicable laws, as well as to
civil liability for the breach of the terms and conditions of this
license.

THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
File Description:

Change History:
Rev   Date         Description
1.0   11/19/2004   Initial release
2.1   02/26/2007   Updated for simplicity and to use common
coding style
********************************************************************/


//	========================	INCLUDES	========================
#ifdef _VISUAL
#include "VisualSpecials.h"
#endif // VISUAL

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"

#include "mtouch.h"

#include "BMA150.h"

#include "oled.h"

#include "soft_start.h"





//	========================	CONFIGURATION	========================

#if defined(PIC18F46J50_PIM)
//Watchdog Timer Enable bit:
#pragma config WDTEN = OFF          //WDT disabled (control is placed on SWDTEN bit)
//PLL Prescaler Selection bits:
#pragma config PLLDIV = 3           //Divide by 3 (12 MHz oscillator input)
//Stack Overflow/Underflow Reset Enable bit:
#pragma config STVREN = ON            //Reset on stack overflow/underflow enabled
//Extended Instruction Set Enable bit:
#pragma config XINST = OFF          //Instruction set extension and Indexed Addressing mode disabled (Legacy mode)
//CPU System Clock Postscaler:
#pragma config CPUDIV = OSC1        //No CPU system clock divide
//Code Protection bit:
#pragma config CP0 = OFF            //Program memory is not code-protected
//Oscillator Selection bits:
#pragma config OSC = ECPLL          //HS oscillator, PLL enabled, HSPLL used by USB
//Secondary Clock Source T1OSCEN Enforcement:
#pragma config T1DIG = ON           //Secondary Oscillator clock source may be selected
//Low-Power Timer1 Oscillator Enable bit:
#pragma config LPT1OSC = OFF        //Timer1 oscillator configured for higher power operation
//Fail-Safe Clock Monitor Enable bit:
#pragma config FCMEN = OFF           //Fail-Safe Clock Monitor disabled
//Two-Speed Start-up (Internal/External Oscillator Switchover) Control bit:
#pragma config IESO = OFF           //Two-Speed Start-up disabled
//Watchdog Timer Postscaler Select bits:
#pragma config WDTPS = 32768        //1:32768
//DSWDT Reference Clock Select bit:
#pragma config DSWDTOSC = INTOSCREF //DSWDT uses INTOSC/INTRC as reference clock
//RTCC Reference Clock Select bit:
#pragma config RTCOSC = T1OSCREF    //RTCC uses T1OSC/T1CKI as reference clock
//Deep Sleep BOR Enable bit:
#pragma config DSBOREN = OFF        //Zero-Power BOR disabled in Deep Sleep (does not affect operation in non-Deep Sleep modes)
//Deep Sleep Watchdog Timer Enable bit:
#pragma config DSWDTEN = OFF        //Disabled
//Deep Sleep Watchdog Timer Postscale Select bits:
#pragma config DSWDTPS = 8192       //1:8,192 (8.5 seconds)
//IOLOCK One-Way Set Enable bit:
#pragma config IOL1WAY = OFF        //The IOLOCK bit (PPSCON<0>) can be set and cleared as needed
//MSSP address mask:
#pragma config MSSP7B_EN = MSK7     //7 Bit address masking
//Write Protect Program Flash Pages:
#pragma config WPFP = PAGE_1        //Write Protect Program Flash Page 0
//Write Protection End Page (valid when WPDIS = 0):
#pragma config WPEND = PAGE_0       //Write/Erase protect Flash Memory pages starting at page 0 and ending with page WPFP[5:0]
//Write/Erase Protect Last Page In User Flash bit:
#pragma config WPCFG = OFF          //Write/Erase Protection of last page Disabled
//Write Protect Disable bit:
#pragma config WPDIS = OFF          //WPFP[5:0], WPEND, and WPCFG bits ignored

#else
#error No hardware board defined, see "HardwareProfile.h" and __FILE__
#endif



//	========================	Global VARIABLES	========================
#pragma udata
//You can define Global Data Elements here

//	========================	PRIVATE PROTOTYPES	========================
static void InitializeSystem(void);
static void ProcessIO(void);
static void UserInit(void);
static void YourHighPriorityISRCode();
static void YourLowPriorityISRCode();

BOOL CheckButtonPressed(void);

//	========================	VECTOR REMAPPING	========================
#if defined(__18CXX)
//On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
//the reset, high priority interrupt, and low priority interrupt
//vectors.  However, the current Microchip USB bootloader 
//examples are intended to occupy addresses 0x00-0x7FF or
//0x00-0xFFF depending on which bootloader is used.  Therefore,
//the bootloader code remaps these vectors to new locations
//as indicated below.  This remapping is only necessary if you
//wish to program the hex file generated from this project with
//the USB bootloader.  If no bootloader is used, edit the
//usb_config.h file and comment out the following defines:
//#define PROGRAMMABLE_WITH_SD_BOOTLOADER

#if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
#define REMAPPED_RESET_VECTOR_ADDRESS     0xA000
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0xA008
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0xA018
#else 
#define REMAPPED_RESET_VECTOR_ADDRESS     0x00
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0x08
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0x18
#endif

#if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
extern void _startup(void);        // See c018i.c in your C18 compiler dir
#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
void _reset(void)
{
	_asm goto _startup _endasm
}
#endif
#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
void Remapped_High_ISR(void)
{
	_asm goto YourHighPriorityISRCode _endasm
}
#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
void Remapped_Low_ISR(void)
{
	_asm goto YourLowPriorityISRCode _endasm
}

#if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
//Note: If this project is built while one of the bootloaders has
//been defined, but then the output hex file is not programmed with
//the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
//As a result, if an actual interrupt was enabled and occured, the PC would jump
//to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
//executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
//(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
//would effective reset the application.

//To fix this situation, we should always deliberately place a 
//"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
//"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
//hex file of this project is programmed with the bootloader, these sections do not
//get bootloaded (as they overlap the bootloader space).  If the output hex file is not
//programmed using the bootloader, then the below goto instructions do get programmed,
//and the hex file still works like normal.  The below section is only required to fix this
//scenario.
#pragma code HIGH_INTERRUPT_VECTOR = 0x08
void High_ISR(void)
{
	_asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
}
#pragma code LOW_INTERRUPT_VECTOR = 0x18
void Low_ISR(void)
{
	_asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
}
#endif  //end of "#if defined(||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER))"

#pragma code

//	========================	Application Interrupt Service Routines	========================
//These are your actual interrupt handling routines.
#pragma interrupt YourHighPriorityISRCode
void YourHighPriorityISRCode()
{
	//Check which interrupt flag caused the interrupt.
	//Service the interrupt
	//Clear the interrupt flag
	//Etc.

} //This return will be a "retfie fast", since this is in a #pragma interrupt section 
#pragma interruptlow YourLowPriorityISRCode
void YourLowPriorityISRCode()
{
	//Check which interrupt flag caused the interrupt.
	//Service the interrupt
	//Clear the interrupt flag
	//Etc.

} //This return will be a "retfie", since this is in a #pragma interruptlow section 
#endif




  //	========================	Board Initialization Code	========================
#pragma code
#define ROM_STRING rom unsigned char*

  /******************************************************************************
  * Function:        void UserInit(void)
  *
  * PreCondition:    None
  *
  * Input:           None
  *
  * Output:          None
  *
  * Side Effects:    None
  *
  * Overview:        This routine should take care of all of the application code
  *                  initialization that is required.
  *
  * Note:
  *
  *****************************************************************************/
void UserInit(void)
{
	/* Initialize the mTouch library */
	mTouchInit();

	/* Call the mTouch callibration function */
	mTouchCalibrate();

	/* Initialize the acce	lerometer */
	InitBma150();

	/* Initialize the oLED Display */
	ResetDevice();
	FillDisplay(0x00);
	oledPutROMString((ROM_STRING)"", 0, 0);
}//end UserInit


 /********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
	// Soft Start the APP_VDD
	while (!AppPowerReady())
		;

#if defined(PIC18F87J50_PIM) || defined(PIC18F46J50_PIM)
	//On the PIC18F87J50 Family of USB microcontrollers, the PLL will not power up and be enabled
	//by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
	//This allows the device to power up at a lower initial operating frequency, which can be
	//advantageous when powered from a source which is not gauranteed to be adequate for 48MHz
	//operation.  On these devices, user firmware needs to manually set the OSCTUNE<PLLEN> bit to
	//power up the PLL.
	{
		unsigned int pll_startup_counter = 600;
		OSCTUNEbits.PLLEN = 1;  //Enable the PLL and wait 2+ms until the PLL locks before enabling USB module
		while (pll_startup_counter--);
	}
	//Device switches over automatically to PLL output after PLL is locked and ready.
#endif

#if defined(PIC18F46J50_PIM)
	//Configure all I/O pins to use digital input buffers
	ANCON0 = 0xFF;                  // Default all pins to digital
	ANCON1 = 0xFF;                  // Default all pins to digital
#endif

	UserInit();

}//end InitializeSystem



 //	========================	Application Code	========================

BOOL CheckButtonPressed(void)
{
	static char buttonPressed = FALSE;
	static unsigned long buttonPressCounter = 0;

	if (PORTBbits.RB0 == 0)
	{
		oledWriteChar1x('B', 0xB1, 2 * 6);
		if (buttonPressCounter++ > 1000)
		{
			buttonPressCounter = 0;
			buttonPressed = TRUE;
		}
	}
	else
	{
		oledWriteChar1x(',', 0xB1, 2 * 6);
		if (buttonPressed == TRUE)
		{
			if (buttonPressCounter == 0)
			{
				buttonPressed = FALSE;
				return TRUE;
			}
			else
			{
				buttonPressCounter--;
			}
		}
	}

	return FALSE;
}
/********************************************************************
* Function:        void main(void)
*
* PreCondition:    None
*
* Input:           None
*
* Output:          None
*
* Side Effects:    None
*
* Overview:        Main program entry point.
*
* Note:            None
*******************************************************************/
unsigned char adh, adl, con;		// registers 8 bits each
unsigned int ans = 0;				// will provide the 16 bits adh.adl
char output[4];
int i;
//int flag = 1;						// flag to distinguish between color contrast

void potentziomeneterPrintScale(int ad) {
	i = 7;
	oledWriteChar1x('|', 0xB0, 6 * 6);
	for (i; i<20; i++) {
		oledWriteChar1x('-', 0xB0, i * 6);
	}
	oledWriteChar1x('|', 0xB0, 20 * 6);
	oledWriteChar1x('|', 0xB0, ad * 6);
}

char buf[20];

void potentziomeneter() {
	adh = ADRESH;
	adl = ADRESL;
	ans = ((int)adh << 8) | adl;		// shift adh 8 bits left and make OR with adl
	sprintf(output, "%x", ans);
	oledPutString(itoa(ans, buf), 0xB0, 1 * 6);
	if (ans<10) {
		oledWriteChar1x('   ', 0xB0, 2 * 6);
	}
	else if (ans > 10 && ans < 100) {
		oledWriteChar1x('  ', 0xB0, 3 * 6);
	}
	else if (ans > 100 && ans < 1000) {
		oledWriteChar1x(' ', 0xB0, 4 * 6);
	}
	potentziomeneterPrintScale(7 + ans / 85);
}

void buttonListener() {
	CheckButtonPressed();
}

void arrowsListener() {
	if (mTouchReadButton(0)>700) {  // right
		oledWriteChar1x('&', 0xB1, 10 * 6);
	}
	else {
		oledWriteChar1x('(', 0xB1, 10 * 6);
	}
	// need to change to soft // ALMOG
	if (mTouchReadButton(1)>700) {  // up
		oledWriteChar1x('"', 0xB1, 4 * 6);
	}
	else {
		oledWriteChar1x('#', 0xB1, 4 * 6);
	}
	// need to change to soft // ALMOG
	if (mTouchReadButton(2)>700) {  // down
		oledWriteChar1x('$', 0xB1, 6 * 6);
	}
	else {
		oledWriteChar1x('%', 0xB1, 6 * 6);
	}
	if (mTouchReadButton(3)>700) {  // left
		oledWriteChar1x(')', 0xB1, 8 * 6);
	}
	else {
		oledWriteChar1x('*', 0xB1, 8 * 6);
	}
}

int tempReg;

void temperature() {
	tempReg = (int)BMA150_ReadByte(0x08);
	tempReg = tempReg / 2 - 30;
	oledPutROMString((ROM_STRING)"T:", 0xB1, 13 * 6);
	oledPutString(itoa(tempReg, buf), 0xB1, 15 * 6);
	oledWriteChar1x('+', 0xB1, 17 * 6);
}

int maxXscale = 2;
int maxYscale = 0;
int maxX = 0;
int maxY = 0;

void acceleratorsPrintScales() {
	// for x
	int currentX = 2 + (maxX / 40);
	int currentY = maxY / 128;
	if (maxXscale < currentX) {
		oledWriteChar1x('-', 0xB6, maxXscale * 6);
		oledWriteChar1x('|', 0xB6, currentX * 6);
		maxXscale = currentX;
	}
	else {
		oledWriteChar1x('|', 0xB6, maxXscale * 6);
	}

	// for y
	if (currentY > maxYscale) {
		maxYscale = currentY;
		oledWriteChar1x('|', maxYscale, 20 * 6);
	}
	switch (maxYscale) {
	case 0:
		oledWriteChar1x('-', 0xB5, 20 * 6);
		break;
	case 1:
		oledWriteChar1x('-', 0xB4, 20 * 6);
		break;
	case 2:
		oledWriteChar1x('-', 0xB3, 20 * 6);
		break;
	case 3:
		oledWriteChar1x('-', 0xB2, 20 * 6);
		break;
	default:
		break;
	}
}

void printAxis() {
	// Y axis
	oledWriteChar1x('-', 0xB1, 20 * 6);
	oledWriteChar1x('|', 0xB2, 20 * 6);
	oledWriteChar1x('|', 0xB3, 20 * 6);
	oledWriteChar1x('|', 0xB4, 20 * 6);
	oledWriteChar1x('|', 0xB5, 20 * 6);
	oledWriteChar1x('-', 0xB6, 20 * 6);

	// x axis
	oledWriteChar1x('|', 0xB6, 1 * 6);
	for (i = 2; i<18; i++) {
		oledWriteChar1x('-', 0xB6, i * 6);
	}
	oledWriteChar1x('|', 0xB6, 18 * 6);

	// print scales
	acceleratorsPrintScales();
}

void resetGlobalVars() {
	maxX = 0;
	maxY = 0;
	maxXscale = 2;
	maxYscale = 0;
}

void accelerometers() {
	int temp = 0;
	int axisX = 0;
	int axisY = 0;
	int axisZ = 0;

	//print x
	axisX = axisX | BMA150_ReadByte(0x03);
	axisX = axisX << 2;
	temp = temp | (BMA150_ReadByte(0x02));
	temp = temp >> 6;
	axisX = axisX | temp;

	if (axisX & 0x200) {
		// negative
		axisX = axisX | 0xfc00;
		axisX = ~axisX;
		axisX = axisX + 1;

	}
	else {
		axisX = axisX | 0x0000;
	}

	if (axisX>maxX) {
		maxX = axisX;
	}

	oledPutROMString((ROM_STRING)"x:", 0xB7, 1 * 6);
	if (maxX >= 0 && maxX<10) {
		oledPutROMString((ROM_STRING)"   ", 0xB7, 4 * 6);
	}
	else if (maxX>10 && maxX < 100) {
		oledPutROMString((ROM_STRING)"  ", 0xB7, 5 * 6);
	}
	else if (maxX>100 && maxX < 1000) {
		oledPutROMString((ROM_STRING)" ", 0xB7, 6 * 6);
	}
	oledPutString(itoa(maxX, buf), 0xB7, 3 * 6);
	oledPutROMString((ROM_STRING)"          ", 0xB7, 6 * 6);


	// print Y
	axisY = axisY | BMA150_ReadByte(0x05);
	axisY = axisY << 2;
	temp = temp | (BMA150_ReadByte(0x04));
	temp = temp >> 6;
	axisY = axisY | temp;

	if (axisY & 0x200) {
		// negative
		axisY = axisY | 0xfc00;
		axisY = ~axisY;
		axisY = axisY + 1;

	}
	else {
		axisY = axisY | 0x0000;
	}

	if (axisY>maxY) {
		maxY = axisY;
	}

	oledPutROMString((ROM_STRING)"y:", 0xB7, 16 * 6);
	if (maxY >= 0 && maxY < 10) {
		oledPutROMString((ROM_STRING)"  ", 0xB7, 18 * 6);
		oledPutString(itoa(maxY, buf), 0xB7, 20 * 6);
	}
	else if (maxY>10 && maxY < 100) {
		oledPutROMString((ROM_STRING)" ", 0xB7, 18 * 6);
		oledPutString(itoa(maxY, buf), 0xB7, 19 * 6);
	}
	else if (maxY>100 && maxY < 1000) {
		oledPutString(itoa(maxY, buf), 0xB7, 18 * 6);
	}

	// axisZ indicator
	axisZ = axisZ | BMA150_ReadByte(0x07);
	axisZ = axisZ << 2;
	temp = temp | (BMA150_ReadByte(0x06));
	temp = temp >> 6;
	axisZ = axisZ | temp;

	// if board is up-side down
	// reset global vars
	if (axisZ & 0x200) {
		resetGlobalVars();
	}

	// Z value indicator (comment this do disable view) ALMOG
	//oledPutROMString((ROM_STRING)"Z:", 0xB4 ,9*6);
	//oledPutString( itoa(axisZ,buf) , 0xB4 , 11*6);

	// function call to print axis
	printAxis();
}

// main
void main(void)
{
	int mask = 0x11;			 		// mask to enable	
	output[3] = '\0';					// If we wanna print the output[] array, make sure the last cell is '\0'	
	InitializeSystem();
	ADCON0 = mask;						// ADCON0 should be redefined only after InitializeSystem();

	while (1)							//Main is Usualy an Endless Loop
	{
		ADCON0 |= 0x02;					// GO
		while (ADCON0 & 0x02);			// While Enable && GO

		potentziomeneter();
		buttonListener();
		arrowsListener();
		temperature();
		accelerometers();

		//sprintf(output, "%x", ans);		// from HEX to char[]
		//WriteCommand(0x81);				// will fade the screen
		ADCON0 = 0x13;
	}
}//end main


