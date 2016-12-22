/*
 Name:		PondMonitor4.ino
 Created:	1/10/2016 6:59:04 PM
 Author:	jimfr
*/

#include <inttypes.h>
#include <Event.h>
#include <Timer.h>			// timer functions, used for all the soft interupts and taking actions at intervals
#include <Time.h>			// time functions, used by Real Time Clock (RTC)
//#include <TimeLib.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <DS1307RTC.h>		// library for RTC module
#include <avr/pgmspace.h>	// library for memory optimization using flash mem
#include <SD.h>				// library for SD card
#include <OneWire.h>		// library for one-wire (I2C) bus used by temperature sensors
#include <DallasTemperature.h>	// library for temperature sensors used
#include <avr/wdt.h>		// watchdog timer used to reset the system after global parameters are changed.  Note this header is built into the compiler

#define Debug			//general debug switch
#ifdef Debug
#define dprint(p)	Serial.print(p)
#define dprintln(p)	Serial.println(p)
#define dprint2(p,u)	Serial.print(p,u)
#define dprintln2(p,u)	Serial.println(p,u)

#else
#define dprint(p)
#define dprintln(p)
#define dprint2(p,u)
#define dprintln2(p,u)
#endif // DEBUG

#define DebugDisplayClass			// debug switch for display class
#ifdef DebugDisplayClass
#define ddcprint(p)	Serial.print(p)
#define ddcprintln(p)	Serial.println(p)
#else
#define ddcprint(p)
#define ddcprintln(p)
#endif // DEBUG display class



File SDfile;			// file object for accessing SD card
Timer Tmr;				// timer object used for polling at timed intervals, used by keyboard routines, display class, and LEDs class
Timer SensTmr;			// timer object used for sensors (temp, flow, water level)
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);	// LCD display
tmElements_t SysTm;		// system time, used by RTC and time logic
String SysTmStr;		// system time as string, format HH:MM using 24 hr time. derived from SysTm
String SysDateStr;		// system date as string, format mm/dd/yyyy. derived from SysTm
String sysDOWstr;		// system day of week as string, 3 chr length.
String LogTm;			// string of date and time used for log functions formatted for XML "2016-07-27T00:00:00"
int SysTmPoleContext;	// ID of timer used to poll system time
#define SysTmPoleFreq 1000	// time polling frequency
#define SetUpDelay 750		// delay used during setup section. It is the length of time to show user a message during system self test done during setup.

//---------------------------------------------------------------------------
//			variables to control global state
//---------------------------------------------------------------------------
//flags determine which section of main loop code processes
boolean	InMonitoringMode = true;	//if true, then system is in monitoring mode and sensors are sampled/recorded
boolean TempSensorsOn, FlowSensorsOn, WaterLvlSensorsOn, RelayBoardOn;	// sensors on/off for the purposes of monitoring.  Values read in from SysStat.txt.  User can change these with UI, but system reads state at startup
String	PumpMode;											// relay state read in from SysStat.txt. Auto=controlled by water level sensor logic, on/off are manual 
boolean InFlowSensTestMode = false;		// for testing and setup of flow sensors
//boolean InFlowSens1TestMode = false;	// for testing and setup of flow sensor #1
boolean InTempSensTestMode = false;	// for testing and setup of temperature sensor #0 & #1
//boolean InTempSens1TestMode = false;	// for testing and setup of temperature sensor #1
boolean InWaterLvlTestMode = false;	// for testing and setup of water level sensor
boolean	TransmitReadings = false;	// if true, then system will radio sensor readings to companion system to upload to cloud

/* variables to monitor duration of main loop.  Used to check if time used processing main loop exceeds nyquist frequency of polling sensors*/
long MainLoopStartTime;
long MainLoopEndTime;
long MainLoopDurration;


//----------------------------------------LS keypad related variables-----------------------------------------------
//
// This library is written for the LinkSprite keypad shield for the Arduino. It is intended to simplify using the keypad
// by allowing code using this class to check a flag to know when a key has been pressed, read the key, and not need
// to do anything other than check for the next key press.
//
// The KeyPoll method is used to start/stop polling. Once started, the class uses the Timer class to poll the keys at intervals (default 30 ms).
// When a button press is detected, the key is debounced by making sure it stays the same for a debounce period (default 5 ms)
// Once debounced, the key is ready to be read.  The public variable LS_KeyReady is true when a key is ready to be read.  The code that uses this class should check this variable to know when to read it.
// When the key is ready to be read, it is fetched using the ReadKey method
// ReadKey toggles the LS_KeyReady flag so that the code will see the key press only once.  Key releases are ignored.
//
// The library leverages the timer class to handle polling and debouncing

#define LS_AnalogPin  0			// analog pin 0 used by keypad
#define LS_Key_Threshold  5		// variability around the analogue read value settings

//keys on LinkSprite keypad-LCD shield are read through a single analog input using a resistor voltage divider setup.
// below are the values read on the A/D converter for the respective keys
#define UPKEY_ARV  144
#define DOWNKEY_ARV  329
#define LEFTKEY_ARV  505
#define RIGHTKEY_ARV  0
#define SELKEY_ARV  742
#define NOKEY_ARV  1023

// constants to ID keys pressed
#define NO_KEY 0
#define UP_KEY 3
#define DOWN_KEY 4
#define LEFT_KEY 2
#define RIGHT_KEY 5
#define SELECT_KEY 1

// constants for configuration
const unsigned long LS_KeyPollRate = 20;			//default poll rate looking for keys pressed = 20 ms
const unsigned long LS_KeyDebounceDelay = 10;		//how long to wait to confirm key press is real...aka debouncing

													//variables
boolean LS_KeyReady; 	// if true, then there is a new key reading ready to read. for use by other classes and/or in the main loop
int LS_keyPin;
int LS_curInput;
int LS_curKey;
int LS_prevInput;
boolean LS_keyChanged;		// if true, key changed since last polling
int	LS_PollContext;	// ID of timer used to poll keypad
int	LS_DebounceContext;	//ID of timer used to debounce keypad

/*----------------------------------------------- ProgMem Routines ----------------------------------------------------*/
String ProgMemGetStr(const char* LUwhich, unsigned int LUlen)
{
	/*
	ProgMem library has functions to store const char arrays in program memory space for later retrieval, thus saving SCRAM.
	This routine returns a string stored in program memory that is needed by the calling routine.
	Strings are saved in program memory as a char array which are read out one char at a time and appended to the String that is returned.
	*/
	dprintln("reading from ProgMemGetStr");	//debug
	String	RetString;					//holds the string that will be returned.
	char	ChrFromString;				//holds the character returned from the string stored in Program memory
	for (unsigned int x = 0; x<< LUlen; x++)
	{
		ChrFromString = pgm_read_byte_near(LUwhich + x);
		dprint(ChrFromString);	//debug
		RetString += ChrFromString;
	}
	dprintln("RetString=" + RetString);	//debug
	return RetString;
}

//-----------------------------------------LED indicator variables and class ---------------------------------------
class LEDs
{

	/*
	Variables for the green and red LEDs.  Green is used to indicate monitoring is going well, red indicates when there are errors.
	LED's are wired to power with the ground wired to a digital pin.  The LED is turned on by bringing the pin low.  The digital pins can sink more current than source.


	Red LED used to report 3 levels of error.  Level 1 is minor and LED blinks slowly, level 2 is more serious and LED blinks rapidly, levle 3 is severe and LED stays on.
	CurrentErrLvl is static and ranges from 0-3, where 0 = no error
	If a called with an error level higher than current level, then set to new level else not changed.  This is because this routine can
	be called from different parts of the code.
	If called with err level 0, the error LED is turned off (reset error)
	*/

#define RedLEDpin 44
#define GreenLEDpin	45
#define lvl1delay	500		//blink rate for minor error
#define lvl2delay	250		//blink rate for moderate error	/*
private:
	byte	ErrLEDlvl;			//value 1-3 to represent increasing severity of error, where 0=no error
	boolean	GreenLEDstate;		// saves the state of the green LED, true=on
	boolean	RedLEDstate;		// saves the state of the red LED, true is on
	int		LEDtmrContext;		// index for timer class used to toggle red LED

public:
	void	LEDinit(void);		//initialize status LEDs
	void	SetErrLED(byte level);	// sets the red LED to error level and adjusts blink rate
	void	ClearErrLED(void);	// clears the error level and turns off red LED
	void	SetGreenLED(boolean turnOn);	// used to turn green LED on/off
	void	ErrLEDtoggle(void);	//called at interval to toggle the red LED based on error level
}statusLEDs;
//----------------------------------------------------------------------
void	LEDs::LEDinit(void)
{
	// initialize status LEDs and initialize them to off
	pinMode(RedLEDpin, OUTPUT);	// red LED attached here, 
	digitalWrite(RedLEDpin, 1);	// turn off red LED
	pinMode(GreenLEDpin, OUTPUT);
	digitalWrite(GreenLEDpin, 1);
	GreenLEDstate = false;
	RedLEDstate = false;
	ErrLEDlvl = 0;	// initialize to no error
}
//----------------------------------------------------------------------
void	LEDs::SetErrLED(byte level)
{
	// sets the red LED to error level and adjusts blink rate
	byte	tmpByte;
	unsigned long int		tmpInt;
	if (level > ErrLEDlvl)
	{
		//only do something if the error level is to be set at a level higher than it already is
		if (level < 1 || level > 3)
		{
			//unexpected calling parameters, report error and do nothing
			//can't log it because this routine called by error log, so will create a loop
			dprint("unexpected parameter in SetErrLED="); dprintln(level);
		}
		else
		{
			// valid error level beign requested that is higher than current level
			ErrLEDlvl = level;	//save the level for reference
			RedLEDstate = true;
			digitalWrite(RedLEDpin, 0);	//pin acts as sink, turning on LED

			if (level == 3)
			{
				// error level = 3, so want to turn LED on, no flashing
				if (level != 0) Tmr.stop(LEDtmrContext);	//turn timer off if it is already flashing	
			}
			else
			{
				if (level == 1) tmpInt = lvl1delay; else tmpInt = lvl2delay;
				LEDtmrContext = Tmr.every(tmpInt, ErrLEDtoggleRedirect, (void*)2);	// call ErrLEDtoggleRedirect at intervals to flash red LED
			}
		}
	}
}
//----------------------------------------------------------------------
void	LEDs::ClearErrLED(void)
{
	// clears the error level and turns off red LED
	digitalWrite(RedLEDpin, 1);	// brings pin high and turns off LED (no longer sinking)
	RedLEDstate = false;
	if (ErrLEDlvl == 1 || ErrLEDlvl == 2) Tmr.stop(LEDtmrContext);	//turn off timer if being used to flash
	ErrLEDlvl = 0;	//0=no error
}
//----------------------------------------------------------------------
void	LEDs::SetGreenLED(boolean turnOn)
{
	// used to turn green LED on/off
	GreenLEDstate = turnOn;
	digitalWrite(GreenLEDpin, !turnOn);	// need to invert because 0=turn on
}
//----------------------------------------------------------------------
void	LEDs::ErrLEDtoggle(void)
{	//called at interval to toggle the red LED based on error level
	RedLEDstate = !RedLEDstate;
	digitalWrite(RedLEDpin, RedLEDstate);
}
//----------------------------------------------------------------------
void	ErrLEDtoggleRedirect(void * context)
{
	// called by timer class at intervals when red LED is flashing
	statusLEDs.ErrLEDtoggle();	//use class to toggle.  routine called by timer class outside of object because compiler cannot find correct instance or I don't know how to tell it to do so :-)
}
//----------------------------------------------------------------------


/* --------------------------------------------------- Log Routines --------------------------------------------------*/
/*
Routines used to log errors to SD card. Used throught code/classes that follow.
Error log is in xml format containing a date, log entry, and error level (used to determine action e.g. turn on red LED).  XML strings are 
saved in program memory to conserve space using the PROGMEM library.  Strings are read from program memory using DisplayClass::ProgMemLU

The file is located in the log directory of the SD card and is named "log/errorlog.xml"  It has a header and footer xml.  TPondMonitor assumes that the file is there and has the 
correct structure.  It opens the file and inserts the log entry before the footer.

xml header
	header is not used by the routine but must already exist in the file.
	<?xml version="1.0" encoding="UTF-8"?>
	<dataroot xmlns:od="urn:schemas-microsoft-com:officedata" generated="2016-07-27T08:46:45">

XML format for error log entry
	<ErrorLog>
		<LogDate> enter date here, use format 2016-07-28T00:00:00 </LogDate>
		<LogEngry> log entry goes here, no quotes, max len 255 </LogEngry>
		<LogLevel> log error level goes here, must be 1-3 </LggLevel>
	</ErrorLog>

XML footer
	</dataroot>

Error levels:
	error level 1=hardware failure, e.g. RTC fails to init
	error level 2= unexpected finding in software, e.g. didn't recognize a menu option from a limited set
	error level ?  other error levels may be needed in the future, but so far, only 1&2 are defined.
*/

void ErrorLog(String error,String param,int errLevel)
{
	// writes error to errorlog.  If not able to write to errorlog, on SD card, writes to monitor and turns on Red alarm.
	// Note that only one file can be open at a time, so you have to close this one before opening another.
	// Note, may need to add transaction for SPI bus becuse SD card uses SPI bus and so do other devices on this system.

	SDfile = SD.open("log/errorlog.xml", FILE_WRITE);				// try to open the file
	if (SDfile)
	{ 
		/*file opened ok if true
		Error log is a XML file.  we want to open the file and insert the error string at the appropriate place, which is just before the xml to end the data root.
		*/
		SDfile.seek(SDfile.size() - 13);	//seek to just before '</dataroot>"
		//make entry in XML and close file
		SDfile.print(F("<ErrorLog><LogDate>"));
		SDfile.print(LogTm);
		SDfile.print(F("</LogDate><LogEntry>"));
		SDfile.print((error+" "+param));		//print error plus paramerer
		SDfile.print(F("</LogEntry><LogLevel>"));
		SDfile.print(errLevel);
		SDfile.println(F("</LogLevel></ErrorLog></dataroot>"));

		SDfile.close();	// close the error log
	}
	else
	{ 
		//error log couldn't be opened, this is a serious error but will continue processing
		statusLEDs.SetErrLED(3);	//error level 3 is max, LED stays on
		dprintln("Cannot open error log file");	//debug
	}
}
//---------------------------------------------------------------------------
void MonitorLog(String name, String ID, String valueStr, float valueFloat, int valueInt, String unitStr, String value1Str, float value1Float, int value1Int, String unit1Str)
{
	/*
	code used by all sensors to make log entry of their readings.  This is general purpose to accomidate a name, ID, and u values.
	The min required info is name, value, and units.  There is no completeness or validity checking done.  If there is an error with the SD card
	this routine will log it.  The sensors will not be made aware of an SD card issue.
	The log file name is "log/SensLog.xml" located on the SD card.  This routine assumes it already exists and the routine needs to append new entries.  The
	XML structure of the file is as follows:

		<?xml version="1.0" encoding="UTF-8"?>
		<dataroot xmlns:od="urn:schemas-microsoft-com:officedata" generated="2016-11-19T08:49:49">
			<MonitorLog>

			//data from first value.  e.g. temp sens outputs in degF and degC.  First value is the most important.
			<LogDate>2016-11-19T08:46:07</LogDate>
			<SensName>name string 20 chr max</SensName>
			<SensID>int of sensor ID, 0 if doesn't exist</SensID>
			<valueString>string representation of the first value logged by the monitor.</valueString>
			<valueFloat>if value being output is a float, then it goes here. if not used, 0 </valueFloat>
			<valueInt>if value being output is an int, then it goes here. if not used, 0</valueInt>
			<unitStr>units string, e.g. "l/min", if not used, " "</unitStr>

			//data from second value, optional.   
			<SensName1>name string 20 chr max</SensName1>
			<SensID1>int of sensor ID, 0 if doesn't exist</SensID1>
			<valueString1>string representation of the first value logged by the monitor.</valueString1>
			<valueFloat1>if value being output is a float, then it goes here. if not used, 0 </valueFloat1>
			<valueInt1>if value being output is an int, then it goes here. if not used, 0</valueInt1>
			<unitStr1>units string, e.g. "l/min", if not used, " "</unitStr1>
			</MonitorLog>
		</dataroot>

	What this routine needs to do is 
	1) open the file for write
	2) seek to the end and back the pointer up to just before "</dataroot>"
	3) make the log entry using the format above for <MonitorLog>
	4) close the file
	note, if an error occurrs with the SD drive, let the SD code handle it
	*/
	SDfile = SD.open("log/SensLog.xml", FILE_WRITE);				// try to open the file
	if (SDfile)
	{
		/*file opened ok if true
		Sensor log is a XML file.  we want to open the file and insert the sensor data at the appropriate place, which is just before the xml to end the data root.
		*/
		SDfile.seek(SDfile.size() - 13);	//seek to just before '</dataroot>"
		//make entry in XML and close file
		SDfile.print(F("<MonitorLog><LogDate>"));
		SDfile.print(LogTm);
		SDfile.print(F("</LogDate><SensName>"));
		SDfile.print(name);
		SDfile.print(F("</SensName><SensID>"));
		SDfile.print(ID);
		SDfile.print(F("</SensID><valueString>"));
		SDfile.print(valueStr);
		SDfile.print(F("</valueString><valueFloat>"));
		SDfile.print(valueFloat);
		SDfile.print(F("</valueFloat><valueInt>"));
		SDfile.print(valueInt);
		SDfile.print(F("</valueInt><unitStr>"));
		SDfile.print(unitStr);
		SDfile.print(F("</unitStr><valueString1>"));
		SDfile.print(value1Str);
		SDfile.print(F("</valueString1><valueFloat1>"));
		SDfile.print(value1Float);
		SDfile.print(F("</valueFloat1><valueInt1>"));
		SDfile.print(value1Int);
		SDfile.print(F("</valueInt1><unitStr1>"));
		SDfile.print(unit1Str);
		SDfile.print(F("</unitStr1></MonitorLog>"));

		SDfile.close();	// close the sensor log file
	}
	else
	{
		//error log couldn't be opened, this is a serious error but will continue processing
		ErrorLog("Could not write to sensor log for sensor name=",name,3);
	}
}

//--------------------------Display Class Definition and Display Related Global Variables -----------------------------

//buffer used to  load/save string arrays used for Display object.  This is max 80 chr X 7 lines
#define DisplayBufLen 100	// max len of strings in DisplayBuf
String DisplayBuf[10];		// buffer used to work with DisplayArrays. Read/written from SD card by Display object

//-------------------------------------------
//preset the strings used for data entry using the methods in the Display class.  
//Defined here instead of in the class because I couldn't figure out how to get it to work within the variables declared for the class.
//Note, these are stored in program memory to conserve space using DisplayClass::ProgMemLU and the PROGMEM library.

const char	DisplayDay[] PROGMEM = { "01020304050607080910111213141516171819202122232425262728293031" };	// day will advance 2 chr at a time
const char DisplayMonth[] PROGMEM = { "010203040506070809101112" };	// month will advance 2 chr at a time
const char DisplayYear[] PROGMEM = { "0123456789" };	// year will advance 1 chr at a time
const char DisplayHour[] PROGMEM = { "000102030405060708091011121314151617181920212223" };	//hour will advance 2 chr at a time
const char DisplayMin[] PROGMEM = { "00010203040506070809101112131415161718192021222324252627282930313233343536373839404142434445464748495051525354555657585960" };	// Min will advance 2 chr at a time
const char DisplayNum[] PROGMEM = { "0123456789" };	// numbers advance 1 chr at a time
const char DisplayChar[] PROGMEM = { " abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_$%&" };	//alpha numeric characters will advance 1 at a time
const char DisplayDOW[] PROGMEM = { "MonTueWedThuFriSatSun" };

class DisplayClass
{
	/*
	The display class supports user interface that leverages the LinkSprite keypad shield for the Arduino, which includes 5 keys (excluding reset) and a 2 line LCD
	display.  User interface elements supported by the Display class include menu processing, data entry, and message display. There are two main goals of this class:
	1) to minimize the amount of effort needed to support a user interface.
	2) to enable the code in the main loop to continue while the user is interacting with the display.  That is, the main loop needs to poll a flag that will be
	set by the Display class when the user is done.

	The Display class leverages an array of String objects. The array is referred to as a display array
	and each String in the array is referred to as a DisplayLine.  The contents of the DisplayLine are used by the class to support menu option selection, data entry,
	message review, and data display. When using the Display class, the programmer defines the display arrays needed for their program, sets flags to tell other
	parts of the code that the display is in use, activates keypad polling, and points the Display class to the display array.  Display class does its thing
	and hands results back to the program when the user is done interacting with the DisplayLines in the Display array.  For example, a single Display array
	can be used to support viewing and setting a real time clock.  Display array will handle instructions, entry of date, time, day of week, and menu options to 'update' or 'cancel'.
	After the call to set up the Display, the class only set the flag to indicate the user is done when the user selects 'update' or 'cancel'.

	The display array is an array of DisplayLines, where a DisplayLine is a String object with 4 fields that are comma separated
	1st element: DisplayLine name- this is passed out to the main loop along with the display array name when the user completes an operation
	2nd element: template, which has the following values/format.  The characters in the templates tell the code how to process Rt, Lt, Up, Down and Select keys.
	-"Display", this indicates that the display line is a Display that the user can scroll through to choose an option
	key-right and key-left move to Display options.  key-up moves to previous display line in the array, key-down moves to next display line, enter passes Display option out
	-"text", this indicates that the display line is a line of text.  Usually used to display a message or instructions
	key-right and key-left move through the text,  key-up moves to previous display line in the array, key-down moves to next display line, Select has no effect
	-"m--d----yy--U-D", this is for entering a date.  It gives the position in the actual display line of the components of a date
	key-right and left move to the next non'-' in the template. Key-up and down use the chr in the template to determine what to do: increment or decrement MM, dd, yy, move up or down in the display line array. Select has no effect
	-"H--M---U-D", this is for entering time in 24 hr format.  It gives the position in the actual display line for the components of time.
	key-right and left move to the next non'-'. Key-up and down use the chr in the template to determine what to do. Increment or decrement the HH, MM, or move up or down in display line array. Select has no effect
	-"-----##--U-D", this is for entering a numeric values.  The "-" can be variable length. '#" can have '.' there.  e.g. '##.#. The routine does the following
	key-right and left move to the next non '-'chr. Key-up and down use the chr in the template to determine what to do. Increment or decrement the #, or move up or down in display line array. Select has no effect
	-"--CCCCCCC--U-D", this is for entering alpha numeric values.  Key-right and left move to the next non'-' chr.  Key-up and down use the chr in the template to determine what to do.  Increment or decrement the alphanumeric or move up or down in the display line array.  Select has no effect.
	3rd element: Display line title....occupies the 1st line of the display
	4th element: actual display line....occupies the 2nd line of the display
	*/

	#define mset  true		//for display set/get functions, dset-->write the value
	#define mget  false		//dget-->get the value
	#define mReadOnly  true	//for DisplaySetup--> menu is ReadOnly
	#define mReadWrite  false // for DisplaySetup--> menue is read/write
	#define mUseSD  true	// for DisplaySetup--> menu array comes from SD

protected:
	boolean DisplayInUse;	// if true, then the display array is in use.  Used by ProcessDisplay to only process key presses when Display is active. Needed because other parts of the program could be using the keypad.
	int	DisplayLineCnt;		// count of lines in string matrix making up display array
	int	DisplayIndex;		// index to DisplayLine being displayed
#define MaxDisplayArrayLen 10	//max number of display line strings in a display array
	String *DisplayPntr;	// pointer to String array that is the Display
	int DisplayMode;	// TemplateLine is interpreted and this variable is set to indicate if the display line being process is Display=1, text=3, or other=2 (date,time, alphanumeric)
	boolean DisplayReadOnly;	//display lines can sometimes be for input, sometimes just for display.  this variable is passed in  when the display deck is set up, and indicates if it is read only or read write
#define DisplayDisplayLen 16	//length of the Display display, 16 chr.
	String DisplayLine;	// storage of display line
	String BlinkLine;	// string used by soft interrupt to make simulate a blinking cursor
	String BlinkCursor;	// character used for cursor.  if read only= O else =*
	String TemplateLine;	// string used to determine how to process the display line. e.g. Display, text, time, date, numeric input
	unsigned int DisplayStartPos, DisplayEndPos, DisplayPos;	//starting, ending, and current position indices of the DisplayLine being processed.  Used for scrolling in small displays
	unsigned int DisplayOptStart, DisplayOptEnd;			// starting and ending position of a selection in the DisplayLine.  Used for returning the string of the Display option selected
	unsigned int MaxLinePos;		// max line position to use for printing to LCD.  Takes 16 chr into account.  Used for Substring

	int DisplayAdvPastSpace(String Mline, int Start);
	int DisplayAdvToSpace(String Mline, int Start);
	int DisplayBkupPastSpace(String Mline, int Start);
	int DisplayBkupToSpace(String Mline, int Start);
	void DisplayLineSetup(String Mline); //takes the input string and parses it into parameters and the Displayline. The format for the input string is: DisplayLineName,Title,DisplayLine
	void CursorBlink(boolean action);

	//Methods for incrementing/decrementing characters for data entry
	void MonthEntry(boolean increment);	//increments or decrements 2 digit month at DisplayPos in DisplayLine
	void DayEntry(boolean increment);	// increments or decrements 2 digit day at DisplayPos in DisplayLine
	void YearEntry(boolean increment);	// increments or decrements 2 remaining digits in the year, one digit at a time 
	void DowEntry(boolean increment);	// increments or decrements day-of-week (Mon, Tues, etc)
	void HourEntry(boolean increment);	// increments or decrements hours, 2 digits at a time
	void MinuteEntry(boolean increment);	// increments or decrements Minutes, 2 digits at a time
	void NumEntry(boolean increment);	// increments or decrements numbers, 1 digit at a time
	void ChrEntry(boolean increment);	// increments or decrements alphanumerics


public:
	boolean FindAndParseDisplayLine(String MnuLineName, int *Idx, String *DisplayTitle, String *TemplateLn, String *DisplayLn); // Used by methods to get/set variables within display (Display) arrays. Find and parse the display line named MnuLineName in the current display array. Parse out the index to the display line, the template line, and the display line.
																																// make above protected after debug

	DisplayClass(void);
	String DisplayName;		// static variable holding name of Display, passed to the Display routines by reference
	String DisplayLineName;	// context for the Display operation.  when user selects an option, code passes out the name of the Display line and the selection
	String DisplayLineTitle;	// title string for the DisplayLine.  used on 1st line of display
	String DisplaySelection;	// option that the user selected in the DisplayLine
	boolean DisplayUserMadeSelection;	// used to poll if Display result is ready to be read.  If true, result can be read by DisplayGetResult()

	void DisplayStartStop(boolean action);	//Used to indicate that the Display is in use and that keypresses should be processed by the Display routines
	void DisplaySetup(boolean isReadOnly, boolean readFromSD, String mnuName, int mnuLines, String *mnu); //sets up the Display.  needs to be passed the name, number of lines, and pointer to an array of strings (formatted as DisplayLines)
	void DisplayLineRefresh(String LineName);	// Refreshes the display line if the current display line== LineName.  Used to update something that is changing. e.g. when testing temp, this is used to show the temp changine.
	void ProcessDisplay(int KeyID); //Main processing routine for Display.  This is called after user has pressed a key (up, dn, rt, lt, or Select) that has been debounced by the LS_Key routines
	void CursorBlinkTimeInt(void);	// soft interrupt routine to make 'cursor' blink 
	boolean DisplayGetSetDate(String *DateStr, String MnuLineName, boolean set);	// gets or sets date in the display line named MnuLineName in the current display array.  date format is mm/dd/yyyy.  if Set is true then sets value of DateStr else gets value
	boolean DisplayGetSetTime(String *TimeStr, String MnuLineName, boolean set);	// gets or sets time in the display line named MnuLineName in the current display array.  time format is HH:MM.  if Set is true then sets value of DateStr else gets value
	boolean DisplayGetSetDOW(String *DayStr, String MnuLineName, boolean set);		// gets or sets day of week (DOW) in the display line named MnuLineName in the current display array.  DOW format is 3 chr abbreviation, see variable DisplayDOW[7] for values  if Set is true then sets value of DateStr else gets value
	boolean DisplayGetSetNum(String *NumStr, String MnuLineName, boolean set);		// gets or sets a numeric value in the display line named MnuLineName in the current display array. If Set is true then sets value of DayStr else gets value
	boolean DisplayGetSetChrs(String *ChrStr, String MnuLineName, boolean set);	// gets or sets a character string in the display line named MnuLineName in the current display array. If Set is true then sets value of ChrStr else gets value
	boolean DisplaySetTxt(String *TxtStr, String MnuLineName);						// sets a text message in the display line named MnuLineName in the current display array. Get not needed as this is only for outputting messages
	boolean DisplayWriteSD(void);													// writes the current display array to a file on the SD card.  Uses DisplayPntr, DisplayName, and DisplayLineCnt.  file is named DisplayName and is overwritten.  returns true if successful

																					//Methods for optimizing SCRAM by using program memory to store const strings. Actual const char * are declaired below outside of the class because I couldn't get them to work here.
	String ProgMemLU(const char* LUwhich, unsigned int LUwhere, unsigned int LUlen);	// uses PROGMEM library to retrieve substrings of char arrays stored in program memory for RAM conservation

} Display;

//-------------------------------------------------Display Class routines----------------------------------------------
DisplayClass::DisplayClass(void)
{
	//constructor
	DisplayPos = 0;
	DisplayLine = "01234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890012345678901234567890012345678901234567890";	//reserve 140 chr length

};
//------------------------------------------
int DisplayClass::DisplayAdvPastSpace(String Mline, int Start)
{
	//starting  at a space in a string, return the position of the next non-space chr
	//dprint("in DisplayAdvPastSpace, values passed in: Mline=|"); dprint(Mline); dprint("|, start="); dprintln(Start);
	unsigned int x = Start;
	while (Mline[x] == ' ' && x < Mline.length() + 1)
	{
		x++;	//advance until encounter a character other than space
	}

	//dprint("chr and index='"); dprint(Mline[x]); dprint("' , "); dprintln(x);//dprint ('|');
	//dprint(Mline[x]);
	//dprintln('|');
	return x;	// return the index of the non-space chr.
}
//------------------------------------------
int DisplayClass::DisplayAdvToSpace(String Mline, int Start)
{
	//starting with a non-space chr in a string, return the  position of the next space chr
	return Mline.indexOf(' ', Start);
}
//------------------------------------------
int DisplayClass::DisplayBkupPastSpace(String Mline, int Start)
{
	//starting  at a space in a string, return the position of the previous non-space chr
	int x = Start;
	while (Mline[x] == ' ' && x >= 0)
	{
		x--;	//backup until encounter a character other than space
	}

	//dprint ('|');
	//dprint(Mline[x]);
	//dprintln('|');
	return x;	// return the index of the non-space chr.
}
//------------------------------------------
int DisplayClass::DisplayBkupToSpace(String Mline, int Start)
{
	//starting with a non-space chr in a string, return the  position of the previous space chr
	int x = Start;
	while (Mline[x] != ' ' && x >= 0)
	{
		x--;	//backup until encounter a space
	}

	//dprint ('|');
	//dprint(Mline[x]);
	//dprintln('|');
	return x;	// return the index of the space chr.
}
//------------------------------------------
String DisplayClass::ProgMemLU(const char* LUwhich, unsigned int LUwhere, unsigned int LUlen)
{
	/*
	ProgMem library has functions to store const char arrays in program memory space for later retrieval, thus saving SCRAM.
	This routine returns a portion of the chr array stored in program memory that is needed by the calling routine.
	LUWhich points to the char array in program memory. The char array holds substrings that are of equal length LUlen,
	and LUwhere serves as an index to the substring to return, (first substring is 0).
	*/

	String	substr;					//holds the substring that will be returned.  Static so it will persist after exiting this routine
	char	ChrFromString;			//holds the character returned from the string stored in Program memory
	if (LUwhere < 0) { LUwhere = 0; }	// prevents reading outside the string in the event that the calling parameters are in error
	unsigned int start = LUwhere * LUlen;		//starting location to read string
	if (start > strlen_P(LUwhich) - LUlen) { start = 0; }	// prevents reading outside of the string in the event that the calling parameters are in error

	for (unsigned int x = start; x<start + LUlen; x++)
	{
		ChrFromString = pgm_read_byte_near(LUwhich + x);
		substr += ChrFromString;
	}
	return substr;
}
//------------------------------------------
void DisplayClass::MonthEntry(boolean increment)
{
	// DisplayPos is pointing to first digit of 2 digit month.  Increment or decrement month using strings in display month
	int idx;
	String tmpStr, MonthStr;
	tmpStr = DisplayLine.substring(DisplayPos, DisplayPos + 2);
	dprintln(tmpStr);
	idx = tmpStr.toInt() - 1;	//month # -1 = index # due to index starting with 0
	dprintln(idx);

	if (increment)
	{
		if (idx <11) { idx++; }
		else { idx = 0; }
	}
	else
	{
		if (idx > 0) { idx--; }
		else { idx = 11; }
	}

	MonthStr = ProgMemLU(DisplayMonth, idx, 2);	//retrieve the 2 character portion (MM) of DisplayMonth indexed by idx
												//dprintln(MonthStr);dprint("displayPos="); dprintln(DisplayPos);
												//dprint("0-DisplayPos=|"); dprint(DisplayLine.substring(0,DisplayPos)); dprint("| displayPos+2-->len=|"); dprint( DisplayLine.substring (DisplayPos+2,DisplayLine.length()));dprintln("|");
	tmpStr = DisplayLine.substring(0, DisplayPos) + MonthStr + DisplayLine.substring(DisplayPos + 2, DisplayLine.length());
	//replace the month 
	//dprint("|"); dprint(DisplayLine); dprintln("|");	//debug
	//dprint("|"); dprint(tmpStr); dprintln("|");//debug
	DisplayLine = tmpStr;
	//dprint("|"); dprint(DisplayLine); dprintln("|");	//debug
}
//------------------------------------------
void DisplayClass::DayEntry(boolean increment)
{
	// DisplayPos is pointing to first digit of 2 digit day.  Increment or decrement day using strings in display month
	int idx;
	String tmpStr, DayStr;
	tmpStr = DisplayLine.substring(DisplayPos, DisplayPos + 2);
	idx = tmpStr.toInt() - 1;	//day # -1 = index # due to index starting with 0

	if (increment)
	{
		if (idx <30) { idx++; }
		else { idx = 0; }
	}
	else
	{
		if (idx > 0) { idx--; }
		else { idx = 30; }
	}

	DayStr = ProgMemLU(DisplayDay, idx, 2);	//retrieve the 2 character portion (dd) of DisplayDay indexed by idx
	tmpStr = DisplayLine.substring(0, DisplayPos) + DayStr + DisplayLine.substring(DisplayPos + 2, DisplayLine.length());	//replace the month
																															//dprint("|"); dprint(DisplayLine); dprintln("|");	//debug
																															//dprint("|"); dprint(tmpStr); dprintln("|");//debug
	DisplayLine = tmpStr;
}
//------------------------------------------
void DisplayClass::YearEntry(boolean increment)
{
	// DisplayPos is pointing to a digit in the last 2 digits of the year. Increment or decrement year using strings in display month
	int idx;
	String tmpStr, YrStr;
	tmpStr = DisplayLine.substring(DisplayPos, DisplayPos + 1);
	idx = tmpStr.toInt();

	if (increment)
	{
		if (idx <9) { idx++; }
		else { idx = 0; }
	}
	else
	{
		if (idx > 0) { idx--; }
		else { idx = 9; }
	}

	YrStr = ProgMemLU(DisplayYear, idx, 1);
	tmpStr = DisplayLine.substring(0, DisplayPos) + YrStr + DisplayLine.substring(DisplayPos + 1, DisplayLine.length());	//replace the month
																															//dprint("|"); dprint(DisplayLine); dprintln("|");	//debug
																															//dprint("|"); dprint(tmpStr); dprintln("|");//debug
	DisplayLine = tmpStr;
}
//------------------------------------------
void DisplayClass::DowEntry(boolean increment)
{
	// increments or decrements days of the week
	int idx = 0;
	String tmpStr, ChrStr;
	tmpStr = DisplayLine.substring(DisplayPos, DisplayPos + 3);

	while ((tmpStr != ProgMemLU(DisplayDOW, idx, 3)) && (idx<6)) idx++; //scan the string array for the day of the week under DisplayPos

	if (increment)
	{
		if (idx <6) { idx++; }
		else { idx = 0; }
	}
	else
	{
		if ((idx > 0) { idx--; }
		else { idx = 6; }
	}

	ChrStr = ProgMemLU(DisplayDOW, idx, 3);
	tmpStr = DisplayLine.substring(0, DisplayPos) + ChrStr + DisplayLine.substring(DisplayPos + 3, DisplayLine.length());	//replace the DOW
	DisplayLine = tmpStr;
}
//------------------------------------------
void DisplayClass::HourEntry(boolean increment)
{
	// increments or decrements hours, 2 digits at a time
	int idx;
	String tmpStr, HrStr;
	tmpStr = DisplayLine.substring(DisplayPos, DisplayPos + 2);
	idx = tmpStr.toInt();	//

	if (increment)
	{
		if (idx <23) { idx++; }
		else { idx = 0; }
	}
	else
	{
		if (idx > 0) { idx--; }
		else { idx = 23; }
	}

	HrStr = ProgMemLU(DisplayHour, idx, 2);
	tmpStr = DisplayLine.substring(0, DisplayPos) + HrStr + DisplayLine.substring(DisplayPos + 2, DisplayLine.length());	//replace the hours
	DisplayLine = tmpStr;
}
//------------------------------------------
void DisplayClass::MinuteEntry(boolean increment)
{
	// increments or decrements Minutes, 2 digits at a time
	int idx;
	String tmpStr, MinStr;
	tmpStr = DisplayLine.substring(DisplayPos, DisplayPos + 2);
	idx = tmpStr.toInt();	//

	if (increment)
	{
		if (idx <59) { idx++; }
		else { idx = 0; }
	}
	else
	{
		if (idx > 0) { idx--; }
		else { idx = 59; }
	}

	MinStr = ProgMemLU(DisplayMin, idx, 2);
	tmpStr = DisplayLine.substring(0, DisplayPos) + MinStr + DisplayLine.substring(DisplayPos + 2, DisplayLine.length());	//replace the hours
	DisplayLine = tmpStr;
}
//------------------------------------------
void DisplayClass::NumEntry(boolean increment)
{
	// increments or decrements numbers, 1 digit at a time
	int idx;
	String tmpStr, NumStr;
	tmpStr = DisplayLine.substring(DisplayPos, DisplayPos + 1);
	idx = tmpStr.toInt();

	if (increment)
	{
		if (idx <9) { idx++; }
		else { idx = 0; }
	}
	else
	{
		if (idx > 0) { idx--; }
		else { idx = 9; }
	}

	NumStr = ProgMemLU(DisplayNum, idx, 1);
	tmpStr = DisplayLine.substring(0, DisplayPos) + NumStr + DisplayLine.substring(DisplayPos + 1, DisplayLine.length());	//replace the Minutes
	DisplayLine = tmpStr;

}
//------------------------------------------
void DisplayClass::ChrEntry(boolean increment)
{
	// increments or decrements alphanumerics
	int idx = 0;
	String tmpStr, ChrStr;
	tmpStr = DisplayLine.substring(DisplayPos, DisplayPos + 1);

	while ((tmpStr != ProgMemLU(DisplayChar, idx, 1)) && (idx<68)) idx++; //scan the string array for the character under DisplayPos and advance by 1 with wraparound

	if (increment)
	{
		if (idx <68) { idx+=1; }
		else { idx = 0; }
		ddcprint(F("inc, idx=")); ddcprint(idx);
	}
	else
	{
		if (idx == 0) { idx = 68; } else { idx -= 1; }
		ddcprint(F("dec, idx=")); ddcprint(idx);
	}


	ChrStr = ProgMemLU(DisplayChar, idx, 1);
	tmpStr = DisplayLine.substring(0, DisplayPos) + ChrStr + DisplayLine.substring(DisplayPos + 1, DisplayLine.length());	//replace the Minutes
	DisplayLine = tmpStr;
}
//------------------------------------------
void DisplayClass::DisplayLineSetup(String Mline)
{
	/* This routine takes the input string and parses it into parameters and the DisplayLine. The format for the input string is as follows:
	DisplayLineName,TemplateLine,Title,DisplayLine
	the format for DisplayLine depends on the template.  If the DisplayLine is for a Display, then it consists of one or more options.See documentation is as follows:
	"option1   option2   option3   etc"
	note, there should be at least 2 spaces between DisplayLine options

	This routine parses out the DisplayLineName, TemplateLine, Title, and DisplayLine.  It then displays the Title on the first row of the display, and the DisplayLine on the
	second row of the display.  How the DisplayLine is processed is determined by the values in the TemplateLine.
	If TemplateLine='Display' then the DisplayLine is a list of Display options and the DisplayMode=1. The DisplayLine processing handles the left and right keys, moving the Display options accordingly.  When the user selects an option,
	processing will return the DisplayLineName and DisplayOption, both of which can be used to direct logic.  The Display processing will change DisplayLines based on
	the up and down buttons.  Details of DisplayLine and Display processing are addressed in appropriate areas of code.

	If TemplateLine='text' then the DisplayLine is informational text and the DisplayMode=3.  Right and left keys scroll as needed.  Up and down move to other display lines in the display array.

	If not "Display' or 'text', TemplateLine contains characters that support entry of date, time, numeric, and alpha numeric values and the DisplayMode=2

	For DisplayMode = 2: The displayline processing handles the left and right, moving the 'cursor' to the locations in the Display string for data entry.
	Using the up and down buttons increments/decrements the date/time/#/chr element accordingly.

	the select button works only in DisplayMode=1 (Display).  The intent of the display array is to combine Display lines +/- data entry.  One of the Display lines will contain
	an option to 'continue', after which the routine hands off to the program logic.  There are public routines that allow logic to set or read the data entered
	by the user.
	*/
	//dprint(F("DisplayLineSetup SCRAM="));dprintln(getFreeSram()); //debug
	int tmp1, tmp2, tmp3;
	String TemplateChar;							// used to process the TemplateLine
	tmp1 = Mline.indexOf(',');						// get position of comma, used to parse
	DisplayLineName = Mline.substring(0, tmp1);		// get DisplayLineName
	tmp2 = Mline.indexOf(',', tmp1 + 1);				// position of next comma
	TemplateLine = Mline.substring(tmp1 + 1, tmp2);	// get TemplateLine
	//dprintln("templateLine=|" + TemplateLine +"|");	
	tmp3 = Mline.indexOf(',', tmp2 + 1);				// position of next comma
	DisplayLineTitle = Mline.substring(tmp2 + 1, tmp3); // get DisplayLineTitle
	DisplayLine = "                        " + Mline.substring(tmp3 + 1, Mline.length());	//snip out the Display options and pre-pend with spaces
	DisplayEndPos = DisplayLine.length();					// set the end of the Display position pointer
	DisplayLine += "                    ";				// pad the DisplayLine with spaces at the end with Display line
														//dprint("DisplayLine=|"); dprint(DisplayLine); dprintln("|");
	DisplayStartPos = DisplayAdvPastSpace(DisplayLine, 0);	// find the first non-space chr
	DisplayPos = DisplayStartPos;							// position index used for scrolling

															//display the Display title and Display
	lcd.begin(16, 2);	//unclear why, but this is needed every time else setCursor(0,1) doesn't work....probably scope related.
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(DisplayLineTitle);
	lcd.setCursor(0, 1);

	//dprintln("DisplayLine=" + DisplayLine);
	//dprint("DisplayStartPos="); dprint(" DisplayEndPos="); dprint(DisplayEndPos); dprint(" DisplayPos="); dprintln(DisplayPos);

	if (DisplayEndPos - DisplayStartPos + 1>DisplayDisplayLen)
	{
		//length of the Display line is wider than display
		//dprintln("length of display line is wider than display");
		lcd.print(DisplayLine.substring(DisplayPos, DisplayPos + DisplayDisplayLen - 1) + '>');	// put up as much as will fit for the Display, with indicator that there is more
	}
	else
	{
		//length of Display line is not as wide as display
		lcd.print(DisplayLine.substring(DisplayPos, DisplayPos + DisplayDisplayLen));		// put up the Display
	}

	//set DisplayMode based on TemplateLine.  DisplayMode is used as a switch to direct processing the display line as Display, text, or other (time, date, alphanumeric)
	if (TemplateLine == "menu")
	{
		DisplayMode = 1;
		CursorBlink(false);		//turn off the cursor soft blink in case it is on
		//dprintln(F("TemplateLine==menu"));
	}
	else
		if (TemplateLine == "text")
		{
			DisplayMode = 3;
			CursorBlink(false);		//turn off the cursor soft blink
		}
		else
		{
			DisplayMode = 2;			// need to use integers because simplified C++ of .pde/.ino format doesn't support switch statements that take case of type string, typedef's or enum types.....at least as of 7/4/2015 I don't know how to do that :-)
			CursorBlink(true);		//turn on the soft interrupt to blink the character under DisplayPos.  Helps with data entry
									//we want the cursor to be on the first chr in template line that is to be processed, which is the first one that is not "-"
			while ((TemplateLine.substring(DisplayPos - DisplayStartPos, DisplayPos - DisplayStartPos + 1) == "-") && (DisplayPos < DisplayStartPos + DisplayDisplayLen - 1)) DisplayPos++;	// advance to non "-" chr		
			if (DisplayReadOnly)	BlinkCursor = "-"; else BlinkCursor = "*";	//set the chr used for cursor based on read only status as a hint for user
		}

}
//------------------------------------------
void DisplayClass::DisplayStartStop(boolean action)
{
	/*
	Used to indicate that the Display is in use and that keypresses should be processed by the Display routines
	*/

	if (action == true)
	{
		// Display processing beginning, set status variable
		DisplayInUse = true;
	}
	else
	{
		// DisplayProcessing is being stopped, set status variable
		DisplayInUse = false;
	}

}
//------------------------------------------
void DisplayClass::DisplaySetup(boolean isReadOnly, boolean readFromSD, String mnuName, int mnuLines, String *mnu)
{
	/*  Starts the Display array passed in by reference.
	Display is an array of DisplayLines. We pass in the name of the array and the number of displayLines (starting with 1).
	IsReadOnly determines if DisplayLines are read only or read/write....which applies only to data entry DisplayLines.
	If readFromSD is true, the routine will read the display string from a file of same name into the DisplayBuff. This does two things:
	1) is how we read back variables that were set and saved in display arrays in prior runs
	2) saves SRAM because DisplayBuf us reused for each display array

	This routine initializes some Display specific variables (pointer to array, name of display array, max number of lines, and index
	After that, it calls a routine to put the first DisplayLine on the physical display.  Subsequent processing occurs when user interacts with the keys (Rt, Lt, Up, Dn, and Select).
	The main loop of the program polls to see if a key is ready to be read via ReadKey().  If true, then the program must pass the key to the ProcessDisplay routine
	which will handle all interaction with the user.  The main loop needs to poll to see if the user has made a selection via checking if DisplayUserMadeSelection==true, and if so, reads
	the results from the Display variables....Display name, row name, and user selection.  These are then used to direct logic in the main loop of the program.
	*/
	if (readFromSD)
	{
		//read the mnuName into DisplayBuf from SD card.
		DisplayPntr = DisplayBuf;	// variable to hold a pointer to where we will load the string array that is the display
		if (ReadStringArraySD(mnuName, mnuLines) != mnuLines)	//read menuName from /Save folder of the SD card into DisplayBuf.  If successfull, then will read all the lines else error.
		{
			//if here then there was an error reading the file.
			ddcprintln(F("Error reading menu in DisplayClass::DisplaySetup"));	//replace with errorlog
		}
		//dprint(F("UsingDisplayPntr")); 
		//for (int z=0; z<mnuLines; z++) {dprintln(DisplayPntr[z]);	}	//debug

	}
	else
	{
		DisplayPntr = mnu;			// using in memory display array.  this variable to hold a pointer to the String array that is the display
	}

	DisplayName = mnuName;		// name of the Display array, returned when user makes a selection
	DisplayIndex = 0;			// initialize index to DisplayLine within String array that is the Display
	DisplayLineCnt = mnuLines;	// max number of DisplayLines in the Display, starting with 1
	DisplayReadOnly = isReadOnly;	// set to determine if this display array (deck) is read only or read/write.  Note applies only to display lines that take input.
	DisplayUserMadeSelection = false;	// flag indicating Display not ready to be read.  If true, user made selection
	DisplayLineSetup(*DisplayPntr);		 // extract and display DisplayLine[0]


}
//------------------------------------------
void DisplayClass::DisplayLineRefresh(String LineName)
{
	// Refreshes the display line if the current display line== LineName.  Used to update something that is changing. e.g. when testing temp, this is used to show the temp changine.
	//dprint("LineName="); dprintln(DisplayLineName);	//debug
	if (LineName == DisplayLineName)
	{
		//if here, then the line currently being displayed is the one we want to refresh, pointed to by DisplayIndex
		DisplayLineSetup(*(DisplayPntr + DisplayIndex)); // redisplay
	}
}
//------------------------------------------
void DisplayClass::ProcessDisplay(int KeyID)
{
	/*
	Main processing routine for Display.  This is called after user has pressed a key (up, dn, rt, lt, or Select) that has been debounced by the LS_Key routines.
	If the Display is in use (DisplayInUse==true), then the user key press is processed.  DisplaySetup was previously called and passed in the name of the Display, the
	array of DisplayLine strings, and the number of DisplayLines.  This routine responds to the keys pressed as follows:
	- Rt: slides the DisplayLine to the right, aligning the next Display option with the left side of the display
	- Lt: slides the DisplayLine to the left, aligning the previous Display option with the left side of the display
	- Down: displays the next DisplayLine (index+1) in the Display
	- Up: displays the previous DisplayLine (index-1) in the Display
	- Select: sets variables to pass out the DisplayName, DisplayLineName, and DisplaySelection.  Tells the code that the user made a selection by setting DisplayUserMadeSelection= true.
	*/
	if (DisplayInUse == true)
	{
		//if here, then Display processing should be occurring, so use the value of the key pressed and act accordingly.
		switch (DisplayMode)
		{
		case 1:	// display line is of type Display.
		{
			int oldPos = DisplayPos;		// holds prior position of DisplayPos
			switch (LS_curKey)
			{
			case (RIGHT_KEY) :
			{
				DisplayPos = DisplayAdvToSpace(DisplayLine, DisplayPos);	//skip past Display option to space
				if (DisplayPos >= DisplayEndPos)
				{
					// if here then the we just went past the last option on the DisplayLine, so restore prior DisplayPos and do nothing
					DisplayPos = oldPos;
				}
				else
				{
					// if here then we are not at the end of the DisplayLine, so advance to next Display option
					DisplayPos = DisplayAdvPastSpace(DisplayLine, DisplayPos);
					lcd.begin(16, 2);
					lcd.clear();
					lcd.setCursor(0, 0);
					lcd.print(DisplayLineTitle);
					lcd.setCursor(0, 1);
					if (DisplayPos + DisplayDisplayLen < DisplayEndPos)
					{
						if (DisplayPos>DisplayStartPos)
						{
							//if here, then there is more Displayline to left and right of display
							lcd.print('<' + DisplayLine.substring(DisplayPos, DisplayPos + DisplayDisplayLen - 2) + '>');
							//debugPrint("Right, more on rt and lt ");
						}
						else
						{
							//if here then the Displayline starts on left of display and extends past display
							lcd.print(DisplayLine.substring(DisplayPos, DisplayPos + DisplayDisplayLen - 1) + '>');
							//debugPrint("Right, fits on lt, more on rt ");
						}

					}
					else
					{
						if (DisplayPos>DisplayStartPos) //just switched
						{
							//if here, Displayline extends past left of display and fits on right side of display
							lcd.print('<' + DisplayLine.substring(DisplayPos, DisplayPos + DisplayDisplayLen - 1));
							//debugPrint("Right, more on lt, fits on rt");
						}
						else
						{
							//if here, then Displayline fits in display
							lcd.print(DisplayLine.substring(DisplayPos, DisplayPos + DisplayDisplayLen));
							//debugPrint("Right, fits in display ");

						}
					}

				}
				break;
			}// done processing RIGHT_KEY

			case (LEFT_KEY) :
			{

				lcd.begin(16, 2);
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print(DisplayLineTitle);	// put up the title
				lcd.setCursor(0, 1);

				if (DisplayPos <= DisplayStartPos)
				{
					// if here then the we just are at the beginning of the Display, so don't back up
					lcd.print(DisplayLine.substring(DisplayPos, DisplayPos + DisplayDisplayLen - 1) + '>');	// put up as much as will fit for the Display, with indicator that there is more
																											//debugPrint("Left, at beginning, no check of len ");
				}
				else
				{
					// if here then we are not at the beginning of the DisplayLine, so backup to previous Display option
					DisplayPos = DisplayBkupPastSpace(DisplayLine, DisplayPos - 1);	//backs DisplayPos to the end of the previous Display option
					DisplayPos = DisplayBkupToSpace(DisplayLine, DisplayPos) + 1;	//DisplayPos now at the first char of the previous Display option
					if (DisplayPos + DisplayDisplayLen < DisplayEndPos)
					{
						if (DisplayPos>DisplayStartPos)
						{
							//if here, then there is more DisplayLine before and after the display window
							lcd.print('<' + DisplayLine.substring(DisplayPos, DisplayPos + DisplayDisplayLen - 2) + '>');
							//debugPrint("Left, more on lt and rt ");
						}
						else
						{
							//if here, then at the start of the Displayline with more Display after the display window
							lcd.print(DisplayLine.substring(DisplayPos, DisplayPos + DisplayDisplayLen - 1) + '>');
							//debugPrint("Left, fit on lt, more on Rt ");
						}

					}
					else
					{
						if (DisplayPos>DisplayStartPos)
						{
							// if here then some Displayline to the left of display window but the remaining Display options fit on the display
							lcd.print('<' + DisplayLine.substring(DisplayPos, DisplayPos + DisplayDisplayLen - 1));
							//debugPrint("Left, more on lt, fit on rt");
						}
						else
						{
							// if here then some Displayline fits in the the display
							lcd.print(DisplayLine.substring(DisplayPos, DisplayPos + DisplayDisplayLen));
							//debugPrint("Left, fits display ");
						}
					}
				}
				break;

			}// done processing LEFT_KEY

			case (SELECT_KEY) :
			{
				// get the option that starts at DisplayPoss
				DisplayOptStart = DisplayPos;
				ddcprintln("displayLine=|" + DisplayLine + "|");
				DisplayOptEnd = (DisplayAdvToSpace(DisplayLine, DisplayPos));
				ddcprint("DisplayOptEnd="); ddcprintln(DisplayOptEnd);
				ddcprint("DisplaySelection=");ddcprint(DisplayLine.substring(DisplayOptStart,DisplayOptEnd));ddcprint("  DisplayStartPos=");ddcprint(DisplayStartPos);ddcprint("  DisplayOptStart=");ddcprint(DisplayOptStart);ddcprint("   DisplayOptEnd=");ddcprint(DisplayOptEnd);ddcprint("   selection length=");ddcprintln(DisplaySelection.length());
				DisplaySelection = DisplayLine.substring(DisplayOptStart, DisplayOptEnd);	// get the string of the option
				DisplayUserMadeSelection = true;	//polling routine will see this and fetch results found in DisplayName, DisplayLineName, DisplaySelection
				break;		// end processing SELECT_KEY
			}	// done with SELECT_KEY

			case (UP_KEY) :
			{
				// user wants to display DisplayLine 'above' the current DisplayLine where [0] is the highest
				DisplayIndex = (((DisplayIndex - 1) + DisplayLineCnt) % DisplayLineCnt);	// needed for correct wrap around at 0
				DisplayLineSetup(*(DisplayPntr + DisplayIndex)); // extract and display DisplayLine[DisplayLineCnt]
				ddcprintln(F("KeyUp")); ddcprintln(DisplayBuf[DisplayIndex]);	//debug
				break;
			}	// done with UP_KEY

			case (DOWN_KEY) :
			{
				// user wants to display DisplayLine 'below' the current DisplayLine where [MaxLines] is the Lowest
				DisplayIndex = (DisplayIndex+1) % DisplayLineCnt;
				DisplayLineSetup(*(DisplayPntr + DisplayIndex)); // extract and display DisplayLine[DisplayLineCnt]
				ddcprintln(F("KeyDown")); ddcprintln(DisplayBuf[DisplayIndex]);	//debug
				break;
			}	// done with DOWN_KEY
			}	// end switch Ls_Key
			break; // done processing DisplayOption
		}	// end case DisplayOption = 1

		case 2:	// display line is of type 'other', which is for input e.g. date, time, numeric, alphanumeric
				/*
				Here we are processing the DisplayLine according to the characters in the TemplateLine.  The index of the TemplateLine and the
				DisplayLine match.  That is, if the TemplateLine indicates that the month in mm/dd/yyyy starts at DisplayPos 5, the value of the month starts at DisplayPos
				in the DisplayLine.

				All the processing of type 'other' uses a blinking cursor, which is handled by the method CursorBlinkTimeInt, which is called by a timer object.
				The display is re-written by that method, so there is no need to re-write the display in this section (case 2, processing type 'other')
				*/
		{
			boolean UpdateDisplayArray = false;	// used to persist changes the user made to the displayline back to the string array (display array).
			int	tmpKey = LS_curKey;
			//String TemplateChar;	// used to drive the switch statement that processes the template
			char TempChar;			//test

			if (LS_curKey == RIGHT_KEY)
			{
				//user pressed right key, advance in template to chr to process (not = '-')
				int tmpInt;
				for (int x = 0; x < TemplateLine.length(); x++)	// look for next non "-" chr but limit in case user made error in formatting display line
				{
					tmpInt = ((DisplayPos - DisplayStartPos) + 1) % TemplateLine.length();	// tmpInt is the next chr to test, modulo to wrap around
					ddcprint(F("next position in TemplateLine to test=")); ddcprintln(tmpInt);	//debug
					DisplayPos = tmpInt + DisplayStartPos;
					if (TemplateLine[DisplayPos - DisplayStartPos] != '-') break;	// exit loop when we find the next non "-" chr
				}

				break;	//done processing LS_curKey
			}

			if (LS_curKey == LEFT_KEY)
			{
				//user pressed left key, back up in template to chr to process (not = '-')
				/*
				if (DisplayPos > DisplayStartPos) DisplayPos--; //move Lt one chr
				while ((TemplateLine[DisplayPos - DisplayStartPos] == '-') && (DisplayPos > DisplayStartPos))
				{
					DisplayPos--;// continue to back up to non "-" chr and process below
								 //dprint(TemplateLine.substring(DisplayPos-DisplayStartPos,DisplayPos-DisplayStartPos+1));	//debug						
				}
				*/
				int tmpInt;
				for (int x = 0; x < TemplateLine.length(); x++)	// look for next non "-" chr but limit in case user made error in formatting display line
				{
					tmpInt = ((DisplayPos - DisplayStartPos) -1) % TemplateLine.length();	// tmpInt is the next chr to test, modulo to wrap around
					ddcprint(F("next position in TemplateLine to test=")); ddcprintln(tmpInt);	//debug
					DisplayPos = tmpInt + DisplayStartPos;
					if (TemplateLine[DisplayPos - DisplayStartPos] != '-') break;	// exit loop when we find the next non "-" chr
				}
				break;	//done processing LS_curKey
			}

			TempChar = TemplateLine[DisplayPos - DisplayStartPos];		//get the character under the cursor					
			ddcprint(F("TempChar=")); ddcprint(TempChar);ddcprint(F("  DisplayPos=")); ddcprintln(DisplayPos);	//debug

			//process the character under the DisplayPos cursor
			if (TempChar == 'm' && !DisplayReadOnly)	// pointing to first digit of month in date, only process if display is read/write
			{
				//dprintln("processing TemplateChar=m");	//debug
				switch (tmpKey)
				{
				case (UP_KEY) :
				{
					//dprintln("month, upkey");							
					MonthEntry(true);
					break;
				}	// done with UP_KEY

				case (DOWN_KEY) :
				{
					MonthEntry(false);
					//dprintln("month, downkey");								
					break;
				}	// done with DOWN_KEY	
				}
				UpdateDisplayArray = true;	// flag to update the DisplayArray to save the changes.
				goto UpdateCheck;			//done processing the LS_CurKey, jump to bottom and update display array if user made changes to the displayLine

			} //if (TempChar=='m')

			if (TempChar == 'd'&& !DisplayReadOnly)	// pointing to first digit of day in date
			{
				//dprintln("processing TemplateChar=d");	//debug
				switch (tmpKey)
				{
				case (UP_KEY) :
				{
					DayEntry(true);
					break;
				}	// done with UP_KEY

				case (DOWN_KEY) :
				{
					DayEntry(false);
					break;
				}	// done with DOWN_KEY
				}
				UpdateDisplayArray = true;	// flag to update the DisplayArray to save the changes.					
				goto UpdateCheck;			//done processing the LS_CurKey, jump to bottom and update display array if user made changes to the displayLine				
			}

			if (TempChar == 'y' && !DisplayReadOnly)	// pointing to 3rd digit of yyyy in year
			{
				//dprintln("processing TemplateChar=y");	//debug					
				switch (tmpKey)
				{
				case (UP_KEY) :
				{
					YearEntry(true);
					break;
				}	// done with UP_KEY

				case (DOWN_KEY) :
				{
					YearEntry(false);
					break;
				}	// done with DOWN_KEY
				}
				UpdateDisplayArray = true;	// flag to update the DisplayArray to save the changes.					
				goto UpdateCheck;			//done processing the LS_CurKey, jump to bottom and update display array if user made changes to the displayLine
			}

			if (TempChar == 'a' && !DisplayReadOnly)	// pointing to 1st chr in day of week
			{
				//dprintln("processing TemplateChar=a");	//debug
				switch (tmpKey)
				{
				case (UP_KEY) :
				{
					DowEntry(true);
					break;
				}	// done with UP_KEY

				case (DOWN_KEY) :
				{
					DowEntry(false);
					break;
				}	// done with DOWN_KEY
				}
				UpdateDisplayArray = true;	// flag to update the DisplayArray to save the changes.					
				goto UpdateCheck;			//done processing the LS_CurKey, jump to bottom and update display array if user made changes to the displayLine
			}

			if (TempChar == 'H'&& !DisplayReadOnly)	// pointing to first digit of hour in HH:MM
			{
				//dprintln("processing TemplateChar=H");	//debug
				switch (tmpKey)
				{
				case (UP_KEY) :
				{
					HourEntry(true);
					break;
				}	// done with UP_KEY

				case (DOWN_KEY) :
				{
					HourEntry(false);
					break;
				}	// done with DOWN_KEY
				}
				UpdateDisplayArray = true;	// flag to update the DisplayArray to save the changes.					
				goto UpdateCheck;			//done processing the LS_CurKey, jump to bottom and update display array if user made changes to the displayLine		
			}

			if (TempChar == 'M' && !DisplayReadOnly)	// pointing to first digit of min in HH:MM
			{
				switch (tmpKey)
				{
				case (UP_KEY) :
				{
					MinuteEntry(true);
					break;
				}	// done with UP_KEY

				case (DOWN_KEY) :
				{
					MinuteEntry(false);
					break;
				}	// done with DOWN_KEY
				}
				UpdateDisplayArray = true;	// flag to update the DisplayArray to save the changes.					
				goto UpdateCheck;			//done processing the LS_CurKey, jump to bottom and update display array if user made changes to the displayLine			
			}

			if (TempChar == '#'&& !DisplayReadOnly)	// pointing to digit in base 10 number
			{
				switch (tmpKey)
				{
				case (UP_KEY) :
				{
					NumEntry(true);
					break;
				}	// done with UP_KEY

				case (DOWN_KEY) :
				{
					NumEntry(false);
					break;
				}	// done with DOWN_KEY
				}
				UpdateDisplayArray = true;	// flag to update the DisplayArray to save the changes.					
				goto UpdateCheck;			//done processing the LS_CurKey, jump to bottom and update display array if user made changes to the displayLine			
			}

			if (TempChar == 'C'&& !DisplayReadOnly)	// pointing to character in string
			{
				switch (tmpKey)
				{
				case (UP_KEY) :
				{
					ChrEntry(true);
					break;
				}	// done with UP_KEY

				case (DOWN_KEY) :
				{
					ChrEntry(false);
					break;
				}	// done with DOWN_KEY
				}
				UpdateDisplayArray = true;	// flag to update the DisplayArray to save the changes.					
				goto UpdateCheck;			//done processing the LS_CurKey, jump to bottom and update display array if user made changes to the displayLine				
			}

			if (TempChar == 'U')	// pointing to the Display Up selection
			{
				ddcprintln(F("processing TemplateChar=U"));	//debug
				switch (tmpKey)
				{
				case (UP_KEY) :
				{
					// user wants to display DisplayLine 'above' the current DisplayLine where [0] is the highest
					ddcprint(F("DisplayLineCnt=")); ddcprint(DisplayLineCnt); ddcprint(F(" DisplayIndex pre=")); ddcprint(DisplayIndex);
					DisplayIndex = (((DisplayIndex - 1) + DisplayLineCnt) % DisplayLineCnt);	// needed for correct wrap around at 0
					ddcprint(F("DisplayIndex post=")); ddcprintln(DisplayIndex);
					DisplayLineSetup(*(DisplayPntr + DisplayIndex)); // extract and display DisplayLine[DisplayLineCnt]
					break;		// done processing 	switch (tmpKey)
				}	// done with UP_KEY

				case (DOWN_KEY) :
				{
					// user wants to display DisplayLine 'below' the current DisplayLine where [MaxLines] is the Lowest
					ddcprint(F("DisplayLineCnt=")); ddcprint(DisplayLineCnt); ddcprint(F(" DisplayIndex pre=")); ddcprint(DisplayIndex);
					DisplayIndex = ((DisplayIndex+1) % DisplayLineCnt);
					ddcprint(F("DisplayIndex post=")); ddcprintln(DisplayIndex);
					DisplayLineSetup(*(DisplayPntr + DisplayIndex)); // extract and display DisplayLine[DisplayLineCnt]
					break;			// done processing 	switch (tmpKey)
				}	// done with DOWN_KEY
				}
				goto UpdateCheck;	//done processing the LS_CurKey, jump to bottom and update display array if user made changes to the displayLine
			}

			if (TempChar == 'D')	// pointing to the Display Down selection
			{
				//dprintln("processing TemplateChar=D");	//debug					
				switch (tmpKey)
				{
				case (UP_KEY) :
				{
					// user wants to display DisplayLine 'above' the current DisplayLine where [0] is the highest
					ddcprint(F("DisplayLineCnt=")); ddcprint(DisplayLineCnt); ddcprint(F(" DisplayIndex pre=")); ddcprint(DisplayIndex);
					DisplayIndex = (((DisplayIndex - 1) + DisplayLineCnt) % DisplayLineCnt);	// needed for correct wrap around at 0;
					ddcprint(F("DisplayIndex post=")); ddcprintln(DisplayIndex);
					DisplayLineSetup(*(DisplayPntr + DisplayIndex)); // extract and display DisplayLine[DisplayLineCnt]
					break;		// done processing 	switch (tmpKey)
				}	// done with UP_KEY

				case (DOWN_KEY) :
				{
					// user wants to display DisplayLine 'below' the current DisplayLine where [MaxLines] is the Lowest
					ddcprint(F("DisplayLineCnt=")); ddcprint(DisplayLineCnt); ddcprint(F(" DisplayIndex pre=")); ddcprint(DisplayIndex);
					DisplayIndex = ((DisplayIndex+1) % DisplayLineCnt);
					ddcprint(F("DisplayIndex post=")); ddcprintln(DisplayIndex);
					DisplayLineSetup(*(DisplayPntr + DisplayIndex)); // extract and display DisplayLine[DisplayLineCnt]					
					break;			// done processing 	switch (tmpKey)
				}	// done with DOWN_KE			

				}
				goto UpdateCheck;	//done processing the LS_CurKey, jump to bottom and update display array if user made changes to the displayLine					
			}

			// update display array if user made changes to the displayLine.
		UpdateCheck:		//target of goto from processing date, time, numeric, and alphanumeric
							//dprintln("testing UpdateDisplayArray");
			if (UpdateDisplayArray)
			{
				String tmpStr = DisplayLine;
				tmpStr.trim();	//trim the whitespace added as padding prior to updating the display array
								//change the entry			
								//dprint("Before= |");dprint(DisplayPntr[DisplayIndex]);dprintln("|");	//debug					
								//DisplayLine.trim();	//trim leading and trailing whitespace					
				DisplayPntr[DisplayIndex] = DisplayLineName + ',' + TemplateLine + ',' + DisplayLineTitle + ',' + tmpStr;	// change the entry in the display array
																															//dprint("After=  |");dprint(DisplayPntr[DisplayIndex]);dprintln("|");	//debug
			}
			break;
		} // end case DisplayOption = 2 

		case 3:	//display line is of type text.  this supports showing long strings of text that can be rapidly scrolled through.
		{
			//int oldPos=DisplayPos;		// holds prior position of DisplayPos *Unused*
			String newline;			// string to display
			int tmpInt;

			switch (LS_curKey)
			{
			case (RIGHT_KEY) :
			{
				//dprintln(DisplayPos);	//debug

				//find the new starting position.  Advance the width of the display and back up to nearest whold word
				DisplayPos += DisplayDisplayLen;
				if (DisplayPos > DisplayEndPos) DisplayPos = DisplayEndPos;

				//if not on a space, back up to one. 
				if (DisplayLine[DisplayPos] != ' ')
				{
					//there is one or more spaces at the right side of the display
					//dprint("Displaypos!=' ' ");
					while (DisplayLine[DisplayPos] != ' ' && DisplayPos>DisplayStartPos) { DisplayPos--; }	//back up to start of nearest whole word
																											//dprintln(DisplayPos);	//debug
				}
				else
				{
					//we're on a space, so back up to the next space
					//dprint("Displaypos==' ' ");
					while (DisplayLine[DisplayPos] == ' ' && DisplayPos>DisplayStartPos) { DisplayPos--; }	//back up to start of nearest whole word
					while (DisplayLine[DisplayPos] != ' ' && DisplayPos>DisplayStartPos) { DisplayPos--; }	//back up to start of nearest whole word
																											//dprintln(DisplayPos);	//debug							
				}


				//form the new display line
				if (DisplayPos == DisplayStartPos)
				{
					if (DisplayPos + DisplayDisplayLen < DisplayEndPos)
					{
						// if here then left most chrs of string are in the display and more chrs are off the display on the right (..string..>)
						//if here then the display line starts on left of display and extends past display
						newline = (DisplayLine.substring(DisplayPos, DisplayPos + DisplayDisplayLen - 1) + '>');
						//debugPrint("Right, fits on lt, more on rt ");									
					}
					else
					{
						// if here, then string fits within the display (..string..)
						//if here, then Displayline fits in display
						newline = (DisplayLine.substring(DisplayPos, DisplayPos + DisplayDisplayLen));
						//debugPrint("Right, fits in display ");
					}

				}
				else
				{
					DisplayPos++;
					if (DisplayPos + DisplayDisplayLen < DisplayEndPos)
					{
						// if here then left most chrs of string off the display and more chrs are off the display on the right (<..string..>)
						newline = ('<' + DisplayLine.substring(DisplayPos, DisplayPos + DisplayDisplayLen - 2) + '>');
						//debugPrint("Right, more on rt and lt ");
					}
					else
					{
						// if here, then left most chrs of string off the display and right most chrs are within the display ( <..string)
						//if here, Displayline extends past left of display and fits on right side of display
						newline = ('<' + DisplayLine.substring(DisplayPos, DisplayPos + DisplayDisplayLen - 1));
						//debugPrint("Right, more on lt, fits on rt");
					}
				}

				// put up the new line							
				lcd.begin(16, 2);
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print(DisplayLineTitle);
				lcd.setCursor(0, 1);
				lcd.print(newline);
				break;
			}// done processing RIGHT_KEY

			case (LEFT_KEY) :
			{
				// user hit left key, go back to start of string
				DisplayPos = DisplayStartPos;
				if (DisplayStartPos + DisplayDisplayLen < DisplayEndPos)
				{
					newline = (DisplayLine.substring(DisplayPos, DisplayPos + DisplayDisplayLen - 1) + '>');
				}
				else
				{
					newline = (DisplayLine.substring(DisplayPos, DisplayPos + DisplayDisplayLen));
				}

				// put up the new line
				lcd.begin(16, 2);
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print(DisplayLineTitle);
				lcd.setCursor(0, 1);
				lcd.print(newline);
				break;
				break;
			}// done processing LEFT_KEY


			case (UP_KEY) :
			{
				ddcprint(F("displayLine= text, key is up, DisplayIndex before decrement=")); ddcprint(DisplayIndex);
				DisplayIndex = (((DisplayIndex - 1)+ DisplayLineCnt)% DisplayLineCnt);	// needed for correct wrap around at 0
				DisplayLineSetup(*(DisplayPntr + DisplayIndex)); // extract and display DisplayLine[DisplayLineCnt]
				ddcprint(F(" DisplayIndex after decrement=")); ddcprint(DisplayIndex); ddcprint(F(" DisplayLineCnt=")); ddcprintln(DisplayLineCnt);
				break;
			}	// done with UP_KEY

			case (DOWN_KEY) :
			{
				// user wants to display DisplayLine 'below' the current DisplayLine where [MaxLines] is the Lowest
				ddcprint(F("displayLine= text, key is down, DisplayIndex before increment=")); ddcprint(DisplayIndex);
				DisplayIndex = ((DisplayIndex+1) % DisplayLineCnt);
				DisplayLineSetup(*(DisplayPntr + DisplayIndex)); // extract and display DisplayLine[DisplayLineCnt]
				ddcprint(F(" DisplayIndex after increment=")); ddcprint(DisplayIndex); ddcprint(F(" DisplayLineCnt=")); ddcprintln(DisplayLineCnt);
				break;
			}	// done with DOWN_KEY
			}	// end switch Ls_Key

			break;
		}	// end case DisplayOption = 3

		}	// end switch DisplayMode
	} // end if DisplayInUse==true

}
//------------------------------------------
boolean DisplayClass::FindAndParseDisplayLine(String MnuLineName, int *Idx, String *DisplayTitle, String *TemplateLn, String *DisplayLn)
{
	// Used by methods to get/set variables within display (Display) arrays. Find and parse the display line named MnuLineName in the current display array. Parse out the index to the display line, the template line, and the display line.


	/*debug
	dprintln("MnuLineName=" + MnuLineName);
	dprint(" Inputs: index=" );dprintln(*Idx);
	dprint(" DisplayTitle="); dprint(*DisplayTitle); dprint(" templateln="); dprint(*TemplateLn); dprint(" displayLn="); dprintln(*DisplayLn);
	*/

	int tmp1, tmp2, tmp3;
	boolean tmpBool = false;
	*Idx = 0;	//set index to 0 as calling routine doesn't need to
				//find the string in the display array that has the name MnuLineName
	for (int x = 1; x<DisplayLineCnt; x++)	//DisplayLineCnt is protected within Display object and is the number of lines in the dispaly array
	{
		//dprintln(x);
		if (DisplayPntr[*Idx].startsWith(MnuLineName))
		{
			tmpBool = true;	// found string
			break;
		}
		else
		{
			(*Idx)++;	//increment the index
		}
	}
	if (!tmpBool) return false;	//// error, didn't find the MnuLineName.  Probably a typo in coding

								//dprint(" Inputs: index=" );dprintln(*Idx);
								//found the entry in the display array, so parse it
	tmp1 = DisplayPntr[*Idx].indexOf(',');						// get position of comma, used to parse
	tmp2 = DisplayPntr[*Idx].indexOf(',', tmp1 + 1);				// position of 2nd comma
	tmp3 = DisplayPntr[*Idx].indexOf(',', tmp2 + 1);				// position of 3rd comma
	*TemplateLn = DisplayPntr[*Idx].substring(tmp1 + 1, tmp2);		// get TemplateLine
	*DisplayTitle = DisplayPntr[*Idx].substring(tmp2 + 1, tmp3);	// get DisplayLine title	
	*DisplayLn = DisplayPntr[*Idx].substring(tmp3 + 1, DisplayPntr[*Idx].length());	//snip out the portion of the display line that is shown on the display and contains the variable that is to be set/returned

	return true;
}
//------------------------------------------
boolean DisplayClass::DisplayGetSetDate(String *DateStr, String MnuLineName, boolean set)
{
	int Index = 0;
	//int tmp1,tmp2,tmp3,tmp4;
	int	tmp1;
	String DisplayTitle, TemplateLine, DisplayLine;							// used to process the TemplateLine
#define LenOfDate 10	// length of the mm/dd/yyyy

																			// find and parse the display line
	if (!FindAndParseDisplayLine(MnuLineName, &Index, &DisplayTitle, &TemplateLine, &DisplayLine))return false;	// problem parsing display line, likely a typo in call, return error	

																												//dprintln ("old entry=" + DisplayPntr[Index]);	//debug
	tmp1 = TemplateLine.indexOf('m');	//find where in the template the first chr of month is.
	if (tmp1 == -1) return false;		//-1 means didn't find chr which is an unexpected error, probably a typo in template or referencing incorrect line in display array	
										//dprint ("index="); dprintln(tmp1);	//debug
	if (set)
	{
		//user wants to set the date
		//dprint("old displayline="); dprintln(DisplayLine);	//debug
		//dprintln(Index);	//debug
		DisplayLine = DisplayLine.substring(0, tmp1) + *DateStr + DisplayLine.substring(tmp1 + LenOfDate, DisplayLine.length());	//splice new date into display line.  works because the chr position in the template matches the those in the display line
																																	//dprint("new displayline="); dprintln(DisplayLine);	//debug	

																																	//change the entry	
		DisplayPntr[Index] = MnuLineName + ',' + TemplateLine + ',' + DisplayTitle + ',' + DisplayLine;	// change the entry in the display array
																										//dprintln ("new entry=" + DisplayPntr[Index]);	//debug
	}
	else
	{
		//user wants to read the date
		*DateStr = DisplayLine.substring(tmp1, tmp1 + LenOfDate);
		//dprintln("date string='" + *DateStr +"'");	//debug

	}

	return true;	//return, successful 
}
//------------------------------------------
boolean DisplayClass::DisplayGetSetTime(String *TimeStr, String MnuLineName, boolean set)
{
	// gets or sets time in the display line named MnuLineName in the current display array.  time format is HH:MM.  if Set is true then sets value of DateStr else gets value
	int Index = 0;
	int	tmp1;
	String DisplayTitle, TemplateLine, DisplayLine, tmpStr;							// used to process the TemplateLine
#define LenOfTime 5	// length of the HH:MM

																					// find and parse the display line
	if (!FindAndParseDisplayLine(MnuLineName, &Index, &DisplayTitle, &TemplateLine, &DisplayLine))return false;	// problem parsing display line, likely a typo in call, return error	

	tmp1 = TemplateLine.indexOf('H');	//find where in the template the first chr of time is.
	if (tmp1 == -1) return false;		//-1 means didn't find chr which is an unexpected error, probably a typo in template or referencing incorrect line in display array
	if (set)
	{
		//user wants to set the time
		//dprint("old displayline="); dprintln(DisplayLine);	//debug
		//dprintln(Index);	//debug
		tmpStr = TimeStr->substring(0, LenOfTime);	// clip time to 5 chrs
		DisplayLine = DisplayLine.substring(0, tmp1) + tmpStr + DisplayLine.substring(tmp1 + LenOfTime, DisplayLine.length());	//splice new time into display line.  works because the chr position in the template matches the those in the display line
																																//change the entry	
		DisplayPntr[Index] = MnuLineName + ',' + TemplateLine + ',' + DisplayTitle + ',' + DisplayLine;	// change the entry in the display array
																										//dprintln ("new entry=" + DisplayPntr[Index]);	//debug
	}
	else
	{
		*TimeStr = DisplayLine.substring(tmp1, tmp1 + LenOfTime);
		//dprintln("time string='" + *TimeStr +"'");	//debug

	}

	return true;	//return, successful 
}
//------------------------------------------
boolean DisplayClass::DisplayGetSetDOW(String *DayStr, String MnuLineName, boolean set)
{
	// gets or sets day of the week (DOW) in the display line named MnuLineName in the current display array.  Day format is 3 chr, see DisplayDOW[x] string.  if Set is true then sets value of DayStr else gets value
	int Index = 0;
	int	tmp1;
	String DisplayTitle, TemplateLine, DisplayLine;							// used to process the TemplateLine
#define LenOfDOW 3	// length of the day abbreviation

																			// find and parse the display line
	if (!FindAndParseDisplayLine(MnuLineName, &Index, &DisplayTitle, &TemplateLine, &DisplayLine))return false;	// problem parsing display line, likely a typo in call, return error

	tmp1 = TemplateLine.indexOf('a');	//find where in the template the first chr of day of week is.
	if (tmp1 == -1) return false;		//-1 means didn't find chr which is an unexpected error, probably a typo in template or referencing incorrect line in display array
	if (set)
	{
		//user wants to set the day
		//dprint("old displayline="); dprintln(DisplayLine);	//debug
		//dprintln(Index);	//debug
		DisplayLine = DisplayLine.substring(0, tmp1) + *DayStr + DisplayLine.substring(tmp1 + LenOfDOW, DisplayLine.length());	//splice new DOW into display line.  works because the chr position in the template matches the those in the display line
																																//change the entry
		DisplayPntr[Index] = MnuLineName + ',' + TemplateLine + ',' + DisplayTitle + ',' + DisplayLine;	// change the entry in the display array
																										//dprintln ("new entry=" + DisplayPntr[Index]);	//debug
	}
	else
	{
		*DayStr = DisplayLine.substring(tmp1, tmp1 + LenOfDOW);
		//dprintln("DOW string='" + *DayhStr +"'");	//debug

	}

	return true;	//return, successful
}
//------------------------------------------
boolean DisplayClass::DisplayGetSetNum(String *NumStr, String MnuLineName, boolean set)
{
	// gets or sets a numeric value in the display line named MnuLineName in the current display array. If Set is true then sets value of DayStr else gets value
	int Index = 0;
	int	tmp1, tmp2;
	String DisplayTitle, TemplateLine, DisplayLine,NewNumStr;							// used to process the TemplateLine

																			// find and parse the display line
	if (!FindAndParseDisplayLine(MnuLineName, &Index, &DisplayTitle, &TemplateLine, &DisplayLine))return false;	// problem parsing display line, likely a typo in call, return error
	tmp1 = TemplateLine.indexOf('#');	//find where in the template the first chr of the number
	tmp2 = TemplateLine.lastIndexOf('#');	//find in the template the last chr of the number

	if (tmp1 == -1) return false;		//-1 means didn't find chr which is an unexpected error, probably a typo in template or referencing incorrect line in display array
	if (set)
	{
		//user wants to set 
		ddcprint("numStr="); ddcprintln(*NumStr);	//debug
		ddcprint("old displayline="); ddcprintln(DisplayLine);	//debug
		ddcprintln(Index);	//debug
		NewNumStr = *NumStr;
		DisplayLine = DisplayLine.substring(0, tmp1) + NewNumStr.substring(0,tmp2-tmp1+1) + DisplayLine.substring(tmp2+1, DisplayLine.length());	//splice new numeric value into display line.  works because the chr position in the template matches the those in the display line
																													//change the entry
		DisplayPntr[Index] = MnuLineName + ',' + TemplateLine + ',' + DisplayTitle + ',' + DisplayLine;	// change the entry in the display array
																										//dprintln ("new entry=" + DisplayPntr[Index]);	//debug
	}
	else
	{
		*NumStr = DisplayLine.substring(tmp1, tmp2 + 1);
		ddcprintln("DOW string='" + *NumStr +"'");	//debug

	}

	return true;	//return, successful
}
//------------------------------------------
boolean DisplayClass::DisplayGetSetChrs(String *ChrStr, String MnuLineName, boolean set)
{
	// gets or sets a character string in the display line named MnuLineName in the current display array. If Set is true then sets value of ChrStr else gets value
	int Index = 0;
	int	tmp1, tmp2;
	String DisplayTitle, TemplateLine, DisplayLine, NewChrStr;							// used to process the TemplateLine

																						// find and parse the display line
	if (!FindAndParseDisplayLine(MnuLineName, &Index, &DisplayTitle, &TemplateLine, &DisplayLine))return false;	// problem parsing display line, likely a typo in call, return error
	
	ddcprint("len of TemplateLine="); ddcprintln(TemplateLine.length());
	tmp1 = TemplateLine.indexOf('C');	//find where in the template the first chr of the string
	tmp2 = TemplateLine.lastIndexOf('C');	//find in the template the last chr of the string

	if (tmp1 == -1) return false;		//-1 means didn't find chr which is an unexpected error, probably a typo in template or referencing incorrect line in display array
	if (set)
	{
		//user wants to set
		ddcprint(F("old displayline=|")); ddcprint(DisplayLine); ddcprintln(F("|"));	//debug
		ddcprintln(Index);	//debug
		NewChrStr = *ChrStr;
		DisplayLine = DisplayLine.substring(0, tmp1) + NewChrStr.substring(0, tmp2 - tmp1 + 1) + DisplayLine.substring(tmp2+1, DisplayLine.length());	//splice new String value into display line.  works because the chr position in the template matches the those in the display line
																																					//change the entry
		DisplayPntr[Index] = MnuLineName + ',' + TemplateLine + ',' + DisplayTitle + ',' + DisplayLine;	// change the entry in the display array
		ddcprint(F("new displayline=|")); ddcprint(DisplayLine); ddcprintln(F("|"));	//debug
		ddcprintln ("new entry=" + DisplayPntr[Index]);	//debug
	}
	else
	{
		*ChrStr = DisplayLine.substring(tmp1, tmp2 + 1);
		ddcprint("string="); ddcprintln(*ChrStr);	//debug

	}

	return true;	//return, successful
}
//------------------------------------------
boolean DisplayClass::DisplaySetTxt(String *TxtStr, String MnuLineName)
{
	// sets a text message in the display line named MnuLineName in the current display array. Get not needed as this is only for outputting messages
	int Index = 0;
	//int	tmp1,tmp2;
	String DisplayTitle, TemplateLine, DisplayLine;							// used to process the TemplateLine

																			// find and parse the display line
	if (!FindAndParseDisplayLine(MnuLineName, &Index, &DisplayTitle, &TemplateLine, &DisplayLine))return false;	// problem parsing display line, likely a typo in call, return error

	dprintln(TemplateLine);	//debug
	if (TemplateLine != "text") return false;		//the template line = text for display lines containing display text, so return error if this is not found

	DisplayPntr[Index] = MnuLineName + ',' + TemplateLine + ',' + DisplayTitle + ',' + *TxtStr;	// change the entry in the display array
	return true;	//return, successful
}
//------------------------------------------
void DisplayClass::CursorBlink(boolean action)
{
	//Routine to start/stop polling for keys being pressed.  Used to turn on/off the keys. would have been in the Display class except for problems getting timer.every to point to a method within a class
	static int	BlinkTimerContext;
	static boolean  BlinkingCursorOn = false;
	if (action)
	{
		if (BlinkingCursorOn == false)	//only turn it on if not currently on
		{
			//start the timer polling at intervals to blink cursor
			BlinkTimerContext = Tmr.every(500, CursorBlinkIntRedirect, (void*)2);	// begin polling keypad, call KeyCheck at intervals of KeyPollRate. timer index = LS_PollContext
			BlinkingCursorOn = true;	// set flag so we know we are using the soft interrupt
										//dprintln("timer for cursor blink started ");	//delete in future
		}
	}
	else
	{
		if (BlinkingCursorOn == true)	// only turn it off if it is currently on
		{
			Tmr.stop(BlinkTimerContext);		// stop polling.  Index previously saved with call above
			BlinkingCursorOn = false;
		}
	}
}
//------------------------------------------
void DisplayClass::CursorBlinkTimeInt(void)
{
	static boolean Blank = true;

	// uses a soft interrupt to blink a character, alternately writing the character and a space.  chr to use
	// is in BlinkCursor, set based on read only status of DisplayLine

	//display the Display title and Display
	lcd.begin(16, 2);	//unclear why, but this is needed every time else setCursor(0,1) doesn't work....probably scope related.
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(DisplayLineTitle);
	lcd.setCursor(0, 1);

	// now write the line
	if (Blank)
	{
		//blank out the character under DisplayPos
		BlinkLine = DisplayLine.substring(DisplayStartPos, DisplayPos) + BlinkCursor + DisplayLine.substring(DisplayPos + 1, DisplayEndPos);
		lcd.print(BlinkLine);
		Blank = false;
	}
	else
	{
		// do not blank out the character under DisplayPos
		lcd.print(DisplayLine.substring(DisplayStartPos, DisplayEndPos));
		Blank = true;
	}
}
//------------------------------------------
boolean DisplayClass::DisplayWriteSD(void)
{
	// writes the current display array to a file on the SD card.  Uses DisplayPntr, DisplayName, and DisplayLineCnt.  file is named DisplayName and is overwritten.  returns true if successful
	if (WriteStringArraySD(DisplayName, DisplayLineCnt, DisplayPntr)) return true; else return false;	//uses method external to Display class because of issues of class containing other classes in arduino's "simplified C/C++"
}
//------------------------------------------
void CursorBlinkIntRedirect(void* context)
{
	// this routine exists outside of the Display class because we can't use some timer.every method within a class in the .pde implementation.  Compiler cannot resolve which routine to call.
	Display.CursorBlinkTimeInt();
}
//------------------------------------------
boolean WriteStringArraySD(String Dname, int Dlines, String *Darray)
{
	/* writes the the string array named Dname to a file on the SD card.  There are Dlines in the array and Darray points to the array.
	file is named Dname and is in the folder /Save in the root of the SD card.  Uses the built in Arduino SD and File classes.  A variable of type File named SDfile
	is declared above and initialized in setup.
	returns true if successful
	*/
	char filename[19];								//SD library uses char* not Sring objects.  max name is 8.3
	Dname = "Save/" + Dname.substring(0, 8) + ".txt";
	Dname.toCharArray(filename, 18);				// room for folder/8.3 filename plus string terminating 0
	if (SD.exists(filename)) SD.remove(filename);			//delete existing file
	SDfile = SD.open(filename, FILE_WRITE);			// create new file

													// if the file opened okay, write to it:
	if (SDfile)
	{
		for (byte x = 0; x<Dlines; x++)
		{
			ddcprintln(*(Darray + x));
			SDfile.println(*(Darray + x));	//	write to end of file
		}

		SDfile.close();		//close the file
		return true;
	}
	else
	{
		// if the file didn't open, print an error:
		dprint(F("error opening SDFile="));	dprintln(filename);//change to error log in future
		return false;
	}
}
//------------------------------------------
byte ReadStringArraySD(String Dname, byte Dlines)
{
	/* Reads Dlines of the string array named Dname from a file on the SD card and loads it into the variable named DisplayBuf.
	The file is named Dname is located in the Save/ folder in the root of the SD care.  Uses the built in Arduino SD and File classes.  A variable of type File named SDfile
	is declared above and initialized in setup.
	Returns the number of lines read if successful else 0.
	*/
	char filename[19];								//SD library uses char* not Sring objects.  max name is 8.3
	Dname = "Save/" + Dname + ".txt";	
	Dname.toCharArray(filename, 18);				// room for folder/8.3 filename plus string terminating 0
	if ((SD.exists(filename)) && (SDfile = SD.open(filename, FILE_READ)))
	{
		// file exists and opened ok
		byte x;
		for (x = 0; x<Dlines; x++)
		{
			char C;						// holds character read from file
			int tmp;					// for int value of C

			DisplayBuf[x] = "";			// initialize display line
			for (byte y = 0; y<200; y++)	//max of DisplayBufLen characters per line. we must read the entire line, but only put a max of DisplayBufLen chr's into the buffer line
			{
				C = SDfile.read();
				tmp = C;					//convert character into int
				if (tmp != 13)			//if not a carriage return = end of line
				{
					if ((y<DisplayBufLen) && (tmp >= 32) && (tmp <= 126))DisplayBuf[x] = DisplayBuf[x] + C;	//add the char to the buffer if there is room & if it is printable
				}
				else break;				//read next line after 
			}
			//dprintln(DisplayBuf[x]);	//debug
		}
		SDfile.close();		//close the file
		return x;			// return the number of lines read

	}
	else
	{
		// file didn't exist or didn't open
		dprint(F("error in ReadStringArraySD, did not find file or could not open it. SDFile="));	dprintln(filename);//change to error log in future
		return 0;
	}
}


/* ----------------------------------------------LS_key routines---------------------------------------------------*/

void KeyPoll(boolean start)
{
	//Routine to start/stop polling for keys being pressed.  Used to turn on/off the keys.
	if (start)
	{
		//start the timer polling at intervals
		LS_curInput = NOKEY_ARV;	// set input value= no key being pressed
		LS_PollContext = Tmr.every(LS_KeyPollRate, CheckKey, (void*)2);	// begin polling keypad, call KeyCheck at intervals of KeyPollRate. timer index = LS_PollContext
		dprint("timer started ");	//delete in future
		dprintln(LS_KeyPollRate);
	}
	else
	{
		Tmr.stop(LS_PollContext);		// stop polling.  Index previously saved with call to every
	}
}
//------------------------------------------
void CheckKey(void* context)
{
	int	x;
	// called by KeyTimer every KeyPollRate to check for if keys changed
	LS_prevInput = LS_curInput;
	LS_curInput = analogRead(LS_AnalogPin);	// read key value through analog pin
											//dprint("previous input=");	//debug
											//dprint(LS_prevInput);//debug
											//dprint("; LS_curInput=");//debug
											//dprintln(LS_curInput);//debug
	x = LS_curInput - LS_prevInput;
	if (abs(x) >= LS_Key_Threshold)
	{
		//current analog value is more than threshold away from previous value then there has been a change, check via debounce
		//_debounce = true;
		// use the timer to delay for KeyDebounceDelay.  After KeyDebounceDelay, GetKey will be called.
		//dprintln("setting up debounce");	//debug
		LS_DebounceContext = Tmr.after(LS_KeyDebounceDelay, GetKey, (void*)5);
	}
	else
	{
		LS_curInput = LS_prevInput;
		//_debounce = false
	}
}
//------------------------------------------
void GetKey(void* context)
{
	// called after debounce delay. Valid key entry is essentially unchanged for the duration of the debounce delay.
	// If valid, then read which key is pressed.
	LS_prevInput = LS_curInput;
	LS_curInput = analogRead(LS_AnalogPin);	// read key value through analog pin
											//dprint("in getKey, after debounce period. previous and current read, abs(cur-prev): ");
											//dprint(LS_prevInput); dprint(",  ");
											//dprint(LS_curInput); dprint(",   ");
											//dprintln(abs(LS_curInput- LS_prevInput));
	if (abs(LS_curInput - LS_prevInput) <= LS_Key_Threshold)
	{
		//Key press is confirmed because there is no substantive change in key input value after debounce delay
		LS_KeyReady = true;		// flag to signal that key is ready to read.  Main loop checks for this.
								//dprintln("in getkey determining which key is pressed"); //debug
								//determine which key is pressed from the voltage on the analog input
		if (LS_curInput > UPKEY_ARV - LS_Key_Threshold && LS_curInput < UPKEY_ARV + LS_Key_Threshold) LS_curKey = UP_KEY;
		else if (LS_curInput > DOWNKEY_ARV - LS_Key_Threshold && LS_curInput < DOWNKEY_ARV + LS_Key_Threshold) LS_curKey = DOWN_KEY;
		else if (LS_curInput > RIGHTKEY_ARV - LS_Key_Threshold && LS_curInput < RIGHTKEY_ARV + LS_Key_Threshold) LS_curKey = RIGHT_KEY;
		else if (LS_curInput > LEFTKEY_ARV - LS_Key_Threshold && LS_curInput < LEFTKEY_ARV + LS_Key_Threshold) LS_curKey = LEFT_KEY;
		else if (LS_curInput > SELKEY_ARV - LS_Key_Threshold && LS_curInput < SELKEY_ARV + LS_Key_Threshold) LS_curKey = SELECT_KEY;
		else
		{
			LS_curKey = NO_KEY;
			LS_KeyReady = false;	// ignore key being released.  will catch the key when pressed.
		}
		//dprint("current key ="); //debug
		//dprintln(LS_curKey); //debug
	}
	else
	{
		//key press in not confirmed because there is a difference in reading > threshold after delay
		LS_curInput = LS_prevInput;
		LS_KeyReady = false;

	}
}
//------------------------------------------
int ReadKey(void)
{
	// gets result of key pressed if it is ready.  Main loop checks LS_KeyReady and calls this to get result when true
	if (LS_KeyReady == true)
	{
		LS_KeyReady = false;	// change the ready to read flag so the main loop only reads the key once
		return LS_curKey;

	}
	else
	{
		return NO_KEY;
	}
}


/* ---------------------------------------------- Real Time Clock routines ---------------------------------------*/
void SysTimePoll(boolean start)
{
	//Routine to start/stop polling for RTC time.  Used to turn on/off the timer.

	if (start)
	{
		//start the polling at for system time at interval 
		SysTmPoleContext = Tmr.every(SysTmPoleFreq, GetSysTime, (void*)3);	// begin polling system timer (RTC), call GetSysTime at intervals of SysTmPoleFreq. timer index = SysTmPoleContext
		dprintln("RTC polling started ");
	}
	else
	{
		Tmr.stop(SysTmPoleContext);		// stop polling
	}

}
//------------------------------------------
void GetSysTime(void* context)
{

	// routine to get system time and save in SysTm and LogTm
	if (RTC.read(SysTm))
	{
		// set up string of system time in 24 hr format
		String tmpTime, strSnip, DayPart, MonthPart, YearPart;
		strSnip = String(SysTm.Hour);	// hr to String
										//dprint("hour="); dprint(strSnip);
		if (strSnip.length() == 1) strSnip = '0' + strSnip;	//hr needs to be 2 chr
		SysTmStr = strSnip + ":";
		strSnip = String(SysTm.Minute);
		//dprint(" Min="); dprint(strSnip);		
		if (strSnip.length() == 1) strSnip = '0' + strSnip;	//min needs to be 2 chr		
		SysTmStr = SysTmStr + strSnip + ":";	// add hrs
		strSnip = String(SysTm.Second);
		//dprint(" Sec="); dprintln(strSnip);
		if (strSnip.length() == 1) strSnip = '0' + strSnip;	//sec needs to be 2 chr
		SysTmStr = SysTmStr + strSnip;	// add sec to complete system time string in 24 hr format as hh:mm:ss

										//set up string of system date as mm/dd/yyyy
		YearPart = String(tmYearToCalendar(SysTm.Year));	// year as yyyy
		MonthPart = String(SysTm.Month);	// month to String
										//dprint("Month="); dprint(strSnip);		
		if (MonthPart.length() == 1) MonthPart = '0' + MonthPart;	//month needs to be 2 chr
		SysDateStr = strSnip + "/";
		DayPart = String(SysTm.Day);
		if (DayPart.length() == 1) DayPart = '0' + DayPart;	//day needs to be 2 chr
		
		SysDateStr = MonthPart + "/" + DayPart + "/" + YearPart;	// format SysDateStr as mm/dd/yyyy		
		LogTm = YearPart + "-" + MonthPart + "-" + DayPart + "T" + SysTmStr;	//create the time string in the format 2016 - 07 - 27T00:00 : 00, used for error log and data log

		sysDOWstr = Display.ProgMemLU(DisplayDOW, SysTm.Wday, 3); // DisplayDOW[SysTm.Wday];	//set day of week string. SysTm.Wday is int where 1=sunday
																  //dprintln ("SysTmStr=" + SysTmStr + ", SysDateStr=" + SysDateStr + ", SysDOW=" + sysDOWstr);
																  /*
																  dprint("Ok, Time = ");
																  dprint(SysTm.Hour);
																  Serial.write(':');
																  dprint(SysTm.Minute);
																  Serial.write(':');
																  dprint(SysTm.Second);
																  dprint(", Date (D/M/Y) = ");
																  dprint(SysTm.Day);
																  Serial.write('/');
																  dprint(SysTm.Month);
																  Serial.write('/');
																  dprint(tmYearToCalendar(SysTm.Year));
																  dprintln();
																  */

	}
	else
	{
		if (RTC.chipPresent())
		{
			dprintln("The DS1307 is stopped.  Please run the SetTime");
		}
		else
		{
			ErrorLog("DS1307 read error!  Please check the circuitry.","",1);
		}
	}
}
//------------------------------------------

/* ---------------------------------------------- Memory Usage Routines ---------------------------------------*/
extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

uint16_t getFreeSram() {
	uint8_t newVariable;
	// heap is empty, use bss as start memory address
	if ((uint16_t)__brkval == 0)
		return (((uint16_t)&newVariable) - ((uint16_t)&__bss_end));
	// use heap end as the start of the memory address
	else
		return (((uint16_t)&newVariable) - ((uint16_t)__brkval));
};

//-------------------------------------- ---------Flow Sensor variables and class--------------------------------------
/*
YF-S201 Hall Effect Water Flow Meter / Sensor
two flow sensors attached, each to its own input pin.
We use a timer event and polling to read the value of the flow sensors, incrementing a flow sensor
specific counter when the state changes (no debounce needed with Hall effect sensors).
Polling frequency just above 2x expected for max flow rate. In current application read
at 200/sec or once every 5 ms.
Four functions used:
FlowCalcSetup: sets input pins and variables
FlowCalcBegin: initiates the timers
FlowCalcTick: called every 5ms, checks for state changes of flow sensors and increments cntrs
FlowCalcRead: called to read and report flow rates of all sensors

Read Water Flow Meter and output reading in liters/hour
*/
class FlowSensors
{
protected: 
#define	flowmeter1  2		// 1st flow Meter on pin 2
#define	flowmeter2  3		//2nd flow meter 0n pin3
#define FlowMonitoringInterval 300000	// default sampling interval in ms = 5 min
#define FlowTestingInterval 5000		// sampling interval used for testing = 5 sec
	unsigned long	ReadFlowInterval = 2000;	// interval to read flow sensors at
	boolean	usingFlowSensors = false;	// if true, then in the state of using flow sensors, else not
	boolean testingFlowSensors = false;	// if true, then testing flow sensors else not.   note, using and testing are mutually exclusive
	
	int8_t flowTickContext;			// index to FlowTick timer in SensTmr
	int8_t flowReadContext;			// index to FlowRead timer in SensTmr
	unsigned long flow1tick = 0;	//frequency counter for flowsensor 1
	unsigned long flow2tick = 0;	//frequency counter for flowsensor 2
	boolean FlowState1 = false;		// high/low state of input for flow sensor 1
	boolean FlowState2 = false;		// state of flow sensor 2

	boolean flow1 = false;
	boolean flow2 = false;
	
	unsigned long flowStartTime;	//starting time for flow calculations
	unsigned long flowEndTime;		//ending time for flow calculationsunsigned 
	long currentTime;
	unsigned long cloopTime;

public:
	String			Flow1Name;					// string name for the sensor.  e.g. upper pump
	String			Flow2Name;					// string name for the sensor.  e.g. Lower pump
	//boolean			IsOn;					// true if activly taking sensor readings else false
	int	Flow1Warn;							// set warning LED if flow less than this level
	int	Flow2Warn;							// set warning LED if flow rate below this level
	unsigned int	FlowValue1;					// flow meter 1 reading in l/min
	unsigned int	FlowValue2;					// reading for meter 2
	unsigned long flow1dur = 0;					//duty cycle for flowsensor 1, useful for adjusting sampling rate.
	unsigned long flow2dur = 0;					//duty cycle for flowsensor 2
	boolean FlowReadReady = false;				//used in main loop, if true that ok to read flow values 1-2

	void FlowCalcSetup(void);				//sets up pins for flow sensors	
	void FlowStartStop(boolean Start);		// if Start=true then enable soft interupts to measure flow, else turn them off.
	void SetReadFlowInterval(unsigned long interval);	//sets soft interupt for how often to read the flow sensors
	unsigned long GetReadFlowInterval(void);	// returns the soft interupt interval for reading flow sensors
	void FlowCalcTick(void);				// called to check for changes of state of flow sensors

	void FlowCalcBegin(void);				// sets counters at the start of a flow calculation
	void FlowCalcRead(void);				// reads the values of all flow sensors

} FlowSens;

void FlowSensors::FlowCalcSetup()		//sets up pins for flow sensors and sets the timers to measure and read flow sensors
{
	pinMode(flowmeter1, INPUT);
	pinMode(flowmeter2, INPUT);
}
//----------------------------------------------------------------------
void FlowSensors::FlowStartStop(boolean Start)		// if Start=true then enable soft interupts to measure flow, else turn them off.
{
	if (Start)
	{
		flowTickContext = SensTmr.every(5, FlowCalcTickRedirect, (void*)2);	// calls FlowCalcTick every 5ms to check for state change
		flowReadContext = SensTmr.every(ReadFlowInterval, FlowCalcReadRedirect, (void*)3);	// calls to read flow sensors
		usingFlowSensors = true;	// tells us we are using the flow sensore....timers on
		FlowCalcBegin();			// reset counters used for flow calculations
	}
	else
	{
		FlowReadReady = false;
		usingFlowSensors = false;			// tells us we are not currently using the flow sensors...no timers
		SensTmr.stop(flowTickContext);		// turn off timer for FlowCalcTickRedirect
		SensTmr.stop(flowReadContext);		// turn off timer to read flow sensors
	}
}
//----------------------------------------------------------------------
void FlowSensors::SetReadFlowInterval(unsigned long interval)
{
	ReadFlowInterval = interval;
	if (usingFlowSensors)
	{
		// we are using the timers for to read the flow sensors, so change the timer interval
		SensTmr.stop(flowReadContext);		// turn off timer to read flow sensors
		flowReadContext = SensTmr.every(ReadFlowInterval, FlowCalcReadRedirect, (void*)3);	// start timer up again with the new interval
		FlowCalcBegin();	//reset counters to insure correct flow readings on the next read cycle
	}
}
//----------------------------------------------------------------------
unsigned long FlowSensors::GetReadFlowInterval(void)
{
	//returns the value of a private variable
	return ReadFlowInterval;
}
//----------------------------------------------------------------------

void FlowSensors::FlowCalcBegin()		// sets counters at the start of a flow calculation
{
	flow1tick = flow2tick = 0;	// zero out flow sensor frequency counters
	FlowState1 = digitalRead(flowmeter1);	//starting state of the flow meters
	FlowState2 = digitalRead(flowmeter1);
	flowStartTime = millis();				 // set starting time for measurement interval. note, millis wraps every ~ 50 days so need to check if end <start when reading
}
//----------------------------------------------------------------------
void FlowSensors::FlowCalcTick(void)			// called by FlowcalcTickRedirect to check for changes of state of flow sensors
{
	//flow1=digitalRead(flowmeter1);
	//flow2=digitalRead(flowmeter2);

	if (digitalRead(flowmeter1) != FlowState1)
	{
		FlowState1 = !FlowState1;	//invert flow state, begin checking for next state change
		flow1tick++;				//increment frequency counter because pin changed state
	}

	if (digitalRead(flowmeter2) != FlowState2)
	{
		FlowState2 = !FlowState2;	//invert flow state, begin checking for next state change
		flow2tick++;				//increment frequency counter because pin changed state
	}
}
//----------------------------------------------------------------------
void FlowSensors::FlowCalcRead(void)			// reads the values of all flow sensors
{	// called by timer ever ReadFlowInterval
	long ElapsedTime;	//local variable for elapsed time since last calculation in milliseconds

	flowEndTime = millis();	//ending time for this flow calculation.

	if (flowEndTime>flowStartTime)
	{
		ElapsedTime = flowEndTime - flowStartTime;
		FlowValue1 = (flow1tick * 60 * 1000 / ElapsedTime / 7.5);// (Pulse frequency x 60 min) * (1000 ms/ElapsedTime in ms) / 7.5Q = flow rate in L/hour
		FlowValue2 = (flow2tick * 60 * 1000 / ElapsedTime / 7.5);
		FlowReadReady = true;	// set flag indicating flow results are ready to read

								//look at duration of wave form duration because of Nyquist, sampling rate=5ms so avg duration should be > 10ms
		flow1dur = ElapsedTime / (flow1tick + 1);
		flow2dur = ElapsedTime / (flow2tick + 1);
	}
	else
	{
		// millisecond timer has wrapped around (~ every 70 days), so can't use this reading
		FlowReadReady = false;
	}

	FlowCalcBegin();	// restart the flow calculation using the current run of millis
}
//----------------------------------------------------------------------
void FlowCalcTickRedirect(void* context)
{
	// this routine exists outside of the FlowSensor class because we can't use some timer.every method within a class in the .pde implementation.  Compiler cannot resolve which routine to call.
	FlowSens.FlowCalcTick();	// check for changes of state of flow sensors
}
//----------------------------------------------------------------------
void FlowCalcReadRedirect(void* context)
{
	// this routine exists outside of the FlowSensor class because we can't use some timer.every method within a class in the .pde implementation.  Compiler cannot resolve which routine to call.
	FlowSens.FlowCalcRead();		// call method that reads the flow sensors and sets the flag in the main loop telling us it is time to do something with the results
}

//-------------------------------------Dallas Temperature Sensor variables and class------------------------------

#define ONE_WIRE_BUS 11	// data is on digital pin 11 of the arduino
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);	// Pass our oneWire reference to Dallas Temperature.

//int numberOfDevices; // Number of temperature devices found

//DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address

class TempSensor
{
	/*
	class for temp sensor.  Temp sensor uses I2C bus and is a DS18b20 temperature sensor.  This class handles set up, reading,
	storage of info needed to display results in the cloud.  Polling is not handled by this class.....mostly because Arduino
	limits this class using another class in an external library, specifically the Timer class.

	*/
protected:
#define TEMPERATURE_PRECISION 10	// temp precision 10 bit, options are 9, 10, 11, or 12
#define TempMonitoringInterval 300000	// default sampling interval in ms = 5 min
#define TempTestingInterval 3000		// sampling interval used for testing = 3 sec
	byte			Snum;			// number of the sensor device on the I2C buss
	String			Sname;			// name of sensor to use with logs and displays
	String			SIDstring;		// string of ID for sensor when used in cloud based data logging

	//boolean			IsOnStorage;	// saves the state of IsOn during testing mode
	long int		PollInterval;	// polling interval
	long int		PollIntervalStorage;	// holds prior poll interval when it is changed for testing
	int				SensorPollContext;	//variable set by SensTmr and passed by code into SensTmr.  It is an index for the timer object
public:
	DeviceAddress	Saddr;			// I2C address of the sensor
	String			SensName;		// string name for the sensor.  e.g. Pond Temp
	String			SensHandle;		// Not currently used: string 'handle' used to ID sensor in internet data stream. 
	//boolean			IsOn;			// true if sensor has been initialized and once 'turned on', we will activly taking sensor readings else false
	float	TempC;					// last temperature reading in celcius.	if -100 then not valid
	float	TempF;					// last temperature reading in farenheight.  If-100 then not valid
	float	AlarmThreshHigh;		// temperature (F) that will trigger high temp alarm...getting too hot!
	float	AlarmThreshLow;			// temperature (F) that will trigger low temp alarm....burrrrrr!
	boolean	TempSensReady;			// tells main loop that there is a temperature reading that is ready to be processed
	boolean	TempSensReadDealyIsDone;// used by ReadTempSensor() to implement 1 sec read delay and continue to process other things


	void	TempSensorInit(byte SN);	//used like constructor because constructor syntax was not working ;-(.  passes in device #
	void	TurnOn(boolean TurnOn);		//turn on/off flags and timers used to take readings at intervals
	void	TestMode(boolean StartTesting);		// Preserves the state of IsOn while in testing mode.  If true, saves the state of IsOn, sets IsOn=true, and changes polling rate. If false, resumes prior state of IsOn and polling interval
	void	ReadTempSensor(void);		// called at polling intervals, reads the temp sensor, and sets results and flags indicating a reading is ready for use
	void	SetPollInterval(int Delay);	//sets the poll interval, changes the poll interval if sensor IsOn=true	//boolean Locate(void);				// locates the temp sensor at Saddr, returns true if found else false
										//float	ReadC(void);				// converts 
										//float	ReadF(void);				// read value of sensor and return temp in degrees F
	void	printAddress(void);			// prints the address in Saddr 
} TempSens0, TempSens1;

/*--------------------------------------------------------------------------------------------------------------------
methods for Temperature sensor class
--------------------------------------------------------------------------------------------------------------------*/

//----------------------------------------------------------------------
void	TempSensor::TempSensorInit(byte SN)
{
	// constructor, 0 stuff out, find device address, set precision
	//IsOn = false;
	Sname = SIDstring = "";
	TempC = TempF = -100;	//preset to value indicating invalid temp...probably not needed because of TempSensReady
	//PollInterval = TempMonitoringInterval;	//default polling rate in MS
	Snum = SN;
	TempSensReady = false;	// tells main loop that the temperature readings are not ready to be read

	if (sensors.getAddress(Saddr, SN))	// Search the I2C bus for address
	{
		//if true, device address in set in Saddr else error
		dprint("Found device number= ");	//debug
		dprint(Snum);	//debug
		dprint(" with address: "); //debug
										 //printAddress(tempDeviceAddress);
		for (uint8_t i = 0; i < 8; i++)
		{
			if (Saddr[i] < 16) dprint("0");
			dprint2(Saddr[i],HEX);
		}
		dprintln();

		dprint("Setting resolution to ");
		dprintln2(TEMPERATURE_PRECISION, DEC);
		sensors.setResolution(Saddr, TEMPERATURE_PRECISION);// set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)

		dprint("Resolution actually set to: ");
		dprint2(sensors.getResolution(Saddr), DEC);
		dprintln();
	}
	else
	{
		dprint("Error, did not find temp sensor address");	//debug
																	//add error log entry here
	}
}
//----------------------------------------------------------------------
/*
void	TempSensor::TurnOn(boolean TurnOn)
{

	//if true, set Turn on sampling 
	if (TurnOn)
	{
		//if here, then we want to turn on the polling for taking temperature readings.  We use SensTmr object of the Timer class
		//IsOn = true;	//flag that we are taking Temperature sensor readings
		SensorPollContext = SensTmr.every(PollInterval, SensorPollRedirect, (void*)2);	// begin polling temp readings, call SensorPollRedirect at intervals of PollInterval. timer index = SensorPollContext
	}
	else
	{
		// turn off polling for temperature sensor readings
		//IsOn = false;	//flag that we are not taking temperature sensor readings
		SensTmr.stop(SensorPollContext);	//turns off the poll timer for this context
	}
	TempSensReadDealyIsDone = false;		// flag used by soft interupt to read temp sensor.
};
*/
//----------------------------------------------------------------------
void	TempSensor::TestMode(boolean StartTesting)
{
	// Preserves the state of IsOn while in testing mode.  If true, saves the state of IsOn, sets IsOn=true, and changes polling rate. If false, resumes prior state of IsOn and polling interval
	if (StartTesting)
	{
		PollIntervalStorage = PollInterval;	// save the polling interval used for measurement
		TempSensReadDealyIsDone = false;		// flag used by soft interupt to read temp sensor.	
		SetPollInterval(TempTestingInterval);	// sample at interval appropriate for testing. e.g. every 3 sec
	}
	else
	{
		//testing done, resume prior state
		TempSensReadDealyIsDone = false;		// flag used by soft interupt to read temp sensor.		
		SetPollInterval(PollIntervalStorage);	//resume prior polling interval 
	}
}
//----------------------------------------------------------------------
void	TempSensor::ReadTempSensor(void)
{
	// called at polling intervals (SensorPollRedirect is called at intervals set up by TurnOn and calls the routing. See SensorPollRedirect for expalination of need for indirection
	// This routine reads the temp sensor, and sets results and flags indicating a reading is ready for use

	//if here, read ok so get result. SensorPollRedirect calls sensors.requestTemperatures() to tell all temp sensors to take temperature.  It then sets up a 1 sec delay 
	//for conversion to occur, then calls this routine to actually read the temperature.
	TempC = sensors.getTempC(Saddr);
	TempF = DallasTemperature::toFahrenheit(TempC);	//convert to farenheit
	TempSensReady = true;	// tells main loop that temp is ready to read
	
}
//----------------------------------------------------------------------
void	TempSensor::SetPollInterval(int Delay)
{
	//saves the poll interval, changes the poll interval if sensor IsOn=true
	PollInterval = Delay;
	SensTmr.stop(SensorPollContext);	//turns off the poll timer for this context	
	SensorPollContext = SensTmr.every(PollInterval, SensorPollRedirect, (void*)2);	// begin polling temp readings at new polling interval
}
//----------------------------------------------------------------------
void	TempSensor::printAddress(void) 	// prints the address of temp sensor
{
	for (uint8_t i = 0; i < 8; i++)
	{
		if (Saddr[i] < 16) dprint("0");
		dprint2(Saddr[i], HEX);
	}
}
//----------------------------------------------------------------------
void SensorPollRedirect(void* context)
{
	
	int	tmpInt;
		/* This routine exists outside of the Sensor class because we can't use some timer.every method within a class in the .pde implementation.  Compiler cannot resolve which routine to call.
		The routine is called by two soft interupts.  1) at repeated intervals to take temperature readings.  2) Once it is time to take a reading, a one-shot
		'soft interupt' is used to create a 1 sec delay, which is the time to wait for the temp sensor to make a measurement.
		*/
	if(TempSens0.TempSensReadDealyIsDone)
	{ 
		// here only if prior call to this routing was made and a 1 sec delay occurred
		TempSens0.ReadTempSensor();	// read temperature now that 1 sec delay has occurred
		TempSens1.ReadTempSensor();
		TempSens0.TempSensReadDealyIsDone = false;				// set to false so when it is time to read the sensors again, we will add a 1 sec delay for conversion time.
	}
	else
	{
		sensors.requestTemperatures();								// broadcast command to all  sensors to measure temperature.  Read after 1 sec.
		tmpInt = SensTmr.after(1000, SensorPollRedirect, (void*)2);	// return to this routine after a 1 sec delay.  Processing continues during this time.
		TempSens0.TempSensReadDealyIsDone = true;					// next time this routine is entered, the temp sensors will be read.
	}
}

//----------------------------------------Water Level Sensor variables and class-----------------------------------
class WaterLvlSensor
{
	/*
	class for moisture sensor, adapted to sense water level.  WaterLvl sensor uses the Fundouino moisture sensor and analogue input
	A1. Polling is not handled by this class.....mostly because Arduino
	limits this class using another class in an external library, specifically the Timer class.

	*/
protected:
#define WaterLvlPowerPin	23		// Digital pin 23 powers the moisture sensor.  Turning on only to read will prolong sensor life due to electroplating
#define WaterLvlInput	1			// Analog input pin 
#define WaterLvlSampleInterval 10000// default sampling interval in ms
#define WaterLvlTestingInterval 2000 // sampeling interval when testing this sensor
#define WaterLvlReadDelay	25		// number of ms to wait to read moisture sensor after turning on the power
#define	WaterDfltNo	20				// Analog reading < means no water touching sensor
#define WaterDfltLow 140			// Analog reading < this and greater than prior threshold means low level
#define WaterDfltMid 220			// Analog reading < this and greater than prior threshold means mid level

	//boolean	IsOn;					// true if activly taking sensor readings else false
	boolean	ReadDelay;				// if true, the window where power to sensor is on and waiting to read
	int		PollInterval;			// polling interval
	int		SensorPollContext;		// variable set by SensTmr and passed by code into SensTmr.  It is an index for the timer object
	int		PowerDelayContext;		// context for timer that turns on power for WaterLvlReadDelay before reading.

public:
	int		WaterLvlNo, WaterLvlLow, WaterLvlMid;	//storage of water level thresholds.  Will default to defines above, but will use variables so they changed by UI
	float	WaterLvl;					// last reading by the moisture sensor
	String	WaterLvlRange;				// sets a range value for the water level: none, low, mid, high.  Used by pump control to turn on and off
	String	PriorLvlRange;				// saves state of prior water level reading, needed to determine if water level is increasing or decreasing
	boolean WaterLvlIncreasing;			// if true, then water level on its way up else down.
	boolean	WaterLvlSensReady;			// tells main loop that there is a water level reading that is ready to be processed
	int		tempWater;					//debug
	void	WaterLvlSensorInit(void);	//used like constructor because constructor syntax was not working ;-(.  passes in device #
	void	TurnOn(boolean TurnOn);		//turn on/off flags and timers used to take readings at intervals
	void	ReadWaterLvlSensor(void);	// called at polling intervals, reads the water level sensor, and sets results and flags indicating a reading is ready for use
	void	SetPollInterval(int Delay);	//sets the poll interval, changes the poll interval if sensor IsOn=true	//boolean Locate(void);				// locates the temp sensor at Saddr, returns true if found else false
	int		GetPollInterval(void);		//gets the value of the polling interval (private to the class)
} WaterSens;

void WaterLvlSensor::WaterLvlSensorInit(void)
{
	analogReference(DEFAULT);				//sets voltage reference for ADC to 5V.  Moisture sensor uses analog input
	PollInterval = WaterLvlSampleInterval;	// set initial polling interval
	pinMode(WaterLvlPowerPin, OUTPUT);		// using a digital output to power the moisture sensor, which used ~25ma.  Digital outputs on Mega can source 40 ma.
	digitalWrite(WaterLvlPowerPin, LOW);	// turn off power to moisture sensor
	ReadDelay = false;						// not in window after turning on power and before reading sensor
											// set thresholds used to determine WaterLvlRange, which is used externally to controll stuff
	WaterLvlNo = WaterDfltNo;
	WaterLvlLow = WaterDfltLow;
	WaterLvlMid = WaterDfltMid;
}
//----------------------------------------------------------------------
void	WaterLvlSensor::TurnOn(boolean TurnOn)
{

	//if true, set Turn on sampling 
	if (TurnOn)
	{
		//if here, then we want to turn on the polling for taking water level readings.  We use SensTmr object of the Timer class
		//digitalWrite(WaterLvlPowerPin, HIGH);	// turn on power
		//IsOn = true;	//flag that we are taking moisture sensor readings

		//set the water level range and prior level.  Needed by logic to tell if water is increasing or decreasing
		digitalWrite(WaterLvlPowerPin, HIGH);	// turn power on
		WaterLvl = analogRead(WaterLvlInput);	//read water level analog input
		digitalWrite(WaterLvlPowerPin, LOW);	// turn power off
												// set level flags based on reading.
		if (WaterLvl < WaterLvlNo) WaterLvlRange = "none"; else
			if (WaterLvl < WaterLvlLow) WaterLvlRange = "low"; else
				if (WaterLvl < WaterLvlMid) WaterLvlRange = "mid"; else
					WaterLvlRange = "high";
		PriorLvlRange = WaterLvlRange;		// current and prior water level ranges set
		WaterLvlIncreasing = true;			// start with assumption that system is not hemoraging water so that when pumps go on, level will be increasing

		SensorPollContext = SensTmr.every(PollInterval, WaterLvlPollRedirect, (void*)4);	// begin polling temp readings, call SensorPollRedirect at intervals of PollInterval. timer index = SensorPollContext
	}
	else
	{
		// turn off polling for temperature sensor readings
		//digitalWrite(WaterLvlPowerPin, LOW);	// turn off power
		//IsOn = false;	//flag that we are not taking water level readings
		SensTmr.stop(SensorPollContext);	//turns off the poll timer for this context
	}
	ReadDelay = false;	//flag related to power up delay when reading moisture sensor.
};
//----------------------------------------------------------------------
void	WaterLvlSensor::ReadWaterLvlSensor(void)
{
	// called at polling intervals (WaterLvlPollRedirect is called at intervals set up by TurnOn and calls the routing. See SensorPollRedirect for expalination of need for indirection
	// based on the value of ReadDelay, This routine either turns on the power to the moisture sensor, sets the delay before reading.
	// or reads the water level sensor, sets results and flags indicating a reading is ready for use, and resets polling interval

	if (!(ReadDelay))
	{
		//time to read the moisture level.  However, the power needs to be turned on and we need to wait for the sensor to settle.  The delay is set here
		ReadDelay = true;	// flag we are entering the power up delay
		digitalWrite(WaterLvlPowerPin, HIGH);	// turn on power
		PowerDelayContext = SensTmr.after(WaterLvlReadDelay, WaterLvlPollRedirect, (void*)5);	//	return to WaterLvlPollRedirect after power up delay
	}
	else
	{
		//power up delay is done, 
		ReadDelay = false;	// flag that power up delay is over
		PriorLvlRange = WaterLvlRange;	//save prior water level.  Needed to determine if level increasing or decreasing
		WaterLvl = analogRead(WaterLvlInput);	//read water level analog input
		digitalWrite(WaterLvlPowerPin, LOW);	// turn power off

												// set level flags based on reading.
		if (WaterLvl < WaterLvlNo) WaterLvlRange = "none"; else
			if (WaterLvl < WaterLvlLow) WaterLvlRange = "low"; else
				if (WaterLvl < WaterLvlMid) WaterLvlRange = "mid"; else
					WaterLvlRange = "high";

		// determine if water level is increasing or decreasing and set WaterLvlIncreasing accordingly
		//if ((PriorLvlRange == "none") && (WaterLvlRange == "none") && WaterLvlIncreasing) WaterLvlIncreasing = true;

		WaterLvlSensReady = true;	//used by main loop to know when to read water level
	}
}
//----------------------------------------------------------------------
void	WaterLvlSensor::SetPollInterval(int Delay)
{
	//saves the poll interval, changes the poll interval if sensor IsOn=true
	PollInterval = Delay;
	//if (IsOn)
	//{
		SensTmr.stop(SensorPollContext);	//turns off the poll timer for this context	
		SensorPollContext = SensTmr.every(PollInterval, WaterLvlPollRedirect, (void*)4);	// begin polling temp readings at new polling interval
	//}
}
//----------------------------------------------------------------------
int		WaterLvlSensor::GetPollInterval(void)
{
	//returns the value of the polling interval, which is private
	return PollInterval;
}
//----------------------------------------------------------------------
void WaterLvlPollRedirect(void* context)
{
	// this routine exists outside of the Sensor class because we can't use some timer.every method within a class in the .pde implementation.  Compiler cannot resolve which routine to call.
	WaterSens.ReadWaterLvlSensor();	//read the water level sensor
}
//--------------------------------------------Relay Board variables and class -------------------------------------
class RelayBoard
{
	/*
	class for Relay module board. Board is made by ### jf here ### and has 4 relays, each activated by a digital pin going low.

	*/
protected:
	boolean	Relay1wiredNO;		//if true then wired as NO (normally open) and circuit is off when relay is off, else wired as NC (normally closed=circuit on when relay is off)
	boolean	Relay2wiredNO;
	boolean	Relay3wiredNO;
	boolean	Relay4wiredNO;
	boolean Relay1State;		//logical on/of state of circuit attached to relay
	boolean Relay2State;
	boolean Relay3State;
	boolean Relay4State;
	boolean IsOn;			// flag for enabled/disabled board. 
public:
#define Relay1	24		// Digital pin to control this relay
#define Relay2	25		
#define Relay3	26		
#define Relay4	27	



	void	InitRelayBoard(boolean NO1, boolean NO2, boolean NO3, boolean NO4);	// initializes Relay board and sets wiring variables identifying if relay is wired NO or NC		
	void	RelaySet(byte RelayNum, boolean CircuitOn);	// controls the logical state of RelayNum.  If CircuitOn=true then Relay is set such the the circuit is on, takning wiring (NO vs NC) into consideration.
	void	RelayToggle(byte RelayNum);		//toggles the state of the relay.  If off, turns on.
	boolean	GetRelayState(byte RelayNum);	// returns logical state of RelayNum
											//void	TurnOn(boolean TurnOn);			// enables / disables the use of the relay board.  When turned on, set all relays to off state
											//void	DefineRelayWiring (boolean 1NO, boolean 2NO, boolean 3NO, boolean 4NO);	//defines the wiring state for the relays.  NO=normally open= when off, circut is open, else when off, circut closed
} Relay;

void RelayBoard::InitRelayBoard(boolean NO1, boolean NO2, boolean NO3, boolean NO4)
{
	//set state of the relay wiring, used to logically turn on/off
	Relay1wiredNO = NO1;		//if true then wired as NO (normally open) and circuit is off when relay is off, else wired as NC (normally closed=circuit on when relay is off)
	Relay2wiredNO = NO2;
	Relay3wiredNO = NO3;
	Relay4wiredNO = NO4;

	// set the logical state of the relays that represent circuit connected to relay is open (off).
	Relay1State = !NO1;
	Relay2State = !NO2;
	Relay3State = !NO3;
	Relay4State = !NO4;

	//initializes and turns off relays.  Relays are activated when digital line is brought low..
	pinMode(Relay1, OUTPUT);
	digitalWrite(Relay1, Relay1State);	// turn off Relay 1
	pinMode(Relay2, OUTPUT);
	digitalWrite(Relay2, Relay2State);	// turn off Relay 2
	pinMode(Relay3, OUTPUT);
	digitalWrite(Relay3, Relay3State);	// turn off Relay 3
	pinMode(Relay4, OUTPUT);
	digitalWrite(Relay4, Relay4State);	// turn off Relay 4


	//dprintln(F("Relay initialized."));
}
//----------------------------------------------------------------------
void	RelayBoard::RelaySet(byte RelayNum, boolean CircuitOn)
{
	// Sets the state of circuit attached RelayNum to RelayState, taking wiring of relay into account.
	byte	PinNum;	// local to hold number of digital pin
	boolean	isNO;	// local to hold wiring of relay (Normally Open vs NC)
	switch (RelayNum)
	{
	case 1: Relay1State = CircuitOn;
		PinNum = Relay1;
		isNO = Relay1wiredNO;
		break;
	case 2: Relay2State = CircuitOn;
		PinNum = Relay2;
		isNO = Relay2wiredNO;
		break;
	case 3: Relay3State = CircuitOn;
		PinNum = Relay3;
		isNO = Relay3wiredNO;
		break;
	case 4: Relay4State = CircuitOn;
		PinNum = Relay4;
		isNO = Relay4wiredNO;
		break;
	default:
		break;
	}

	if (isNO) digitalWrite(PinNum, !CircuitOn); else digitalWrite(PinNum, CircuitOn);
}
//----------------------------------------------------------------------
void	RelayBoard::RelayToggle(byte RelayNum)
{
	// toggle the state of the circuit attached to relay.  Take wiring into account (relay is wired Normally Open vs NC)
	byte	PinNum;	// local to hold number of digital pin
	boolean	isNO;	// local to hold wiring of relay (Normally Open vs NC)
	boolean RelayState;	// local variable to hold state to write to relay
	switch (RelayNum)
	{
	case 1:
		RelayState = Relay1State = !(Relay1State); //toggle variable holding the logical state of the circuit attached to relay and save locally to use to write to the port
		PinNum = Relay1;
		isNO = Relay1wiredNO;
		break;
	case 2:
		RelayState = Relay2State = !(Relay2State);
		PinNum = Relay2;
		isNO = Relay2wiredNO;
		break;
	case 3:
		RelayState = Relay3State = !(Relay3State);
		PinNum = Relay3;
		isNO = Relay3wiredNO;
		break;
	case 4:
		RelayState = Relay4State = !(Relay4State);
		PinNum = Relay4;
		isNO = Relay4wiredNO;
		break;
	default:
		break;
	}
	if (isNO) digitalWrite(PinNum, !RelayState); else digitalWrite(PinNum, RelayState);	// change relay state.
}
//----------------------------------------------------------------------
boolean	RelayBoard::GetRelayState(byte RelayNum)
{
	// get the logical state of the circuit attached to the relay 

	switch (RelayNum)
	{
	case 1:
		return Relay1State;
		break;
	case 2:
		return Relay2State;
		break;
	case 3:
		return Relay3State;
		break;
	case 4:
		return Relay4State;
		break;
	default:
		return 0;	// error.
		ErrorLog("Error in GetRelayState, unrecognized realy num", String(RelayNum), 2);	// make error log entry
		break;
	}
}
//----------------------------------------------Pump variables and class ------------------------------------------
class H2Opumps
{
	/*
	there are 2 water pumps, lower and upper.  They have a PumpMode that is in the SysStat display array: On, Off, and Auto.
	This class works with the Relay class as the pumps are controlled by the relay board.  The relay board has 4 relays, and the pumps use only two
	*/
public:
#define uPump	1	//relay number for the upper pump
#define lPump	2	// relay number for the lower pump
#define pumpOn	true
#define pumpOff	false
	boolean uPumpIsOn;
	boolean lPumpInOn;
	void pumpInit(void);	// initialize pump logical states to off
	void SetPump(byte PumpNum, boolean TurnOn);		// checks the current state of PumpNum and changes it to match TurnOn as needed
} pump;

void H2Opumps::pumpInit(void)
{
	//initializes pump state variables to false
	uPumpIsOn = lPumpInOn = false;
}
//----------------------------------------------------------------------
void H2Opumps::SetPump(byte PumpNum, boolean TurnOn)
{
	String tmpStr;
	if (TurnOn) tmpStr = "On"; else tmpStr = "Off";	//strings for logging new state of pumps but only if there is a change.
	if (PumpNum == uPump)
	{
		if (uPumpIsOn != TurnOn)
		{
			Relay.RelaySet(PumpNum, TurnOn);
			uPumpIsOn = !uPumpIsOn;	//toggle state as is changing
			MonitorLog("UpperPump", "0", tmpStr, 0, TurnOn, "n/a", "n/a", 0, 0, "n/a");
		}
	}
	else
		if (PumpNum == lPump)
		{
			if (lPumpInOn != TurnOn)
			{
				Relay.RelaySet(PumpNum, TurnOn);
				lPumpInOn = !lPumpInOn;	//toggle state as is changing
				MonitorLog("LowerPump", "0", tmpStr, 0, TurnOn, "n/a", "n/a", 0, 0, "n/a");
			}
		}
			else
			{
				// error, unexpected pump number
				dprint(F("Error, unexpected pump number in H2Opumps::SetPump.  Pump requested was=")); dprintln(PumpNum);
				ErrorLog("Error, unexpected pump number in H2Opumps::SetPump",String(PumpNum), 2);
			}
}



//--------------------------------------------------------Set Up --------------------------------------------------

void setup()
{


	String tempString;
	String Str;
	int tempInt, tempInt1;
	float tmpFloat;
	boolean tmpBool;

	wdt_disable();	// disable the watchdog timer.  It is used to reset the system when user updates global variables that impact the setup. Search code for wdt_disable() to see where we software reset.

	// display splash screen before getting under way
	lcd.begin(16, 2);	//unclear why, but this is needed every time else setCursor(0,1) doesn't work....probably scope related.
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("PondMonitor");
	lcd.setCursor(0, 1);
	lcd.print("Ver 1.0");
	delay(SetUpDelay);	//delay 5 sec

	// set up and test LEDs
	statusLEDs.LEDinit();	//initialize and turn off

	// tell user we are testing the LEDs
	lcd.begin(16, 2);	//unclear why, but this is needed every time else setCursor(0,1) doesn't work....probably scope related.
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("testing LEDs");
	lcd.setCursor(0, 1);
	lcd.print("Grn & Red are on");
	//delay(SetUpDelay);	//delay 5 sec
	statusLEDs.SetGreenLED(true);
	//statusLEDs.SetErrLED(1);	//slowly flashing red LED
	//delay(SetUpDelay);	//delay 5 sec
	//statusLEDs.SetErrLED(2);	// red LED flashing faster
	//delay(SetUpDelay);	//delay 5 sec
	statusLEDs.SetErrLED(3);	//led on
	delay(SetUpDelay);	//delay 5 sec
	statusLEDs.ClearErrLED();	// clear error level and turn off RedLED
	statusLEDs.SetGreenLED(false);	//turn off Green LED


	//initialize the SD library	
	pinMode(53, OUTPUT);	//pin 53 = CS for SD card reader
	
	lcd.begin(16, 2);	//unclear why, but this is needed every time else setCursor(0,1) doesn't work....probably scope related.
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("starting SD ");
	lcd.setCursor(0, 1);
	
	if (!SD.begin(53))
	{
		dprintln(F("SD initialization failed!"));	// can't log because error log is on SD, so use display
		lcd.print("failed, halt");
		statusLEDs.SetErrLED(3);	//critical error, RED LED on
		while (true)
		{
			// loop forever as SD failure is fatal.
		}

	}
	else
	{
		lcd.print("SD init OK");
		delay(SetUpDelay);	//delay 5 sec
	}

	/*
	get the monitoring state of sensors from SysStat.txt and set flags accordingly
		SysStat.txt
		text1,text,-Config System-,Start up status of sensors. Can be changed through main menu. (cont)
		text2,text,-Config System-,Used to enable / disable sensor functions. (cont)
		text3,text,-Config System-,note: WLvl=water level sensor, RelB= relay board.
		Temp,U-D----------CCC,--Temp: On/Off--,U/D  Temp is On
		Flow,U-D----------CCC,--Flow: On/Off--,U/D  Flow is On
		Wlvl,U-D----------CCC,--WLvl: On/Off--,U/D  WLvl is On
		Relay,U-D----------CCC,--RelB: On/Off--,U/D  RelB is On
		Pumps,U-D---------CCCC,Pump:On/Off/Auto,U/D  Pumps@ Auto
		action,menu,---Action---,Update  Cancel
	*/
	Display.DisplaySetup(mReadOnly, mUseSD, "SysStat", 9, DisplayBuf); // get settings from SysStat.txt  Will use the Display class to retrieve the values.  User can modify these via UI
	Display.DisplayGetSetChrs(&tempString, "Temp", mget);	//get the string from the SysStat file for the line pertaining to the temp sensors
	if (tempString == "On") { TempSensorsOn = true; } else {TempSensorsOn = false;}
	Display.DisplayGetSetChrs(&tempString, "Flow", mget);	//get 
	if (tempString == "On") { FlowSensorsOn = true; } else { FlowSensorsOn = false; }
	Display.DisplayGetSetChrs(&tempString, "Wlvl", mget);
	dprint(F("tempString for Wlvl=")); dprintln(tempString);
	if (tempString == "On") { WaterLvlSensorsOn = true; } else { WaterLvlSensorsOn = false; }
	Display.DisplayGetSetChrs(&tempString, "Relay", mget);
	if (tempString == "On") { RelayBoardOn = true; } else { RelayBoardOn = false; }
	Display.DisplayGetSetChrs(&PumpMode, "Pumps", mget);	//PumpMode = On, Off, or Auto
	dprint(F("WaterLvlSensorsOn=")); dprintln(WaterLvlSensorsOn);

	// look up variables for the sensors that are on
	if (TempSensorsOn)
	{

		//temp sensors are to be used, so set them up!
		lcd.begin(16, 2);
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("tmp sens chk ");
		lcd.setCursor(0, 1);

		sensors.begin();								// Start up the temp sensor library
		tempInt1 = sensors.getDeviceCount();			// get count of devices on the wire
		if (tempInt1 != 2)
		{
			//error, expecting 2 temperature sensors, one for water and one for temp inside monitor enclosure.  Log the error.
			ErrorLog("In setup, found less than 2 temp sensors", " ",1);	// make error log entry
			lcd.print("error tmp sens");
			delay(SetUpDelay);
		}
		else
		{
			lcd.print("tmp sens ok");
			delay(SetUpDelay);
		}



		dprint("Found "); dprint(tempInt1);	dprintln(" temp sensors.");	//debug

		TempSens0.TempSensorInit(0);		// initialize device, get address, set precision.  Address comes from device.  Precision is #define; IsOn initially set to false, but read from TSens0and TSens1 and set below.
		TempSens1.TempSensorInit(1);		// initialize device, get address, set precision

		/* Set the polling interval for these temp sensors.  The polling interval is stored on SD in TempRate.txt
			TempRate.txt
			text1,text,--Sample Rate--,Set sample rate for both temp sensors
			rate,U-D---------###-,--Sample Rate--,U/D   Every 060s
			action,menu,---Action---,Update  Cancel
		*/
		Display.DisplaySetup(mReadOnly, mUseSD, "TempRate", 3, DisplayBuf);	//get the display array into the buffer
		Display.DisplayGetSetNum(&tempString, "rate", mget);				// read the polling rate
		tempInt = tempString.toInt() * 1000;								//convert to integer polling rate and convert to ms.
		TempSens0.SetPollInterval(tempInt);									//sets the polling interval and begins polling
		TempSens1.SetPollInterval(tempInt);

		// this sensor uses the polling set by TempSens.  TempSens uses a soft interupt which calls 'SensorPollRedirect', which in turn calls the class specific ReadTempSensor for each instance (tempSens 0 and 1).
		/*	load data from the SD card.  Data stored on SD card using Display class, instances Tsens0 & 1.  Some of the Tsens (device) objects need to be written into Tsens0 Display on SD, and some need to be read from SD.
		The Display will be written back to SD at the end of initialization to save the sate of all parameters.  e.g. sensor address comes from the sensor
		and needs to be saved. */
		Display.DisplaySetup(mReadWrite, mUseSD, "TSens0", 8, DisplayBuf); // get info from SD card related to temp sensor 0 and use it to populate class variables or read from class variables

		/*	TSens0 has the following 8 lines
		Text1, text, -- - Tmp Sensor 0-- - , Parameter set up for temperature sensor 0
		tempThreshH, U - D--###------, --H Temp Thresh-, U / D  090 deg F
		tempThreshL, U - D--###------, --L Temp Thresh-, U / D  036 deg F
		tempAddrLt, U - D----CCCCCCCC - , ----Addr_Lt----, U / D    28FFA4F9
		tempAddrRt, U - D----CCCCCCCC - , ----Addr_Rt----, U / D    541400B4
		tempName, U - D--CCCCCCCCCC, ----Name Str----, U / D  Pond Temp
		handle, U - D--CCCCCCCCCC, -- - Cloud Str-- - , U / D ???????????
		action,menu,---Action---,Update   Cancel
		*/
		//if here, temp sensors are on based on global variable.  However, we have 2 temp sensors, and it is possible to have one ontemperature sensors initialized to 'off' in TempSens.TempSensorInit.   initial on/off status is set above in initialization.  Therefore, the IsOn value must be written into "TSens0"
		//if (TempSens0.IsOn) tempString = "Y"; else tempString = "N";
		//Display.DisplayGetSetChrs(&tempString, "IsOn", mset); // write the status of TempSensorsOn into the line on Display named "IsOn"

		// read temperature alarm thresholds from that is saved in TSens0.txt into appropriate Tsens class variable.
		Display.DisplayGetSetNum(&tempString, "tempThreshH", mget);	//get the value as string
		TempSens0.AlarmThreshHigh = tempString.toFloat();
		Display.DisplayGetSetNum(&tempString, "tempThreshL", mget);	//get the value as string
		TempSens0.AlarmThreshLow = tempString.toFloat();

		/* populate the sensor address in the display line.  This comes from the sensor during initialization and therefore must be written into the Display (TSens0.txt),
		which is being used for both storage and display.  The Address is too long to fit on the LCD so it is broken up into 2 segments.
		Read the lt 4 Hex digits into a temp string */
		tempString = "";	// blank out string
		for (uint8_t i = 0; i < 4; i++)
		{
			if (TempSens0.Saddr[i] < 16) tempString += '0';
			tempString += String(TempSens0.Saddr[i], HEX);
		}
		dprint(F("lt half of address=")); dprintln(tempString);
		Display.DisplayGetSetChrs(&tempString, "tempAddrLt", true);	// add lt part of address into display line
		// Read the rt  4 Hex digits into a temp string
		tempString = "";	// blank out string
		for (uint8_t i = 4; i < 8; i++)
		{
			if (TempSens0.Saddr[i] < 16) tempString += '0';
			tempString += String(TempSens0.Saddr[i], HEX);
		}
		dprint(F("Rt half of address=")); dprintln(tempString);
		Display.DisplayGetSetChrs(&tempString, "tempAddrRt", true);	// add rt part of address into display line

		Display.DisplayGetSetChrs(&TempSens0.SensName, "tempName", false); //read the sensor name from the display into the class
		Display.DisplayGetSetChrs(&TempSens0.SensHandle, "handle", false); //read the sensor handle from the display into the class

		Display.DisplayWriteSD();	// write the display containing TSens1 back to SD card.

		//-----------------------------------
		// set up the 2nd temperature sensor the exact same way as above
		Display.DisplaySetup(false, true, "TSens1", 8, DisplayBuf); // get info from SD card related to temp sensor 0 and use it to populate class variables or read from class variables

		//temperature sensor initial on/off status is set above in initialization.  Therefore, the IsOn value must be written into "TSens1"
		//if (TempSens1.IsOn) tempString = "Y"; else tempString = "N";
		//Display.DisplayGetSetChrs(&tempString, "IsOn", true); // write the status of TempSensorsOn into the line on Display named "IsOn"

		// read temperature alarm thresholds from that is saved in TSens1.txt into appropriate Tsens class variable.
		Display.DisplayGetSetNum(&tempString, "tempThreshH", mget);	//get the value as string
		TempSens1.AlarmThreshHigh = tempString.toFloat();
		Display.DisplayGetSetNum(&tempString, "tempThreshL", mget);	//get the value as string
		TempSens1.AlarmThreshLow = tempString.toFloat();

		/* populate the sensor address in the display line.  This comes from the sensor during initialization and therefore must be written into the Display (TSens1.txt),
		which is being used for both storage and display.  The Address is too long to fit on the LCD so it is broken up into 2 segments.
		Read the lt 4 Hex digits into a temp string */
		tempString = "";	// blank out string
		for (uint8_t i = 0; i < 4; i++)
		{
			if (TempSens1.Saddr[i] < 16) tempString += '0';
			tempString += String(TempSens1.Saddr[i], HEX);
		}
		Display.DisplayGetSetChrs(&tempString, "tempAddrLt", mset);	// add lt part of address into display line
																	// Read the rt  4 Hex digits into a temp string
		tempString = "";	// blank out string
		for (uint8_t i = 4; i < 8; i++)
		{
			if (TempSens1.Saddr[i] < 16) tempString += '0';
			tempString += String(TempSens1.Saddr[i], HEX);
		}
		Display.DisplayGetSetChrs(&tempString, "tempAddrRt", mset);	// add rt part of address into display line

		Display.DisplayGetSetChrs(&TempSens1.SensName, "tempName", false); //read the sensor name from the display into the class
		Display.DisplayGetSetChrs(&TempSens1.SensHandle, "handle", false); //read the sensor handle from the display into the class

		Display.DisplayWriteSD();	// write the display containing TSens1 back to SD card.
	}

	//--------------------------------------------------------------
	if (FlowSensorsOn)
	{
		/*
		Set up for flow sensors 1 & 2.  Read the flow sensor names, warning thresholds, and sample rate off the SD card from FlowEdit.txt and
		set the variables in the FlowSens object.  After that, set up the hardware for sampling flow rates, and enable readings.

			FlowEdit.txt
			Text1,text,-Flow Edit-,Used to set flow sensor parameters
			FlowName1,U-D--CCCCCCCCCC,-Flow1 Name str-,U/D  Upper Pump
			FlowWarn1,U-D--###------,-Low Flow1 Lvl @,U/D  ### l/min
			FlowName2,U-D--CCCCCCCCCC,-Flow2 Name str-,U/D  Lower Pump
			FlowWarn2,U-D--###------,-Low Flow2 Lvl @,U/D  ### l/min
			rate,U-D---------###-,--Sample Rate--,U/D   Every 300s
			action,menu1,---Action---,Save_Changes   Cancel
		*/
		Display.DisplaySetup(mReadOnly, mUseSD, "FlowEdit", 8, DisplayBuf);	//get display array from SD card into DisplayBuf
		Display.DisplayGetSetChrs(&FlowSens.Flow1Name, "FlowName1", mget);	//get name of flow sensor 1 and save in object
		Display.DisplayGetSetChrs(&FlowSens.Flow2Name, "FlowName2", mget);	//get name of flow sensor 2
		Display.DisplayGetSetChrs(&tempString, "FlowWarn1", mget);	//get low flow warning threshold for sensor 1
		FlowSens.Flow1Warn = tempString.toInt();
		Display.DisplayGetSetChrs(&tempString, "FlowWarn2", mget);	//get low flow warning for sensor 2
		FlowSens.Flow2Warn = tempString.toInt();
		Display.DisplayGetSetNum(&tempString, "rate", mget);		//get the sampling rate in sec
		FlowSens.SetReadFlowInterval(tempString.toInt() * 1000);	//save the sampling rate in ms

		FlowSens.FlowCalcSetup();		// set up the pins for reading
		FlowSens.FlowStartStop(true);	// if global setting for turning on flow sensors is on, then enable soft interupts to measure flow.  Note, global settings loaded via SysStat.txt 	
	}
	//--------------------------------------------------------------

	if (WaterLvlSensorsOn)
	{
		/*
		water level sensors are enabled so need to initialize and read variables in from SD card.

			H2OEdit.txt has state of variables 
			{snip}
			H2OLvlNo,U-D--####------,--No H2O--,U/D  #### =none
			H2OLvlLow,U-D--####------,--Mid H2O--,U/D  #### = Mid
			H2OLvlMid,U-D--####-------,--High H2O--,U/D  #### = High
			rate,U-D---------###-,--Sample Rate--,U/D   Every 300s
			action,menu,---Action---,Update   Cancel
		*/

		WaterSens.WaterLvlSensorInit();		//sets the pins up 
		Display.DisplaySetup(mReadOnly, mUseSD, "H2OEdit", 9, DisplayBuf);	//get variables from SD into display array

		Display.DisplayGetSetNum(&tempString, "H2OLvlNo", mget);
		WaterSens.WaterLvlNo = tempString.toInt();

		Display.DisplayGetSetNum(&tempString, "H2OLvlLow", mget);
		WaterSens.WaterLvlLow = tempString.toInt();

		Display.DisplayGetSetNum(&tempString, "H2OLvlMid", mget);
		WaterSens.WaterLvlMid = tempString.toInt();

		Display.DisplayGetSetNum(&tempString, "rate", mget);
		WaterSens.SetPollInterval(tempString.toInt() * 1000);	// convert sampling rate from sec to ms

		// water level sensor is in use so let's test it....show user that it is working

		lcd.begin(16, 2);
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("H2O Sens chk");
		lcd.setCursor(0, 1);
		lcd.print("Should flash twice");

		for (tempInt = 2; tempInt < 6; tempInt++)
		{
			tmpBool = tempInt % 2;						// value will be either 0 or 1
			digitalWrite(WaterLvlPowerPin, tmpBool);	// toggle power LED
			delay(SetUpDelay);
		}
		digitalWrite(WaterLvlPowerPin, LOW);			// make sure power is off
		WaterSens.TurnOn(true);	//global settings has water sensor on, so begin polling

	}
	//--------------------------------------------------------------

	if (RelayBoardOn)
	{
		// relay board is in use so let's test it....show user that it is working
		lcd.begin(16, 2);
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("Relay board chk");
		lcd.setCursor(0, 1);
		lcd.print("all should flash twice");

		for (tempInt = 1; tempInt < 5; tempInt++)
		{
			tmpBool = tempInt % 2;						// value will be either 0 or 1
			Relay.InitRelayBoard(tmpBool, tmpBool, tmpBool, tmpBool);	//initialize all as normally open, so should all turn on
			delay(SetUpDelay);
		}
		Relay.InitRelayBoard(false, false, false, false);	//initialize all as normally closed.

		pump.pumpInit();	//set state variables for pump on/off
		if (PumpMode =="Auto")
		{
			// in auto state, need to set value for prior level.  This will guarentee that pumps are turned on when water level sensor is read for first time during monitoring
			WaterSens.PriorLvlRange = "none";
		}
		else
		{
			// if here, then need to set the pumps according to the value in PumpMode.  
			//Auto is handled by the water level sensors.
			lcd.begin(16, 2);
			lcd.clear();
			lcd.setCursor(0, 0);
			if(PumpMode=="On")
			{
				pump.SetPump(uPump, pumpOn);	//trun on pump.  uses relay board.
				pump.SetPump(lPump, pumpOn);
				lcd.print("Pumps turned on");	// tell user
			}
			else
			{
				pump.SetPump(uPump, pumpOff);	//trun off pump.  uses relay board.
				pump.SetPump(lPump, pumpOff);
				lcd.print("Pumps turned off");	// tell user
			}

			lcd.setCursor(0, 1);
			lcd.print("based on 'SetUp'");
			delay(SetUpDelay);
		}
	}
	//--------------------------------------------------------------
// all sensors are set up and we are ready to start polling for keyboard use and sensor readings
	KeyPoll(true);		// Begin polling the keypad S
	SysTimePoll(true);	// begin to poll the Real Time Clock to get system time into SysTm
	statusLEDs.ClearErrLED();		//turn off the red LED as it was tested above.
	statusLEDs.SetGreenLED(true);	//turn on the green LED because we are monitoring

	Display.DisplayStartStop(true);		// indicate that menu processing will occur. Tells main loop to pass key presses to the Menu
	Display.DisplaySetup(mReadWrite, mUseSD, "Main_UI", 1, DisplayBuf); // Prepare main-UI display array and display the first line, mode is read-write.
//--------------------------------------------------------------------------------------------------------------------------------------------------
}


//-----------------------------------------------------------------main loop -----------------------------------------------------------------------

void loop()
{
	Tmr.update();		//timer object used by keyboard and displays
	SensTmr.update();	//timer object used for sensor reading intervals

	//Begin--------------------------------------------------------Processing User Selection -------------------------------------------------------
	if (ReadKey() != NO_KEY)
	{
		// key was pressed and debounced result is ready for processing
		Display.ProcessDisplay(LS_curKey);	// routine will process key only if DisplayInUse==true, global set by DisplayStartStop()	
	}//if (ReadKey() != NO_KEY)

	if (Display.DisplayUserMadeSelection == true)
	{
		String tempString;
		
		//If here, then user has made a Display selection, passing results through DisplayName, DisplayLineName, DisplaySelection
		if (Display.DisplayName == "Main_UI")
		{

			/*if here then processing Main_UI.txt display array
			SetUp,menu,--Home Screen--,Status   Temp_sensor  Flow_sensor   H20_Lvl_sensor  Pumps  RTC
			*/

			if (Display.DisplayLineName == "SetUp")
			{
				if (Display.DisplaySelection == "Status")
				{
					dprintln(F("Main_UI-->Setup-->Status"));
					Display.DisplaySetup(mReadWrite,mUseSD,"SysStat",9,DisplayBuf);	// put up the display array with the global variables
				}
				else
		
				if (Display.DisplaySelection == "Temp_sensor")
				{
					if (TempSensorsOn)
					{
						//global for temperature sensor (set is SysStat.txt) is on, so can proceed, else error
						dprintln(F("Main_UI-->SetUp-->Temp_sensor"));	//debug
						Display.DisplaySetup(false, true, "tempsens", 2, DisplayBuf); // put up entry screen for temperature sensor display array and display the first line
					}
					else
					{
						Display.DisplaySetup(mReadOnly, mUseSD, "TestErr", 3, DisplayBuf);	//put up error message telling user to turn on sensor first
					}									
				}
				else
				
				if (Display.DisplaySelection == "Flow_sensor")
				{
					if (FlowSensorsOn)
					{
						dprintln(F("Main_UI-->SetUp-->Flow_sensor"));	//debug
						Display.DisplaySetup(mReadOnly, mUseSD, "FlowSens", 2, DisplayBuf);	//put up the display for user to make selections regarding flow sensors
					}
					else
					{
						Display.DisplaySetup(mReadOnly, mUseSD, "TestErr", 3, DisplayBuf);	//put up error message telling user to turn on sensor first
					}

				}
				else

				if (Display.DisplaySelection == "H20_Lvl_sensor")
				{
					if (WaterLvlSensorsOn)
					{
						dprintln(F("Main_UI-->Setup-->H20_Lvl_sensor"));
						Display.DisplaySetup(mReadWrite, mUseSD, "H2OLvl", 4, DisplayBuf);	//put up display array for water level sensor
					}
					else
					{
						Display.DisplaySetup(mReadOnly, mUseSD, "TestErr", 3, DisplayBuf);	//put up error message telling user to turn on sensor first
					}

				}
				else

				if (Display.DisplaySelection == "RTC")
				{
					boolean	rslt;
					dprintln(F("Main_UI-->Setup-->RTC"));
					Display.DisplaySetup(mReadWrite, mUseSD, "SetRTC", 5, DisplayBuf); // Prepare SetRTC display array, display the first line, mode is read/write, retrieve SetRTC from the SD card

					//RTC is read every second and sets strings for day of week, time, and date
					//modify the display lines in the SetRTC array

					rslt = Display.DisplayGetSetDate(&SysDateStr, "Date", true);	// replace the date string in display line named 'Date' in the SetRTC_up array
					rslt = Display.DisplayGetSetTime(&SysTmStr, "Time", true);		// replace the time string in display line named 'Time'.  need to clip off sec
					rslt = Display.DisplayGetSetDOW(&sysDOWstr, "DOW", true);		// replace the day of week string in display line named 'DOW'
					dprintln(F("main loop, clicked RCT")); for (int z=0; z<5; z++) {dprintln(DisplayBuf[z]);}	//debug

				}
				else

				if(Display.DisplaySelection == "Pumps")
				{
					Display.DisplaySetup(mReadWrite, mUseSD, "Pumps", 3, DisplayBuf);	//put up pump control display
				}
				else

				{
					//error, should have identified the DisplaySelection
					ErrorLog("error processing Main_UI, Setup: did not match DisplaySection", Display.DisplaySelection, 2);
					dprint(F("error processing Main_UI-->Setup: unrecognized DisplaySelection=")); dprintln(Display.DisplaySelection);	//debug
				}

				goto EndDisplayProcessing; //exit processing Display	
			}

		}		

		if (Display.DisplayName == "Pumps")
		{
			/*			
			Pumps.txt
			text1,text,--Pond Pumps--,Functions related to the water pumps, On/Off are static (cont)
			text2,text,--Pond Pumps--,Auto controls pumps by water level sensor.
			Pumps,menu,--Set Pump Pwr--,Auto   Off   On  Cancel
			*/

			if (Display.DisplayLineName == "action")
			{
				if (Display.DisplaySelection == "Auto") 
				{
					dprintln(F("Pumps-->Pumps-->Auto"));	//debug
					if (WaterLvlSensorsOn)
					{
						PumpMode = "Auto";	//set the pump mode to Auto.  Water level sensor logic will take care of turning pumps on and off
						Display.DisplaySetup(mReadWrite, mUseSD, "Main_UI", 1, DisplayBuf);	// return to main menu
					}
					else
					{
						//error, user is trying to set pumps to 'Auto' but water level sensor is not on
						Display.DisplaySetup(mReadWrite, mUseSD, "PumpErr", 3, DisplayBuf);	// put up error message
					}
				}
				else
				if(Display.DisplaySelection == "Off")
				{
					dprintln(F("Pumps-->Pumps-->Off"));	//debug
					pump.SetPump(uPump, pumpOff);
					pump.SetPump(lPump, pumpOff);
					Display.DisplaySetup(mReadWrite, mUseSD, "Main_UI", 1, DisplayBuf); // Return to main-UI display array and display the first line
				}
				else
				if (Display.DisplaySelection == "On")
				{
					dprintln(F("Pumps-->Pumps-->On"));	//debug
					pump.SetPump(uPump, pumpOn);
					pump.SetPump(lPump, pumpOn);
					Display.DisplaySetup(mReadWrite, mUseSD, "Main_UI", 1, DisplayBuf); // Return to main-UI display array and display the first line
				}
				else
				if(Display.DisplaySelection=="Cancel")
				{
					Display.DisplaySetup(mReadWrite, mUseSD, "Main_UI", 1, DisplayBuf); // Return to main-UI display array and display the first line
				}
				else
				{
					ErrorLog("error processing Pumps-->action: unrecognized DisplaySelection",Display.DisplaySelection,2);
					dprint(F("error processing Pumps-->action: unrecognized DisplaySelection=")); dprintln(Display.DisplaySelection);	
				}
			}
			goto EndDisplayProcessing; //exit processing pumps
		}

		if (Display.DisplayName == "PumpErr")
		{
			/*
			PumpErr.txt
			Text1,text,--Error--,Auto uses the water level sensor and it is turned off globally. --cont--
			Text2,text,--Error--,Turn it on before setting pumps to 'Auto' (Main --> Pumps).
			action,menu,Continue
			*/

			if (Display.DisplayLineName == "action")
			{
				if (Display.DisplaySelection == "Continue")
				{
					dprintln(F("PumpErr-->Continue"));
					Display.DisplaySetup(mReadWrite, mUseSD, "Main_UI", 1, DisplayBuf);	//Return to main-UI display array and display the first line
				}
				else
				{
					ErrorLog("error processing PumpErr-->action: unrecognized DisplaySelection",Display.DisplaySelection, 2);
					dprint(F("error processing PumpErr-->action: unrecognized DisplaySelection=")); dprintln(Display.DisplaySelection);
				}
			}
			goto EndDisplayProcessing;
		}

		if (Display.DisplayName == "SetRTC")
		{

			//if here then processing SetRTC.txt
			//{"Text,text,---RTC Setup---,Used to view/update the date, time, and day of week settings",
			//"Date,m--d----yy--U-D,---RTC Date---,08/01/2015  U/D",
			//"Time,H--M---U-D,---RTC Time---,13:27  U/D",
			//"DOW,-a---a---U-D,--RTC DOW--,-Mon-Tue-U/D",
			//"action,menu,---Action---,Update   Cancel"}

			dprintln(Display.DisplayLineName);
			if (Display.DisplayLineName == "action")
			{
				if (Display.DisplaySelection == "Update")
				{
					dprintln(F("RTC_ui-->action-->update"));
					//user wants to update system time with their changes made via display array 'RTC_ui'. Note that display 'RTC_ui' insures valid date, time, and DOW.  However, there is no check if DOW goes with date.
					// will set the RTC and the soft interrupt will set the system time strings with the next read of RTC 
					dprint(F("Free Scram =")); dprintln(getFreeSram());


					String TmpSysTimeStr, TmpSysDateStr, tmpTime, tmpStr;
					int	tmpVal;
					//dprint("SysDateStr=" ); dprint(SysDateStr); dprint(", sysTimeStr="); dprint(SysTmStr); dprint("' SysDOW="); dprintln(sysDOWstr);
					//dprintln("SysDateStr=" + SysDateStr + ", sysTimeStr=" + SysTmStr + "' SysDOW=" + sysDOWstr);
					Display.DisplayGetSetDate(&TmpSysDateStr, "Date", false);					// read the date string from display line named 'Date' in the SetRTC_up array
																								//dprintln("new SysDateStr=" + TmpSysDateStr);
					tmpStr = TmpSysDateStr.substring(0, 2);		//get 2 digit month
																//dprintln("month from RTC_ui="+tmpStr);
					SysTm.Month = tmpStr.toInt();
					//dprint("month int="); dprintln(tmpVal);
					tmpStr = TmpSysDateStr.substring(3, 5);	//get 2 digit day
															//dprintln("day from RTC_ui="+tmpStr);
					SysTm.Day = tmpStr.toInt();
					//dprint("Day int="); dprintln(tmpStr.toInt());				
					tmpStr = TmpSysDateStr.substring(6);	//get 4 digit yr
															//dprintln("year from RTC_ui="+tmpStr);
					SysTm.Year = tmpStr.toInt() - 1970;		//year has offset from 1970	
															//dprint("year int="); dprintln(SysTm.Year);	
															//dprint("Free Scram after date ="); dprintln(getFreeSram());

					Display.DisplayGetSetTime(&SysTmStr, "Time", false);					// read the time string from display line named 'Time'. 
																							//dprintln("new time string=" +SysTmStr);
					tmpStr = SysTmStr.substring(0, 2);										// get	2 digit Hr
																							//dprintln("hrs from RTC_ui=" + tmpStr);
					SysTm.Hour = tmpStr.toInt();
					tmpStr = SysTmStr.substring(3, 5);										// get 2 digit min
																							//dprintln("min from RTC_ui=" + tmpStr);
					SysTm.Minute = tmpStr.toInt();
					SysTm.Second = 0;
					//dprint("Free Scram after time ="); dprintln(getFreeSram());				
					// no seconds in RTC_ui						
					Display.DisplayGetSetDOW(&sysDOWstr, "DOW", false);						// read the day of week string from display line named 'DOW'

					for (tmpVal = 0; tmpVal<7; tmpVal++)
					{
						tmpStr = Display.ProgMemLU(DisplayDOW, tmpVal, 3);	//get the DOW string from storage
						if (tmpStr == sysDOWstr) break;
					}
					//tmpVal++;		// increment because Sunday=1 and index for sunday=0
					//dprint	("RTC_ui DOW index="); dprintln(tmpVal);
					SysTm.Wday = tmpVal;
					//dprint("Free Scram after DOW ="); dprintln(getFreeSram());


					if (!RTC.write(SysTm))		//  write time to RTC, false if fails
					{
						ErrorLog("RTC write failed"," ",1);
					}
					//dprint("Free Scram after set time ="); dprintln(getFreeSram());

				}
				else
				{
					if (Display.DisplaySelection == "Cancel")
					{
						dprintln(F("RTC_ui-->action-->Cancel"));
						Display.DisplaySetup(true, true, "Main_UI", 1, DisplayBuf); // user wants to cancel, so return to main-UI display array and display the first line, mode is read only.
					}
					else
					{
						ErrorLog("error processing SetRTC-->action: unrecognized DisplaySelection",Display.DisplaySelection,2);
						dprint(F("error processing SetRTC-->action: unrecognized DisplaySelection=")); dprintln(Display.DisplaySelection);	//debug
					}

				}

			}
			else
			{
				ErrorLog("error processing RTC_ui: unrecognized DisplayLineName", Display.DisplayLineName,2);
				dprint(F("error processing SetRTC-->action: unrecognized DisplayLineName=")); dprintln(Display.DisplayLineName);	//debug
				dprint((F("length="))); dprintln(Display.DisplayLineName.length());

			}
			Display.DisplaySetup(false, true, "Main_UI", 1, DisplayBuf); // Return to main-UI display array and display the first line
			goto EndDisplayProcessing; //exit processing Display	
		}

		if (Display.DisplayName == "tempsens")
		{
			/*  
			Tempsens.txt
			Text1,text,---Tmp Sensor---,Functions related to temperature sensors
			action1,menu,---Edit/Test---,Sample_Rate  Edit_0   Edit_1   Test_0&1  Cancel
			*/

			if (Display.DisplayLineName == "action1")
			{
				if (Display.DisplaySelection == "Sample_Rate")
				{
					dprint(F("TempSens-->Action1-->Sample_Rate"));	//debug
					Display.DisplaySetup(mReadWrite, mUseSD, "TempRate", 3, DisplayBuf);	//put up display array so we can set the sampling rate for the temp sensors
				}
				else
				if (Display.DisplaySelection == "Edit_0")
				{

					dprintln(F("TempSens-->Action-->Edit_0"));	//debug
					Display.DisplaySetup(mReadWrite, mUseSD, "TSens0", 8, DisplayBuf); // Put up first temp sens setup display array and display the first line
				}
				else
				if (Display.DisplaySelection == "Edit_1")
				{
					// see comments for Edit_0
					dprintln(F("TempSens-->Action-->Edit_1"));	//debug
					Display.DisplaySetup(mReadWrite, mUseSD, "TSens1", 8, DisplayBuf); // Put up 2nd temp sens setup display array and display the first line
				}
				else
				if (Display.DisplaySelection == "Test_0&1")
				{
					dprintln(F("TempSens-->Action-->Test_0&1"));	//debug
					Display.DisplaySetup(mReadWrite, mUseSD, "TempTst", 5, DisplayBuf); // Put up test temp sens 0 display array and display the first line
				}
				else
				if (Display.DisplaySelection == "Cancel")
				{
					dprint(F("TempSens-->Action1-->Cancel"));	//debug					
					Display.DisplaySetup(mReadWrite, mUseSD, "Main_UI", 1, DisplayBuf); // Return to main-UI display array and display the first line
				}
				else
				{
					ErrorLog("error processing TempSens-->action1: unrecognized DisplaySelection",Display.DisplaySelection, 2);
					dprint(F("error processing TempSens-->action1: unrecognized DisplaySelection=")); dprintln(Display.DisplaySelection);
				}
			}
			goto EndDisplayProcessing; //exit processing Display
		}

		if (Display.DisplayName == "TempRate")
		{
			/*
			if here, then processing user interaction with display array used to set the sampling rate for both temp sensors.
				TempRate.txt
				text1,text,--Sample Rate--,Set sample rate for both temp sensors
				rate,U-D---------###-,--Sample Rate--,U/D   Every 060s  
				action,menu,---Action---,Update  Cancel
			*/
			int tempInt;
			if (Display.DisplayLineName == "action")
			{
				if (Display.DisplaySelection == "Update")
				{
					dprintln(F("TempRate-->Action-->Update"));	//debug
					//read the rate from the display array, set the rate, and save the display array back to SD
					Display.DisplayGetSetNum(&tempString, "rate",mget);	//get the string of the sampling rate
					tempInt = tempString.toInt() * 1000;				//convert to integer polling rate and convert to ms.
					TempSens0.SetPollInterval(tempInt);					// set the polling interval for both temp sensors.
					TempSens1.SetPollInterval(tempInt);
					Display.DisplayWriteSD();							// save the display array on SD
					Display.DisplaySetup(mReadWrite, mUseSD, "tempsens", 4, DisplayBuf); // return to the entry screen for temperature sensor display array and display the first line
				}
				else
					if (Display.DisplaySelection == "Cancel")
					{
						dprintln(F("TempRate-->Action-->Cancel"));	//debug					
						Display.DisplaySetup(mReadWrite,mUseSD, "tempsens", 4, DisplayBuf); // return to the entry screen for temperature sensor display array and display the first line
					}
					else
					{
						ErrorLog("error processing TempRate-->action: unrecognized DisplaySelection",Display.DisplaySelection, 2);
						dprint(F("error processing TempRate-->action: unrecognized DisplaySelection=")); dprintln(Display.DisplaySelection);
					}
			}
			goto EndDisplayProcessing; //exit processing Display
		}

		if (Display.DisplayName == "TSens0")
		{
			/*if here then processing TSens0
			Text1, text, -- - Tmp Sensor 0-- - , Parameter set up for temperature sensor 0
			tempThreshH, U - D--###------, --H Temp Thresh-, U / D  090 deg F
			tempThreshL, U - D--###------, --L Temp Thresh-, U / D  036 deg F
			tempAddrLt, U - D----CCCCCCCC - , ----Addr_Lt----, U / D    28FFA4F9
			tempAddrRt, U - D----CCCCCCCC - , ----Addr_Rt----, U / D    541400B4
			tempName, U - D--CCCCCCCCCC, ----Name Str----, U / D  Pond Temp
			handle, U - D--CCCCCCCCCC, -- - Cloud Str-- - , U / D ???????????
			action,menu,---Action---,Update   Cancel
			*/
			if (Display.DisplayLineName == "action")
			{

				if (Display.DisplaySelection == "Update")
				{
					dprintln(F("TSens0-->action-->Update"));	//debug 
					/*
					User asking to update parameters, will assume that they changed some/all.  However, even though they can change the address in Temp0 display array,
					they cannot change the actual address of the device, because that comes from the device.  Soooooo, we assume they changed it but
					change it back :-)
					What we need to do is read parameters that the user can change from Temp0 display array into the object, read the device address from the object
					into Temp0 display array, and write Temp0 display array back to SD as storage.
					*/
					// read temperature alarm thresholds 
					Display.DisplayGetSetNum(&tempString, "tempThreshH", mget);	//get the value as string
					TempSens0.AlarmThreshHigh = tempString.toFloat();
					Display.DisplayGetSetNum(&tempString, "tempThreshL", mget);	//get the value as string
					TempSens0.AlarmThreshLow = tempString.toFloat();

					/*populate the sensor address in the display line.  This comes from the sensor during initialization and the user is not allowed to change it.
					The Address is too long to fit on the LCD so it is broken up into 2 segments.
					Read the lt 4 Hex digits into a temp string */
					tempString = "";	// blank out string
					for (uint8_t i = 0; i < 4; i++)
					{
						if (TempSens0.Saddr[i] < 16) tempString += '0';
						tempString += String(TempSens0.Saddr[i], HEX);
					}
					Display.DisplayGetSetChrs(&tempString, "tempAddrLt", mset);	// add lt part of address into display line
	
					// Read the rt  4 Hex digits into a temp string
					tempString = "";	// blank out string
					for (uint8_t i = 4; i < 8; i++)
					{
						if (TempSens0.Saddr[i] < 16) tempString += '0';
						tempString += String(TempSens0.Saddr[i], HEX);
					}
					Display.DisplayGetSetChrs(&tempString, "tempAddrRt", mset);	// add rt part of address into display line

					Display.DisplayGetSetChrs(&TempSens0.SensName, "tempName", mget); //read the sensor name from the display into the class
					Display.DisplayGetSetChrs(&TempSens0.SensHandle, "handle", mget); //read the sensor handle from the display into the class

					Display.DisplayWriteSD();	// write the display containing TSens0 back to SD card.
					Display.DisplaySetup(mReadWrite, mUseSD, "tempsens", 4, DisplayBuf); // return to the entry screen for temperature sensor display array and display the first line
				}
				else
				if(Display.DisplaySelection=="Cancel")
				{
					Display.DisplaySetup(mReadWrite, mUseSD, "tempsens", 4, DisplayBuf); // return to the entry screen for temperature sensor display array and display the first line
				}
				else
				{
					ErrorLog("error processing TSens0-->action: unrecognized DisplaySelection",Display.DisplaySelection,2);
					dprint(F("error processing TSens0-->action: unrecognized DisplaySelection=")); dprintln(Display.DisplaySelection);
	
				}
			}
			goto EndDisplayProcessing; //exit processing Display
		}	// end processing DisplayName== "TSens0"

		if (Display.DisplayName == "TSens1")
		{
			/*
			TSens1.txt
			Text1,text,---Tmp Sensor 1---,Parameter set up for temperature sensor 1
			tempThreshH,U-D--###------,--Alrm Thresh--,U/D  110 deg F
			tempThreshL,U-D--###------,--Alrm Thresh--,U/D  032 deg F
			tempAddrLt,U-D----CCCCCCCC-,----Addr_Lt----,U/D    ######## 
			tempAddrRt,U-D----CCCCCCCC-,----Addr_Rt----,U/D    ######## 
			tempName,U-D--CCCCCCCCCC,----Name Str----,U/D  Device Tmp  
			handle,U-D--CCCCCCCCCC,---Cloud Str---,U/D  ???????????
			action,menu,---Action---,Update   Cancel
			*/
			if (Display.DisplayLineName == "action")
			{

				if (Display.DisplaySelection == "Update")
				{
					dprintln(F("TSens1-->action-->Update"));	//debug 
					/*
					User asking to update parameters, will assume that they changed some/all.  However, even though they can change the address in Temp0 display array,
					they cannot change the actual address of the device, because that comes from the device.  Soooooo, we assume they changed it but
					change it back :-)
					What we need to do is read parameters that the user can change from Temp0 display array into the object, read the device address from the object
					into Temp0 display array, and write Temp0 display array back to SD as storage.
					*/
					// read temperature alarm thresholds 
					Display.DisplayGetSetNum(&tempString, "tempThreshH", mget);	//get the value as string
					TempSens1.AlarmThreshHigh = tempString.toFloat();
					Display.DisplayGetSetNum(&tempString, "tempThreshL", mget);	//get the value as string
					TempSens1.AlarmThreshLow = tempString.toFloat();

					/*populate the sensor address in the display line.  This comes from the sensor during initialization and the user is not allowed to change it.
					The Address is too long to fit on the LCD so it is broken up into 2 segments.
					Read the lt 4 Hex digits into a temp string */
					tempString = "";	// blank out string
					for (uint8_t i = 0; i < 4; i++)
					{
						if (TempSens1.Saddr[i] < 16) tempString += '0';
						tempString += String(TempSens1.Saddr[i], HEX);
					}
					Display.DisplayGetSetChrs(&tempString, "tempAddrLt", mset);	// add lt part of address into display line

																				// Read the rt  4 Hex digits into a temp string
					tempString = "";	// blank out string
					for (uint8_t i = 4; i < 8; i++)
					{
						if (TempSens1.Saddr[i] < 16) tempString += '0';
						tempString += String(TempSens1.Saddr[i], HEX);
					}
					Display.DisplayGetSetChrs(&tempString, "tempAddrRt", mset);	// add rt part of address into display line

					Display.DisplayGetSetChrs(&TempSens1.SensName, "tempName", mget); //read the sensor name from the display into the class
					Display.DisplayGetSetChrs(&TempSens1.SensHandle, "handle", mget); //read the sensor handle from the display into the class

					Display.DisplayWriteSD();	// write the display containing TSens0 back to SD card.
					Display.DisplaySetup(mReadWrite, mUseSD, "tempsens", 4, DisplayBuf); // return to the entry screen for temperature sensor display array and display the first line
				}
				else
					if (Display.DisplaySelection == "Cancel")
					{
						Display.DisplaySetup(mReadWrite, mUseSD, "tempsens", 4, DisplayBuf); // return to the entry screen for temperature sensor display array and display the first line
					}
					else
					{
						ErrorLog("error processing TSens1-->action: unrecognized DisplaySelection",Display.DisplaySelection, 2);
						dprint(F("error processing TSens1-->action: unrecognized DisplaySelection=")); dprintln(Display.DisplaySelection);

					}
			}
			goto EndDisplayProcessing; //exit processing Display
		}	// end processing for DisplayName=="TSens1"

		if (Display.DisplayName == "TempTst")
		{
			/*
			TempTst.txt
			Text1, text, --Tmp Test_0 & 1--, Halts measurement and tests both temp sensors(cont)
			Text2, text, --Tmp Test_0 & 1--, Temp 0 & 1 values on separate lines.
			tempValue0, U - D--### - #----, --Temp_0 Value--, U / D  ###.# F
			tempValue1, U - D--### - #----, --Temp_1 Value--, U / D  ###.# F
			action, menu, -- - Action-- - , Begin_Test   End_Test
			*/
			if (Display.DisplayLineName == "action")
			{
				if (Display.DisplaySelection == "Begin_Test")
				{
					dprintln(F("TempTst-->action-->Begin_Test"));	//debug
					InMonitoringMode = false;	// flag to end monitoring mode....stop reading and logging sensor readings
					statusLEDs.SetGreenLED(false);
					InTempSensTestMode = true;	// flag to start temp sens testing
					TempSens0.TestMode(true);	// sets the soft interupts and state variables for both TempSens0 AND TempSens1 for testing

					//Note, soft interupt for temp sensor will check if monitoring vs testing and act accordingly.
				}
				else
				if (Display.DisplaySelection == "End_Test")
				{
					dprintln(F("TempTst-->action-->End_Test"));	//debug
					InMonitoringMode = true;		// flag to resume monitoring mode....resume reading and logging sensor readings
					statusLEDs.SetGreenLED(true);
					InTempSensTestMode = false;	// flag to end temp sens 0 testing
					TempSens0.TestMode(false);		// sets the soft interupts and state variables for both TempSens0 AND TempSens1 for monitoring
					Display.DisplaySetup(mReadWrite, mUseSD, "tempsens", 4, DisplayBuf); // return to the entry screen for temperature sensor display array and display the first line
				}
				else						
				{
					ErrorLog("error processing TempTst-->action: unrecognized DisplaySelection",Display.DisplaySelection,2);
					dprint(F("error processing TempTst-->action: unrecognized DisplaySelection=")); dprintln(Display.DisplaySelection);

				}
			}
			goto EndDisplayProcessing; //exit processing Display
		}	// end processing DisplayName== "TempTst"

		if (Display.DisplayName == "FlowSens")	
		{
			/*if here then processing FloSens
				FlowSens.txt
				Text1,text,--Flow Sensors--,Functions related to Flow sensors
				action1,menu,---Edit/Test---,Edit_setings  Test_sensors   Cancel

			*/
			if (Display.DisplayLineName == "action1")
			{
				if (Display.DisplaySelection == "Edit_settings")
				{
					dprintln(F("FlowSens-->action1-->Edit_settings"));	//debug
					Display.DisplaySetup(mReadWrite, mUseSD, "FlowEdit", 7, DisplayBuf);	//put up the display array to edit flow sensor related settings
				}
				else
					if (Display.DisplaySelection == "Test_sensors")
					{
						dprint(F("FlowSens-->action1-->FlowTest"));	//debug
						Display.DisplaySetup(mReadWrite, mUseSD, "FlowTest", 8, DisplayBuf);	//put up display array to test flow sensors.
					}
					else
						if (Display.DisplaySelection == "Cancel")
						{
							dprint(F("FlowSens-->action1-->Cancel"));	//debug
							Display.DisplaySetup(mReadWrite, mUseSD, "Main_UI", 1, DisplayBuf);	// user canceled, so return to Main_UI
						}					
						else
						{
							ErrorLog("error processing FlowSens-->action1: unrecognized DisplaySelection",Display.DisplaySelection, 2);
							dprint(F("error processing FlowSens-->action1: unrecognized DisplaySelectionn=")); dprint(Display.DisplaySelection);
						}
			}
			goto EndDisplayProcessing; //exit processing Display
		}	// end processing DisplayName== FlowSens

		if (Display.DisplayName == "FlowEdit")
		{
			/*if here then processing FlowEdit, settings for flow sensors 1 & 2.  User makes changes then on Save, we write to SD card, and update the sampling rate

				FlowEdit.txt
				Text1,text,-Flow Edit-,Used to set flow sensor parameters
				FlowName1,U-D--CCCCCCCCCC,-Flow1 Name str-,U/D  Upper Pump
				FlowWarn1,U-D--###------,-Low Flow1 Lvl @,U/D  ### l/min
				FlowName2,U-D--CCCCCCCCCC,-Flow2 Name str-,U/D  Lower Pump
				FlowWarn2,U-D--###------,-Low Flow2 Lvl @,U/D  ### l/min
				rate,U-D---------###-,--Sample Rate--,U/D   Every 300s
				action,menu,---Action---,Save_Changes   Cancel

			*/
			if (Display.DisplayLineName == "menu")
			{
				if (Display.DisplaySelection == "Save_Changes")
				{
					//Save the array on SD, get the settings from the display array and save values in object variables
					dprintln(F("FlowEdit-->menu-->Save_Changes"));	//debug
					Display.DisplayWriteSD();	// save what user may have changed
					Display.DisplayGetSetChrs(&FlowSens.Flow1Name, "FlowName1", mget);	//get name of flow sensor 1 and save in object
					Display.DisplayGetSetChrs(&FlowSens.Flow2Name, "FlowName2", mget);	//get name of flow sensor 2
					Display.DisplayGetSetChrs(&tempString, "FlowWarn1", mget);	//get low flow warning threshold for sensor 1
					FlowSens.Flow1Warn = tempString.toInt();
					Display.DisplayGetSetChrs(&tempString, "FlowWarn2", mget);	//get low flow warning for sensor 2
					FlowSens.Flow2Warn = tempString.toInt();
					Display.DisplayGetSetNum(&tempString, "rate", mget);		//get the sampling rate in sec
					FlowSens.SetReadFlowInterval(tempString.toInt() * 1000);	//save the sampling rate in ms

					Display.DisplaySetup(mReadWrite, mUseSD, "FlowSens", 2, DisplayBuf); // Return to FlowSens display array and display the first line
				}
				else
					if (Display.DisplaySelection == "Cancel")
					{
						Display.DisplaySetup(mReadWrite, mUseSD, "FlowSens", 2, DisplayBuf); // Return to FlowSens display array and display the first line
					}
					else
						{
							ErrorLog("error processing FlowEdit-->menu: unrecognized DisplaySelection",Display.DisplaySelection, 2);
							dprint(F("error processing FlowEdit-->menu: unrecognized DisplaySelection=")); dprintln(Display.DisplaySelection);
						}
			}
			goto EndDisplayProcessing; //exit processing Display

		}	// end processing DisplayName== "FlowEdit"

		if (Display.DisplayName == "FlowTest")
		{
			/*if here then processing FlowTest
			FlowTest.txt
			Text1,text,---Flow Test---,Halts measurement and tests flow sensors 1 & 2 (cont)
			Text2,text,---Flow Test---,Each flow sensor has 2 outputs, flow in l/min and waveform duration. (cont)
			Text3,text,---Flow Test---,Sampling rate=5ms so duration should be <= 10 ms (Nyquist).
			Flow1Value,U-D--###,--Flow1 Value--,U/D  ### l/min
			Flow2Value,U-D--###,--Flow2 Value--,U/D  ### l/min
			Flow1Dur,U-D--##----,--Flow1Dur--,U/D  ## ms
			Flow2Dur,U-D--##----,--Flow2Dur--,U/D  ## ms
			action,menu,---Action---,Begin_Test   End_Test
			*/
			if (Display.DisplayLineName == "Action")
			{
				if (Display.DisplaySelection == "Begin_Test")
				{
					dprintln(F("FlowTest-->action-->Begin_Test"));	//debug
					InMonitoringMode = false;	// flag to end monitoring mode....stop reading and logging sensor readings
					statusLEDs.SetGreenLED(false);
					InFlowSensTestMode = true;	// flag to tell system we are testing the flow sensors
					FlowSens.SetReadFlowInterval(FlowTestingInterval);	// sets the testing interval, e.g. make reading every 5 sec
					Display.DisplayLineRefresh("Flow1Value");	// show the display line for where the flow rate for flow sens1 will be displayed
				}
				else
					if (Display.DisplaySelection == "End_Test")
					{
						dprintln(F("FlowTest-->action-->End_Test"));	//debug
						InMonitoringMode = true;		// flag to resume monitoring mode....resume reading and logging sensor readings
						statusLEDs.SetGreenLED(true);
						InFlowSensTestMode = false;		// flag to end flow sens testing
						FlowSens.SetReadFlowInterval(FlowSens.GetReadFlowInterval());	//retrore soft interupt for when to read the flow sensors during monitoring
						Display.DisplaySetup(mReadWrite,mUseSD,"FlowSens",2,DisplayBuf); // return to the entry screen for flow  sensor display array and display the first line
					}					
					else
					{
						ErrorLog("error processing FlowTest-->action: unrecognized DisplaySelection",Display.DisplaySelection, 2);
						dprint(F("error processing FlowTest-->action:: unrecognized DisplaySelection=")); dprintln(Display.DisplaySelection);
					}
			}
			goto EndDisplayProcessing; //exit processing Display
		}	// end processing DisplayName== "FlowTest"

		if (Display.DisplayName == "TestErr")
		{
			/*
			TestErr.txt
			Text1,text,--Error--,The sensor you are trying to test is turned off by global setting. --cont--
			Text2,text,--Error--,To test, must first turn sensor on via Main_Menu -->Status.
			action,menu,Continue
			*/

			if (Display.DisplayLineName == "action")
			{
				if (Display.DisplaySelection == "Continue")
				{
					dprintln(F("TestErr-->Continue"));
					Display.DisplaySetup(mReadWrite, mUseSD, "Main_UI", 1, DisplayBuf);	//Return to main-UI display array and display the first line
				}
				else
				{
					ErrorLog("error processing TestErr-->action: unrecognized DisplaySelection",Display.DisplaySelection, 2);
					dprint(F("error processing Pumps-->action: unrecognized DisplaySelection=")); dprintln(Display.DisplaySelection);
				}
			}
			goto EndDisplayProcessing; //exit processing pumps
		}

		if (Display.DisplayName == "H2OLvl")
		{
			/*
			H2OLvl.txt
			Text1,text,--H2O Sensor--,Functions related to water level sensor. (cont)
			Text2,text,--H2O Sensor--,Detects multiple levels of water in filter and (cont)
			Text3,text,--H2O Sensor--,turns on/off upper and lower pump accordingly.
			action1,menu,---Edit/Test---,Edit_setings  Test_sensor  Cancel
			*/
			if (Display.DisplayLineName == "action1")
			{
				if (Display.DisplaySelection == "Edit_settings")
				{
					dprintln(F("H2OLvl-->action1-->Edit_settings"));	//debug
					Display.DisplaySetup(mReadWrite, mUseSD, "H2OEdit", 9, DisplayBuf);	//put up display array to edit settings for H2O sensor
				}
				else
					if (Display.DisplaySelection == "Test_sensor")
					{
						dprintln(F("H2OLvl-->action1-->Test_sensor"));	//debug
						Display.DisplaySetup(mReadWrite, mUseSD, "H2OTest", 7, DisplayBuf);	//put up display array to test the H2O sensor
					}
					else
						if (Display.DisplaySelection == "Cancel")
						{
							Display.DisplaySetup(mReadWrite, mUseSD, "Main_UI", 1, DisplayBuf); // Return to main-UI display array and display the first line
							goto EndDisplayProcessing; //exit processing Display	
						}
						else
						{
							ErrorLog("error processing H2OLvl-->action1: unrecognized DisplaySelection",Display.DisplaySelection, 2);
							dprint(F("error processing H2OLvl-->action: unrecognized DisplaySelection=")); dprintln(Display.DisplaySelection);

						}
			}
			goto EndDisplayProcessing; //exit processing Display
		}	// end processing DisplayName== "H2OLvl"

		if (Display.DisplayName == "H2OEdit")
		{
			/*
				H2OEdit.txt
				Text1,text,-H2O Sens Edit-,Used to set water level sensor parameters to control pumps (cont)
				Text1,text,-H2O Sens Edit-,No H2O = water not touching sensor, both pumps on (cont)
				Text1,text,-H2O Sens Edit-,mid H2O = lower pump on, upper off (cont)
				Text1,text,-H2O Sens Edit-,high H20 = both pumps off.
				H2OLvlNo,U-D--####------,--No H2O--,U/D  #### =none
				H2OLvlLow,U-D--####------,--Mid H2O--,U/D  #### = Mid
				H2OLvlMid,U-D--####-------,--High H2O--,U/D  #### = High
				rate,U-D---------###-,--Sample Rate--,U/D   Every 300s
				action,menu,---Action---,Update   Cancel
			*/
			if (Display.DisplayLineName == "action")
			{
				if (Display.DisplaySelection == "Update")
				{
					dprintln(F("H2OEdit-->action-->Update"));	//debug
					Display.DisplayGetSetNum(&tempString, "H2OLvlNo", mget);
					WaterSens.WaterLvlNo = tempString.toInt();

					Display.DisplayGetSetNum(&tempString, "H2OLvlLow", mget);
					WaterSens.WaterLvlLow = tempString.toInt();

					Display.DisplayGetSetNum(&tempString, "H2OLvlMid", mget);
					WaterSens.WaterLvlMid = tempString.toInt();

					Display.DisplayGetSetNum(&tempString, "rate", mget);
					WaterSens.WaterLvlMid = tempString.toInt();
					WaterSens.SetPollInterval(tempString.toInt() * 1000);	//set the new polling rate
				}
				else
					if (Display.DisplaySelection == "Cancel")
					{
						Display.DisplaySetup(mReadWrite, mUseSD, "Main_UI", 1, DisplayBuf); // Return to main-UI display array and display the first line
						goto EndDisplayProcessing; //exit processing Display	
					}
					else
					{
						ErrorLog("error processing H2OEdit-->action: unrecognized DisplaySelection",Display.DisplaySelection, 2);
						dprint(F("error processing H2OEdit-->action: unrecognized DisplaySelection=")); dprintln(Display.DisplaySelection);
					}
			}
			goto EndDisplayProcessing; //exit processing Display
		}	// end processing DisplayName== "H2OEdit"

		if (Display.DisplayName == "H2OTest")
		{
			/*if here then processing H2OTest
				H2OTest.txt
				Text1,text,--H2O Lvl Test--,Halts measurement and tests water level sensor (cont)
				Text2,text,--H2O Lvl Test--,Value should be between 0 and 1024. (cont)
				Text3,text,--H2O Lvl Test--,Determine reading for no level--> water not touching sensor (cont)
				Text4,text,--H2O Lvl Test--,Determine reading for mid level--> water mid filter (cont)
				Text5,text,--H2O Lvl Test--,Determine reading for high level--> water 1 inch from top outflow.
				H2OLvl,U-D--####------,-H2O Lvl Value-,U/D  #### l/min
				action,menu,---Action---,Begin_Test   End_Test
			*/
			if (Display.DisplayLineName == "action")
			{
				if (Display.DisplaySelection == "Begin_Test")
				{
					dprintln(F("H2OTest-->action-->Begin_Test"));	//debug
					InMonitoringMode = false;	// flag to end monitoring mode....stop reading and logging sensor readings
					statusLEDs.SetGreenLED(false);
					InWaterLvlTestMode = true;	// flag to tell system we are testing the water level sensor
					WaterSens.SetPollInterval(WaterLvlTestingInterval);	//sampeling delay used for testing. e.g. read every 2 sed
					Display.DisplayLineRefresh("H2OLvl");	// show the display line for where the water level value will be shown
				}
				else
					if (Display.DisplaySelection == "End_Test")
					{
						dprintln(F("H2OTest-->action-->End_Test"));	//debug
						InMonitoringMode = true;		// flag to resume monitoring mode....resume reading and logging sensor readings
						statusLEDs.SetGreenLED(true);
						InWaterLvlTestMode = false;		// flag to end water level sensor testing
						WaterSens.SetPollInterval(WaterSens.GetPollInterval());	//retrore soft interupt for when to read the sensor during monitoring
						Display.DisplaySetup(mReadWrite, mUseSD, "H2OLvl", 4, DisplayBuf); // return to the entry screen for water  sensor display array and display the first line
					}
						else
						{
							ErrorLog("error processing H2OTest-->action: unrecognized DisplaySelection",Display.DisplaySelection, 2);
							dprint(F("error processing H2OTest-->action: unrecognized DisplaySelection=")); dprintln(Display.DisplaySelection);

						}
			}
			goto EndDisplayProcessing; //exit processing Display
		}	// end processing DisplayName== "H2OTest"

		if (Display.DisplayName == "SysStat")
		{
			/*
				SysStat.txt
				text1,text,-Config System-,Start up status of sensors. Can be changed through main menu. (cont)
				text2,text,-Config System-,Used to enable / disable sensor functions. (cont)
				text3,text,-Config System-,note: WLvl=water level sensor, RelB= relay board.
				Temp,U-D----------CCC,--Temp: On/Off--,U/D  Temp is On
				Flow,U-D----------CCC,--Flow: On/Off--,U/D  Flow is On
				Wlvl,U-D----------CCC,--WLvl: On/Off--,U/D  WLvl is On
				Relay,U-D----------CCC,--RelB: On/Off--,U/D  RelB is On
				Pumps,U-D---------CCCC,Pump:On/Off/Auto,U/D  Pumps@ Auto
				action,menu,---Action---,Update  Cancel
			*/
			if (Display.DisplayLineName == "action")
			{
				if (Display.DisplaySelection == "Update")
				{
					dprintln(F("SysStat-->action-->Update"));	//debug
					/*need to copy the contents of the display array named SysStat to SD card and reset the system.  We need to reset
					because there is a lot of code in start up that depends on global settings.  e.g. TempSensorsOn.
					To reset the system, we use the watchdog timer
					*/
					Display.DisplayWriteSD();	// writes the current info set by the user to SD
					
	
					// tell the user we are going to reset the system
					lcd.begin(16, 2);	//unclear why, but this is needed every time else setCursor(0,1) doesn't work....probably scope related.
					lcd.clear();
					lcd.setCursor(0, 0);
					lcd.print("Rebooting now to");
					lcd.setCursor(0, 1);
					lcd.print("take effect");

					//use the watchdog timer to reset the system
					wdt_disable();
					wdt_enable(WDTO_4S);	//this will cause a reset if the watchdog timer is not re-triggered within 4 sec					
					while (1) {}	//loop until reset
				}
				else
					if (Display.DisplaySelection == "Cancel")
					{
						Display.DisplaySetup(mReadWrite, mUseSD, "Main_UI", 1, DisplayBuf); // Return to main-UI display array and display the first line
						goto EndDisplayProcessing; //exit processing Display	
					}
					else
					{
						ErrorLog("error processing SysStat-->action: unrecognized DisplaySelection",Display.DisplaySelection, 2);
						dprint(F("error processing SysStat-->action: unrecognized DisplaySelection=")); dprintln(Display.DisplaySelection);

					}
			}
			goto EndDisplayProcessing; //exit processing Display
		}	// end processing DisplayName== "SysStat"

			//jf add  processing for new screens here
		
		ErrorLog("error, unrecognized Display.DisplayName",Display.DisplayName,2);	//should have recognized displayName
		dprint(F("error, unrecognized Display.DisplayName=")); dprintln(Display.DisplayName);
	}	// end DisplayUserMadeSelection=true

	EndDisplayProcessing:	//target of goto. common exit for processing display array entries for object Display

	Display.DisplayUserMadeSelection = false;		// reset flag because we are processing the response
	//END----------------------------------------------------------Processing User Selection -------------------------------------------------------


	//Begin--------------------------------------------------------Process Sensors in Monitoring Mode-----------------------------------------------
	if (InMonitoringMode)
	{
		/* check the flags set by timers used for sensor management.  Each sensor will set a flag indicating if it is
		ready to be processed
		*/

		//-------------------------------------------Temp Sensors----------------------------
		if (TempSens0.TempSensReady)
		{
			TempSens0.TempSensReady = false;	//reset because we are processing this 
			dprint("Temperature for pond (0) = "); dprintln(TempSens0.TempF);
			dprintln(F("___________________________________________________________________"));
			//log values for deg F and deg C.
			MonitorLog(TempSens0.SensName, TempSens0.SensHandle, String(TempSens0.TempF), TempSens0.TempF, 0, "deg F", String(TempSens0.TempC), TempSens0.TempC, 0, "deg C");
		}

		if (TempSens1.TempSensReady)
		{
			TempSens1.TempSensReady = false;	//reset because we are processing this 
			dprint("Temperature for internal sensor (1) = "); dprintln(TempSens1.TempF);
			dprintln(F("___________________________________________________________________"));
			MonitorLog(TempSens1.SensName, TempSens1.SensHandle, String(TempSens1.TempF), TempSens1.TempF, 0, "deg F", String(TempSens1.TempC), TempSens1.TempC, 0, "deg C");

		}
		//-------------------------------------------Water Level Sensor and pump relays----------
		if (WaterSens.WaterLvlSensReady)
		{
			WaterSens.WaterLvlSensReady = false;	//reset ready flag
			dprint(F(" water sensor reading=")); dprint(WaterSens.WaterLvl);
			dprint(F(" WaterLvlRange=")); dprintln(WaterSens.WaterLvlRange);
			dprintln(F("___________________________________________________________________"));

			//log the value and water level interpretation 
			MonitorLog("Water level sensor","",String(WaterSens.WaterLvl),WaterSens.WaterLvl,0,"",WaterSens.WaterLvlRange,0,0,"");

			/*
			Water sensor level range is used to turn on/off water pumps.  The water pump from the
			bottom of the pond is attached to relay 1, and the skimmer pump attached to relay 2.
			Connections for both are in the normally closed position, so that when the relay is 'on',
			the pump is turned off.  

			When the filters get dirty or if there is a blockage in the outflow of the filter, the
			water rises in the filter.
			When the water level reaches the 'mid' value, we want to turn off the upper skimmer pump
			and resume it when water level is 'low'.
			If the water continues to rise, then the water level reaches 'high', at which time
			we want to turn off the bottom filter pump, and turn it on when the water level is mid.
			We will use WaterLvlRange and PriorLvlRange to determine if water level is increasing or decreasing

			if the pump/filter circuit develops a serious leak and the water level drops below the sensor (level = 'none')
			then we want to turn off both pumps.

			We use the prior state and current state to determine if the water level is increasing or decreasing. The code should work if there is a
			sudden change in level....e.g. bump the filter with a wave :-)
			*/

			if ((WaterSens.WaterLvlRange == "none") && (WaterSens.PriorLvlRange == "none"))
			{
				//Starting with the assumption on no leak, so assume water level rising so we want both pumps circuits on
				Relay.RelaySet(1, true);
				Relay.RelaySet(2, true);	//skimmer on relay 2
			}
			else if (((WaterSens.PriorLvlRange == "none") || (WaterSens.PriorLvlRange == "low")) && (WaterSens.WaterLvlRange == "low"))
			{
				// water level rising so we want both pumps on
				Relay.RelaySet(1, true);
				Relay.RelaySet(2, true);	//skimmer on relay 2
			}
			else if (((WaterSens.PriorLvlRange == "none") || (WaterSens.PriorLvlRange == "low")) && WaterSens.WaterLvlRange == "mid")
			{
				// water level rising  and reached mid level, so we want to turn off the skimmer and leave the bottom on
				Relay.RelaySet(1, true);
				Relay.RelaySet(2, false);
			}
			else if (((WaterSens.PriorLvlRange == "none") || (WaterSens.PriorLvlRange == "low") || (WaterSens.PriorLvlRange == "mid")) && WaterSens.WaterLvlRange == "mid")
			{
				// water level rising  and still mid level, so we want to leave skimmer off
				Relay.RelaySet(1, true);
				Relay.RelaySet(2, false);	//skimmer on relay 2
			}
			else if (((WaterSens.PriorLvlRange == "none") || (WaterSens.PriorLvlRange == "low") || (WaterSens.PriorLvlRange == "mid")) && WaterSens.WaterLvlRange == "high")
			{
				// water level rising  and reached high level, so we want to turn off both pumps
				Relay.RelaySet(1, false);
				Relay.RelaySet(2, false);	//skimmer on relay 2
			}
			else if ((WaterSens.PriorLvlRange == "high") && (WaterSens.WaterLvlRange == "high"))
			{
				// water level high and still high so we want to leave both pumps off
				Relay.RelaySet(1, false);
				Relay.RelaySet(2, false);
			}
			else if ((WaterSens.PriorLvlRange == "high") && (WaterSens.WaterLvlRange == "mid"))
			{
				// water level was high and dropped to mid, so turn bottom pump on again
				Relay.RelaySet(1, true);	// bottom pump on relay 1
				Relay.RelaySet(2, false);
			}
			else if (((WaterSens.PriorLvlRange == "high") || (WaterSens.PriorLvlRange == "mid")) && (WaterSens.WaterLvlRange == "mid"))
			{
				// water level was high and dropped to mid and still at mid, so bottom pump on, skimmer off
				Relay.RelaySet(1, true);
				Relay.RelaySet(2, false);
			}
			else if (((WaterSens.PriorLvlRange == "high") || (WaterSens.PriorLvlRange == "mid")) && (WaterSens.WaterLvlRange == "low"))
			{
				// water level was up (high or mid) and dropped to low, so turn an both pumps
				Relay.RelaySet(1, true);
				Relay.RelaySet(2, true);
			}
			else if (((WaterSens.PriorLvlRange == "mid") || (WaterSens.PriorLvlRange == "low")) && (WaterSens.WaterLvlRange == "low"))
			{
				// water level was up (mid or low) and remains at low, so keep both pumps on
				Relay.RelaySet(1, true);
				Relay.RelaySet(2, true);
			}
			else if (((WaterSens.PriorLvlRange == "high") || (WaterSens.PriorLvlRange == "mid") || (WaterSens.PriorLvlRange == "low")) && (WaterSens.WaterLvlRange == "none"))
			{
				// water level was up and now at none, so filter is emptying, which could be due to a big leak, so turn off both pumps to prevent draining the pond
				Relay.RelaySet(1, false);
				Relay.RelaySet(2, false);
				ErrorLog("Water level sensor suggests big leak, turning both pumps off","",3);	// log error, indicate most severe level			
			}
		}
		//-------------------------------------------Flow Sensors----------------------------
		if (FlowSens.FlowReadReady)
		{
			FlowSens.FlowReadReady = false;			// reset ready flag because wee are processing this
			dprint(F("Flow Sensor 1 =")); dprint(FlowSens.FlowValue1); dprint(F(" l/min, duty cycle in ms=")); dprintln(FlowSens.flow1dur);
			dprint(F("Flow Sensor 2 =")); dprint(FlowSens.FlowValue2); dprint(F(" l/min, duty cycle in ms=")); dprintln(FlowSens.flow2dur);
			dprintln(F("___________________________________________________________________"));

			//log values for both flow sensors.  log the flow as value 1 and low flow status as value 2
			boolean	isLow;
			if (FlowSens.FlowValue1 <= FlowSens.Flow1Warn) isLow = true; else isLow = false;
			MonitorLog(FlowSens.Flow1Name, "",String(FlowSens.FlowValue1), 0, FlowSens.FlowValue1, "l/min",String(isLow),0,0,"low flow warning");
			MonitorLog(FlowSens.Flow2Name, "", String(FlowSens.FlowValue2), 0, FlowSens.FlowValue2, "l/min", String(isLow), 0, 0, "low flow warning");
		}

	}	// end if in monitoring mode
	//END----------------------------------------------------------Process Sensors in Monitoring Mode-----------------------------------------------

	//Begin--------------------------------------------------------Testing Mode---------------------------------------------------------------------

	if(!InMonitoringMode)
	{
		if (InTempSensTestMode) 
		{
				/*if here, then in temperature sensor testing mode.  check for valid readings and update display
					TempTst.txt
					Text1,text,--Tmp Test_0&1--,Halts measurement and tests both temp sensors (cont)
					Tex22,text,--Tmp Test_0&1--,Temp 0 & 1 values on separate lines.
					tempValue0,U-D--###-#----,--Temp_0 Value--,U/D  ###.# F
					tempValue1,U-D--###-#----,--Temp_1 Value--,U/D  ###.# F
					action,menu,---Action---,Begin_Test   End_Test
				*/			
			
			String tmpStr;
			if (TempSens0.TempSensReady)
			{

				TempSens0.TempSensReady = false;					//reset because we are processing this 
				tmpStr = String(TempSens0.TempF, 1);				//convert float to string with 1 decimal point
				if ((TempSens0.TempF) < 100) tmpStr = "0" + tmpStr;	//pad if needed because format = ###.#

				Display.DisplayGetSetNum(&tmpStr, "tempValue0", true);	//write the temperature interger as a string with 1 decimal point.
				Display.DisplayLineRefresh("tempValue0");						//refresh the line but only if it is currently displayed
			}

			if (TempSens1.TempSensReady)
			{
				TempSens1.TempSensReady = false;					//reset because we are processing this 
				tmpStr = String(TempSens1.TempF, 1);				//convert float to string with 1 decimal point
				if ((TempSens1.TempF) < 100) tmpStr = "0" + tmpStr;	//pad if needed because format = ###.#

				Display.DisplayGetSetNum(&tmpStr, "tempValue1", true);	//write the temperature interger as a string with 1 decimal point.
				Display.DisplayLineRefresh("tempValue1");						//refresh the line but only if it is currently displayed
			}		
		}
		//------------------------------------------------------------------------------------------

		if (InFlowSensTestMode && FlowSens.FlowReadReady)
		{
			/*
			if here, then in flow sensor testing mode and we have a valid reading.  
			We want to update 2 display lines in display named FlowTest with the flow rates in l/min

				FlowTest.txt
				{snip}
				Flow1Value,U-D--###,--Flow1 Value--,U/D  ### l/min
				Flow2Value,U-D--###,--Flow2 Value--,U/D  ### l/min
				Flow1Dur,U-D--##----,--Flow1Dur--,U/D  ## ms
				Flow2Dur,U-D--##----,--Flow2Dur--,U/D  ## ms
				action,menu,---Action---,Begin_Test   End_Test
			*/
			String tmpStr;
			FlowSens.FlowReadReady = false;	//reset because we are processing this

			//enter flow rate and frequency of cycling.  We are sampling at 5ms, so want the cycle time to be <= 10ms (nyquist freq, sample rate should be minimym of 2x frequency)
			tmpStr = String(FlowSens.FlowValue1, 0);
			Display.DisplayGetSetNum(&tmpStr, "Flow1Value", mset);	// set the value, 0 dec
			Display.DisplayLineRefresh("Flow1Value");				// update display if this is the line user is looking at

			tmpStr = String(FlowSens.flow1dur, 0);
			Display.DisplayGetSetNum(&tmpStr, "Flow1Dur", mset);
			Display.DisplayLineRefresh("Flow1Dur");

			tmpStr = String(FlowSens.FlowValue2, 0);
			Display.DisplayGetSetNum(&tmpStr, "Flow2Value", mset);
			Display.DisplayLineRefresh("Flow2Value");

			tmpStr = String(FlowSens.flow2dur, 0);
			Display.DisplayGetSetNum(&tmpStr, "Flow2Dur", mset);
			Display.DisplayLineRefresh("Flow2Dur");

			FlowSens.FlowCalcBegin();	// begin the next reading
		}
		//------------------------------------------------------------------------------------------

		if (InWaterLvlTestMode && WaterSens.WaterLvlSensReady)
		{
			/*	
			if here, then in water level sensor testing mode and reading is ready. The display array H2OTest is on the display.
			We want to update the water level reading on the line "H2OLvl"
			H2OTest.txt
			Text1,text,--H2O Lvl Test--,Halts measurement and tests water level sensor (cont)
			Text2,text,--H2O Lvl Test--,Value should be between 0 and 1024. (cont)
			Text3,text,--H2O Lvl Test--,Determine reading for no level--> water not touching sensor (cont)
			Text4,text,--H2O Lvl Test--,Determine reading for mid level--> water mid filter (cont)
			Text5,text,--H2O Lvl Test--,Determine reading for high level--> water 1 inch from top outflow.
			H2OLvl,U-D--####------,-H2O Lvl Value-,U/D  #### l/min
			action,menu,---Action---,Begin_Test   End_Test
			*/

			String tmpStr;
			WaterSens.WaterLvlSensReady = false;	//reset because we are processing this		
			tmpStr = String(WaterSens.WaterLvl);	//convert the WaterLvl value (float to string)
			Display.DisplayGetSetNum(&tmpStr, "H2OLvl", mset);	//enter the value into the display array
			Display.DisplayLineRefresh("H2OLvl");	// update the display
		}
	} // end 	if(!InMonitoringMode)
	    //------------------------------------------------------------------------------------------
	//End----------------------------------------------------------Testing Mode---------------------------------------------------------------------


} // end main loop
