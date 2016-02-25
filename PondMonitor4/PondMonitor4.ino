/*
 Name:		PondMonitor4.ino
 Created:	1/10/2016 6:59:04 PM
 Author:	jimfr
*/

#include <inttypes.h>
#include <Event.h>
#include <Timer.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <Time.h>			// time functions, used by Real Time Clock (RTC)
#include <DS1307RTC.h>		// library for RTC module
#include <avr/pgmspace.h>	// library for memory optimization using flash mem
#include <SD.h>				// library for SD card

File SDfile;			// file object for accessing SD card
Timer Tmr;	// timer object used for polling at timed intervals
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);	// LCD display
tmElements_t SysTm;		// system time
String SysTmStr;		// system time as string, format HH:MM using 24 hr time. derived from SysTm
String SysDateStr;		// system date as string, format mm/dd/yyyy. derived from SysTm
String sysDOWstr;		// system day of week as string, 3 chr length.
int SysTmPoleContext;	// ID of timer used to poll system time
#define SysTmPoleFreq 1000	// time polling frequency


	//int ledEvent;
	//------------------------ String arrays used by the Display class to support the user interface------------------------------

	/* these variables replaced by routines that read from SD card into reusable DisplayBuf
	String Main_UI[4]=
	{"SetUp,menu,---Set Up---,RTC   Temp_sensor  Flow_sensor",
	"Row_2,menu,---2nd Display---,run2   set_time2   sensor_tst2   calibrate2",
	"Row_3,menu,---3rd Display---,run3   set_time3   sensor_tst3   calibrate3",
	"Row_4,menu,---4th Display---,Yes   No"};

	String SetRTC_ui[5]=
	{"Text,text,---RTC Setup---,Used to view/update the date, time, and day of week settings",
	"Date,m--d----yy--U-D,---RTC Date---,01/01/2015  U/D",
	"Time,H--M---U-D,---RTC Time---,01:01  U/D",
	"DOW,a-------U-D,--RTC DOW--,Mon     U/D",
	"action,menu,---Action---,Update   Cancel"};

	String TempSensor_ui[2]=
	{"Text1,text,---Tmp Sensor---,Used to find, name, and test temp sensors",
	"action,menu,---Action---,Discover  Name  Test  Cancel"};
	*/

//buffer used to  load/save string arrays used for Display object.  This is max 80 chr X 7 lines
#define DisplayBufLen 100	// max len of strings in DisplayBuf
String DisplayBuf[7];		// buffer used to work with DisplayArrays. Read/written from SD card by Display object
							//merge test
							/*String DisplayBuf[6]=
							{"01234567890123456789012345678901234567890123456789012345678901234567890123456789",
							"01234567890123456789012345678901234567890123456789012345678901234567890123456789",
							"01234567890123456789012345678901234567890123456789012345678901234567890123456789",
							"01234567890123456789012345678901234567890123456789012345678901234567890123456789",
							"01234567890123456789012345678901234567890123456789012345678901234567890123456789",
							"01234567890123456789012345678901234567890123456789012345678901234567890123456789"};
							*/

							/*
							String menu3[7]=
							{"mnu1,menu,---Main Menu---,Make-Mnu-R/W   Make-Mnu-R/O",
							"Date1,m--d----yy--U-D,---Date Entry1---,11/29/1955  U/D",
							"Time1,H--M---U-D,---Time Entry1---,13:27  U/D",
							"Numeric1,-----##--U-D,---# Entry1---,Int= 00  U/D",
							"Alpha1,--CCCCCCC--U-D,---Label 1---,--abcdefg  U/D",
							"DOW,-a---a---U-D,--Day of Week--,-Mon-Tue-U/D",
							"Text1,text,---Message---,Now is the time for all good men to come to the aid of their country"};
							*/


/* ---------------------------------------------- ErrorLog Routines --------------------------------------------------*/
/*
these are routines to log errors to Logs/errorlog.txt on the file on the SD card.  SD care is initialized in setup routine.  
The error log entry includes system date, time, and log entry.
*/
void ErrorLog(String error)
{
	// writes error to errorlog.  If not able to write to errorlog, on SD card, writes to monitor and turns on LCD backlight strobe alarm.
	// Note that only one file can be open at a time, so you have to close this one before opening another.
	// Note, need to add transaction for SPI bus becuse SD card uses SPI bus and so do other devices on this system.
	String errorLine = SysDateStr + " " + SysTmStr + ": " + error;	//form error line = system date, time, and error string
	
	SDfile = SD.open("log/errorlog.txt", FILE_WRITE);

	if (SDfile) 
	{
		SDfile.println(errorLine);	//write system date, time, and error string
		SDfile.close();
		Serial.println(errorLine);	//debug
	}
	// if the file isn't open, print error on monitor:
	else 
	{
		Serial.println(errorLine);
		Display.LCDflashAlarm(true);	//turn on 'flash alarm' which blinks LCD backlight to get attention
	}
}


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

//#include <inttypes.h>


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


//--------------------------Display Class Definition and Display Related Global Variables -----------------------------
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
protected:
	boolean DisplayInUse;	// if true, then the display array is in use.  Used by ProcessDisplay to only process key presses when Display is active. Needed because other parts of the program could be using the keypad.
	int	DisplayLineCnt;		// count of lines in string matrix making up display array
	int	DisplayIndex;		// index to DisplayLine being displayed
	#define MaxDisplayArrayLen 20	//max number of display line strings in a display array
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
	#define DimTimeout 6000			// number of milliseconds to wait with no key being pressed to turn off the LCD backlight
	time_t	TimeToDim;			// used for elapsed time counter for dimming LCD backlight
	boolean LCDbacklightOn;		// if true, then the backlight is on. Used for strobe alarm to get attention
	#define	LCDstrobeRate	750	// strobe interval in milliseconds for LCD backlight alarm
	//boolean	FlashAlarmIsOn;		// if true, then LCD backlight is strobing to get some attention
	void DisplayStartStop(boolean action);	//Used to indicate that the Display is in use and that keypresses should be processed by the Display routines
	void DisplaySetup(boolean isReadOnly, boolean readFromSD, String mnuName, int mnuLines, String *mnu); //sets up the Display.  needs to be passed the name, number of lines, and pointer to an array of strings (formatted as DisplayLines)
	void ProcessDisplay(int KeyID); //Main processing routine for Display.  This is called after user has pressed a key (up, dn, rt, lt, or Select) that has been debounced by the LS_Key routines
	void CursorBlinkTimeInt(void);	// soft interrupt routine to make 'cursor' blink 
	boolean DisplayGetSetDate(String *DateStr, String MnuLineName, boolean set);	// gets or sets date in the display line named MnuLineName in the current display array.  date format is mm/dd/yyyy.  if Set is true then sets value of DateStr else gets value
	boolean DisplayGetSetTime(String *TimeStr, String MnuLineName, boolean set);	// gets or sets time in the display line named MnuLineName in the current display array.  time format is HH:MM.  if Set is true then sets value of DateStr else gets value
	boolean DisplayGetSetDOW(String *DayStr, String MnuLineName, boolean set);		// gets or sets day of week (DOW) in the display line named MnuLineName in the current display array.  DOW format is 3 chr abbreviation, see variable DisplayDOW[7] for values  if Set is true then sets value of DateStr else gets value
	boolean DisplayGetSetNum(String *NumStr, String MnuLineName, boolean set);		// gets or sets a numeric value in the display line named MnuLineName in the current display array. If Set is true then sets value of DayStr else gets value
	boolean DisplayGetSetChrs(String *ChrStr, String MnuLineName, boolean set);	// gets or sets a character string in the display line named MnuLineName in the current display array. If Set is true then sets value of ChrStr else gets value
	boolean DisplaySetTxt(String *TxtStr, String MnuLineName);						// sets a text message in the display line named MnuLineName in the current display array. Get not needed as this is only for outputting messages
	boolean DisplayWriteSD(void);													// writes the current display array to a file on the SD card.  Uses DisplayPntr, DisplayName, and DisplayLineCnt.  file is named DisplayName and is overwritten.  returns true if successful
	void LCDflashAlarm(boolean TurnOn);												// starts/stops the LCD strobe alarm for major errors.  If true then turns on flash until key is hit

																					//Methods for optimizing SCRAM by using program memory to store const strings. Actual const char * are declaired below outside of the class because I couldn't get them to work here.
	String ProgMemLU(const char* LUwhich, unsigned int LUwhere, unsigned int LUlen);	// uses PROGMEM library to retrieve substrings of char arrays stored in program memory for RAM conservation

} Display;

//-------------------------------------------
//preset the strings used for data entry using the methods in the Display class.  
//Defined here instead of in the class because I couldn't figure out how to get it to work within the variables declared for the class.
//Note, these are stored in program memory to conserve space using DisplayClass::ProgMemLU and the PROGMEM ligrary.

const char	DisplayDay[] PROGMEM = { "01020304050607080910111213141516171819202122232425262728293031" };	// day will advance 2 chr at a time
const char DisplayMonth[] PROGMEM = { "010203040506070809101112" };	// month will advance 2 chr at a time
const char DisplayYear[] PROGMEM = { "0123456789" };	// year will advance 1 chr at a time
const char DisplayHour[] PROGMEM = { "000102030405060708091011121314151617181920212223" };	//hour will advance 2 chr at a time
const char DisplayMin[] PROGMEM = { "00010203040506070809101112131415161718192021222324252627282930313233343536373839404142434445464748495051525354555657585960" };	// Min will advance 2 chr at a time
const char DisplayNum[] PROGMEM = { "0123456789" };	// numbers advance 1 chr at a time
const char DisplayChar[] PROGMEM = { "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_$%&" };	//alpha numeric characters will advance 1 at a time
const char	DisplayDOW[] PROGMEM = { "MonTueWedThuFriSatSun" };


/*String DisplayDay[31]= {"01","02","03","04","05","06","07","08","09","10",
"11","12","13","14","15","16","17","18","19","20",
"21","22","23","24","25","26","27","28","29","30","31"};	// day will advance 2 chr at a time

String DisplayYear[10]= {"0","1","2","3","4","5","6","7","8","9"};	//year will advance 1 chr at a time
String DisplayHour[24]= {"00","01","02","03","04","05","06","07","08","09","10",
"11","12","13","14","15","16","17","18","19","20","21","22","23"};	// hour will advance 2 chr at a time
String DisplayMin[60]= {"00","01","02","03","04","05","06","07","08","09","10",
"11","12","13","14","15","16","17","18","19","20",
"21","22","23","24","25","26","27","28","29","30",
"31","32","33","34","35","36","37","38","39","40",
"41","42","43","44","45","46","47","48","49","50",
"51","52","53","54","55","56","57","58","59"};
String DisplayNum[10]= {"0","1","2","3","4","5","6","7","8","9"};	//numbers advance 1 chr at a time
String DisplayChar[64]= {"a","b","c","d","e","f","g","h","i","j","k","l","m","n","o","p","q","r","s","t","u","v","w","x","y","z",
"A","B","C","D","E","F","G","H","I","J","K","L","M","N","O","P","Q","R","S","T","U","V","W","X","Y","Z",
"0","1","2","3","4","5","6","7","8","9","-","_"};
String DisplayDOW[7]=	{"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};	//days of the week start on sunday and are 3 chr's each

*/
//-------------------------------------------------Display Class routines----------------------------------------------
DisplayClass::DisplayClass(void)
{
	//constructor
	DisplayPos = 0;
	DisplayLine = "01234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890012345678901234567890012345678901234567890";	//reserve 140 chr length
	LCDbacklightOn = true;	// LCD backlight is on
	//FlashAlarmIsOn = false;	// LCD strobe alarm is off
};
//------------------------------------------
int DisplayClass::DisplayAdvPastSpace(String Mline, int Start)
{
	//starting  at a space in a string, return the position of the next non-space chr
	//Serial.print("in DisplayAdvPastSpace, values passed in: Mline=|"); Serial.print(Mline); Serial.print("|, start="); Serial.println(Start);
	unsigned int x = Start;
	while (Mline[x] == ' ' && x < Mline.length() + 1)
	{
		x++;	//advance until encounter a character other than space
	}

	//Serial.print("chr and index='"); Serial.print(Mline[x]); Serial.print("' , "); Serial.println(x);//Serial.print ('|');
	//Serial.print(Mline[x]);
	//Serial.println('|');
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

	//Serial.print ('|');
	//Serial.print(Mline[x]);
	//Serial.println('|');
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

	//Serial.print ('|');
	//Serial.print(Mline[x]);
	//Serial.println('|');
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
	Serial.println(tmpStr);
	idx = tmpStr.toInt() - 1;	//month # -1 = index # due to index starting with 0
	Serial.println(idx);

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
												//Serial.println(MonthStr);Serial.print("displayPos="); Serial.println(DisplayPos);
												//Serial.print("0-DisplayPos=|"); Serial.print(DisplayLine.substring(0,DisplayPos)); Serial.print("| displayPos+2-->len=|"); Serial.print( DisplayLine.substring (DisplayPos+2,DisplayLine.length()));Serial.println("|");
	tmpStr = DisplayLine.substring(0, DisplayPos) + MonthStr + DisplayLine.substring(DisplayPos + 2, DisplayLine.length());
	//replace the month 
	//Serial.print("|"); Serial.print(DisplayLine); Serial.println("|");	//debug
	//Serial.print("|"); Serial.print(tmpStr); Serial.println("|");//debug
	DisplayLine = tmpStr;
	//Serial.print("|"); Serial.print(DisplayLine); Serial.println("|");	//debug
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
																															//Serial.print("|"); Serial.print(DisplayLine); Serial.println("|");	//debug
																															//Serial.print("|"); Serial.print(tmpStr); Serial.println("|");//debug
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
																															//Serial.print("|"); Serial.print(DisplayLine); Serial.println("|");	//debug
																															//Serial.print("|"); Serial.print(tmpStr); Serial.println("|");//debug
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
		if (idx > 0) { idx--; }
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

	while ((tmpStr != ProgMemLU(DisplayChar, idx, 1)) && (idx<64)) idx++; //scan the string array for the day of the week under DisplayPos
																		  //scan the string array for the chr under DisplayPos
																		  /*for (idx=0, idx<64, idx++)
																		  {
																		  tmpStr1= ProgMemLU(DisplayChar,idx,1);
																		  if (tmpStr1==tmpStr) {break;}		//exit when we find it
																		  }
																		  */
	if (increment)
	{
		if (idx <63) { idx++; }
		else { idx = 0; }
	}
	else
	{
		if (idx > 0) { idx--; }
		else { idx = 63; }
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
	//Serial.print(F("DisplayLineSetup SCRAM="));Serial.println(getFreeSram()); //debug
	int tmp1, tmp2, tmp3;
	String TemplateChar;							// used to process the TemplateLine
	tmp1 = Mline.indexOf(',');						// get position of comma, used to parse
	DisplayLineName = Mline.substring(0, tmp1);		// get DisplayLineName
	tmp2 = Mline.indexOf(',', tmp1 + 1);				// position of next comma
	TemplateLine = Mline.substring(tmp1 + 1, tmp2);	// get TemplateLine
													//Serial.println("templateLine=|" + TemplateLine +"|");	
	tmp3 = Mline.indexOf(',', tmp2 + 1);				// position of next comma
	DisplayLineTitle = Mline.substring(tmp2 + 1, tmp3); // get DisplayLineTitle
	DisplayLine = "                        " + Mline.substring(tmp3 + 1, Mline.length());	//snip out the Display options and pre-pend with spaces
	DisplayEndPos = DisplayLine.length();					// set the end of the Display position pointer
	DisplayLine += "                    ";				// pad the DisplayLine with spaces at the end with Display line
														//Serial.print("DisplayLine=|"); Serial.print(DisplayLine); Serial.println("|");
	DisplayStartPos = DisplayAdvPastSpace(DisplayLine, 0);	// find the first non-space chr
	DisplayPos = DisplayStartPos;							// position index used for scrolling

															//display the Display title and Display
	lcd.begin(16, 2);	//unclear why, but this is needed every time else setCursor(0,1) doesn't work....probably scope related.
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(DisplayLineTitle);
	lcd.setCursor(0, 1);

	//Serial.println("DisplayLine=" + DisplayLine);
	//Serial.print("DisplayStartPos="); Serial.print(" DisplayEndPos="); Serial.print(DisplayEndPos); Serial.print(" DisplayPos="); Serial.println(DisplayPos);

	if (DisplayEndPos - DisplayStartPos + 1>DisplayDisplayLen)
	{
		//length of the Display line is wider than display
		//Serial.println("length of display line is wider than display");
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
			Serial.println(F("Error reading menu in DisplayClass::DisplaySetup"));	//replace with errorlog
		}
		//Serial.print(F("UsingDisplayPntr")); 
		//for (int z=0; z<mnuLines; z++) {Serial.println(DisplayPntr[z]);	}	//debug

	}
	else
	{
		DisplayPntr = mnu;			// using in memory display array.  this variable to hold a pointer to the String array that is the display
	}

	DisplayName = mnuName;		// name of the Display array, returned when user makes a selection
	DisplayIndex = 0;			// initialize index to DisplayLine within String array that is the Display
	if (mnuLines == 1) DisplayLineCnt = 1; else DisplayLineCnt = mnuLines - 1; // max number of DisplayLines in the Display, starting with 1
	DisplayReadOnly = isReadOnly;	// set to determine if this display array (deck) is read only or read/write.  Note applies only to display lines that take input.
	DisplayUserMadeSelection = false;	// flag indicating Display not ready to be read.  If true, user made selection
	DisplayLineSetup(*DisplayPntr);		 // extract and display DisplayLine[0]


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
				//Serial.println("displayLine=|" + DisplayLine + "|");
				DisplayOptEnd = (DisplayAdvToSpace(DisplayLine, DisplayPos));
				//Serial.print("DisplayOptEnd="); Serial.println(DisplayOptEnd);
				//Serial.print("DisplaySelection=");Serial.print(DisplayLine.substring(DisplayOptStart,DisplayOptEnd));Serial.print("  DisplayStartPos=");Serial.print(DisplayStartPos);Serial.print("  DisplayOptStart=");Serial.print(DisplayOptStart);Serial.print("   DisplayOptEnd=");Serial.print(DisplayOptEnd);Serial.print("   selection length=");Serial.println(DisplaySelection.length());
				DisplaySelection = DisplayLine.substring(DisplayOptStart, DisplayOptEnd);	// get the string of the option
				DisplayUserMadeSelection = true;	//polling routine will see this and fetch results found in DisplayName, DisplayLineName, DisplaySelection
				break;		// end processing SELECT_KEY
			}	// done with SELECT_KEY

			case (UP_KEY) :
			{
				// user wants to display DisplayLine 'above' the current DisplayLine where [0] is the highest
				if (DisplayIndex != 0)
				{
					DisplayIndex--;	//decrement DisplayIndex
					DisplayLineSetup(*(DisplayPntr + DisplayIndex)); // extract and display DisplayLine[DisplayIndex]
				}
				break;
			}	// done with UP_KEY

			case (DOWN_KEY) :
			{
				// user wants to display DisplayLine 'below' the current DisplayLine where [MaxLines] is the Lowest
				if (DisplayIndex != DisplayLineCnt)
				{
					DisplayIndex++;	//Increment DisplayIndex
					DisplayLineSetup(*(DisplayPntr + DisplayIndex)); // extract and display DisplayLine[DisplayLineCnt]
					Serial.println("KeyDown"); Serial.println(DisplayBuf[DisplayIndex]);	//debug
				}
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
				if (DisplayPos < DisplayStartPos + DisplayDisplayLen - 1) DisplayPos++; //move right one chr
				while ((TemplateLine[DisplayPos - DisplayStartPos] == '-') && (DisplayPos < DisplayStartPos + DisplayDisplayLen - 1))
				{
					DisplayPos++;	// continue to advance to non "-" chr and process below
									//Serial.print(TemplateLine[DisplayPos-DisplayStartPos]);	//debug					
				}
				if (DisplayPos - DisplayStartPos >= TemplateLine.length())
				{
					DisplayPos = DisplayStartPos + TemplateLine.length() - 1;	//don't advance past then end of the templateLine, even if there is displayLine available
				}
				break;	//done processing LS_curKey
			}

			if (LS_curKey == LEFT_KEY)
			{
				//user pressed left key, back up in template to chr to process (not = '-')
				if (DisplayPos > DisplayStartPos) DisplayPos--; //move Lt one chr
				while ((TemplateLine[DisplayPos - DisplayStartPos] == '-') && (DisplayPos > DisplayStartPos))
				{
					DisplayPos--;// continue to back up to non "-" chr and process below
								 //Serial.print(TemplateLine.substring(DisplayPos-DisplayStartPos,DisplayPos-DisplayStartPos+1));	//debug						
				}
				break;	//done processing LS_curKey
			}

			TempChar = TemplateLine[DisplayPos - DisplayStartPos];		//get the character under the cursor					
																		//Serial.print("TempChar="); Serial.print(TempChar);Serial.print("  DisplayPos="); Serial.println(DisplayPos);	//debug

																		//process the character under the DisplayPos cursor
			if (TempChar == 'm' && !DisplayReadOnly)	// pointing to first digit of month in date, only process if display is read/write
			{
				//Serial.println("processing TemplateChar=m");	//debug
				switch (tmpKey)
				{
				case (UP_KEY) :
				{
					//Serial.println("month, upkey");							
					MonthEntry(true);
					break;
				}	// done with UP_KEY

				case (DOWN_KEY) :
				{
					MonthEntry(false);
					//Serial.println("month, downkey");								
					break;
				}	// done with DOWN_KEY	
				}
				UpdateDisplayArray = true;	// flag to update the DisplayArray to save the changes.
				goto UpdateCheck;			//done processing the LS_CurKey, jump to bottom and update display array if user made changes to the displayLine

			} //if (TempChar=='m')

			if (TempChar == 'd'&& !DisplayReadOnly)	// pointing to first digit of day in date
			{
				//Serial.println("processing TemplateChar=d");	//debug
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
				//Serial.println("processing TemplateChar=y");	//debug					
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
				//Serial.println("processing TemplateChar=a");	//debug
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
				//Serial.println("processing TemplateChar=H");	//debug
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
				//Serial.println("processing TemplateChar=U");	//debug
				switch (tmpKey)
				{
				case (UP_KEY) :
				{
					// user wants to display DisplayLine 'above' the current DisplayLine where [0] is the highest
					if (DisplayIndex != 0)
					{
						DisplayIndex--;	//decrement DisplayIndex
						DisplayLineSetup(*(DisplayPntr + DisplayIndex)); // extract and display DisplayLine[DisplayIndex]
					}
					break;		// done processing 	switch (tmpKey)
				}	// done with UP_KEY

				case (DOWN_KEY) :
				{
					// user wants to display DisplayLine 'below' the current DisplayLine where [MaxLines] is the Lowest
					if (DisplayIndex != DisplayLineCnt)
					{
						DisplayIndex++;	//Increment DisplayIndex
						DisplayLineSetup(*(DisplayPntr + DisplayIndex)); // extract and display DisplayLine[DisplayLineCnt]
					}
					break;			// done processing 	switch (tmpKey)
				}	// done with DOWN_KEY
				}
				goto UpdateCheck;	//done processing the LS_CurKey, jump to bottom and update display array if user made changes to the displayLine
			}

			if (TempChar == 'D')	// pointing to the Display Down selection
			{
				//Serial.println("processing TemplateChar=D");	//debug					
				switch (tmpKey)
				{
				case (UP_KEY) :
				{
					// user wants to display DisplayLine 'above' the current DisplayLine where [0] is the highest
					if (DisplayIndex != 0)
					{
						DisplayIndex--;	//decrement DisplayIndex
						DisplayLineSetup(*(DisplayPntr + DisplayIndex)); // extract and display DisplayLine[DisplayIndex]
					}
					break;		// done processing 	switch (tmpKey)
				}	// done with UP_KEY

				case (DOWN_KEY) :
				{
					// user wants to display DisplayLine 'below' the current DisplayLine where [MaxLines] is the Lowest
					if (DisplayIndex != DisplayLineCnt)
					{
						DisplayIndex++;	//Increment DisplayIndex
						DisplayLineSetup(*(DisplayPntr + DisplayIndex)); // extract and display DisplayLine[DisplayLineCnt]
					}
					break;			// done processing 	switch (tmpKey)
				}	// done with DOWN_KE			

				}
				goto UpdateCheck;	//done processing the LS_CurKey, jump to bottom and update display array if user made changes to the displayLine					
			}

			// update display array if user made changes to the displayLine.
		UpdateCheck:		//target of goto from processing date, time, numeric, and alphanumeric
							//Serial.println("testing UpdateDisplayArray");
			if (UpdateDisplayArray)
			{
				String tmpStr = DisplayLine;
				tmpStr.trim();	//trim the whitespace added as padding prior to updating the display array
								//change the entry			
								//Serial.print("Before= |");Serial.print(DisplayPntr[DisplayIndex]);Serial.println("|");	//debug					
								//DisplayLine.trim();	//trim leading and trailing whitespace					
				DisplayPntr[DisplayIndex] = DisplayLineName + ',' + TemplateLine + ',' + DisplayLineTitle + ',' + tmpStr;	// change the entry in the display array
																															//Serial.print("After=  |");Serial.print(DisplayPntr[DisplayIndex]);Serial.println("|");	//debug
			}
			break;
		} // end case DisplayOption = 2 

		case 3:	//display line is of type text.  this supports showing long strings of text that can be rapidly scrolled through.
		{
			//int oldPos=DisplayPos;		// holds prior position of DisplayPos *Unused*
			String newline;			// string to display

			switch (LS_curKey)
			{
			case (RIGHT_KEY) :
			{
				//Serial.println(DisplayPos);	//debug

				//find the new starting position.  Advance the width of the display and back up to nearest whold word
				DisplayPos += DisplayDisplayLen;
				if (DisplayPos > DisplayEndPos) DisplayPos = DisplayEndPos;

				//if not on a space, back up to one. 
				if (DisplayLine[DisplayPos] != ' ')
				{
					//there is one or more spaces at the right side of the display
					//Serial.print("Displaypos!=' ' ");
					while (DisplayLine[DisplayPos] != ' ' && DisplayPos>DisplayStartPos) { DisplayPos--; }	//back up to start of nearest whole word
																											//Serial.println(DisplayPos);	//debug
				}
				else
				{
					//we're on a space, so back up to the next space
					//Serial.print("Displaypos==' ' ");
					while (DisplayLine[DisplayPos] == ' ' && DisplayPos>DisplayStartPos) { DisplayPos--; }	//back up to start of nearest whole word
					while (DisplayLine[DisplayPos] != ' ' && DisplayPos>DisplayStartPos) { DisplayPos--; }	//back up to start of nearest whole word
																											//Serial.println(DisplayPos);	//debug							
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
				if (DisplayIndex != 0)
				{
					DisplayIndex--;	//decrement DisplayIndex
					DisplayLineSetup(*(DisplayPntr + DisplayIndex)); // extract and display DisplayLine[DisplayIndex]
				}
				break;
			}	// done with UP_KEY

			case (DOWN_KEY) :
			{
				// user wants to display DisplayLine 'below' the current DisplayLine where [MaxLines] is the Lowest
				if (DisplayIndex != DisplayLineCnt)
				{
					DisplayIndex++;	//Increment DisplayIndex
					DisplayLineSetup(*(DisplayPntr + DisplayIndex)); // extract and display DisplayLine[DisplayLineCnt]
				}
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
	Serial.println("MnuLineName=" + MnuLineName);
	Serial.print(" Inputs: index=" );Serial.println(*Idx);
	Serial.print(" DisplayTitle="); Serial.print(*DisplayTitle); Serial.print(" templateln="); Serial.print(*TemplateLn); Serial.print(" displayLn="); Serial.println(*DisplayLn);
	*/

	int tmp1, tmp2, tmp3;
	boolean tmpBool = false;
	*Idx = 0;	//set index to 0 as calling routine doesn't need to
				//find the string in the display array that has the name MnuLineName
	for (int x = 1; x<DisplayLineCnt + 1; x++)	//DisplayLineCnt is protected within Display object and is the number of lines in the dispaly array
	{
		//Serial.println(x);
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

								//Serial.print(" Inputs: index=" );Serial.println(*Idx);
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

																												//Serial.println ("old entry=" + DisplayPntr[Index]);	//debug
	tmp1 = TemplateLine.indexOf('m');	//find where in the template the first chr of month is.
	if (tmp1 == -1) return false;		//-1 means didn't find chr which is an unexpected error, probably a typo in template or referencing incorrect line in display array	
										//Serial.print ("index="); Serial.println(tmp1);	//debug
	if (set)
	{
		//user wants to set the date
		//Serial.print("old displayline="); Serial.println(DisplayLine);	//debug
		//Serial.println(Index);	//debug
		DisplayLine = DisplayLine.substring(0, tmp1) + *DateStr + DisplayLine.substring(tmp1 + LenOfDate, DisplayLine.length());	//splice new date into display line.  works because the chr position in the template matches the those in the display line
																																	//Serial.print("new displayline="); Serial.println(DisplayLine);	//debug	

																																	//change the entry	
		DisplayPntr[Index] = MnuLineName + ',' + TemplateLine + ',' + DisplayTitle + ',' + DisplayLine;	// change the entry in the display array
																										//Serial.println ("new entry=" + DisplayPntr[Index]);	//debug
	}
	else
	{
		//user wants to read the date
		*DateStr = DisplayLine.substring(tmp1, tmp1 + LenOfDate);
		//Serial.println("date string='" + *DateStr +"'");	//debug

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
		//Serial.print("old displayline="); Serial.println(DisplayLine);	//debug
		//Serial.println(Index);	//debug
		tmpStr = TimeStr->substring(0, LenOfTime);	// clip time to 5 chrs
		DisplayLine = DisplayLine.substring(0, tmp1) + tmpStr + DisplayLine.substring(tmp1 + LenOfTime, DisplayLine.length());	//splice new time into display line.  works because the chr position in the template matches the those in the display line
																																//change the entry	
		DisplayPntr[Index] = MnuLineName + ',' + TemplateLine + ',' + DisplayTitle + ',' + DisplayLine;	// change the entry in the display array
																										//Serial.println ("new entry=" + DisplayPntr[Index]);	//debug
	}
	else
	{
		*TimeStr = DisplayLine.substring(tmp1, tmp1 + LenOfTime);
		//Serial.println("time string='" + *TimeStr +"'");	//debug

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
		//Serial.print("old displayline="); Serial.println(DisplayLine);	//debug
		//Serial.println(Index);	//debug
		DisplayLine = DisplayLine.substring(0, tmp1) + *DayStr + DisplayLine.substring(tmp1 + LenOfDOW, DisplayLine.length());	//splice new DOW into display line.  works because the chr position in the template matches the those in the display line
																																//change the entry
		DisplayPntr[Index] = MnuLineName + ',' + TemplateLine + ',' + DisplayTitle + ',' + DisplayLine;	// change the entry in the display array
																										//Serial.println ("new entry=" + DisplayPntr[Index]);	//debug
	}
	else
	{
		*DayStr = DisplayLine.substring(tmp1, tmp1 + LenOfDOW);
		//Serial.println("DOW string='" + *DayhStr +"'");	//debug

	}

	return true;	//return, successful
}
//------------------------------------------
boolean DisplayClass::DisplayGetSetNum(String *NumStr, String MnuLineName, boolean set)
{
	// gets or sets a numeric value in the display line named MnuLineName in the current display array. If Set is true then sets value of DayStr else gets value
	int Index = 0;
	int	tmp1, tmp2;
	String DisplayTitle, TemplateLine, DisplayLine;							// used to process the TemplateLine

																			// find and parse the display line
	if (!FindAndParseDisplayLine(MnuLineName, &Index, &DisplayTitle, &TemplateLine, &DisplayLine))return false;	// problem parsing display line, likely a typo in call, return error
	tmp1 = TemplateLine.indexOf('#');	//find where in the template the first chr of the number
	tmp2 = TemplateLine.lastIndexOf('#');	//find in the template the last chr of the number

	if (tmp1 == -1) return false;		//-1 means didn't find chr which is an unexpected error, probably a typo in template or referencing incorrect line in display array
	if (set)
	{
		//user wants to set 
		//Serial.print("old displayline="); Serial.println(DisplayLine);	//debug
		//Serial.println(Index);	//debug
		DisplayLine = DisplayLine.substring(0, tmp1) + *NumStr + DisplayLine.substring(tmp2, DisplayLine.length());	//splice new numeric value into display line.  works because the chr position in the template matches the those in the display line
																													//change the entry
		DisplayPntr[Index] = MnuLineName + ',' + TemplateLine + ',' + DisplayTitle + ',' + DisplayLine;	// change the entry in the display array
																										//Serial.println ("new entry=" + DisplayPntr[Index]);	//debug
	}
	else
	{
		*NumStr = DisplayLine.substring(tmp1, tmp2 + 1);
		//Serial.println("DOW string='" + *NumStr +"'");	//debug

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
	tmp1 = TemplateLine.indexOf('C');	//find where in the template the first chr of the string
	tmp2 = TemplateLine.lastIndexOf('C');	//find in the template the last chr of the string

	if (tmp1 == -1) return false;		//-1 means didn't find chr which is an unexpected error, probably a typo in template or referencing incorrect line in display array
	if (set)
	{
		//user wants to set
		//Serial.print("old displayline="); Serial.println(DisplayLine);	//debug
		//Serial.println(Index);	//debug
		NewChrStr = *ChrStr;
		DisplayLine = DisplayLine.substring(0, tmp1) + NewChrStr.substring(0, tmp2 - tmp1 + 1) + DisplayLine.substring(tmp2, DisplayLine.length());	//splice new String value into display line.  works because the chr position in the template matches the those in the display line
																																					//change the entry
		DisplayPntr[Index] = MnuLineName + ',' + TemplateLine + ',' + DisplayTitle + ',' + DisplayLine;	// change the entry in the display array
																										//Serial.println ("new entry=" + DisplayPntr[Index]);	//debug
	}
	else
	{
		*ChrStr = DisplayLine.substring(tmp1, tmp2 + 1);
		//Serial.println("DOW string='" + *NumStr +"'");	//debug

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

	Serial.println(TemplateLine);	//debug
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
										//Serial.println("timer for cursor blink started ");	//delete in future
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
void DisplayClass::LCDflashAlarm(boolean TurnOn)
{
	//Routine to start/stop strobing backlight of LCD for alarm purposes. Will be turned off from main loop when key is pressed.
	// Sets up the 'soft' interupt using the timer.every object.  The actual turn on/off of the backlight is done by the
	// routine LCDflashAlarm, which is outside the Display class because of problems getting timer.every to point to a method within a class
	static int		LCDalarmContext;
	static boolean  LCDalarmOn = false;
	if (TurnOn)
	{
		if (LCDalarmOn == false)	//only turn alarm on if not currently on
		{
			//start the timer polling at intervals to blink cursor
			LCDalarmContext = Tmr.every((int)LCDstrobeRate, LCDflashAlarmtoggle, (void*)2);	// begin flashing backlight at LCDstrobeRate intervals. timer index = LCDalarmContext
			LCDalarmOn = true;	// set flag so we know we are using the soft interrupt
			lcd.noDisplay();	// turn off the backlight
			LCDbacklightOn = false;	// saves the state of the backlight and is used by LCDflashAlarm
		}
	}
	else
	{
		if (LCDalarmOn == true)	// only turn it off if it is currently on
		{
			Tmr.stop(LCDalarmContext);		// turn off the 'soft' interupt.  Index previously saved with call above
			LCDalarmOn = false;
			lcd.display();	// make sure the backlight is on
			LCDbacklightOn = true;	// save the state
		}
	}
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
			Serial.println(*(Darray + x));
			SDfile.println(*(Darray + x));	//	write to end of file
		}

		SDfile.close();		//close the file
		return true;
	}
	else
	{
		// if the file didn't open, print an error:
		Serial.print(F("error opening SDFile="));	Serial.println(filename);//change to error log in future
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
	Dname = "Save/" + Dname.substring(0, 8) + ".txt";
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
			//Serial.println(DisplayBuf[x]);	//debug
		}
		SDfile.close();		//close the file
		return x;			// return the number of lines read

	}
	else
	{
		// file didn't exist or didn't open
		Serial.print(F("error in ReadStringArraySD, did not find file or could not open it. SDFile="));	Serial.println(filename);//change to error log in future
		return 0;
	}
}
//------------------------------------------
void LCDflashAlarmtoggle(void* context)
{
	/* this routine exists outside of the Display class because we can't use some timer.every method within a class in the .pde implementation.  Compiler cannot resolve which routine to call.
	 Uses LCD backlight to get attention in an error situation.  It strobes the backlight of the LCD.  strobing is turned off when
	 any key is pressed.  This happens in the main loop.
	*/
	if (Display.LCDbacklightOn)
	{
		Display.LCDbacklightOn = false;	// flag that we are turning it off
		lcd.noDisplay();					// turn backlight off
	}
	else
	{
		Display.LCDbacklightOn = true;	// flag that we are turning it on
		lcd.display();					// turn backlight on
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
		Serial.print("timer started ");	//delete in future
		Serial.println(LS_KeyPollRate);
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
											//Serial.print("previous input=");	//debug
											//Serial.print(LS_prevInput);//debug
											//Serial.print("; LS_curInput=");//debug
											//Serial.println(LS_curInput);//debug
	x = LS_curInput - LS_prevInput;
	if (abs(x) >= LS_Key_Threshold)
	{
		//current analog value is more than threshold away from previous value then there has been a change, check via debounce
		//_debounce = true;
		// use the timer to delay for KeyDebounceDelay.  After KeyDebounceDelay, GetKey will be called.
		//Serial.println("setting up debounce");	//debug
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
											//Serial.print("in getKey, after debounce period. previous and current read, abs(cur-prev): ");
											//Serial.print(LS_prevInput); Serial.print(",  ");
											//Serial.print(LS_curInput); Serial.print(",   ");
											//Serial.println(abs(LS_curInput- LS_prevInput));
	if (abs(LS_curInput - LS_prevInput) <= LS_Key_Threshold)
	{
		//Key press is confirmed because there is no substantive change in key input value after debounce delay
		LS_KeyReady = true;		// flag to signal that key is ready to read.  Main loop checks for this.
								//Serial.println("in getkey determining which key is pressed"); //debug
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
		//Serial.print("current key ="); //debug
		//Serial.println(LS_curKey); //debug
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
		Serial.println("RTC polling started ");
	}
	else
	{
		Tmr.stop(SysTmPoleContext);		// stop polling
	}

}
//------------------------------------------
void GetSysTime(void* context)
{

	// routine to get system time and save in SysTm
	if (RTC.read(SysTm))
	{
		// set up string of system time in 24 hr format
		String tmpTime, strSnip;
		strSnip = String(SysTm.Hour);	// hr to String
										//Serial.print("hour="); Serial.print(strSnip);
		if (strSnip.length() == 1) strSnip = '0' + strSnip;	//hr needs to be 2 chr
		SysTmStr = strSnip + ":";
		strSnip = String(SysTm.Minute);
		//Serial.print(" Min="); Serial.print(strSnip);		
		if (strSnip.length() == 1) strSnip = '0' + strSnip;	//min needs to be 2 chr		
		SysTmStr = SysTmStr + strSnip + ":";	// add hrs
		strSnip = String(SysTm.Second);
		//Serial.print(" Sec="); Serial.println(strSnip);
		if (strSnip.length() == 1) strSnip = '0' + strSnip;	//sec needs to be 2 chr
		SysTmStr = SysTmStr + strSnip;	// add sec to complete system time string in 24 hr format as hh:mm:ss


										//set up string of system date as mm/dd/yyyy
		strSnip = String(SysTm.Month);	// month to String
										//Serial.print("Month="); Serial.print(strSnip);		
		if (strSnip.length() == 1) strSnip = '0' + strSnip;	//month needs to be 2 chr
		SysDateStr = strSnip + "/";
		strSnip = String(SysTm.Day);
		//Serial.print(" day="); Serial.print(strSnip);		
		if (strSnip.length() == 1) strSnip = '0' + strSnip;	//day needs to be 2 chr
		SysDateStr = SysDateStr + strSnip + "/";
		SysDateStr = SysDateStr + tmYearToCalendar(SysTm.Year);	// complete SysDateStr as mm/dd/yyyy
																//Serial.print(" year="); Serial.println(SysTm.Year);	

		sysDOWstr = Display.ProgMemLU(DisplayDOW, SysTm.Wday, 3); // DisplayDOW[SysTm.Wday];	//set day of week string. SysTm.Wday is int where 1=sunday
																  //Serial.println ("SysTmStr=" + SysTmStr + ", SysDateStr=" + SysDateStr + ", SysDOW=" + sysDOWstr);
																  /*
																  Serial.print("Ok, Time = ");
																  Serial.print(SysTm.Hour);
																  Serial.write(':');
																  Serial.print(SysTm.Minute);
																  Serial.write(':');
																  Serial.print(SysTm.Second);
																  Serial.print(", Date (D/M/Y) = ");
																  Serial.print(SysTm.Day);
																  Serial.write('/');
																  Serial.print(SysTm.Month);
																  Serial.write('/');
																  Serial.print(tmYearToCalendar(SysTm.Year));
																  Serial.println();
																  */

	}
	else
	{
		if (RTC.chipPresent())
		{
			Serial.println("The DS1307 is stopped.  Please run the SetTime");
		}
		else
		{
			ErrorLog("DS1307 read error!  Please check the circuitry.");
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

//--------------------------------------------------------Set Up --------------------------------------------------

void setup()
{
	String Tst;
	Serial.begin(9600);
	boolean tstBool;
	int tmp1 = 0, tmp2 = 0, tmp3 = 0;
	String str1 = "string one", str2 = "string two", str3 = "string three";

	Display.TimeToDim = now() + DimTimeout;	// set time to dim display if no key is hit

	//initialize the SD library	
	pinMode(53, OUTPUT);	//pin 53 = CS 

	if (!SD.begin(53))
	{
		Serial.println(F("SD initialization failed!"));	// change to log in future
	}
	else
	{
		Serial.println(F("SD initialized ok."));	//change to log in future
	}

	/*testing
	Tst="SetRTC_ui";
	tstBool=WriteStringArraySD(Tst,5,SetRTC_ui);
	Serial.println(F("calling ReadStringArray"));
	tmp1=ReadStringArraySD(Tst,5);
	Serial.print(F("returned from ReadStringArraySD=")); Serial.println(tmp1);
	Tst="TempSensor_ui";
	tstBool=WriteStringArraySD(Tst,2,TempSensor_ui);
	Serial.println(F("calling ReadStringArray"));
	tmp1=ReadStringArraySD(Tst,2);
	Serial.print(F("returned from ReadStringArraySD=")); Serial.println(tmp1);
	*/


	KeyPoll(true);		// Begin polling the keypad S
	SysTimePoll(true);	// begin to poll the Real Time Clock to get system time into SysTm

	Display.DisplayStartStop(true);		// indicate that menu processing will occur. Tells main loop to pass key presses to the Menu
	Display.DisplaySetup(true, true, "Main_UI", 4, DisplayBuf); // Prepare main-UI display array and display the first line, mode is read only.

	/*
	set the RTC with starting point: mon 9/7/2015 @ 16:15
	*/

	SysTm.Month = 9;
	SysTm.Day = 7;
	SysTm.Year = (2015 - 1970);
	SysTm.Hour = 16;
	SysTm.Minute = 15;
	SysTm.Second = 27;
	SysTm.Wday = 2;

	if (RTC.write(SysTm))		//  write time to RTC, false if fails
	{
		Serial.println(F("RTC initialized OK to mon 9/7/2015 @ 16:15 "));
	}
	else
	{
		ErrorLog("RTC initialization in SetUp failed");
	}


}


//--------------------------------------------------------main loop -----------------------------------------------

void loop()
{
	Tmr.update();
	if (ReadKey() != NO_KEY)
	{
		// key was pressed and debounced result is ready for processing
		//Serial.println(LS_curKey);
		if (!Display.LCDbacklightOn)
		{
			//backlight is off and key was hit, so turn on display.
			//The display could be off because it was dimmed (because keypad not in use), or because of LCD strobe alarm.
			// In either case, we want to turn it on and reset timers/alarm. 
			lcd.display();	// turn on display
			Display.TimeToDim = now() + DimTimeout;	//set next time to dim
			Display.LCDbacklightOn = true;			// flag that the backlight is one
			Display.LCDflashAlarm(false);			// this will turn off the flash alarm if it was on
		}

		Display.ProcessDisplay(LS_curKey);	// routine will process key only if DisplayInUse==true, global set by DisplayStartStop()	
	}//if (ReadKey() != NO_KEY)

	// check if time to dim the LCD because keypad is not in use
	if (Display.LCDbacklightOn)
	{
		if (now() > Display.TimeToDim)
		{
			lcd.noDisplay();				// turn off display
			Display.LCDbacklightOn = false;	// indicate LCD is dimmed
		}
	}

	if (Display.DisplayUserMadeSelection == true)
	{
		//If here, then user has made a Display selection, passing results through DisplayName, DisplayLineName, DisplaySelection
		if (Display.DisplayName == "Main_UI")
		{

			//if here then processing this display array
			//{"SetUp,menu,---Set Up---,RTC   Temp_sensor  Flow_sensor",
			//"Row_2,menu,---2nd Display---,run2   set_time2   sensor_tst2   calibrate2",
			//"Row_3,menu,---3rd Display---,run3   set_time3   sensor_tst3   calibrate3",
			//"Row_4,menu,---4th Display---,Yes   No"}

			if (Display.DisplayLineName == "SetUp")
			{
				//Serial.print("DisplaySelection=|"); Serial.print(Display.DisplaySelection); Serial.println("|");
				if (Display.DisplaySelection == "RTC")
				{
					boolean	rslt;
					Display.DisplaySetup(false, true, "SetRTC_ui", 5, DisplayBuf); // Prepare SetRTC_ui display array, display the first line, mode is read/write, retrieve SetRTC_ui from the SD card

																				   //RTC is read every second and sets strings for day of week, time, and date
																				   //modify the display lines in the SetRTC_ui array

					rslt = Display.DisplayGetSetDate(&SysDateStr, "Date", true);	// replace the date string in display line named 'Date' in the SetRTC_up array
					rslt = Display.DisplayGetSetTime(&SysTmStr, "Time", true);		// replace the time string in display line named 'Time'.  need to clip off sec
					rslt = Display.DisplayGetSetDOW(&sysDOWstr, "DOW", true);		// replace the day of week string in display line named 'DOW'
																					//Serial.println(F("main loop, clicked RCT")); for (int z=0; z<5; z++) {Serial.println(DisplayBuf[z]);}	//debug

				}
				else
				{
					if (Display.DisplaySelection == "Temp_sensor")
					{
						Serial.println(F("Main_UI-->SetUp-->Temp_sensor"));	//debug
						Display.DisplaySetup(false, true, "tempsens", 4, DisplayBuf); // put up entry screen for temperature sensor display array and display the first line
					}
					else
					{
						if (Display.DisplaySelection == "Flow_sensor")
						{
							Serial.println(F("Main_UI-->SetUp-->Flow_sensor"));	//debug
						}
						else
						{
							//error, should have identified the DisplaySelection
							ErrorLog("error processing Main_UI, Setup: did not match DisplaySection");
						}

					}

				}
				goto EndDisplayProcessing; //exit processing Display	

			}

			//if here then entry not processed, which is an error
			ErrorLog("error processing Main_UI: unrecognized DisplayLineName");
		}

		if (Display.DisplayName == "SetRTC_ui")
		{

			//if here then processing SetRTC_ui[5]=
			//{"Text,text,---RTC Setup---,Used to view/update the date, time, and day of week settings",
			//"Date,m--d----yy--U-D,---RTC Date---,08/01/2015  U/D",
			//"Time,H--M---U-D,---RTC Time---,13:27  U/D",
			//"DOW,-a---a---U-D,--RTC DOW--,-Mon-Tue-U/D",
			//"action,menu,---Action---,Update   Cancel"}

			Serial.println(Display.DisplayLineName);
			if (Display.DisplayLineName == "action")
			{
				if (Display.DisplaySelection == "Update")
				{
					Serial.println(F("RTC_ui-->action-->update"));
					//user wants to update system time with their changes made via display array 'RTC_ui'. Note that display 'RTC_ui' insures valid date, time, and DOW.  However, there is no check if DOW goes with date.
					// will set the RTC and the soft interrupt will set the system time strings with the next read of RTC 
					Serial.print(F("Free Scram =")); Serial.println(getFreeSram());


					String TmpSysTimeStr, TmpSysDateStr, tmpTime, tmpStr;
					int	tmpVal;
					//Serial.print("SysDateStr=" ); Serial.print(SysDateStr); Serial.print(", sysTimeStr="); Serial.print(SysTmStr); Serial.print("' SysDOW="); Serial.println(sysDOWstr);
					//Serial.println("SysDateStr=" + SysDateStr + ", sysTimeStr=" + SysTmStr + "' SysDOW=" + sysDOWstr);
					Display.DisplayGetSetDate(&TmpSysDateStr, "Date", false);					// read the date string from display line named 'Date' in the SetRTC_up array
																								//Serial.println("new SysDateStr=" + TmpSysDateStr);
					tmpStr = TmpSysDateStr.substring(0, 2);		//get 2 digit month
																//Serial.println("month from RTC_ui="+tmpStr);
					SysTm.Month = tmpStr.toInt();
					//Serial.print("month int="); Serial.println(tmpVal);
					tmpStr = TmpSysDateStr.substring(3, 5);	//get 2 digit day
															//Serial.println("day from RTC_ui="+tmpStr);
					SysTm.Day = tmpStr.toInt();
					//Serial.print("Day int="); Serial.println(tmpStr.toInt());				
					tmpStr = TmpSysDateStr.substring(6);	//get 4 digit yr
															//Serial.println("year from RTC_ui="+tmpStr);
					SysTm.Year = tmpStr.toInt() - 1970;		//year has offset from 1970	
															//Serial.print("year int="); Serial.println(SysTm.Year);	
															//Serial.print("Free Scram after date ="); Serial.println(getFreeSram());

					Display.DisplayGetSetTime(&SysTmStr, "Time", false);					// read the time string from display line named 'Time'. 
																							//Serial.println("new time string=" +SysTmStr);
					tmpStr = SysTmStr.substring(0, 2);										// get	2 digit Hr
																							//Serial.println("hrs from RTC_ui=" + tmpStr);
					SysTm.Hour = tmpStr.toInt();
					tmpStr = SysTmStr.substring(3, 5);										// get 2 digit min
																							//Serial.println("min from RTC_ui=" + tmpStr);
					SysTm.Minute = tmpStr.toInt();
					SysTm.Second = 0;
					//Serial.print("Free Scram after time ="); Serial.println(getFreeSram());				
					// no seconds in RTC_ui						
					Display.DisplayGetSetDOW(&sysDOWstr, "DOW", false);						// read the day of week string from display line named 'DOW'

					for (tmpVal = 0; tmpVal<7; tmpVal++)
					{
						tmpStr = Display.ProgMemLU(DisplayDOW, tmpVal, 3);	//get the DOW string from storage
						if (tmpStr == sysDOWstr) break;
					}
					//tmpVal++;		// increment because Sunday=1 and index for sunday=0
					//Serial.print	("RTC_ui DOW index="); Serial.println(tmpVal);
					SysTm.Wday = tmpVal;
					//Serial.print("Free Scram after DOW ="); Serial.println(getFreeSram());


					if (!RTC.write(SysTm))		//  write time to RTC, false if fails
					{
						ErrorLog("RTC write failed");
					}
					//Serial.print("Free Scram after set time ="); Serial.println(getFreeSram());

				}
				else
				{
					if (Display.DisplaySelection == "Cancel")
					{
						Serial.println(F("RTC_ui-->action-->Cancel"));
						Display.DisplaySetup(true, true, "Main_UI", 4, DisplayBuf); // user wants to cancel, so return to main-UI display array and display the first line, mode is read only.
					}
					else
					{
						ErrorLog("error processing setRTC_ui-->action: unrecognized DisplaySelection");
						Serial.print(F("error processing setRTC_ui-->action: unrecognized DisplaySelection=")); Serial.println(Display.DisplaySelection);	//debug
					}

				}

			}
			else
			{
				ErrorLog("error processing RTC_ui: unrecognized DisplayLineName");
				Serial.print(F("error processing setRTC_ui-->action: unrecognized DisplayLineName=")); Serial.println(Display.DisplayLineName);	//debug
				Serial.print((F("length="))); Serial.println(Display.DisplayLineName.length());

			}
			Display.DisplaySetup(false, true, "Main_UI", 4, DisplayBuf); // Return to main-UI display array and display the first line
			goto EndDisplayProcessing; //exit processing Display	
		}

		if (Display.DisplayName == "tempsens")
		{
			/*  Text1, text, -- - Tmp Sensor-- - , Used for actions effecting all temp sens
				sensNum, U - D--# - O - Sens--#, --# of Sensors--, U / D  # - O - Sens = #
				rate, U - D-------- - ### - , --Sample Rate--, U / D   Every 060s
				action, menu, -- - Action-- - , Update  Cancel  Individual-Setup  Discover  Test
			*/
			if (Display.DisplayLineName == "action")
			{
				if (Display.DisplaySelection == "Update") 
				{
					Serial.println(F("TempSens-->Action-->Update"));	//debug
				}
				else
				if(Display.DisplaySelection == "Cancel")
				{
					Display.DisplaySetup(false, true, "Main_UI", 4, DisplayBuf); // Return to main-UI display array and display the first line
				}
				else
				if (Display.DisplaySelection == "Individual-Setup")
				{
					Display.DisplaySetup(false, true, "TSens1", 6, DisplayBuf); // Put up first temp sens setup display array and display the first line
				}
				else
				if(Display.DisplaySelection=="Discover")
				{
					Serial.println(F("TempSens-->Action-->Discover"));	//debug
				}
				else
				if (Display.DisplaySelection == "Test")
				{
					Serial.println(F("TempSens-->Action-->Test"));	//debug
				}
				else
				{
					ErrorLog("error processing TempSens-->action: unrecognized DisplaySelection");
					Serial.print(F("error processing TempSens-->action: unrecognized DisplaySelection=")); Serial.println(Display.DisplaySelection);
				}
			}
			goto EndDisplayProcessing; //exit processing Display
		}

		if (Display.DisplayName == "TSens1")
		{
			/*if here then processing setup display array for temperature senser 1
			Text1,text,---Tmp Sensor 1---,Parameter set up for temperature sensor 1
			IsOn,U-D---------C-,-On/Off status-,U/D Is on?  Y
			tempAddr,U-D----CCCCCCCC-,----Address----,U/D    00000000
			tempName,U-D--CCCCCCCCCC,----Name Str----,U/D  Pond Temp
			handle,U-D--CCCCCCCCCC,---Cloud Str---,U/D  ???????????
			action,menu,---Action---,Next  Update-#1-Next   Cancel
			*/
			if (Display.DisplayLineName == "action")
			{
				if (Display.DisplaySelection == "Next")
				{
					Serial.println(F("TSens1-->Action-->Next"));	//debug
					Display.DisplaySetup(false, true, "TSens2", 6, DisplayBuf);		//don't save anything and put up TSens2 dispaly array																			
				}
				else
				if (Display.DisplaySelection == "Update-#1-Next")
				{
					// read variables from display array TSens1 into temp sensor variables
					if (!(Display.DisplayWriteSD())) ErrorLog("error writing TSens1 to SD");	//save settings on SD
					Display.DisplaySetup(false, true, "TSens2", 6, DisplayBuf); // Put up 2nd temp sens setup display array and display the first line
					Serial.println(F("TSens1-->Action-->Update-#1-Next"));	//debug
				}
				else
				if (Display.DisplaySelection == "Cancel")
				{
					Display.DisplaySetup(true, true, "Main_UI", 4, DisplayBuf); // Return to main-UI display array and display the first line
					goto EndDisplayProcessing; //exit processing Display	
				}
				else
				{
					ErrorLog("error processing TSens1-->action: unrecognized DisplaySelection");
					Serial.print(F("error processing TSens1-->action: unrecognized DisplaySelection=")); Serial.println(Display.DisplaySelection);
				}
			}
			goto EndDisplayProcessing; //exit processing Display
		}

		//Serial.println("----------------------------------------------------------");
		//Serial.println(Display.DisplayName);
		//Serial.println(Display.DisplayLineName	);
		//Serial.println(Display.DisplaySelection);
		//Serial.println("");

		ErrorLog("error, unrecognized Display.DisplayName");	//should have recognized displayName
		Serial.print(F("error, unrecognized Display.DisplayName=")); Serial.println(Display.DisplayName);
	}	// end DisplayUserMadeSelection=true
EndDisplayProcessing:	//target of goto. common exit for processing display array entries for object Display

	Display.DisplayUserMadeSelection = false;		// reset flag because we are processing the response
} // end main loop
