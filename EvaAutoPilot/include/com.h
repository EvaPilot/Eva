//*----------------------------------------------------------------------------
//*      ATMEL Microcontroller Software Support  -  ROUSSET  -
//*----------------------------------------------------------------------------
//* The software is delivered "AS IS" without warranty or condition of any
//* kind, either express, implied or statutory. This includes without
//* limitation any warranty or condition with respect to merchantability or
//* fitness for any particular purpose, or against the infringements of
//* intellectual property rights of others.
//*----------------------------------------------------------------------------
//* File Name           : com.h
//* Object              : 
//*
//* 1.0 27/03/03 HIi    : Creation
//*----------------------------------------------------------------------------
#ifndef com_h
#define com_h

#define AT91C_CB_SIZE	40			// size of the console buffer 

//* Escape sequences */
#define ESC				\033
#define CLRSCREEN		"ESC[2J"	//\033 = ESC in octal
#define ClEARLINE 		"ESC[K"		// Clear line, from cursor position to the right most position of line 

//* Cursor Movement */
#define MOVEUP(num) 	"ESC[numA" 		// Move the cursor up num positions  
#define MOVEDOWN(num)   "ESC[numB" 		// Move the cursor down num positions  
#define MOVERIGHT(num)  "ESC[numC" 		// Move the cursor right num positions  
#define MOVELEFT(num) 	"ESC[numD"  	// Move the cursor left num positions  
#define MOVETO(row,col) "ESC[row;colH" 	// Move the cursor to the (col, row) position. Note that the row comes before column; that is, y comes before x. Either col or row can be omitted. Row and column both start with "1," not zero. (1, 1) corresponds to the top-left corner of the screen.  

//* Character Mode */

#define CHANGE_CHAR_MODE(attr) "ESC[attrm" 	//Change the character mode with attribute attr. The attributes are numbers listed below.  

#define ALL_ATTRIB_OFF 		0		// All attributes turned off. (Except for foreground and background color).  
#define HIGH_INTENSITY		1 		// Bold.  
#define LOW_INTENSITY		2		// Normal.  
#define UNDERLINE			4		// Underline font.  
#define BLINK				5		// Blinking font.  
#define RAPID_BLINK			6 		// Works only on some systems.  
#define REVERSE_VIDEO		7		// Swapping the foreground color and the background color.  
#define FOREGROUND_BLACK 	30		// Black.  
#define FOREGROUND_RED		31		// Red.  
#define FOREGROUND_GREEN	32 		// Green.  
#define FOREGROUND_YELLOW	33 		// Yellow.  
#define FOREGROUND_BLUE		34 		// Blue.  
#define FOREGROUND_MAGENTA	35 		// Magenta.  
#define FOREGROUND_CYAN		36 		// Cyan.  
#define FOREGROUND_WHITE 	37 		// White.  
#define BACKGROUND_BLACK 	40 		// Black.  
#define BACKGROUND_RED		41 		// Red.  
#define BACKGROUND_GREEN 	42 		// Green.  
#define BACKGROUND_YELLOW 	43 		// Yellow.  
#define BACKGROUND_BLUE 	44 		// Blue.  
#define BACKGROUND_MAGENTA 	45 		// Magenta.  
#define BACKGROUND_CYAN 	46 		// Cyan.  
#define BACKGROUND_WHITE	47 		// White.  

struct __FILE { 
    int handle; 
    /* Whatever you need here (if the only files you are using
       is the stdoutput using printf for debugging, no file
       handling is required) */ 
};


extern char message[AT91C_CB_SIZE];
extern void AT91F_ClrScr(void);
extern int AT91F_ReadLine (const char *const prompt, char *console_buffer);
extern void AT91F_WaitKeyPressed(void);

#endif