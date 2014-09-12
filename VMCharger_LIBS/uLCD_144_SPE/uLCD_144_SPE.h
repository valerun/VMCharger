/*
uLCD_144_SPE Arduino Library

Copyright (c) 2011 Valery Miftakhov.  All right reserved.
Version 1.0, April 8, 2011

See README.txt for usage & more info
*/


//==================================== SCREEN FUNCTION LIBRARY ========================
#ifndef uLCD_144_SPE_h
#define uLCD_144_SPE_h

// without this include, nothing works 
// #include <Arduino.h> // only for 1.0+ Arduino version
#include "WProgram.h"
#include <avr/pgmspace.h>

class uLCD_144_SPE
{
  private:
    int waitAck();
	int isAlive_;
  public:
    uLCD_144_SPE(int);
    byte getMSB(byte, byte, byte);
    byte getLSB(byte, byte, byte);
	int isAlive();
	void wrapStr(const char*, char*, int, const char*);
    void clrScreen();
    void setOpacity(int);
    void setYspacing(int);
    void setFont(int);
	void moveCursor(int, int);
	void setFGcolor(byte, byte, byte);
    void printStr(int, int, int, byte, byte, byte, const char*);
    void printStr(int, int, int, byte, byte, byte, const prog_char*, int); // has to be prog_char in pre-1.0, __FlashStringHelper after
};
#endif
//================================ END SCREEN FUNCTION LIBRARY ========================
