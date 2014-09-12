/*
uLCD_144 Arduino Library

Copyright (c) 2011 Valery Miftakhov.  All right reserved.
Version 1.0, April 8, 2011

See README.txt for usage & more info
*/


// without this include, nothing works 
#ifndef uLCD_144_includes
  #define uLCD_144_includes
  #include "WProgram.h"
#endif


//==================================== SCREEN FUNCTION LIBRARY ========================
#ifndef uLCD_144_h
#define uLCD_144_h

class uLCD_144
{
  private:
    int waitAck();
	int isAlive_;
  public:
    uLCD_144(int);
    byte getMSB(byte, byte, byte);
    byte getLSB(byte, byte, byte);
	int isAlive();
    void setContrast(int);
    void clrScreen();
    void setBgColor(byte, byte, byte);
    void setPenSize(int);
    void setFont(int);
    void setOpacity(int);
    void drawPixel(int, int, byte, byte, byte);
    void drawCircle(int, int, int, byte, byte, byte);
    void printStr(int, int, int, byte, byte, byte, const char*);
    void printGRStr(int, int, int, byte, byte, byte, int, int, const char*);
    void drawLine(int, int, int, int, byte, byte, byte);
    void drawRect(int, int, int, int, byte, byte, byte);
};
#endif
//================================ END SCREEN FUNCTION LIBRARY ========================
