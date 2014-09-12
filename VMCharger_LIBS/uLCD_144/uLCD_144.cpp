#include "uLCD_144.h"


//==================================== SCREEN FUNCTION LIBRARY ========================
//-------------- define all function members ----------    
uLCD_144::uLCD_144(int baud) {
  Serial.begin(baud);
  // display requires some time to init before sending auto-baud command
  delay(1000);
  Serial.print(0x55, BYTE);
  isAlive_=waitAck();
}

// process color data
byte uLCD_144::getMSB(byte red, byte green, byte blue) {
  return red*8 + green/8;
}
byte uLCD_144::getLSB(byte red, byte green, byte blue) {
  return (green & 0x7)*32 + blue;
}
int uLCD_144::waitAck() {
  int x=0;  
  int z=0;
  do {
    x=Serial.read();
    delay(5);
    z++;
  } while(x!=0x06 && z<100); // wait for screen acknowledgement or 100 cycles
  
  if(x==0x06) return 1; // LCD is there
  // LCD not there
  return 0;
}

int uLCD_144::isAlive() {
  return isAlive_;
}

void uLCD_144::setContrast(int contrast) {
  // range from 0 to 0x0f
  Serial.print(0x59,BYTE);
  Serial.print(0x02,BYTE);
  Serial.print(contrast,BYTE);
  waitAck();  
}

void uLCD_144::clrScreen() {
  Serial.print(0x45,BYTE);
  waitAck();
}  
  
void uLCD_144::setBgColor(byte red, byte green, byte blue) {
  Serial.print(0x42, BYTE); // set screen color
  Serial.print(getMSB(red, green, blue), BYTE);
  Serial.print(getLSB(red, green, blue), BYTE);
  waitAck();
}

void uLCD_144::setPenSize(int size) {
  // 0 = solid, 1 = wire
  Serial.print(0x70,BYTE);
  Serial.print(size,BYTE);
  waitAck();
}

void uLCD_144::setFont(int font) {
  // 0 = small, 1 = med, 2 = large
  Serial.print(0x46,BYTE);
  Serial.print(font,BYTE);
  waitAck();
}

void uLCD_144::setOpacity(int opacity) {
  // 0 = transparent, 1 = opaque
  Serial.print(0x4f,BYTE);
  Serial.print(opacity,BYTE);
  waitAck();
}

// draw pixel
void uLCD_144::drawPixel(int x, int y, byte red, byte green, byte blue) {
  Serial.print(0x50,BYTE);
  Serial.print(x, BYTE);
  Serial.print(y, BYTE);
  Serial.print(getMSB(red, green, blue), BYTE);
  Serial.print(getLSB(red, green, blue), BYTE);
  waitAck();
}

// draw circle
void uLCD_144::drawCircle(int x, int y, int r, byte red, byte green, byte blue) {
  Serial.print(0x43,BYTE);
  Serial.print(x, BYTE);
  Serial.print(y, BYTE);
  Serial.print(r, BYTE);
  Serial.print(getMSB(red, green, blue), BYTE);
  Serial.print(getLSB(red, green, blue), BYTE);
  waitAck();
}

void uLCD_144::printStr(int col, int row, int font, byte red, byte green, byte blue, const char *str) {
  Serial.print(0x73,BYTE);
  Serial.print(col, BYTE);
  Serial.print(row, BYTE);
  Serial.print(font, BYTE);
  Serial.print(getMSB(red, green, blue), BYTE);
  Serial.print(getLSB(red, green, blue), BYTE);
  Serial.print(str);
  Serial.print(0, BYTE);
  waitAck();
}

void uLCD_144::printGRStr(int x, int y, int font, byte red, byte green, byte blue, int w, int h, const char *str) {
  Serial.print(0x53,BYTE);
  Serial.print(x, BYTE);
  Serial.print(y, BYTE);
  Serial.print(font, BYTE);
  Serial.print(getMSB(red, green, blue), BYTE);
  Serial.print(getLSB(red, green, blue), BYTE);
  Serial.print(w, BYTE);
  Serial.print(h, BYTE);
  Serial.print(str);
  Serial.print(0, BYTE);
  waitAck();
}

void uLCD_144::drawLine(int  x1, int y1, int x2, int y2, byte red, byte green, byte blue) {
  Serial.print(0x4C,BYTE);
  Serial.print(x1, BYTE);
  Serial.print(y1, BYTE);
  Serial.print(x2, BYTE);
  Serial.print(y2, BYTE);
  Serial.print(getMSB(red, green, blue), BYTE);
  Serial.print(getLSB(red, green, blue), BYTE);
  waitAck();
}

void uLCD_144::drawRect(int x1, int y1, int x2, int y2, byte red, byte green, byte blue) {
  Serial.print(0x72,BYTE);
  Serial.print(x1, BYTE);
  Serial.print(y1, BYTE);
  Serial.print(x2, BYTE);
  Serial.print(y2, BYTE);
  Serial.print(getMSB(red, green, blue), BYTE);
  Serial.print(getLSB(red, green, blue), BYTE);
  waitAck(); 
}
//================================ END SCREEN FUNCTION LIBRARY ========================
