#include "uLCD_144_SPE.h"


//==================================== SCREEN FUNCTION LIBRARY ========================
//-------------- define all function members ----------    
uLCD_144_SPE::uLCD_144_SPE(int baud) {
  Serial.flush();
   
  delay(3000);
  
  Serial.begin(baud);

   // clr screen here - do NOT call function as we need to get an Ack here 
  Serial.print((char)0xFF);
  Serial.print((char)0xD7);

  isAlive_=waitAck();
}

// process color data
byte uLCD_144_SPE::getMSB(byte red, byte green, byte blue) {
  return red*8 + green/8;
}
byte uLCD_144_SPE::getLSB(byte red, byte green, byte blue) {
  return (green & 0x7)*32 + blue;
}
int uLCD_144_SPE::waitAck() {
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

int uLCD_144_SPE::isAlive() {
  return isAlive_;
}

void uLCD_144_SPE::clrScreen() {
  Serial.print((char)0xFF);
  Serial.print((char)0xD7);
  waitAck();
}  

void uLCD_144_SPE::setYspacing(int pixels) {
  Serial.print((char)0xFF);
  Serial.print((char)0x79);
  Serial.print((char)0x00);
  Serial.print((char)pixels);

  waitAck();
}
  
void uLCD_144_SPE::setOpacity(int opacity) {
  // 0 = transparent, 1 = opaque
  Serial.print((char)0xFF);
  Serial.print((char)0x77);
  Serial.print((char)0x00);
  Serial.print((char)opacity);

  waitAck();
}

void uLCD_144_SPE::moveCursor(int col, int row) {
  Serial.print((char)0xFF);
  Serial.print((char)0xE4);
  Serial.print((char)0x00);
  Serial.print((char)row);
  Serial.print((char)0x00);
  Serial.print((char)col);
  
  waitAck();
}

void uLCD_144_SPE::setFGcolor(byte red, byte green, byte blue) {
  Serial.print((char)0xFF);
  Serial.print((char)0x7F);
  Serial.print((char)getMSB(red, green, blue));
  Serial.print((char)getLSB(red, green, blue));
  
  waitAck();
}

void uLCD_144_SPE::setFont(int font) {
  Serial.print((char)0xFF);
  Serial.print((char)0x7D);
  Serial.print((char)0x00);
  Serial.print((char)0x00);
  
  waitAck();
}

void uLCD_144_SPE::printStr(int col, int row, int font, byte red, byte green, byte blue, const char *str) {
  moveCursor(col, row);
  setFont(0);
  setFGcolor(red, green, blue);
  setYspacing(2);
  
  Serial.print((char)0x00);
  Serial.print((char)0x06);
  Serial.print(str);
  Serial.print((char)0);
  waitAck();
}
void uLCD_144_SPE::printStr(int col, int row, int font, byte red, byte green, byte blue, const prog_char *str, int type) { // has to have a different sig from the above function
  moveCursor(col, row);
  setFont(0);
  setFGcolor(red, green, blue);
  setYspacing(2);
  
  Serial.print((char)0x00);
  Serial.print((char)0x06);
  Serial.print(str);
  Serial.print((char)0);
  waitAck();
}

void uLCD_144_SPE::wrapStr(const char *s, char *sto, int lineSize, const char *prefix) {
    const char *head = s;
    int pos, lastSpace;
    pos = lastSpace = 0;

    while(head[pos]!=0) {
        int isLf = (head[pos]=='\n');
        if (isLf || pos==lineSize) {
            if (isLf || lastSpace == 0) { lastSpace = pos; } // just cut it
            if (prefix!=NULL) { sprintf(sto, "%s%s", sto, prefix); }
            while(*head!=0 && lastSpace-- > 0) { sprintf(sto, "%s%c", sto, *head++); }
            sprintf(sto, "%s\n", sto);
            if (isLf) { head++; } // jump the line feed
            while (*head!=0 && *head==' ') { head++; } // clear the leading space
            lastSpace = pos = 0;
        } else {
            if (head[pos]==' ') { lastSpace = pos; }
            pos++;
        }
    }
    printf("%s\n", head);
}

//================================ END SCREEN FUNCTION LIBRARY ========================
