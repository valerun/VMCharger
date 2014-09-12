/*
EMotorWerks SmartCharger-12000 - a 12kW+ charging system

DIY charger inspired by the work of SimonRafferty & jackbauer on DIYelectriccar.com:
http://www.diyelectriccar.com/forums/showthread.php/200-build-your-own-intelligent-charger-36627.html. 

DETAILED FORUM DISCUSSION OF THIS DESIGN IS AT 
http://www.diyelectriccar.com/forums/showthread.php/10kw-60a-diy-charger-open-source-59210p39.html

Controller: Arduino Pro Mini 5V (based on a ATmega328P microcontroller)

------ Startup: 
* check for LCD presence. if none (programming button in the programming state), launch in the serial-controlled mode 
* 2 timeouts - one 5 sec for config, one 10 sec for power setting. can be interrupled by any button
* check mains voltage. If 110, limit power to ~1.5kW
------ Charging (CV or CC):
* modulate duty cycle per PID loop calculations running at 250Hz (double the line frequency)
* break when exit condition satisfied or stop / pause commands received

============================== SERIAL COMMAND SYNTAX ===============================
M,ccc,vvv,sss,E - start charger from 'READY' state with ccc CC point and vvv CV point
                  charger will echo settings. make sure they are what you sent,
                  sss is a checksum = (ccc + vvv) % 1000 
M,001,000,001,E - stop charge
=================== END SERIAL COMMAND SYNTAX ======================================
============================== SERIAL STATUS REPORTING =============================
every 100ms or so, the charger will report its status. 
Generally a dump of the critical charging parameters in the following format:
'M,D0,C965,V334,T-68,O1,R0,E' - [D]uty 0%, output [C]urrent 96.5A, output 
[V]oltage 334V, heatsink [T]emp -68C, [O]utput charge 0.1AH, [R]untime 0 minutes
=================== END SERIAL STATUS REPORTING FORMAT =============================

************************ HARDWARE MODS REQUIRED *********************************
********************** TO RUN V14+ ON EMOTORWERKS *******************************
****************************** CHARGERS! ****************************************
* change RC filter on maxC line (pin 10 Arduino) - ensure C is no more than 1uF
* ensure C<=0.1uF on all current and voltage readings
*********************************************************************************

Original version created Jan 2011 by Valery Miftakhov, Electric Motor Werks, LLC & Inc. 
All rights reserved. Copyright 2014

This software is released as open source and is free for any personal use.
Commercial use prohibited without written approval from the Author and / or Emotorwerks
Absolutely no warranty is provided and no guarantee for fit for a specific purpose
*/

#include <avr/interrupt.h> 
#include <avr/pgmspace.h>
#include <MemoryFree.h>
// need this to remap PWM frequency
#include <EEPROM.h>
#include "EEPROM_VMcharger.h"
#include <TimerOne.h>

//----------- DEBUG switch - careful, this disables many safety features...---------------
// #define DEBUG0 // ad-hoc stuff - targetC at the moment
// #define DEBUG1 // additional printouts / delays / etc.
// #define DEBUG2 // even more printouts etc
//----------------------------------------------------------------------------------------

//============================== MAIN SWITCHES FOR EMotorWerks chargers =================
#define VerStr "V14.4"
//#define SmartCharge // 12kW chargers
//#define PFC // is this a PFC unit? no need to have this when QuickCharge is selected

// #define QuickCharge // 25kW chargers - ONLY FOR AC

 #define DCDC // module is used in DC-DC mode (buck or boost)
 #define DCDC_BUCK // the voltage readings are swapped

// #define SLOWPID // for universal voltage non-CHAdeMO units, slow down PID; may also be needed for any PFCDirect units due to time constant of the PFC circuit
// ===================================================================================================


// ================= SUB-SWITCHES DEFAULT VALUES FOR EMotorWerks chargers ========================
// by-version defaults
#ifdef SmartCharge // the latest version of the PFC version of SmartCharge-12000 12kW chargers
  #define OUTC_SENSOR_Allegro_100U // default is Allegro_100U
  #define A7520_V // using A7520 optoisolation for outV sensing? (as opposed to ISO124)
  #define PC817 // mains voltage sensing based on a crude regular opto (V13 boards)
#endif
#ifdef QuickCharge // the latest version of the AC-FED PFCDirect or QuickCharge-25000 25kW chargers
  #define OUTC_SENSOR_Tamura_150B // Tamura_150B is default for PFCdirect 25kW units only V3 (after Dec 2 2013), Tamura_50B for high-voltage units (800V)
  #define A7520_V // using A7520 optoisolation for outV sensing? (as opposed to ISO124)
  #define A7520_mV // using A7520 optoisolation for mV sensing? (as opposed to ISO124)
  #define PFC // all QuickCharge units are PFC
  #define PFCdirect // is this a PFCDirect 25kW unit?
  #define NEG_CSENSE // in post-Oct'13 PFCdirect units, current runs in opposite direction to make 3.3V logic compatible
#endif
#ifdef DCDC
//  #define OUTC_SENSOR_Tamura_600B // Tamura_600B - this is for low-voltage DC-DC only
  #define OUTC_SENSOR_Tamura_150B // Tamura_150B is default for high-voltage DC-DC
  #define A7520_V // using A7520 optoisolation for outV sensing? (as opposed to ISO124)
  #define A7520_mV // using A7520 optoisolation for mV sensing? (as opposed to ISO124)
  #define DCinput // is this being connected to the DC input or AC? matters for input voltage sense 
  #define PFCdirect
  // #define LOWVOLTAGE // low voltage = lower PWM freq
  
  // in all DCDC units, current runs in opposite direction (for BUCK units, this means that we run the output
  // wire through the same sensor as normally used on the high side)
  #define NEG_CSENSE 
  
  // #define INC_ASOUT // in early DCDC_BUCK units, input sensor is used as output sensor; also, need to spec positive current direction in this case
#endif

// universal defines
#define LCD_SPE // are we using the SPE version of the LCD (shipped after October 2013)
#ifndef DCDC
  // #define drop110power // reduce power to ~1.5kW when connected to 110VAC?
  // #define CHECKMAINS
#endif
// Zero motorcycles special - run charger for DeltaQ signal - this assumes D6 is pulled up by 5k to 3.3v and is 
// connected to Zero's deltaQ line on a battery or bike. Test by pulling D6 to ground with 2.2k resistor
// #define DELTAQ  
// ===================================================================================================

// sensor constants - moved them to here as they are changed very often depending on what unit we have
// 1M in low-voltage units (up to 150V)
// 2M in SmartCharge-12000 standard units
// 2.4M in most PFCdirect units
// 3M in newer PFCdirect units
// 5-6M in high voltage units (but usually only on high-side)
const float upperR0_mV=3000.; 
const float upperR0_bV=3000.; 


//------------------------------ MAIN SWITCHES STORAGE AREA - OVERRIDES ONLY! --------------------
// #define MCC100A // 100A output rating - use ONLY with a custom-wound inductor
// #define A7520_V // using A7520 optoisolation for outV sensing? (as opposed to ISO124 chip used in earlier versions)
// #define A7520_mV // using A7520 optoisolation for mV sensing? (as opposed to ISO124 chip used in earlier versions)
// #define PC817 // mains voltage sensing based on a crude regular opto (V13 boards)
// #define PFC // is this a PFC unit?
// #define PFCdirect // is this a PFCDirect 25kW unit?
// #define UVLO // enable gate supply undervoltage protection?
// #define IND_Temp // do we have a second temp probe on the inductor?
 #define debugpower // doubles the power limits - careful - it may blow up your charger
//------------------------------- END MAIN SWITCHES STORAGE AREA ------------------------------------


//----------------------------------- some hardcoded constants - DO NOT CHANGE
const unsigned long MAXDMILLIDUTY=9700000; // to run precise PID loop   

// from Oct 10 2013, this defaults to 20kHz due to use of faster IGBTs. For kits with older IGBTs, use 60-70
// for 60hz line frequency, period has to ideally be 260 / N, where N is an integer
// for 50hz line frequency, 312 / N
#ifdef LOWVOLTAGE
  const int period=130; // 8khz for low-voltage, high-current applications; 60hz line freq
#else 
//   const int period=52; // 52us (~20khz) for normal-voltage applications; 60hz line freq
//  const int period=65; // 65us (~16khz) for high-voltage applications; 60hz line freq
//  const int period=86; // 86us (~12kHz) for high-voltage applications with high inductance or low inductor voltage 
  const int period=130; // 130us (~8kHz) for medium-voltage applications with high inductance or low inductor voltage 
#endif

const unsigned int linefreq=60; // 60Hz in the US - for best performance, set this to your country's line frequency!
// scaler from PWM frequency. There is a method to the madness here - basically we want to be sampling things at 4x the line
// frequency and then use two adjacent readings to produce an average. This cancels out most of the 120Hz ripple from the readings
// in the sampling interrupt, every variable is sampled only every 1/16th period, hence the 16 divider below
// with 50uS period, 60Hz, this will produce 10.4 which will be rounded to 10, producing 4% phase error which is fine
const int MEASFREQPWMPRESCALE=(1000/period)*1000/(linefreq*4)/16; 

byte PWM_enable_=0;
const unsigned int serialspeed=19200; // 115 is unstable, 38.4 is a bit unstable...

#define SerialStrSize 15 // M,ccc,vvv,sss,E
char SerialStr[SerialStrSize+2]; // buffer for the serial command
char SerialCommand[SerialStrSize+2]; // this is where the command will actually be stored

// LCD includes - have to be here in the code as they depend on some switches above
// LCD library for 4D systems display (http://www.4dsystems.com.au/prod.php?id=121)
#ifdef LCD_SPE
  #include <uLCD_144_SPE.h>
  uLCD_144_SPE *myLCD;
#else
  #include <uLCD_144.h>
  uLCD_144 *myLCD;
#endif
byte LCD_on=0; // this defines manual vs serial-controlled operation
int cmd[2]={0, 0}; // command variables, used in serial comms

//============================================== define messages ==================
// using program memory as we are now running out of SRAM...
// see http://arduino.cc/en/Reference/PROGMEM for reference
// some additional good memory tips at http://liudr.wordpress.com/2011/02/04/how-to-optimize-your-arduino-memory-usage/
//-----------------------Navigate Menus--------------------
const char * configMenu[] = { "Run ", "Pwr ", "Time"  };
const byte configMenuLen = 3;

const byte MSG_THX = 0x00;
const byte MSG_NOBATT	= 0x01;
const byte MSG_WRONGPROF = 0x02;
const byte MSG_BMSSTOP = 0x03;
const byte MSG_TIMEOUT = 0x04;
const byte MSG_USRPAUSE = 0x05;
const byte MSG_LOSTIN	= 0x06;
const byte MSG_SENSEERROR = 0x07;
const byte MSG_NORMEXIT = 0x08;
const byte MSG_DONE = 0x09;

#ifndef LCD_SPE
  prog_char msg_long_0[] PROGMEM = "Thank you for choosing EMotorWerks! BTN to CFG";
  prog_char msg_short_0[] PROGMEM = "INIT";
  prog_char msg_long_1[] PROGMEM = "No batt or reverse! ANY BTN to ignore";
  prog_char msg_short_1[] PROGMEM = "NOBATT";
  prog_char msg_long_2[] PROGMEM = "Wrong profile!";
  prog_char msg_short_2[] PROGMEM = "WRONGPROF";
  prog_char msg_long_3[] PROGMEM = "BMS Stop";
  prog_char msg_short_3[] PROGMEM = "BMSSTOP";
  prog_char msg_long_4[] PROGMEM = "Timeout";
  prog_char msg_short_4[] PROGMEM = "TIMEOUT";
  prog_char msg_long_5[] PROGMEM = "Paused. RED BTN to exit, GRN to resume";
  prog_char msg_short_5[] PROGMEM = "USRPAUSE";
  prog_char msg_long_6[] PROGMEM = "Lost AC";
  prog_char msg_short_6[] PROGMEM = "LOSTIN";
  prog_char msg_long_7[] PROGMEM = "Sensor/cal error. Recal/chk wiring";
  prog_char msg_short_7[] PROGMEM = "SENSEERROR";
  prog_char msg_long_8[] PROGMEM = "Step complete";
  prog_char msg_short_8[] PROGMEM = "NORMEXIT";
  prog_char msg_long_9[] PROGMEM = "Complete! GRN BTN to repeat";
  prog_char msg_short_9[] PROGMEM = "DONE";
#else 
  prog_char msg_long_0[] PROGMEM = "Thank you for\nchoosing\nEMotorWerks!\nBTN to CFG";
  prog_char msg_short_0[] PROGMEM = "INIT";
  prog_char msg_long_1[] PROGMEM = "No batt or reverse! \nANY BTN to ignore";
  prog_char msg_short_1[] PROGMEM = "NOBATT";
  prog_char msg_long_2[] PROGMEM = "Wrong profile!";
  prog_char msg_short_2[] PROGMEM = "WRONGPROF";
  prog_char msg_long_3[] PROGMEM = "BMS Stop";
  prog_char msg_short_3[] PROGMEM = "BMSSTOP";
  prog_char msg_long_4[] PROGMEM = "Timeout";
  prog_char msg_short_4[] PROGMEM = "TIMEOUT";
  prog_char msg_long_5[] PROGMEM = "Paused. RED BTN \nto exit, GRN to \nresume";
  prog_char msg_short_5[] PROGMEM = "USRPAUSE";
  prog_char msg_long_6[] PROGMEM = "Lost AC";
  prog_char msg_short_6[] PROGMEM = "LOSTIN";
  prog_char msg_long_7[] PROGMEM = "Sensor/cal error. \nRecal/chk wiring";
  prog_char msg_short_7[] PROGMEM = "SENSEERROR";
  prog_char msg_long_8[] PROGMEM = "Step complete";
  prog_char msg_short_8[] PROGMEM = "NORMEXIT";
  prog_char msg_long_9[] PROGMEM = "Complete! GRN BTN \nto repeat";
  prog_char msg_short_9[] PROGMEM = "DONE";
#endif

PROGMEM const char *msg_long_table[] = 	  
{   
  msg_long_0,
  msg_long_1,
  msg_long_2,
  msg_long_3,
  msg_long_4,
  msg_long_5, 
  msg_long_6,
  msg_long_7,
  msg_long_8,
  msg_long_9
};

PROGMEM const char *msg_short_table[] = 	  
{   
  msg_short_0,
  msg_short_1,
  msg_short_2,
  msg_short_3,
  msg_short_4,
  msg_short_5, 
  msg_short_6,
  msg_short_7,
  msg_short_8,
  msg_short_9
};

const byte MSG_LCD_CELLTYPE	= 0x00;
const byte MSG_LCD_CV = 0x01;
const byte MSG_LCD_NCELLS = 0x02;
const byte MSG_LCD_CAPACITY = 0x03;
const byte MSG_LCD_CAL0 = 0x04;
const byte MSG_LCD_CAL1 = 0x05;
const byte MSG_LCD_CAL2 = 0x06;
const byte MSG_LCD_CONFIRM = 0x07;
const byte MSG_LCD_PARAMS = 0x08;
const byte MSG_LCD_CFG = 0x09;
const byte MSG_LCD_TOPMENU = 0x0A;
const byte MSG_LCD_INC = 0x0B;
const byte MSG_LCD_OUTC = 0x0C;
const byte MSG_LCD_TOUT = 0x0D;
const byte MSG_LCD_RUN = 0x0E;
const byte MSG_LCD_BLANK = 0x0F;

#ifndef LCD_SPE
  prog_char msg_lcd_0[] PROGMEM = "Cell Type:       ";
  prog_char msg_lcd_1[] PROGMEM = "CV cutoff:       ";
  prog_char msg_lcd_2[] PROGMEM = "Number of cells: ";
  prog_char msg_lcd_3[] PROGMEM = "Capacity:        ";
  prog_char msg_lcd_4[] PROGMEM = "Calibrated zero";
  prog_char msg_lcd_5[] PROGMEM = "Connect batt. BTN to skip";
  prog_char msg_lcd_6[] PROGMEM = "Enter actual batt voltage:";
  prog_char msg_lcd_7[] PROGMEM = "Confirm:      ";
  prog_char msg_lcd_8[] PROGMEM = "Params      ";
  prog_char msg_lcd_9[] PROGMEM = "press BTN to adjust";
  prog_char msg_lcd_10[] PROGMEM = "Action:                   ";
  prog_char msg_lcd_11[] PROGMEM = "max INput current ";
  prog_char msg_lcd_12[] PROGMEM = "max OUTput current";
  prog_char msg_lcd_13[] PROGMEM = "timeout (#min or 0):";
  prog_char msg_lcd_14[] PROGMEM = "Confirm CHARGE:";
  prog_char msg_lcd_15[] PROGMEM = "[           ]";
#else
  prog_char msg_lcd_0[] PROGMEM = "Cell Type:       ";
  prog_char msg_lcd_1[] PROGMEM = "CV cutoff:       ";
  prog_char msg_lcd_2[] PROGMEM = "Number of cells: ";
  prog_char msg_lcd_3[] PROGMEM = "Capacity:        ";
  prog_char msg_lcd_4[] PROGMEM = "Calibrated zero";
  prog_char msg_lcd_5[] PROGMEM = "Connect batt. BTN \nto skip";
  prog_char msg_lcd_6[] PROGMEM = "Enter actual \nbatt voltage:";
  prog_char msg_lcd_7[] PROGMEM = "Confirm:      ";
  prog_char msg_lcd_8[] PROGMEM = "Params      ";
  prog_char msg_lcd_9[] PROGMEM = "press BTN to\nadjust";
  prog_char msg_lcd_10[] PROGMEM = "Action:                   ";
  prog_char msg_lcd_11[] PROGMEM = "max INput current ";
  prog_char msg_lcd_12[] PROGMEM = "max OUTput current";
  prog_char msg_lcd_13[] PROGMEM = "timeout (#min or 0):";
  prog_char msg_lcd_14[] PROGMEM = "Confirm CHARGE:";
  prog_char msg_lcd_15[] PROGMEM = "[           ]";
#endif

PROGMEM const char *msg_lcd_table[] = 	  
{   
  msg_lcd_0,
  msg_lcd_1,
  msg_lcd_2,
  msg_lcd_3,
  msg_lcd_4,
  msg_lcd_5,
  msg_lcd_6,
  msg_lcd_7,
  msg_lcd_8,
  msg_lcd_9,
  msg_lcd_10,
  msg_lcd_11,
  msg_lcd_12,
  msg_lcd_13,
  msg_lcd_14,
  msg_lcd_15
};
//=========================== end define messages ==================================


//---------------- pin-out constants ----------------
//========== analog pins
const byte pin_C=0; // output current pin
const byte pin_bV=1; // output / battery voltage pin
const byte pin_heatSinkT=2; // charger heatsink temp - for thermal derating 
const byte pin_12Vsense=3; // implementing undervoltage protection
const byte pin_temp2=4; // 4 - spare prewired as temp input
const byte pin_mV=5;
// const byte pin_mC=7; // will only work in V12+ control boards (June 2013). needed only for full digital PFC control
//========== digital pins
// 0/1 reserved for serial comms with display etc
const byte pin_pwrCtrlButton=2; // this is wired to the button (used for menu step)
const byte pin_pwrCtrl2Button=3; // this is wired to the button2 (used for menu select)
const byte pin_inrelay=4; // precharges input caps - normally pin 4, in some units pin 6 running fan relay
const byte pin_TEST=5; 
const byte pin_DELTAQ=6; // deltaQ pin
const byte pin_J1772=7; // J1772 pilot input. 1k is hardwired on V14+ pcbs so J1772 will power on on connect
const byte pin_fan=8; // fan control - this is pin4 in all kits shipped before Mar '12
const byte pin_PWM=9; // main PWM pin

// max current reference voltage (using PWM) -  was 6 in the V13 pcb (kits shipped before March 2012)
// now moved to pin 10 so that we can use higher PWM frequency 20kHz PWM
const byte pin_maxC=10; 

const byte pin_EOC=12; // end-of-charge output (see pinout diagram) - pulled low when charge is complete

// end-of-charge input from BMS. Pull low / disconnect from positive TTL signal to activate
//     (normallly will be realized via connecting NC BMS loop between this pin and EOC pin (or +5V)
const byte pin_BMS=13; 
//---------------- END PINOUTS -----------------------


//============= BATTERY INFO  =====
struct config_t {
  int nCells;
  int AH;
  int CV; // per cell
  int CC; // max output current
  int mainsC; // max input current
  // sensor config
  float Vcal;
  float Vcal_k;
  float mVcal;
  float Ccal;
} configuration;


//---------------- MAX CURRENTS
// absolute maximum average output current (used in CV mode) - leave at 0 here - will be set via power menu
const float min_CV_Crating=0.05; // wait until the current goes to XC (use values from your battery's datasheet)
const float Cstep=0.5; // how quickly the current tapers off in CV step - A / second. cannot be too high to prevent false stops
float maxOutC=0., maxOutC1=0;
int n=0;
const float peakMaxC=1.8; // ratio between the average and max current in the inductor before overcurrent kicks in

#ifdef debugpower // increase power limits for testing runs - careful!
  const float absMaxChargerCurrent=300; // 300A...
  const float absMaxChargerPower=50000; // 50kW...
#else
  #ifdef MCC100A
    const float absMaxChargerCurrent=99; // 99A (need to be a 2-digit number!) rating with high-current output toroid inductor
  #else
    const float absMaxChargerCurrent=70; // 70A default rating with new toroid inductors
  #endif
  
  #ifdef PFCdirect
    const float absMaxChargerPower=25000; // 25kW rating for PFCDirect units with new 5-6" toroid inductors
  #else
    const float absMaxChargerPower=12000; // 12kW rating for regular units with new 4" toroid inductors
  #endif
#endif

// input currents (used only for DC-DC units (if DCinput switch is active)
#ifdef DCinput
  const float MAXinputC=absMaxChargerCurrent;
#endif

//------------- THERMAL DERATING OF CHARGER 
// for now, simple protection by pausing charger until cooldown to certain temp
// note that heatSink temp at the point of measurement is generally 20-30 deg C LOWER than temperature 
// of critical components attached to heatsink (due to distance from components to probe)
// use maxHeatSinkT of <60 to ensure <85 deg C temp of components
// this assumes thermistor placement near the heat generating components
// BTW, modest airflow (a single 120mm PC fan) with a large (8x10x3" heatsink should be sufficient for 
// up to 30A output at max power 
#ifndef MCC100A
  const byte maxHeatSinkT=55; // in Centigrades - will start derating here
#else
  const byte maxHeatSinkT=47; // more aggressive derating at high current output
#endif
const byte ABSmaxHeatSinkT=85; // in Centigrades - will stop the charger altogether here
const byte midHeatSinkT=45; // turn on the fans here; also wait until cool down to this temp before resuming at the prev power level 
const byte lowHeatSinkT=35; // turn off the fans here 
//--------------------------------------------------------

const float Aref=5.0; // 5V for ATMega328 (Pro Mini), 3.3V for ATSAM (Due) 

//=============== voltage dividers settings ===========================
//--------- some constants for 7520 chips
const float gain_7520=5./0.512*0.99; // calculate effective gain (Vcc/0.512 per datasheet, plus 1% correction for input resistance)
const float lowerR0_V_7520=2.7;

//--------- mains voltage 
// INPUT side constants
float divider_k_mV=-1.; 
#ifdef A7520_mV
  // resistor from -5V regulator; should form a ~-.25V divider together with the 
  // bottom resistor => >20x * bottom resistor 
  // for 2.7k bottom 3resistor, pick between 60k and 82k; 68k is a good choice... 
  const float V_o_mV0=2.5-5.*lowerR0_V_7520/68.*gain_7520; // -5V input, 2.7k bottom resistor, 9.76 gain; // ~2.5V for A7520
  const float lowerR0_mV=lowerR0_V_7520*gain_7520; // +-0.256V range for input, ~10x gain, 2.7k bottom resistor
  const float lowerR_mV=lowerR0_mV;
#else
  const float V_o_mV0=0;
  const float lowerR_mV=23.79; // 1/(1/27.+1/200.) - in parallel with ISO124 input resistance of 200k
#endif
float V_o_mV=V_o_mV0; // need to reassign to non-const as it will be adjusted below

//--------- battery voltage 
// OUTPUT side constants
float divider_k_bV=-1.;
#ifdef A7520_V
  // resistor from -5V regulator; should form a ~-.25V divider together with the 
  // bottom resistor => >20x * bottom resistor 
  // for 2.7k bottom resistor, pick between 60k and 82k; 68k is a good choice... 
  const float V_o_bV0=2.5-5.*lowerR0_V_7520/68.*gain_7520; // -5V input, 2.7k bottom resistor, 9.76 gain; // ~2.5V for A7520
  const float lowerR0_bV=lowerR0_V_7520*gain_7520;   // +-0.256V range for input, Vref/0.512x gain
  const float lowerR_bV=lowerR0_bV;
#else
  const float V_o_bV0=0;
  const float lowerR_bV=23.79; // in parallel with 200k input resistance of the iso124
#endif
float V_o_bV=V_o_bV0; // need to reassign to non-const as it will be adjusted below
//==================================== end voltage dividers setup =========================

//=================================== charger current sensor ==============================
// V/A constant for the charger output current sensor 
float V_o_C=
#ifdef OUTC_SENSOR_Allegro_100U
                  0.6; // allegros are 0.6
#else
                  2.5; // tamuras are 2.5
#endif

// sensitivity of the sensor
const float k_V_C=
#ifdef OUTC_SENSOR_Tamura_50B
                  0.03;
#endif
#ifdef OUTC_SENSOR_Tamura_150B
                  0.01;
#endif
#ifdef OUTC_SENSOR_Tamura_600B
                  0.0025;
#endif
#ifdef OUTC_SENSOR_Allegro_100U
                  0.04;
#endif
//=================================== END charger current sensor ==========================

//===================== charger cycle timers =====================================
// when changing stepDelay, keep stepDelay*measCycle_len within 0.5-1 sec
const byte stepDelay=30; // primary charger loop delay in milliseconds - should be less than 50 in order to run QC loop properly
const byte measCycle_len=15; // how many primary loop cycles per display cycle 
const byte AVGCycles=10; // how many readings of C,V,T (taken every 4ms)  to average for reporting (to CHAdeMO) and display
const byte stopCycles=5; // how many primary charger cycles to require stop condition to exist before exiting
const byte CV_timeout=20; // what is the max duration (in secs) CV loop is allowed to spend below C stop; should be > ramp time of charger
byte breakCnt=0;
byte breakCycle=0;
//===================== end charger cycle timers =================================

unsigned long timer=0, timer_ch=0, timer_comm=0, timer_irq=0, deltat=0;
unsigned int sec_up=0;

float mainsV=0, outV=0, outC=0;
float AH_charger=0;
char str[64];

byte charger_run=0;
byte state;
byte normT=0;
const byte minMains=30; // min mains voltage to (1) test sensor connectivity and (2) detect mains disconnect 
int timeOut=0; // in min, 0 means no timeout
float maxOutV=0; // absolute maximum output voltage - will be set later in the code
float maxMainsC=0; // allowed charger power - will be changed in code


//---------------------------------------------------------------------------------------------------------
// as of V13, completely new way to control the charger! proper PID loop and interrupt-based fast ADC
// this was originally driven by the need to meet requirements from the Leaf CHAdeMO protocol
//---------------------------------------------------------------------------------------------------------
// all these have to be ints or very unpleasant wrapping will occur in PID loop 
// having these unsigned has cost EmotorWerks over $1,000 in parts during testing ;-)
int targetC_ADC=0; // this is an ADC reference point for our output current
int outC_ADC_0=0, outC_ADC=0, outV_ADC=0, outmV_ADC=0, T_ADC=0, T2_ADC=0;
float outC_ADC_f=0;

// ADC interrput handler
// this is always aligned with Timer1 interrupt
// ADC conversions are always done at 4kHz frequency (or every 250uS) so by the next Timer1 interrupt, 
// we should ALWAYS have the result! 
ISR(ADC_vect) { // Analog->Digital Conversion Complete
  byte ul, uh;
  cli(); // disable interrupt until function exit. otherwise nested interrupts...
  ul=ADCL;
  uh=ADCH;
  sei();
  
  unsigned int val= (uh << 8 | ul); // assuming ADLAR=0
  
  // just load things into the variables and exit interrupt - processing all in main loop
  // for most variable, average 2 values offset 180 degrees wrt haversine wave
  // for current measurement, average 16 measurements over 8ms, or one full haversine period
  switch(ADMUX & B00000111) {
   case pin_C: { // this is measured at 2kHz frequency
     if(outC_ADC==0) {
       outC_ADC_f=outC_ADC=val;
     } else {
       // 16 cycles is 8ms here or a full haversine period
       outC_ADC_f=(outC_ADC_f*15+val)/16; // this emulates an RC filter with time constant of ~half of averaged periods
       outC_ADC=int(outC_ADC_f);
     }
     break;
   }
   // rest of vars measured at 250Hz
   case pin_bV: {
     if(outV_ADC==0) {
       outV_ADC=val;
     } else {
       outV_ADC=(outV_ADC+val)/2;
     }
     break;
   }
   case pin_mV: {
     if(outmV_ADC==0) {
       outmV_ADC=val;
     } else {
       outmV_ADC=(outmV_ADC+val)/2;
     }
     break;
   }
   case pin_heatSinkT: {
     if(T_ADC==0) {
       T_ADC=val;
     } else {
       T_ADC=(T_ADC+val)/2;
     }
     break;
   }
   case pin_temp2: {
     if(T2_ADC==0) {
       T2_ADC=val;
     } else {
       T2_ADC=(T2_ADC+val)/2;
     }
     break;
   }
   default: break;
  }

} // end ADC interrupt


//---------------- interrupt magic to initiate ADC and calc PID loop ---------------------------
// PID loop setup - see http://en.wikipedia.org/wiki/PID_controller for some definitions
// using only PI part of it here
// parameter approximations ------------
// all constants below are effectively in 0.0001 units for the formula
// at 10-bit duty counter, 250Hz loop speed and 1000 unit range
// Example: 50A target ramp from zero, measured with a 50A bidir sensor: error is ~300
//          ramp rate is pids_Kp * 8 duty pts / sec (pids_Kp * 300 / 10000 duty points in one cycle (~4ms))
// for CHAdeMO unit with 50A max C, ramp to 50A in 2 seconds requires pids_Kp>60 (assuming full duty sweep would be required)
// OTOH, typical single-stage charger's stiffness is 10-20A per 10 duty points 
// so we don't want to be making changes of more than 10 duty points per cycle
// which corresponds to pids_Kp<330
// so the meaningfull range is probably between 50 and 300
// our motor controller has Kp=3200, Ki=30, Kd=0 
//----------------------- tuning charger PID:
// Zieglerâ€“Nichols method: the Ki and Kd gains are first set to zero. 
// The P gain is increased until it reaches the ultimate gain, Ku, at which the output of the loop 
// starts to oscillate. Ku and the oscillation period Pu are used to set the gains as shown:
// Control Type	Kp	Ki	        Kd
//    P    	0.50Ku	-	        -
//    PI	        0.45Ku	1.2Kp / Pu	-
//    PID	        0.60Ku	2Kp / Pu	KpPu / 8
// for this charger:
// on 330V pack (LiFePo4, milli-ohm total IR), at Kp=1000, see oscillations at Hz) - hence 
// setting Kp=, Ki=
const long pids_Kp_SLOW=60; // revert to slow PID once the current shows up
const long pids_Kp_FAST=300; // fast PID to start with
long pids_Kp=0; 
const long pids_Ki=1; // need small integral term - otherwise we get some constant offset error
const long pids_Kd=0; // for now, just PI loop

long pids_err=0, pids_perr=0, pids_i=0, pids_d=0; // all have to be signed longs in order to not screw up pid calcs
long deltaDuty=0, milliduty=0; // have to be signed
byte tickerPWM=0; // short counter used only to skip cycles
byte tickerPWM1=0; // counter counting unskipped cycles - ok to overwrap

// called on overflow of Timer1 - called every 'period' uS (20 kHz by default)
// overflow with TimerOne library means we are in the center of the PWM cycle (TimerOne is phase correct)
void sampleInterrupt() {
  // trigger actual work only on every Nth period
  tickerPWM++;
  
  // prescale is defined based on period to make constant frequency of work below
  if(tickerPWM < MEASFREQPWMPRESCALE) return; 
  // prescaler is calculated at startup so that we always end up with ~4kHz frequency here
  // therefore, every ADC conversion has 250 microseconds - which should be ok given that ADC on ATMega328P takes 100us

  tickerPWM=0;
  tickerPWM1++; // this counts at lower frequency
    
  ADMUX &= B11111000; // reset the channel to zero

  // then current is measured every second cycle - or at ~2kHz frequency
  if(tickerPWM1 & 0x1) {
     ADMUX |= pin_C;
     ADCSRA |= B11000000; // manually trigger next one
  } else {
    // Every parameter is measured every 16 cycles => 250 Hz measurement frequency for every variable
    // PID loop runs at the same frequency, as well    
    switch(tickerPWM1/2 & 0x7) {  
       // case set below is MISSING 0,4,7 - available for other sensors
       case 0: {
         // average outC
         if(fabs(outC)<1.) outC=readC(); // 
         outC=(outC*float(AVGCycles-1)+readC())/AVGCycles; 
         break;
       }
       case 1: {
         ADMUX |= pin_bV;
         ADCSRA |= B11000000; // manually trigger next one
         break;
       }
       case 2: {
         ADMUX |= pin_mV;
         ADCSRA |= B11000000; // manually trigger next one
         break;
       }
       case 3: {
         ADMUX |= pin_heatSinkT;
         ADCSRA |= B11000000; // manually trigger next one
         break;
       }
       case 4: {
         // average outV
         if(fabs(outV)<1.) outV=readV();
         outV=(outV*float(AVGCycles-1)+readV())/AVGCycles;
         break;
       }
       case 5: {
         ADMUX |= pin_temp2;
         ADCSRA |= B11000000; // manually trigger next one
         break;
       }
       case 6: {
          //====================   PID loop   ====================
          // remember that targetC is a 10-bit ADC reference point that we are trying to keep - NOT the actual current!
          pids_err = targetC_ADC - outC_ADC; 
  #ifdef NEG_CSENSE
          pids_err *= -1; // the current signal (and hence the error sign) runs in a different direction
  #endif        
          
          // if the current is non-zero already, slow down 
          // allow deviation of 5% of full range of Tamura sensor (which at 1.5V deviation produces ADC output of 300 units)
//          if(abs(outC_ADC-outC_ADC_0)>15) {
//            pids_Kp=pids_Kp_SLOW;
//            digitalWrite(pin_TEST, HIGH);
//          }
          
          deltaDuty = pids_Kp * pids_err;
      
          pids_i += pids_err;
          deltaDuty += pids_Ki * pids_i;
      
          pids_d = pids_err - pids_perr;
          pids_perr = pids_err;
          deltaDuty += pids_Kd * pids_d;
          //==================== end PID loop ====================
          
          // protect against overpowering
          if( (deltaDuty>0) && (outC > maxOutC) ) deltaDuty=0;
          
          milliduty += deltaDuty;
          if(milliduty < 0) {
            milliduty=0;
            // stop accumulation
            pids_i=0;
          }
          if(milliduty > MAXDMILLIDUTY) {
            milliduty=MAXDMILLIDUTY;
            // stop error accumulation
            if(pids_i>0) pids_i=0;
          }
  
          // immediate protection from overvoltage - zero out duty
          // this also stops any term's accumulation before PWM_enable_ is turned on (e.g. before charger start)
          if( (PWM_enable_ == 0) || (outV > 1.05*maxOutV) ) {
            milliduty=0;
            pids_i=0; // need to stop accumulation, as well
          }
          
          Timer1.setPwmDuty(pin_PWM, milliduty/10000); 
  
          break;  
       }

       default: break;

    } // end switch
  
  } // end if(tickerPWM1 & 0x1)
  
}  // end timer interrupt


//-------------------------------------------- START MAIN CODE ---------------------------------------------
void setup() {
  // digital inputs
  pinMode(pin_pwrCtrlButton, INPUT);
  pinMode(pin_pwrCtrl2Button, INPUT);
  pinMode(pin_J1772, INPUT);
  pinMode(pin_BMS, INPUT);
  pinMode(pin_DELTAQ, INPUT);

  // set output digital pins
  pinMode(pin_TEST, OUTPUT);
  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_maxC, OUTPUT);
  pinMode(pin_EOC, OUTPUT);
  pinMode(pin_fan, OUTPUT);
  pinMode(pin_inrelay, OUTPUT);
  
  // setup ADC
  ADMUX = B01000000;  // default to AVCC VRef, ADC Right Adjust, and ADC channel 0 (current)
  ADCSRB = B00000000; // Analog Input bank 1
  // ADC enable, ADC start, manual trigger mode, ADC interrupt enable, prescaler = 128 (3 bits in the end)
  // standard prescaler is 128 resulting in 125kHz ADC clock. 1 conversion takes 13 ADC cycles = 100uS using standard prescaler
  // 64 prescaler results in ~50uS conversion time
  ADCSRA = B11001111; 
  
  // setup timer - has to be before any ADC readouts
  Timer1.initialize(period); 
  Timer1.pwm(pin_PWM, 0); // need this here to enable interrupt
  Timer1.pwm(pin_maxC, 0); // need this here to enable interrupt
  Timer1.attachInterrupt(&sampleInterrupt); // attach our main ADC / PID interrupt
  delay(50); // allow interrupts to fill in all analog values 

  //=================================== finalize init of the sensors =============================
  // reset voltage dividers to account for the input resistance of ISO124
  divider_k_mV=upperR0_mV/lowerR_mV;
  divider_k_bV=upperR0_bV/lowerR_bV;
  //=============================== END finalize init of the sensors =============================

  //================= initialize the display ===========================================
#ifdef LCD_SPE
  *myLCD=uLCD_144_SPE(9600);
#else
  *myLCD=uLCD_144(9600);
#endif
  //================= finish display init ==============================================
  
  // check if the display started / is present
  // if not present, we will assume that the charger is controlled by serial data instead
  LCD_on=myLCD->isAlive();

  //==================================== ONE-TIME CONFIG =================================
  // check if needed to go into config 
  byte forceConfig=255; // default is 255 - has to be different from 0 or 1
  EEPROM_readAnything(0, configuration);
  // reset configuration if the green button is pressed at charger start
  // on first connection, do zero cal of mainsV, as well
  if(configuration.CC<=0 || digitalRead(pin_pwrCtrl2Button)==1) {
    forceConfig=1; // first time running the charger after assembly
    configuration.CV=350;
    // set the rest of the vars
    configuration.Vcal=0;
    configuration.Vcal_k=1.; // prefill the calibration with unity so we don't get zero readings if calibration menu is skipped
    configuration.mVcal=0;
    configuration.Ccal=0;
  }
  
  const byte STATE_DONE = 0xff;
  const byte STATE_CV = 0x1;
  const byte STATE_CELLS = 0x2;
  const byte STATE_CAPACITY = 0x4;
  const byte STATE_CALIBRATE = 0x5; // sensitivity calibration only. zero point calibration done automatically on power-on
  state = STATE_CV;
    
  if(LCD_on) {  
    myLCD->clrScreen();
    myLCD->setOpacity(1);
  } else {
    state=STATE_DONE; // skip config altogether if no LCD
    // reset serial to faster speed
    Serial.end();
    Serial.begin(serialspeed);
  }    
  
  while(state != STATE_DONE)
  {
    switch(state)
   {
     case STATE_CV:
       // if config is not forced, just timeout and send to end of config. Else, wait until button press
       if(forceConfig==255) {
         printClrMsg(MSG_THX, 50, 0, 0x3f, 0);
         forceConfig=BtnTimeout(5, 7); // this will return 0 if no button pressed; 1 otherwise; 5 seconds, line #7
       }
       if(forceConfig==0) {
         state=STATE_DONE;
       } else { // forceConfig=1 here
         myLCD->clrScreen();
         printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_CV);
         configuration.CV = DecimalDigitInput3(configuration.CV); 
         state = STATE_CELLS;       
       }
       break;
     case STATE_CELLS:
       printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_NCELLS);
       configuration.nCells = DecimalDigitInput3(configuration.nCells); 
       state = STATE_CAPACITY;
       break;
     case STATE_CAPACITY:
       printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_CAPACITY);
       configuration.AH = DecimalDigitInput3(configuration.AH); 
       state = STATE_CALIBRATE;       
       break;
     case STATE_CALIBRATE:
       // output current zero calibration - this assumes that there is no load on startup 
       // this is especially important for PFCdirect units which should not have anything plugged into charger output at this point!
       outC_ADC_0=outC_ADC; // ADC reference
       outC=readC();  
#ifdef NEG_CSENSE
       configuration.Ccal=-outC*k_V_C;
#else
       configuration.Ccal=outC*k_V_C;
#endif

       // prep for output voltage zero calibration
       // this will generally NOT work on PFCdirect units as there is always voltage on the output
       // to calibrate at the factory / right after build, power 12V ONLY and follow through calibration
       outV=readV();
       sprintf(str, "Drain %dV, BTN", int(outV));  
       printMsg(str, 0, 0, 0, 0x1f, 0x3f, 0x00);
       while(!(digitalRead(pin_pwrCtrlButton) || digitalRead(pin_pwrCtrl2Button)));
       outV=readV(); // re-read after discharge

       // now actual zero cal
       if(fabs(outV)<40) { // if too far off, fault out
         // output voltage calibration
         configuration.Vcal=outV/divider_k_bV; 
         V_o_bV+=configuration.Vcal; // this needs to be adjusted HERE because we are calling readV() again below for sensitivity calibration
         printConstStr(0, 5, 2, 0x1f, 0x3f, 0x00, MSG_LCD_CAL0);
         delay(1000);
       }
       
       // now calibrate voltage sensor slope
       // first, double-check we have reset to zero point
       // for PFCdirect units, this will only work if ONLY 12V is powered up, no main AC connected!
       outV=readV(); // get the readings with zero-point already calibrated
       if(fabs(outV)<3) { // should be pretty tight after zero calibration
         // this is a good time to also do mains calibration - assuming that on the very first power-up and forced config in general
         // we have zero input
         if(forceConfig==1) {
           mainsV=read_mV();
           // only recal if not too far from truth
           if(mainsV<50) {
             configuration.mVcal=mainsV/divider_k_mV;
           } else {
             configuration.mVcal=0;
           }
         }
         myLCD->clrScreen();
         printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_CAL1); // this asks to connect the battery
         delay(1000); // to avoid reading same button state as in prev step
         while(1) {
           outV=readV();
           if(digitalRead(pin_pwrCtrlButton) || digitalRead(pin_pwrCtrl2Button))  break;
           if(outV>10) { // loop until battery not connected
             delay(5000); // let settle
             outV=readV(); // read settled voltage
             // calibrate
             printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_CAL2);
             // calibration routine here - if actual voltage > shown, REDUCE the constant
             configuration.Vcal_k=DecimalDigitInput3(int(outV))/outV;
             break; // from while() loop
           }
         }
       }
       
       state = STATE_DONE;
       break;

     default: break;
   } 
  }

  // parameters calculated from config variables go here
  // adjust core sensor constants
  V_o_bV=V_o_bV0+configuration.Vcal;
  V_o_mV=V_o_mV0+configuration.mVcal;
  V_o_C+=configuration.Ccal;
  divider_k_bV*=configuration.Vcal_k; 
  
  // write out the configuration to EEPROM for next time
  EEPROM_writeAnything(0, configuration);

#ifdef DEBUG1
  Serial.print("MFP: ");
  Serial.println(MEASFREQPWMPRESCALE);
#endif
}
  

void loop() {  
  // ---------------real loop()
  byte x=255; // default, has to be different from 0 or 1
  int J1772_dur;

  mainsV=read_mV();
  outV=readV();
  
  maxOutV=float(configuration.CV)/100*configuration.nCells;

  // run charger if: 
  //         (1) charger has NOT been run yet in this cycle, or 
  //         (2) has been run over a week ago
  //         (3) green button is pressed to override
  if(LCD_on==0 || digitalRead(pin_pwrCtrl2Button)==HIGH || charger_run==0) {
      //----------------------------
      // run state machine:
      const byte STATE_TOP_MENU = 0x00;
      const byte STATE_CONFIG_PWR = 0x01;
      const byte STATE_CONFIG_TIMER = 0x02;
      const byte STATE_CHARGE = 0x04;
      const byte STATE_WAIT_TIMEOUT = 0x05;
      const byte STATE_SERIALCONTROL = 0x10;
      const byte STATE_SHUTDOWN = 0xff;
      if(!LCD_on) {
        // drop us directly into a serial control loop
        state=STATE_SERIALCONTROL;
      } else {
        state = STATE_WAIT_TIMEOUT;
      }
      if(configuration.CC<=0) state=STATE_CONFIG_PWR;
      
      while(state != STATE_SHUTDOWN)
      {
        // reload voltages
        mainsV=read_mV();
        outV=readV();
        
        if(LCD_on) {
          myLCD->clrScreen();
          printConstStr(0, 6, 2, 0x1f, 0x3f, 0, MSG_LCD_PARAMS);
          sprintf(str, "IN: %dV, %dA", int(mainsV), configuration.mainsC); myLCD->printStr(1, 7, 2, 0x1f, 0x3f, 0, str);
          sprintf(str, "OUT: %dV, %dA", int(outV), configuration.CC); myLCD->printStr(1, 8, 2, 0x1f, 0x3f, 0, str);
          sprintf(str, "T-OUT: %d min", timeOut); myLCD->printStr(1, 9, 2, 0x1f, 0x3f, 0, str); 
        }
        
        //======================== MAIN STATE MACHINE ======================
        switch(state)
        {
       
        case STATE_WAIT_TIMEOUT:
          printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_CFG);
          
          // check J1772
          J1772_dur=pulseIn(pin_J1772, HIGH);
          if(J1772_dur>50) { // noise control. also, deals with the case when no J1772 signal present at all
            configuration.mainsC=0.06*J1772_dur+3; // J1772 spec - every 100uS = 6A input - this will work up to 48A
            if(LCD_on) {
              sprintf(str, "IN: %dV, %dA", int(mainsV), configuration.mainsC); myLCD->printStr(1, 7, 2, 0x1f, 0x3f, 0, str);
            }
          }
          
          x=BtnTimeout(10, 3);

          if(x == 1) state = STATE_TOP_MENU; // some button was pressed
          if(x == 0) // nothing pressed
           { 
            state = STATE_CHARGE;
           }
          break;
       
        case STATE_TOP_MENU:
          printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_TOPMENU);
          x=MenuSelector2(configMenuLen, configMenu);
          switch(x)
          {
            case 0: state = STATE_CHARGE; break;
            case 1: state = STATE_CONFIG_PWR; break;
            case 2: state = STATE_CONFIG_TIMER; break;
            default: break;
          }
          break;

        case STATE_CONFIG_PWR:
          printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_INC);      
          configuration.mainsC = DecimalDigitInput3(configuration.mainsC); 
          printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_OUTC);      
          configuration.CC = DecimalDigitInput3(configuration.CC); 
          state = STATE_TOP_MENU;
          break;

        case STATE_CONFIG_TIMER:
            // now set the timer using the same button       
            printConstStr(0, 0, 2, 0x1f, 0x3f, 0x1f, MSG_LCD_TOUT);
            timeOut=DecimalDigitInput3(0); 
           state = STATE_TOP_MENU;
           break;

        case STATE_SERIALCONTROL:
          // serial control of the charger is enabled
          configuration.mainsC=300; // remove limit on input side?
          sprintf(str, "R:M%03d,V%03d,c%03d,v%03d", int(mainsV), int(outV), int(configuration.CC), int(maxOutV));
          EMWserialMsg(str); // send 'ready' status - expect controller to respond within 200ms

          // listen to commands - format 'M,ccc,vvv,sss,E' where sss is a checksum
          cmd[0]=cmd[1]=0; // reset all
          readSerialCmd(cmd);
          
          if(cmd[0]>0 && cmd[1]>0) {
            // echo reception
            sprintf(str, "E:%d,%d", cmd[0], cmd[1]);
            sprintf(SerialCommand, "M,%03d,%03d,%03d,E", cmd[0], cmd[1], getCheckSum(cmd[0], cmd[1]) );
            EMWserialMsg(str);
            configuration.CC=cmd[0];
            maxOutV=cmd[1];     
            // move to charge stat
            state=STATE_CHARGE;
          } else {
            delay(200); // wait a bit and do another check for a command - cannot wait too long due to QC timing. 
          }
          break;
         
         case STATE_CHARGE: 
           // cannot delay from here to charging function QC operation requires quick ramp after the command
           // write out the configuration to EEPROM for next time
           EEPROM_writeAnything(0, configuration);
  
           maxMainsC=configuration.mainsC; 
           mainsV=read_mV(); // for power adjustments
#ifdef drop110power       
           if(J1772_dur<50) { // but only if no J signal
             // curb power on 110VAC
             if(mainsV<160) { 
               maxMainsC=min(configuration.mainsC/2, 9.); // equivalent 15A from 110VAC // DEBUG
             }
           }
#endif 

            maxOutC=getAllowedC(configuration.CC); 
            
            //  set max hardware current protection to the fixed absMaxChargeCurrent value
            setMaxC(peakMaxC*getAllowedC(absMaxChargerCurrent)); 
            
            timer_ch=millis(); // set the timer          
            AH_charger=0; // reset AH counter
           
// zero motorcycles special now
#ifdef DELTAQ
            while(digitalRead(pin_DELTAQ)); // wait until the pin is pulled down by the BMS
#endif

            // reset the EOC pin (this pin is active LOW so reset means setting to HIGH) 
            // high means charging is commencing. this will be pulled down by the charger when charge ends
            // this also feeds a closed-loop BMS
            digitalWrite(pin_EOC, HIGH); 
              
            //========================== MAIN RUN CHARGER FUNCTION=======================
            // by this point, at least 15sec have passed since AC connection
            //                and at least 10sec since battery connection
            //                therefore, all caps should be pre-charged => close relays
            //   (note: this requires precharge resistors across relays - MAX of 300R for inrelay
            //          and MAX of 1k for outrelay. >3W power rating. Place small 1000V diode in
            //          series with the outrelay resistor - anode to battery - to avoid precharge on
            //          reverse polarity connection)     
            digitalWrite(pin_inrelay, HIGH);
            
            // check for invalid sensor configs
            // most dangerous is disconnection of the current sensor
            // generally will manifest itself by non-zero current reading while duty is zero 
            // (which it should be at this point)
            // 10A is a lot of margin for that
            if(fabs(readC())>10) {
              return;
            }
  
            // CC-CV profile, end condition - voltage goes to CV and current goes to X% of CC
            runChargeStep();
            PWM_enable_=0; // HAS to be here to ensure complete stop on any condition
  
            // make sure everything is off
            digitalWrite(pin_inrelay, LOW);
            digitalWrite(pin_fan, LOW);    
            digitalWrite(pin_EOC, LOW); // active low
            //==================== charger routine exited ===============================
  
            printClrMsg(MSG_DONE, 500, 0x1f, 0x3f, 0); 
            sprintf(str, "%dAH", int(AH_charger)); 
            if(LCD_on) {
              myLCD->printStr(0, 6, 2, 0x1f, 0x3f, 0x1f, str);      
              charger_run=1; // charger has run this mains cycle...
              state = STATE_SHUTDOWN; //STATE_TOP_MENU;   
            } else {
              EMWserialMsg(str);
              state = STATE_SERIALCONTROL; // ready for next run   
            }
  
            break; 

          default: break;
        }
        //=========================== END MAIN STATE MACHINE
      }
    }
  
}


//-------------------------------- main charger routine ------------------------------------------------
int runChargeStep() {
  pids_Kp=pids_Kp_FAST; // start with fast PID - will be changed to slow when we see some current
  digitalWrite(pin_TEST, LOW); // just to detect the transition between FAST AND SLOW pid constant
  
  maxOutC1=maxOutC;
  
  // reset V,C readings - otherwise averaging gets screwed up really badly
  outC=0; 
  outV=0; 
  outC_ADC_0=0, outC_ADC=0, outV_ADC=0, outmV_ADC=0, T_ADC=0, T2_ADC=0;

  if(LCD_on) {
    myLCD->clrScreen();
    sprintf(str, "CC=%dA, CV=%dV", int(maxOutC), int(maxOutV)); 
    myLCD->printStr(0, 4, 2, 0x1f, 0x3f, 0x1f, str);      
    delay(5000);
    myLCD->clrScreen();
  } else {
    // machine-readable
    // this assumes that only CC commands will be issued to the charger via serial
    sprintf(str, "I:%d,%d,%d", int(configuration.AH*min_CV_Crating), int(maxOutC), int(maxOutV)); 
    EMWserialMsg(str);
  }

  // reset timers - for AH metering and serial comms
  timer=millis(); // this will be reset every cycle below after AH delta is calculated
  timer_comm=timer;
  
  // turn on PWM output
  PWM_enable_=1;
  
  //============================================== MAIN CHARGER LOOP =============================================
  byte si=0; // serial command byte counter
  
  while(1) {
    // NOTE THAT outC / outV readings are all set in the interrupts

    // track targetC to maxOutC1
    targetC_ADC=1024*(k_V_C*maxOutC1
#ifdef NEG_CSENSE
          *-1
#endif
#ifdef INC_ASOUT
          *outV/mainsV
#endif
          +V_o_C)/Aref;

    if(!LCD_on) {
      while(Serial.available()) {
        str[0]=Serial.read();
        if(str[0]=='M') si=0; // reset to the beginning of command
        if(si<SerialStrSize-1) {
          SerialStr[si++]=str[0];
        } else {
          SerialStr[SerialStrSize-1]=0; // this is supposed to be the end of the command
          if(str[0]=='E') strcpy(SerialCommand, SerialStr);
          si=0;
        }
      }
    }
    
    // process serial commands and print out status only every 50ms or so
    // in LCD mode, this just increments the cycle counter for proper timing of the LCD printout
    if(millis()-timer_comm > stepDelay) { 
      n++; 
      timer_comm=millis();
      
      normT=getNormT();
  
  #ifdef DEBUG2
      Serial.print("free RAM: "); Serial.println(freeRam());
      Serial.print("       -outC_ADC="); Serial.println(outC_ADC);
      Serial.print("            -outC="); Serial.println(int(outC));
      Serial.print("            -maxOutC="); Serial.println(maxOutC);
      Serial.print("            -targetC="); Serial.println(targetC_ADC);
      Serial.print("       -outV_ADC="); Serial.println(outV_ADC);
      Serial.print("       -outmV_ADC="); Serial.println(outmV_ADC);
      Serial.print("       -T_ADC="); Serial.println(T_ADC);
      Serial.print("            -normT="); Serial.println(normT);
      Serial.print("       -duty="); Serial.println(milliduty/10000);
  //    Serial.print(""); Serial.println();
  #endif
    
      // if in Serial mode, check for commands here
      if(!LCD_on) {
        cmd[0]=cmd[1]=0; // reset all
        str[0]=SerialCommand[2]; // skipping 'M,'
        str[1]=SerialCommand[3];
        str[2]=SerialCommand[4];
        str[3]=0;
        cmd[0]=atoi(str);
        str[0]=SerialCommand[6]; // skipping ','
        str[1]=SerialCommand[7];
        str[2]=SerialCommand[8];
        str[3]=0;
        cmd[1]=atoi(str);
        str[0]=SerialCommand[10]; // skipping ','
        str[1]=SerialCommand[11];
        str[2]=SerialCommand[12];
        str[3]=0;
        if(atoi(str)!=getCheckSum(cmd[0], cmd[1])) { // checksum did not check out
          cmd[0]=0;
          cmd[1]=0;
        }
        if(cmd[0]>1 && cmd[1]>1) {
          // valid output power command
          maxOutV=cmd[1];
          maxOutC=maxOutC1=getAllowedC(cmd[0]); // this also allows for temp derating
        } else {
          // could be a special command
          // 'M,001,000,001,E' is STOP 
          if(cmd[0]==1 && cmd[1]==0) {
            PWM_enable_=0;
            return 0; // full stop
          }
        }
        // send status now - this is ~45 symbols. At 19200 bps, this is  ~30ms
        printParams(outV, outC, normT, AH_charger, maxOutC1, maxOutV);
      } else {        
        // recalc maxOutC1 - this will account for temp derating
        maxOutC1=getAllowedC(maxOutC);
        delay(30); // a delay equivalent to non-LCD execution
      }
    }
    
    //------------------------------------------------ print out stats ----------------------------------------
    // but only every few hundred cycles. defaults: measCycle_len=20, stepDelay=30
    if(n>measCycle_len) {
      n=0;

      // timer
      sec_up=(unsigned int)1.*(millis()-timer_ch)/1000;

      // check for break conditions 
      // mask the first few seconds
      if(sec_up>CV_timeout && outV > maxOutV && outC < configuration.AH*min_CV_Crating) {
        breakCycle=1;
      } else {
        breakCycle=0; // reset 
      }
      // do we REALLY need to break?
      if(breakCycle) {
        breakCnt++;
        if(breakCnt>stopCycles) {
          printClrMsg(MSG_NORMEXIT, 5000, 0, 0x3f, 0);
          return 0; 
        }
      } else {
        breakCnt=0;
      }
    
      // slow voltage control cycle here. AT Cstep=0.5A default, we are ramping down at ~0.5A/second
      // this may not be enough to avoid a bit of overvoltage beyond CV
      if(outV > maxOutV) {
        maxOutC1-=Cstep;
        if(maxOutC1<0) maxOutC1=0;
      }    
      
      // AH meter
      AH_charger+=outC*int(millis()-timer)/1000/3600;
      timer=millis();
      
      // check HVC signal from BMS
      if(digitalRead(pin_BMS)==LOW
#ifdef DELTAQ
            || digitalRead(pin_DELTAQ)
#endif
      ) { // active LOW (so you need to pull up by connecting miniBMS loop to EOC signal)
        // BMS commanding charger to stop
        // noise protection - ensure signal stays on for 100ms or so
        delay(100);
        if(digitalRead(pin_BMS)==LOW
#ifdef DELTAQ
            || digitalRead(pin_DELTAQ)
#endif        
        ) {
          // this is for real
          printClrMsg(MSG_BMSSTOP, 5000, 0x1f, 0x3f, 0);
          return 0; 
        }
      } 
            
      // check the timer
      if(timeOut>0 && (millis()-timer_ch)/60000>timeOut) {
        // timer run out
        printClrMsg(MSG_TIMEOUT, 5000, 0x1f, 0x3f, 0);
        return 0; 
      }
      
      //==================== print all parameters
      // print here only if LCD is on - otherwise print in faster loop over Serial
      if(LCD_on) {
        printParams(outV, outC, normT, AH_charger, maxOutC1, maxOutV);
      }
      
      if(outV < -10 || outC < -10) {
        // sensor polarity problems. abort
        printClrMsg(MSG_SENSEERROR, 300, 0, 0x3f, 0);
        return 1; // full stop
      }
      
      // check if need to stop - RED button pressed? - both in LCD and non-LCD modes
      if(digitalRead(pin_pwrCtrlButton)==HIGH) return 0; 
      
      mainsV=read_mV(); // read input voltage only infrequentry

#ifdef CHECKMAINS         
      // check mains
      if(mainsV<minMains) {
        delay(2000);
        printClrMsg(MSG_LOSTIN, 5000, 0x1f, 0x3f, 0);
        return 1; // error
      }
#endif
      
    }  // end measCycle loop    
    
  }; //======================================== END MAIN CHARGER LOOP ===================================

  return 0;
} // end runChargeStep()


//============================ HELPER FUNCTIONS ====================================
//================== serial comms ========================
// message FROM the charger via serial
void EMWserialMsg(const char *txt) {
  Serial.print("M,");
  Serial.print(txt);
  Serial.println(",E");
}

// message TO the charger via serial
// command syntax: M,ccc,vvv,sss,E
void readSerialCmd(int *cmd_) {
      //+++++++++++++++++++ REWRITE into either async buffer or blocking reads +++++++++++++++++++++ 
  if(Serial.available()>0) {
    if(Serial.read()=='M') {
      // this is a legit command
      Serial.read(); // dispose of comma
      str[0]=Serial.read();
      str[1]=Serial.read();
      str[2]=Serial.read();
      str[3]=0;
      cmd_[0]=atoi(str);
      Serial.read(); // dispose of comma
      str[0]=Serial.read();
      str[1]=Serial.read();
      str[2]=Serial.read();
      str[3]=0;
      cmd_[1]=atoi(str);
      Serial.read(); // dispose of comma
      str[0]=Serial.read();
      str[1]=Serial.read();
      str[2]=Serial.read();
      str[3]=0;
      Serial.read(); // dispose of comma
      if( Serial.read()!='E' || atoi(str)!=getCheckSum(cmd_[0], cmd_[1]) ) {
        cmd_[0]=cmd_[1]=0;
      }
    }
  }
}

int getCheckSum(int val1, int val2) {
  return (val1+val2)%1000;
}
//================== END serial comms ========================


//------------ ensure output power does not exceed other limits
float getAllowedC(float userMaxC) {
  userMaxC=min(userMaxC, absMaxChargerPower/maxOutV );
  userMaxC=min(userMaxC, absMaxChargerCurrent); 
  userMaxC=min(userMaxC, maxMainsC*mainsV/maxOutV);
  
  // check thermal 
  if(normT>=maxHeatSinkT) {
    // start derating
    // map 0-100% derating (relative to the current max current) to 
    // maxHeatSinkT-ABSmaxHeatSinkT heatsink range
    userMaxC=userMaxC*abs(ABSmaxHeatSinkT-normT)/(ABSmaxHeatSinkT-maxHeatSinkT);
    
    if(normT>ABSmaxHeatSinkT) {
      // overheating, stop charger, wait until temp drops enough to restart
      PWM_enable_=0;
      if(LCD_on) myLCD->clrScreen();        

      while(1) {
        sprintf(str, "Cool from %dC", (int)normT);
        printMsg(str, 1000, 0, 1, 0x1F, 0, 0);
        normT=getNormT();
        if(normT<midHeatSinkT) {
          if(LCD_on) myLCD->clrScreen();
          PWM_enable_=1; // restart PWM
          maxOutC1=maxOutC; // full power
          break; // exit cycle when the temp drops enough
        }
      }

    } // ABSmaxHeatSink condition   

  } // end thermal management            

  return userMaxC;
}

void setMaxC(float maxC) {
#ifdef NEG_CSENSE
  // hardware limits in case of opposite direction of the sensor
  Timer1.setPwmDuty(pin_maxC, 1023); // need something more than 3 volts as zero-current output is 2.5V...
#else
  Timer1.setPwmDuty(pin_maxC, 1024./Aref*(V_o_C+k_V_C*maxC));
#endif
}


//============================ current readout functions =====================
//------------ calc current value from ADC value
float readC() {
  return (Aref/1024.*outC_ADC-V_o_C)/k_V_C
#ifdef NEG_CSENSE
          *-1
#endif
#ifdef INC_ASOUT
          *mainsV/outV
#endif
          ;
}

//============================ voltage readout functions =====================
// output voltage
float readV() {  
  return (Aref/1024.*
#ifdef DCDC_BUCK // output voltage is lV in case of buck
          outmV_ADC
#else 
          outV_ADC
#endif
          -V_o_bV)*divider_k_bV;
}

// input voltage
float read_mV() {
#ifdef PC817 
  // 3V is a threashold between 120V and 240V - but may require adjustment on a per-unit basis
  if(Aref/1024.*outmV_ADC < 3) return 240;
  return 120;
#endif

  return (Aref/1024.*
#ifdef DCDC_BUCK // input voltage is hV in case of buck
          outV_ADC
#else 
          outmV_ADC
#endif
          -V_o_mV)*divider_k_mV
#ifndef DCinput
          /1.41
#endif
          ;
}
//============================ end voltage readout functions =====================


//====================== temperature readout functions ===========================
// compute the equivalent (normalized) charger temp
byte getNormT() {
  // assume max sink T is 55, max t2 (inductor) is 85 - approx but reasonably close to reality
  // therefore need to rescale t2 by 55/85=0.65
  // BUT ONLY IF WE HAVE THIS SECONDARY MEASUREMENT
#ifdef IND_Temp
  return max(read_T(T_ADC), read_T(T2_ADC)*0.65);
#else
  return read_T(T_ADC);
#endif
}
// master temp readout, using formulas from http://en.wikipedia.org/wiki/Thermistor
byte read_T(unsigned int ADC_val) {
  return (byte)1/( log(1/(1024./ADC_val-1)) /4540.+1/298.)-273; 
}


//=========================================== Communication (LCD / Serial) Functions =========================
// main parameter printing function
void printParams(float outV, float outC, int t, float curAH, float maxC, float maxV) {
  if(LCD_on) {
    sprintf(str, "%s - D: %d  ", VerStr, int(milliduty/10000), 1); myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x1f, str);      
    sprintf(str, "I: %dV   ", int(mainsV)); myLCD->printStr(0, 3, 2, 0x1f, 0x3f, 0, str);      
    sprintf(str, "O: %dV, %d.%dA   ", int(outV), int(outC), abs(int(outC*10)%10) ); myLCD->printStr(0, 4, 2, 0x1f, 0x3f, 0, str);      
#ifdef DEBUG0
    sprintf(str, "* t%d-a%d %d %d", targetC_ADC, outC_ADC, int(10000*k_V_C), int(10*V_o_C)); myLCD->printStr(1, 5, 2, 0x1f, 0, 0, str);
#endif
    sprintf(str, "T: %dC ", t); myLCD->printStr(0, 7, 2, 0x1f, 0, 0, str);
    sprintf(str, "%d dAH, %u sec", int(curAH*10), sec_up); myLCD->printStr(0, 8, 2, 0x1f, 0x3f, 0, str);      
  } else {
    // machine-readable
    // format: [D]uty in 0.1%, [C]urrent in 0.1A, [V]oltage in 1.0V, [T]emp in C, [O]utput AH in 0.1AH, [S]um (checksum)
    sprintf(str, "S:D%03d,C%03d,V%03d,T%03d,O%03d,S%03d", int(milliduty/10000), int(outC*10), int(outV), t, int(curAH*10), getCheckSum(int(outC*10), int(outV)));
    EMWserialMsg(str);
#ifdef DEBUG1
    sprintf(str, "S2:c%03d,v%03d,%05u", int(maxC*10), int(maxV), (unsigned int)millis());
    EMWserialMsg(str);
#endif
  }
}
// printing primitives
void printClrMsg(const byte msg_id, const int del, const byte red, const byte green, const byte blue) {
  if(LCD_on) {
    strcpy_P(str, (char*)pgm_read_word(&(msg_long_table[msg_id]))); 
    myLCD->clrScreen();
  } else {
    strcpy_P(str, (char*)pgm_read_word(&(msg_short_table[msg_id]))); 
  }
  printMsg(str, del, 0, 2, red, green, blue);
}
void printConstStr(int col, int row, int font, byte red, byte green, byte blue, const byte msg_id) {
  strcpy_P(str, (char*)pgm_read_word(&(msg_lcd_table[msg_id]))); 
  printMsg(str, 0, col, row, red, green, blue);
}
void printLabel(const char * label, const byte col, const byte row, const byte red, const byte green, const byte blue) {
  strcpy(str, label);
  printMsg(str, 0, col, row, red, green, blue); 
}
void printMsg(char *str_, const int del, const byte col, const byte row, const byte red, const byte green, const byte blue) {
  if(LCD_on) {
    myLCD->printStr(col, row, 2, red, green, blue, str_);      
    delay(del);
  } else {
    EMWserialMsg(str_);
  }  
}


unsigned int MenuSelector2(byte selection_total, const char * labels[])
{
  byte selection = 0;
  byte temp_selection = 1;
  
  printConstStr(0, 3, 2, 0x1f, 0x3f, 0x1f, MSG_LCD_BLANK);
  printLabel(labels[temp_selection-1], 1, 3, 0x1f, 0x3f, 0x1f);

  while(!selection)
  {
    if(digitalRead(pin_pwrCtrlButton) == HIGH)
    {
      ++temp_selection;
      if(temp_selection > selection_total) temp_selection = 1;
      printConstStr(0, 3, 2, 0x1f, 0x3f, 0x1f, MSG_LCD_BLANK);
      printLabel(labels[temp_selection-1], 1, 3, 0x1f, 0x3f, 0x1f);
      
      // ideally, this should call a StatusDisplay method and simply pass selection index
      // StatusDisplay should encapsulate all the complexities of drawing status info onto the screen
      // alternatively myLCD can be re-purposed for this
    }
    else
    if(digitalRead(pin_pwrCtrl2Button) == HIGH)
    {
      selection = temp_selection;
      printConstStr(0, 3, 2, 0x1f, 0x0, 0x0, MSG_LCD_BLANK);
      printLabel(labels[selection-1], 1, 3, 0x1f, 0x0, 0x0);
      // similar to the above, should delegate display to StatusDisplay object
    } 
    delay(80);
  }

  delay(200);
  
  return selection - 1;
}


byte BtnTimeout(byte n, byte line) {
  while(n > 0) {
    sprintf(str, "%d sec ", n); 
    printMsg(str, 0, 0, line, 0x1f, 0x3f, 0);

    for(byte k=0; k<100; k++) {
      if(digitalRead(pin_pwrCtrlButton)==HIGH || digitalRead(pin_pwrCtrl2Button) == HIGH) return 1;
      delay(10);
    }
    --n;
  }
  return 0;
}

int DecimalDigitInput3(int preset)
{
  byte digit[3] = { preset/100, (preset/10)%10, (preset%10) };
  byte x = 0; // 0-1-2-3-4
  // 0x30 ascii for "0"
  str[1] = 0x0; // eol 

  while(x < 4)
  { 
    if(digitalRead(pin_pwrCtrlButton) == HIGH) {
      if(x > 2) x = 0;
      else {
        // increment digit
        ++digit[x];
        // wrap at 4 (for 100s) or at 9 (for 10s and 1s) 
        if(x == 0 && digit[x] > 4) digit[x] = 0;
        if(digit[x] > 9) digit[x] = 0;
      }      
    } else 
    if(digitalRead(pin_pwrCtrl2Button) == HIGH) {
      ++x;
    } 

    printDigits(0, digit, 1);
  
    if(x < 3) {
      // still on digits. Reprint the digit we are on in a different color now so we see what's being changed
      str[0] = 0x30+digit[x];
      printDigit(x, 0, str);
    } else 
    if(x == 3) {
      // selection made - show all in the 'changing' color
      printDigits(0, digit, 0);
    }
    
    delay(150);
  }
  
  printDigits(8, digit, 0);

  return (digit[0]*100+digit[1]*10+digit[2]);
}

void printDigits(byte start, byte * digit, byte stat) {
  str[0] = 0x30+digit[0];
  printDigit(start++, stat, str);
  str[0] = 0x30+digit[1];
  printDigit(start++, stat, str);
  str[0] = 0x30+digit[2];
  printDigit(start, stat, str);
}
void printDigit(byte x, byte stat, char * str) {
  if(stat==0) printMsg(str, 0, x, 5, 0x1f, 0x3f, 0x0); // yellow
  if(stat==1) printMsg(str, 0, x, 5, 0x8, 0x8, 0x1f); // blue
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}


//========================================= GRAVEYARD ================================================
//================================ ALL HELPER FUNCTIONS ======================
//int runChargeStep(int cycleType, float CX, int stopType, float stopValue);
//void stopPWM();
//float readV(unsigned int);
//float read_mV(unsigned int);
//void setMaxC(float maxC);
//float read_mC();
//float readC(unsigned int);
//int getNormT();
//int read_T(unsigned int);
//float sampleRead(byte pin);
//float get_k_V_C(int selection);
//float get_V_o_C(int selection);
//void resetDelayParams();
//void printParams(float duty, float outV, float outC, int t, float curAH, float dVdt);
//void printConstStr(int col, int row, int font, byte red, byte green, byte blue, const byte msg_id);
//void printClrMsg(const byte msg_id, const int del, const byte red, const byte green, const byte blue);
//void EMWserialMsg(const char *txt);
//void readSerialCmd(int *cmd_);
//char *ftoa(char *a, double f, int precision);
//void updateMovingAverages(float V);
//unsigned int MenuSelector2(unsigned int selection_total, const char * labels[]);
//byte isBtnPressed();
//int BtnTimeout(int n, int line);
//int DecimalDigitInput3(int preset);
//void printDigits(int start, int * digit, int stat);
//void printDigit(int x, int stat, char * str);
//=================================== END helper functions ======================================



