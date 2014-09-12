 /*
EMW SmartCharger-12000
A 12kW+ charging system

DIY charger inspired by the work of SimonRafferty & jackbauer on DIYelectriccar.com:
http://www.diyelectriccar.com/forums/showthread.php/200-build-your-own-intelligent-charger-36627.html. 

DETAILED FORUM DISCUSSION OF THIS DESIGN IS AT 
http://www.diyelectriccar.com/forums/showthread.php/10kw-60a-diy-charger-open-source-59210p39.html

Controller: Arduino Pro Mini 5V (based on a ATmega328P-PU microcontroller)

Pinout assignments: see below in code


----------- Basic code structure:
------ Startup: 
* check for LCD presence. if none (programming button in the programming state), launch in the serial-controlled mode 
* 2 timeouts - one 5 sec for config, one 10 sec for power setting. can be interrupled by any button
* check mains voltage. If 110, limit power to ~1.5kW
* set duty cycle to 0
------ Charging (CV or CC):
* increase duty cycle until the condition is met (target voltage or target current)
* monitor condition by taking frequent samples averaging over 120Hz ripple waveform
* based on average value of condition, change duty cycle accordingly (slow down update frequency
  as we get closer to the target voltage)
* break when exit condition satisfied or stop / pause commands received

============================== SERIAL COMMAND SYNTAX ===============================
M,ccc,vvv,E - start charger from 'READY' state with ccc CC point and vvv CV point
              charger will echo settings. make sure they are what you sent
M,001,000,E - emulate red button press (to pause the charger when in running state)
              second 'RED BUTTON' command will result in termination of charge
M,000,001,E - emulate green button press (to resume from paused state)
=================== END SERIAL COMMAND SYNTAX ======================================
============================== SERIAL STATUS REPORTING =============================
every second or so, the charger will report its status. This will be contextual -
either a simple 'READY' string or a dump of the critical charging parameters in the 
following format:
'M,D0,C965,V334,T-68,O1,R0,E' - [D]uty 0%, output [C]urrent 96.5A, output 
[V]oltage 334V, heatsink [T]emp -68C, [O]utput charge 0.1AH, [R]untime 0 minutes
=================== END SERIAL STATUS REPORTING FORMAT =============================

Original version created Jan 2011 by Valery Miftakhov, Electric Motor Werks, LLC & Inc. 
Copyright 2011-2013

This software is released as open source and is free for any personal use.
Commercial use prohibited without written approval from the Author and / or EMW.
*/

#include <avr/pgmspace.h>
#include <MemoryFree.h>
// need this to remap PWM frequency
#include <EEPROM.h>
#include "EEPROM_VMcharger.h"
#include <TimerOne.h>

// define sensor configurations
#define Tamura_50B   0  // Tamura 50A sensor (bidirectional) - half flow
#define Tamura_600B   1  // Tamura 600A sensor (bidirectional) 
#define Tamura_150B   2  // Tamura 150A sensor (bidirectional) 
#define Allegro_150U 10 // Allegro 150A sensor (unidirectional)
#define Allegro_100U 11 // Allegro 100A sensor (unidirectional)
#define Allegro_050U 12 // Allegro 50A sensor (unidirectional)

//----------- DEBUG switch - careful, this disables many safety features...---------------
// #define DEBUG0 // just increase maximums
// #define DEBUG // additional printouts / delays / etc.
//----------------------------------------------------------------------------------------


//============================== MAIN SWITCHES
 #define SmartCharge // 12kW chargers
// #define QuickCharge // 25kW chargers
 #define PFC // is this a PFC unit?


// ================= MAIN SWITCHES DEFAULT VALUES FOR EMotorWerks chargers ========================
// by-version defiles
#ifdef SmartCharge // the latest version of the PFC version of SmartCharge-12000 12kW chargers
  #define OUTC_SENSOR Allegro_100U // default is Allegro_100U
  #define A7520_V // using A7520 optoisolation for outV sensing? (as opposed to ISO124)
  #define PC817 // mains voltage sensing based on a crude regular opto (V13 boards)
#endif
#ifdef QuickCharge // the latest version of the PFCDirect or QuickCharge-25000 25kW chargers
  #define OUTC_SENSOR Tamura_150B // Tamura_150B - this is for PFCdirect 25kW units only V3 (after Dec 2 2013)
  #define A7520_V // using A7520 optoisolation for outV sensing? (as opposed to ISO124)
  #define A7520_mV // using A7520 optoisolation for mV sensing? (as opposed to ISO124)
  #define PFC // all QuickCharge units are PFC
  #define PFCdirect // is this a PFCDirect 25kW unit?
  #define NEG_CSENSE // in post-Oct'13 PFCdirect units, current runs in opposite direction to make 3.3V logic compatible
#endif
// universal defines
#define INmC_SENSOR Allegro_150U // this needs to stay uncommented!
#define LCD_SPE // are we using the SPE version of the LCD (shipped after October 2013)
// #define drop110power // reduce power to ~1.5kW when connected to 110VAC?
// ===================================================================================================

#ifdef PFCdirect
  const float upperR0_mV=2400.; // 2.4M in PFC direct units to extend sensing to 420V
  const float upperR0_bV=2400.; // 2.4M in PFC direct units to extend sensing to 420V
#else
  const float upperR0_mV=2000.; // 2M in regular units
  const float upperR0_bV=2000.; // 2M in regular units
#endif

//------------------------------ MAIN SWITCHES STORAGE AREA - DO NOT UNCOMMENT --------------------
// #define INC_ASOUT // what we specified as an output sensor above actually sits on the input (some initial UPP units)
// #define MCC100A // 100A output rating - use ONLY with a custom-wound inductor
// #define buck_Ecore // Ecore output inductor - limit output current to lower value
// #define DCinput // is this being connected to the DC input of AC? matters for input voltage sense 
// #define UVLO // enable gate supply undervoltage protection?
// #define NiXX // do we want support for Nickel chemistries?
// #define IND_Temp // do we have a second temp probe on the inductor?
// #define debugpower // doubles the power limits - careful - it may blow up your charger
//------------------------------- END MAIN SWITCHES STORAGE AREA ------------------------------------


// some hardcoded constants - DO NOT CHANGE
#define PFCvoltage 370 // normally 370
#define PWM_res 1024
#define MAXDUTY 980 // very short off pulses are bad (diode does not recover by the time IGBT turns on again - generally limit Toff to MORE than 1us)
#define period 50 // microseconds; from Oct 10 2013, this defaults to 20kHz due to use of faster IGBTs. For kits with older IGBTs, use 60-70


// LCD includes - have to be here in the code as they depend on some switches above
// LCD library for 4D systems display (http://www.4dsystems.com.au/prod.php?id=121)
#ifdef LCD_SPE
  #include <uLCD_144_SPE.h>
  uLCD_144_SPE *myLCD;
#else
  #include <uLCD_144.h>
  uLCD_144 *myLCD;
#endif
int LCD_on=0; // this defines manual vs serial-controlled operation
int cmd[2]={0, 0}; // command variables, used in serial comms

//============================================== define messages ==================
// using program memory as we are now running out of SRAM...
// see http://arduino.cc/en/Reference/PROGMEM for reference
// some additional good memory tips at http://liudr.wordpress.com/2011/02/04/how-to-optimize-your-arduino-memory-usage/
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
const byte MSG_10 = 0x0A;
const byte MSG_11 = 0x0B;

#ifndef LCD_SPE
  prog_char msg_long_0[] PROGMEM = "Thank you for choosing EMW! Any BTN to CFG";
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
  prog_char msg_long_10[] PROGMEM = "X";
  prog_char msg_short_10[] PROGMEM = "X";
  prog_char msg_long_11[] PROGMEM = "X";
  prog_char msg_short_11[] PROGMEM = "X";
#else 
  prog_char msg_long_0[] PROGMEM = "Thank you for \nchoosing EMW! \nAny BTN to CFG";
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
  prog_char msg_long_10[] PROGMEM = "X";
  prog_char msg_short_10[] PROGMEM = "X";
  prog_char msg_long_11[] PROGMEM = "X";
  prog_char msg_short_11[] PROGMEM = "X";
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
  msg_long_9,
  msg_long_10,
  msg_long_11
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
  msg_short_9, 
  msg_short_10,
  msg_short_11
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
  prog_char msg_lcd_9[] PROGMEM = "press BTN to\n adjust";
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
const byte pin_mC=7; // will only work in V12 control boards (June 2013). needed only for full digital PFC control
//========== digital pins
// 0/1 reserved for serial comms with display etc
const byte pin_pwrCtrlButton=2; // this is wired to the button (used for menu step)
const byte pin_pwrCtrl2Button=3; // this is wired to the button2 (used for menu select)
const byte pin_inrelay=4; // precharges input caps - normally pin 4, in some units pin 6 running fan relay
const byte pin_outrelay=5; // protects from reverse polarity on traction battery, precharges output resistors
const byte pin_PWMpulldown=6; 
const byte pin_J1772=7; // J1772 pilot input. 1k is hardwired on V14+ pcbs so J1772 will power on on connect
const byte pin_fan=8; // fan control - this is pin4 in all kits shipped before Mar '2912
const byte pin_PWM=9; // main PWM pin

// max current reference voltage (using PWM) -  was 6 in the V13 pcb (kits shipped before March 2012)
// now moved to pin 10 so that we can use higher PWM frequency 20kHz PWM
const byte pin_maxC=10; 

// 110/220vac relay control - for non-PFC units only
// If PFC is connected, relay would never close so can be removed
// also can be left unused / unconnected if separate 110V / 220V inputs are used 
const byte pin_110relay=11; // in kits shipped before Mar 2012, this is pin 5

const byte pin_EOC=12; // end-of-charge output (see pinout diagram) - pulled low when charge is complete

// end-of-charge input from BMS. Pull low / disconnect from positive TTL signal to activate
//     (normallly will be realized via connecting NC BMS loop between this pin and EOC pin (or +5V)
const byte pin_BMS=13; 
//---------------- END PINOUTS -----------------------


//============= BATTERY INFO  =====
struct config_t {
  int battType;
  int nCells;
  int AH;
  int CV; // per cell
  int CC; // max output current
  int mainsC; // max input current
  // sensor config
  float Vcal;
  float Vcal_k;
  float Ccal;
} configuration;

#ifdef NiXX
  // Nickel chemistries dVdt cutoff
  const float dVdt_stop=0.; // in %/s. at 1C, safe value is between -1E-05 and +1E-05
#endif

// DO NOT CHANGE THESE!
const int minMains=30; // min mains voltage to (1) test sensor connectivity and (2) detect mains disconnect 
const float min_CV_Crating=0.05; // wait until the current goes to XC (use values from your battery's datasheet)
// spread for duty cycle ramp conditions to avoid jitter - in volts
// With 10k resistor, 40ma sensor, 51V/A constant, voltage sensitivity is ~1V so no point setting this lower
const int spreadV=2.;
// With 50A sensor, 0.06V/A constant, current sensitivity is ~0.1A. But current being off is not a big deal...
const int spreadC=1.; 
float maxOutV=0; // absolute maximum output voltage - will be set later in the code
int minBattV;
#ifdef PFC
  const float charger_efficiency=0.93; 
#else
  const float charger_efficiency=0.95; 
#endif
// ------------------------------- END battery constants -----------------------------


//---------------- MAX CURRENTS
// input currents (used only for DC-DC units (if DCinput switch is active)
#ifdef DCinput
  float MAXinputC=100;
#endif
// absolute maximum average output current (used in CV mode) - leave at 0 here - will be set via power menu
float maxOutC=0.; 
#ifdef MCC100A
  float absMaxChargerCurrent=100; // 100A rating with high-current output toroid inductor
#else
  #ifdef buck_Ecore
    float absMaxChargerCurrent=40; // 40A default rating with old Ecore inductors
  #else
    float absMaxChargerCurrent=70; // 70A default rating with new toroid inductors
  #endif
#endif
#ifdef PFCdirect
  float absMaxChargerPower=25000; // 25kW rating for PFCDirect units with new 5-6" toroid inductors
#else
  #ifdef debugpower
    float absMaxChargerPower=20000; // 20kW for testing
  #else
    float absMaxChargerPower=12000; // 12kW rating for regular units with new 4" toroid inductors
  #endif
#endif
// when does the current limiter kick in? 1.2-1.3 is a good compromise to get the most out of 
// the input caps while keeping overall ripple below 50% 
// this is mostly relevant for 120Hz ripple. Switching frequency ripple is controlled by the automatic
// frequency selection and low-ESR high-freq output cap
// if using smaller inductors (e.g., <200 uH), may have to use higher ratio here (e.g., 1.6-1.8)
const float instantMaxCRatio=1.6; 
int timeOut=0; // in min, 0 means no timeout

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
byte midHeatSinkT=45; // turn on the fans here; also wait until cool down to this temp before resuming at the prev power level 
byte lowHeatSinkT=35; // turn off the fans here 
//--------------------------------------------------------

// sensor supply
const float Vcc=5.0; 
float V_o_V0=0.; // 0.0 for voltage transducers (ISO124)
float V_o_C0=Vcc/2;
const float Aref=Vcc; 

//=============== voltage dividers settings ===========================
const float gain_7520=Vcc/0.512*0.99; // per datasheet, assuming Vref=Vcc, accounting for input resistance

//--------- mains voltage 
float divider_k_mV=-1.; 
#ifdef A7520_mV
  // resistor from -5V regulator; should form a ~-.25V divider together with the 
  // bottom resistor => >20x * bottom resistor 
  // for 2.7k bottom resistor, pick between 60k and 82k; 68k is a good choice... 
  const float V_o_mV0=Vcc/2-Vcc*2.7/68.*gain_7520; // -5V input, 2.7k bottom resistor, ~10x gain; // ~2.5V for A7520
  const float lowerR0_mV=2.7*gain_7520; // +-0.256V range for input, ~10x gain, 2.7k bottom resistor
  const float lowerR_mV=lowerR0_mV;
#else
  const float V_o_mV0=V_o_V0;
  const float lowerR_mV=23.79; // 1/(1/27.+1/200.)
#endif

//--------- battery voltage 
float divider_k_bV=-1.;
#ifdef A7520_V
  // resistor from -5V regulator; should form a ~-.25V divider together with the 
  // bottom resistor => >20x * bottom resistor 
  // for 2.7k bottom resistor, pick between 60k and 82k; 68k is a good choice... 
  const float V_o_bV0=Vcc/2-Vcc*2.7/68.*gain_7520; // -5V input, 2.7k bottom resistor, ~10x gain; // ~2.5V for A7520
  const float lowerR0_bV=2.7*gain_7520; // +-0.256V range for input, ~10x gain, 2.7k bottom resistor
  const float lowerR_bV=lowerR0_bV;
#else
  const float V_o_bV0=V_o_V0;
  const float lowerR0_bV=27.; // 27k
  const float lowerR_bV=23.79; // in parallel with 200k input resistance of the iso124
#endif
float V_o_bV=V_o_bV0;
//==================================== end voltage dividers setup =========================

//=================================== charger current sensor ==============================
// V/A constant for the charger output current sensor 
// some small value so that we don't overcurrent by mistake. this will be replaced later in code
float k_V_C=0.01; 
float k_V_mC=0.01; 
float V_o_C=0.6; 
float V_o_mC=0.6; 
//=================================== END charger current sensor ==========================

//===================== charger cycle timers =====================================
#define SLOWUPDATE // slow down update upon reaching the target condition
// for stepDelay=1000, use measCycle_len=300, dVdt_measCycles=150
// when changing stepDelay, change the other 2 variables so that stepDelay*measCycle_len = 0.5-1 sec
// and stepDelay*measCycle_len*dVdt_measCycles = 100-200 sec
const byte stepDelay0=4; // primary charger loop delay in millioseconds
const byte measCycle_len0=100; // how many primary loop cycles per display cycle
const byte nSamplesStopVar0=10; // how many samples for moving averages of output voltage / current
int stepDelay; // this will be changed in the loop
int measCycle_len; // this will be changed in the loop
byte nSamplesStopVar;
const byte stopCycles=5; // how many primary charger cycles to require stop condition to exist before exiting
const byte waitReadSamples=200; // wait between samples for voltage / current readouts in microseconds
const byte nReadSamples0=2; // how many samples to average in a single call of readX() functions
const byte nReadSamples_mV=50; // how many samples to average in a single call of read_mV() function
byte nReadSamples;
int CV_timeout=10; // what is the max duration (in mins) CV loop is allowed to spend below C stop; should be > ramp time of charger
//===================== end charger cycle timers =================================

//=========== these should be global vars
float duty=0, duty_crit=0;
float mainsV=0, outV=0, outC=0;
float outC_avg=0., outV_avg=0.;
byte charger_run=0;
unsigned long timer=0, timer_ch=0, timer_step=0;
float AH_charger=0;
unsigned int min_up=0;
char str[64];
byte state;
float temp;

#ifdef NiXX  
  //------------------------- Running Averages for dV/dt calcs -------------
  float V_ravg[2];
  unsigned long t_ms = 0;
  unsigned long ele_counter=0;
  float dVdt = 0.0;
#endif

//-----------------------Navigate Menu--------------------
const char * configMenu[] = { " Run  ", " Pwr  ", " Time "  };
const unsigned int configMenuLen = 3;
const char * menuNavigate[] = { " Yes ", " No " };
const unsigned int menuNavigateLen = 2;
// ------------- end global vars ---------------------------


//================================ ALL HELPER FUNCTIONS ======================
int runChargeStep(int cycleType, float CX, int stopType, float stopValue);
void stopPWM();
float readV();
float read_mV();
void setMaxC(float maxC);
float readC();
float read_mC();
int getNormT();
int read_T(byte pin);
float sampleRead(byte pin);
float get_k_V_C(int selection);
float get_V_o_C(int selection);
void resetDelayParams();
void printParams(float duty, float outV, float outC, int t, float curAH, float dVdt);
void printConstStr(int col, int row, int font, byte red, byte green, byte blue, const byte msg_id);
void printClrMsg(const byte msg_id, const int del, const byte red, const byte green, const byte blue);
void EMWserialMsg(const char *txt);
void readSerialCmd(int *cmd_);
char *ftoa(char *a, double f, int precision);
void updateMovingAverages(float V);
unsigned int MenuSelector2(unsigned int selection_total, const char * labels[]);
byte isBtnPressed();
int BtnTimeout(int n, int line);
int DecimalDigitInput3(int preset);
void printDigits(int start, int * digit, int stat);
void printDigit(int x, int stat, char * str);
//=================================== END helper functions ======================================


void setup() {
  // battery type 
// 0 = Lithium, 1 = NiMh/NiCad
const char * battTypeLabel[] = { "LiFePo4", "NiXX   " };
int battTypeLen = 2;

// min battery voltage - if below this, do not start the charger
const int minBattVs[2]={25, 9}; // in 0.1V units

// CV constant (N/A for LA, Ni)
const int CVs[4]={35, -10}; // in 0.1V units; charging voltage for CALB 3.6V per cell. Using 3.5 here to ensure reliable detecion of end-of-charge for a bottom-balanced pack


  #ifdef DEBUG0
    // double ratings for testing 
    absMaxChargerCurrent*=2; // this may result in current sensor saturation! watch out!  
    absMaxChargerPower*=2; 
  #endif

  // set analog input pins
  pinMode(pin_C, INPUT);
  pinMode(pin_bV, INPUT);
  pinMode(pin_heatSinkT, INPUT);
  pinMode(pin_12Vsense, INPUT);
  pinMode(pin_mV, INPUT);

  // digital inputs
  pinMode(pin_pwrCtrlButton, INPUT);
  pinMode(pin_pwrCtrl2Button, INPUT);
  pinMode(pin_J1772, INPUT);
  pinMode(pin_BMS, INPUT);

  // set output digital pins
  pinMode(pin_PWMpulldown, OUTPUT);
  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_maxC, OUTPUT);
  pinMode(pin_EOC, OUTPUT);
  pinMode(pin_fan, OUTPUT);
  pinMode(pin_inrelay, OUTPUT);
  pinMode(pin_outrelay, OUTPUT);
  pinMode(pin_110relay, OUTPUT);
  
  //=================================== finalize init of the sensors =============================
  // input current
  k_V_mC=get_k_V_C(INmC_SENSOR);
  V_o_mC=get_V_o_C(INmC_SENSOR);
  // output current
  k_V_C=get_k_V_C(OUTC_SENSOR);
  V_o_C=get_V_o_C(OUTC_SENSOR);
  
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

  nReadSamples=nReadSamples0; // need this here (otherwise all measurements in setup are screwed)

  //==================================== ONE-TIME CONFIG =================================
  // check if needed to go into config 
  int forceConfig=0;
  EEPROM_readAnything(0, configuration);
  // reset configuration if the green button is pressed at charger start
  if(configuration.CC<=0 || digitalRead(pin_pwrCtrl2Button)==1) {
    forceConfig=1; // first time running the charger after assembly
    configuration.Vcal_k=1.; // prefill the calibration with unity so we don't get zero readings if calibration menu is skipped
    configuration.CV=CVs[configuration.battType]*10;
    // set the rest of the vars
    configuration.Vcal=0;
    configuration.Ccal=0;
  }
  
  int x = 0;
  const byte STATE_DONE = 0xff;
  const byte STATE_BT = 0x0;
  const byte STATE_CV = 0x1;
  const byte STATE_CELLS = 0x2;
  const byte STATE_CONFIRM = 0x3;
  const byte STATE_CAPACITY = 0x4;
  const byte STATE_CALIBRATE = 0x5; // sensitivity calibration only. zero point calibration done automatically on power-on
  state = STATE_BT;
    
  if(LCD_on) {  
    myLCD->clrScreen();
    myLCD->setOpacity(1);
  } else {
    state=STATE_DONE; // skip config altogether if no LCD
  }    
  
  while(state != STATE_DONE)
  {
    switch(state)
   {
     case STATE_BT:
       printClrMsg(MSG_THX, 50, 0, 0x3f, 0);
       // if config is not forced, just timeout and send to end of config. Else, wait until button press
       if(forceConfig==0) {
         forceConfig=BtnTimeout(5, 5); // -1 if no button pressed; 1 otherwise
       }
       if(forceConfig==-1) {
         state=STATE_DONE;
       } else { // forceConfig=1 here
         myLCD->clrScreen();
         printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_CELLTYPE);
         configuration.battType=MenuSelector2(battTypeLen, battTypeLabel);
         state = STATE_CV;
       }
       break;
     case STATE_CV:
       printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_CV);
       configuration.CV = DecimalDigitInput3(configuration.CV); 
       state = STATE_CELLS;       
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
       // output current zero calibration - this assumes that there is no load on startup - this is especially important for 
       // PFCdirect units!
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
#ifndef PFCdirect
       // no discharging for PFCdirect units
#ifndef LCD_SPE
       sprintf(str, "Discharge output (now at %dV), press BTN", int(outV));  
#else
       sprintf(str, "Discharge output\n (now at %dV), \npress BTN", int(outV));  
#endif
       myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, str);
       while(!(digitalRead(pin_pwrCtrlButton) || digitalRead(pin_pwrCtrl2Button)));
       outV=readV(); // re-read after discharge
#endif
       // now actual zero cal
       if(fabs(outV)<40) { // if too far off, fault out
         // output voltage calibration
         temp=outV/divider_k_bV;
         V_o_bV+=temp; // this needs to be adjusted HERE because we are calling readV() again below for sensitivity calibration
         configuration.Vcal=temp; 
         printConstStr(0, 5, 2, 0x1f, 0x3f, 0x00, MSG_LCD_CAL0);
         delay(1000);
       }
       
       // now calibrate voltage sensor slope
       // first, double-check we have reset to zero point
       // for PFCdirect units, this will only work if ONLY 12V is powered up, no main AC connected!
       outV=readV(); // get the readings with zero-point already calibrated
       if(fabs(outV)<3) { // should be pretty tight after zero calibration
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
       state = STATE_CONFIRM;
       break;
     case STATE_CONFIRM:
       myLCD->clrScreen();
       printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_CONFIRM);
       sprintf(str, "%d %s cells, %dAH", configuration.nCells, battTypeLabel[configuration.battType], configuration.AH);       
       myLCD->printStr(0, 1, 2, 0x1f, 0x3f, 0x00, str);
       x=MenuSelector2(menuNavigateLen, menuNavigate);
       if(x == 0) state = STATE_DONE;
       if(x == 1) state = STATE_BT;
       break;
     default: break;
   } 
  }

  // parameters calculated from config variables go here
  // adjust core sensor constants
  V_o_bV=V_o_bV0+configuration.Vcal;
  V_o_C+=configuration.Ccal;
  divider_k_bV*=configuration.Vcal_k; 
  
  minBattV=minBattVs[configuration.battType]*configuration.nCells/10;
}
  

void loop() {  
  // ---------------real loop()
  float pwr;
  int J1772_dur;
  mainsV=read_mV();
  outV=readV();
  
  maxOutV=float(configuration.CV)/100*configuration.nCells;

  // check for battery connection
  int x=1; 
  if(outV<minBattV) { // as minBattV is given in 0.1V units
    // either no battery or reverse polarity
    printClrMsg(MSG_NOBATT, 50, 0x1f, 0x3f, 0);
    if(LCD_on) x=BtnTimeout(10, 7);
  }

  // protect buck's freewheeling diode from high current at low duty - limit to average of 80A through diode
  absMaxChargerCurrent=min(absMaxChargerCurrent, 80./(1-outV/400));
  
  delay(1000);

  // run charger if: 
  //         (1) charger has NOT been run yet in this cycle, or 
  //         (2) has been run over a week ago
  //         (3) positive battery voltage has been detected or zero/negative been commanded to ignore
  //         (4) green button is pressed to override
    if(x==1 && (isBtnPressed()==0b01 || charger_run==0 || (charger_run==1 && millis()-timer_ch>1000*3600*24*7))) {
      // get the charger going
      x=0;
      
      //----------------------------
      // run state machine:
      const byte STATE_TOP_MENU = 0x0;
      const byte STATE_CONFIG_PWR = 0x1;
      const byte STATE_CONFIG_TIMER = 0x2;
      const byte STATE_CHARGE = 0x4;
      const byte STATE_WAIT_TIMEOUT = 0x5;
      const byte STATE_SERIALCONTROL = 0x10;
      const byte STATE_SHUTDOWN = 0xff;
      state = STATE_WAIT_TIMEOUT;
      if(configuration.CC<=0) state=STATE_CONFIG_PWR;
      if(!LCD_on) state=STATE_SERIALCONTROL;
      
      while(state != STATE_SHUTDOWN)
      {
        if(LCD_on) {
          myLCD->clrScreen();
          printConstStr(0, 6, 2, 0x1f, 0x3f, 0, MSG_LCD_PARAMS);
          sprintf(str, "IN: %dV, %dA", int(mainsV), configuration.mainsC); myLCD->printStr(1, 7, 2, 0x1f, 0x3f, 0, str);
          sprintf(str, "OUT: %dV, %dA", int(outV), configuration.CC); myLCD->printStr(1, 8, 2, 0x1f, 0x3f, 0, str);
          sprintf(str, "T-OUT: %dmin", timeOut); myLCD->printStr(1, 9, 2, 0x1f, 0x3f, 0, str); 
        }
        
        //======================== MAIN STATE MACHINE ======================
        switch(state)
        {
       
        case STATE_SERIALCONTROL:
          // serial control of the charger is enabled
          configuration.mainsC=200; // remove limit on input side
          sprintf(str, "READY:C%d,V%d,E", maxOutC, maxOutV);
          EMWserialMsg(str);
          delay(500); // wait for command to start

          // listen to commands       
          // format of the command: McccvvvE
          cmd[0]=cmd[1]=0; // reset all
          readSerialCmd(cmd);
          
          if(cmd[0]>0 && cmd[1]>0) {
            // echo reception
            sprintf(str, "ECHO:%d,%d,E", cmd[0], cmd[1]);
            EMWserialMsg(str);
            configuration.CC=cmd[0];
            maxOutV=cmd[1];     
            // move to charge stat
            state=STATE_CHARGE;
          }
          break;
         
        case STATE_WAIT_TIMEOUT:
          printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_CFG);
          x=BtnTimeout(10, 3);
          
          // check J1772
          J1772_dur=pulseIn(pin_J1772, HIGH);
          if(J1772_dur>50) { // noise control. also, deals with the case when no J1772 signal present at all
            absMaxChargerPower=mainsV*6/100*J1772_dur; // J1772 spec - every 100uS = 6A input
          }
          
          if(x == 1) state = STATE_TOP_MENU; // some button was pressed
          if(x == -1) // nothing pressed
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

         case STATE_CHARGE:
            pwr=configuration.mainsC; 
            // curb power on 110VAC
#ifdef drop110power            
            // 110VAC=160VDC rectified, 220VAC=320VDC - 240VDC is a midpoint in non-PFC
            // 165VDC is a midpoint in PFC version. Use below midpoint between 240 and 160 = 180
            if(mainsV<180) { 
              pwr/=2; // later, pwr is assumed to be a 220VAC-equivalent current
              pwr=min(pwr, 9.); // equivalent 15A from 110VAC // DEBUG
  #ifndef PFC              
              if(mainsV>minMains) {
                // protection against faulty mains reading
                // close 110VAC relay for doubler to operate
                digitalWrite(pin_110relay, HIGH); 
                delay(1000);
              }
  #endif
            }
#endif            
            // initialize timer here - this way will reset every time when returning from no-mains break
            Timer1.initialize(period); 
            Timer1.pwm(pin_PWM, 0);           
           
            //------------ ensure output power does not exceed other limits
            // output power limit for this session
            maxOutC=min(pwr*charger_efficiency*240/maxOutV, absMaxChargerCurrent);
            // lifetime output power limit 
            maxOutC=min(maxOutC, absMaxChargerPower/maxOutV );
            
            // set hardware protection for max current
            // need this here before 5 sec delay so the RC net stabilizes
            // also need this before user settings affect maxOutC 
            setMaxC(maxOutC*instantMaxCRatio); 
            
            // curb further if user-spec'ed current is less
            maxOutC=min(maxOutC, configuration.CC); 
            // finally, input current limit - only for DCinput
#ifdef DCinput
            maxOutC=min(maxOutC, MAXinputC*mainsV/maxOutV);
#endif

            // write out the configuration to EEPROM for next time
            EEPROM_writeAnything(0, configuration);
  
            timer_ch=millis(); // set the timer
            
            //========================== MAIN RUN CHARGER FUNCTION=======================
            // by this point, at least 15sec have passed since AC connection
            //                and at least 10sec since battery connection
            //                therefore, all caps should be pre-charged => close relays
            //   (note: this requires precharge resistors across relays - MAX of 300R for inrelay
            //          and MAX of 1k for outrelay. >3W power rating. Place small 1000V diode in
            //          series with the outrelay resistor - anode to battery - to avoid precharge on
            //          reverse polarity connection)     
            digitalWrite(pin_inrelay, HIGH);
            digitalWrite(pin_outrelay, HIGH);

            // reset AH counter
            AH_charger=0;
           
            // reset the EOC pin (this pin is active LOW so reset means setting to HIGH) 
            // high means charging is commencing. this will be pulled down by the charger when charge ends
            digitalWrite(pin_EOC, HIGH); 
              
            if(configuration.battType==0) {
              //---------------- CCCV for LiFePo4 or LiPoly --------------------
              // CC step, end condition - voltage goes to CV
              if(!runChargeStep(1, round(maxOutC), 1, maxOutV)) {
                // CV step
                delay(30000);
                runChargeStep(2, maxOutV, 2, configuration.AH*min_CV_Crating);
              }
            }
            
 #ifdef NiXX  
            if(configuration.battType==1) {
              //---------------- CC with dVdt termination for NiMh --------------------
              // CC step, end condition - dVdt goes below pre-determined value
              runChargeStep(1, round(maxOutC), 3, dVdt_stop);
            }
 #endif 
                    
            digitalWrite(pin_fan, LOW);    
            digitalWrite(pin_EOC, LOW); // active low

  
            printClrMsg(MSG_DONE, 500, 0x1f, 0x3f, 0);
            if(LCD_on) {
              sprintf(str, "%dAH in", int(AH_charger)); 
              myLCD->printStr(0, 6, 2, 0x1f, 0x3f, 0x1f, str);      
              charger_run=1; // charger has run this mains cycle...
              state = STATE_SHUTDOWN; //STATE_TOP_MENU;   
            } else {
              sprintf(str, "%dAH", int(AH_charger)); 
              EMWserialMsg(str);
              charger_run=0; // in serial command mode, we would not prevent running again
              state = STATE_SERIALCONTROL; // ready for next run   
            }
            
            digitalWrite(pin_inrelay, LOW);
            digitalWrite(pin_outrelay, LOW);
            //===========================================================================

           break; 
           default: break;
        }
        //=========================== END MAIN STATE MACHINE
      }
    }
  
}


//-------------------------------- main charger routine ------------------
// cycle type = 1 for CC, 2 for CV
// universal charging stage function - stop variable is different from start 
//    i.e. if cycleType=1 (CC), CX is amps, stop on voltage. if cycleType=2 (CV), CX is Volts, stop on amps 
int out1Reached=0;

int runChargeStep(int cycleType, float CX, int stopType, float stopValue) {
  float outV0=0; // initial voltage at start of cycle
  float out1=0.;
  float out2=0.;
  int spread=0;
  byte breakCnt=0;
  byte breakCycle=0;
  duty=0; // start with the duty cycle = 0 until you hit the constraint

#ifdef NiXX  
  V_ravg[0] = 0;
  V_ravg[1] = 0;
  t_ms = 0; 
#endif
  
  // start sensor readouts at some value so we can feed the averages
  outV_avg=outV=outV0=readV();
  outC_avg=outC=readC();
  // for derating
  float maxOutC1=maxOutC;
  
  // reset timer for AH metering
  timer=millis(); // this will be reset every cycle below after AH delta is calculated
  timer_step=timer; // this will keep track of step duration
  int n=0;
  
  // here, out1 is what is being controlled (kept constant), out2 is a termination criterion
  char typestr[4];
  if(cycleType==1) {
    // CC - constant current, stop by max voltage
    strcpy(typestr, "CC");
    spread=spreadC;
  } else if(cycleType==2) {
    // CV - constant voltage, stop by min current
    strcpy(typestr, "CV");
    spread=spreadV;
  } else {
    // wrong charge profile
    printClrMsg(MSG_WRONGPROF, 5000, 0x1f, 0x3f, 0);
    return -1;
  }
  
  if(LCD_on) {
    myLCD->clrScreen();
#ifndef LCD_SPE
    sprintf(str, "t=%s, CX=%d, sT=%d, Y=%d", typestr, int(CX), stopType, int(stopValue)); 
#else
    sprintf(str, "t=%s, CX=%d, \nsT=%d, Y=%d", typestr, int(CX), stopType, int(stopValue)); 
#endif
    myLCD->printStr(0, 4, 2, 0x1f, 0x3f, 0x1f, str);      
    delay(5000);
    myLCD->clrScreen();
  } else {
    // machine-readable
    // this assumes that only CC commands will be issued to the charger via serial
    sprintf(str, "START,T%s,C%d,S%d,V%d", typestr, int(CX), stopType, int(stopValue)); 
    EMWserialMsg(str);
  }

// define threshold duty - the charger will fast-ramp to this level before slowing down
#ifdef PFC    
  duty_crit=PWM_res*(outV0/PFCvoltage);
#else
  duty_crit=PWM_res*(outV0/330);  
#endif
// but for PFCdirect units, do not allow any fast ramp
#ifdef PFCdirect
  duty_crit=0;
#endif

  resetDelayParams();

  //============================================== MAIN CHARGER LOOP =============================================
  while(1) {
    // loop counter
    n++;
    min_up=(unsigned int)1.*(millis()-timer_ch)/60000;

    delay(stepDelay); // reasonable delay but not so that everything slows down too much
    
#ifdef UVLO
    // resistor divider is 6.8k/3.3k = 
    if(sampleRead(pin_12Vsense)<3.5) { // 12V supply dropped below 10.5V
      duty=0;
      // double protection
      digitalWrite(pin_PWMpulldown, HIGH); // disable PWM
    } else {
      digitalWrite(pin_PWMpulldown, LOW); // enable PWM
    } 
#endif

    Timer1.setPwmDuty(pin_PWM, duty);           
        
    // here, out1 is what is being controlled (kept constant), out2 is a termination criterion
    outC=readC(); // every cycle
    outC_avg=(outC_avg*(nSamplesStopVar-1)+outC)/nSamplesStopVar; // moving average
    outV=readV();
    outV_avg=(outV_avg*(nSamplesStopVar-1)+outV)/nSamplesStopVar; // moving average
    if(cycleType==1) {
      // CC - constant current, stop by max voltage
      out1=outC_avg; // controlled variable
      out2=outV_avg; // stop variable
    } else if(cycleType==2) {
      // CV - constant voltage, stop by min current
      out1=outV_avg; // controlled variable
      out2=outC_avg;  // stop variable
    }

    // print out stats - but only every few hundred cycles; n=300 generally corresponds to ~2-3Hz
    if(n>measCycle_len) {

      n=0;

      // AH meter
      AH_charger+=outC_avg*int(millis()-timer)/1000/3600;
      timer=millis();

#ifdef NiXX
      // this preps for the dVdT etc
      updateMovingAverages(outV_avg);
#endif      
      
      // check thermal parameters
      int normT=getNormT();
      if(normT<lowHeatSinkT) {
        // turn off the fans
        digitalWrite(pin_fan, LOW);
      } else {
        if(normT>midHeatSinkT) {
          digitalWrite(pin_fan, HIGH);  
          if(normT>=maxHeatSinkT) {
            // start derating
            // map 0-100% derating (relative to the current max current) to 
            // maxHeatSinkT-ABSmaxHeatSinkT heatsink range
            maxOutC1=maxOutC*abs(ABSmaxHeatSinkT-normT)/(ABSmaxHeatSinkT-maxHeatSinkT);
            
            if(normT>ABSmaxHeatSinkT) {
              // overheating. stop charger.  wait until drops enough to restart
              stopPWM();
              out1=out2=outV=outC=outC_avg=outV_avg=0; // force duty ramp after this condition clears
              resetDelayParams();
              if(LCD_on) myLCD->clrScreen();        
  
              while(1) {
                if(LCD_on) {
                  sprintf(str, "Overheat, T=%dC", (int)normT);
                  myLCD->printStr(0, 1, 2, 0x1F, 0, 0, str);
                  delay(1000);
                } else {
                  sprintf(str, "HOT%d", (int)normT);
                  EMWserialMsg(str);
                }
                normT=getNormT();
                if(normT<midHeatSinkT) {
                  if(LCD_on) myLCD->clrScreen();
                  break; // exit cycle when the temp drops enough
                }
              }
            } // ABSmaxHeatSink condition
            
          } 
        }

      } // end thermal management
      
      
      // check HVC signal from BMS
      if(digitalRead(pin_BMS)==LOW) { // active LOW (so you need to pull up by connecting miniBMS loop to EOC signal)
        // BMS commanding charger to stop
        // noise protection - ensure signal stays on for 100ms or so
        delay(100);
        if(digitalRead(pin_BMS)==LOW) {
          // this is for real
          printClrMsg(MSG_BMSSTOP, 5000, 0x1f, 0x3f, 0);
          stopPWM();          
          return 1; // assume this is a normal end-of charge but do not allow any more steps (return 1)
        }
      } 
            
      // check the timer
      if(timeOut>0 && (millis()-timer_ch)/60000>timeOut) {
        // timer run out
        printClrMsg(MSG_TIMEOUT, 5000, 0x1f, 0x3f, 0);
        stopPWM();      
        return 1; // assume this is a normal end-of charge but do not allow any more steps (return 1)
      }
      
      //==================== print all parameters
#ifdef NiXX        
      printParams(duty, outV_avg, outC_avg, normT, AH_charger, dVdt);
#else      
      printParams(duty, outV_avg, outC_avg, normT, AH_charger, 0);
#endif

      // check if need to stop - RED button pressed?
      if(isBtnPressed()==0b10) {
        byte b=0;
        printClrMsg(MSG_USRPAUSE, 1000, 0x1f, 0x3f, 0);   
        stopPWM();
        out1=out2=outV=outC=outC_avg=outV_avg=0; // force duty ramp
  
        do {
          delay(100);
          b=isBtnPressed();
        } while(!b);
        if(LCD_on) myLCD->clrScreen();
        // button pressed. which one?
        if(b==0b10) { // another RED!
          stopPWM();           
          return 1; // out of the main charger loop, do not allow second cycle
        }
        
        // if we are here, this means GREEN button was pressed
        // resume operation
        if(LCD_on) myLCD->clrScreen();
        resetDelayParams();
      }
      
      mainsV=read_mV(); // only infrequently as the averaging interval is pretty long (10mS)

#ifndef DEBUG        
      // check mains
      if(mainsV<minMains) {
        delay(2000);
        printClrMsg(MSG_LOSTIN, 5000, 0x1f, 0x3f, 0);
        stopPWM();           
        return 1;
      }
#endif
      
      // end measCycle loop
    }
        
    //-------------- MAIN DUTY CYCLE MANAGEMENT LOGIC ----------------------------
    // safety - fast-acting protection - zero out duty immediately
    if(outV>maxOutV*1.05) { // 5% on top of CV point
      duty=0;
      Timer1.setPwmDuty(pin_PWM, 0);
    } 
    
    // use small hysteresis (spread*2) to avoid jitter   
    // if current or voltage too LOW, INcrease duty cycle
    if(out1 < CX-spread) {
      // only if we are safe to go up - this is slow protection as it just stops duty from rising
      if(duty<MAXDUTY && outC<maxOutC1) {
        duty++; 
      }
    } else {
      out1Reached=1;
    }
    // if current or voltage too HIGH, DEcrease duty cycle       
    if(out1 > CX || outC>maxOutC1) {
      if(duty>0) {
        duty--;
      }
    }
    
    // slow down when getting close to the resting battery voltage - for stability
#ifdef SLOWUPDATE
    // ramp fast to approximate duty cycle that would be needed to match battery voltage
    //               then slow down to 0.1% duty / sec
    // duty_crit is set earlier to the value depending on the DC rail voltage
    // duty>duty_crit || 
    if(outC>5) { // only slow down after we reached some current or voltage
      stepDelay=100; // 0.1 second per cycle
      measCycle_len=10; 
      nSamplesStopVar=10; 
      nReadSamples=10; 
    }
#endif
    
    if(out1 < -10 || out2 < -10) {
      // sensor polarity problems. abort
      printClrMsg(MSG_SENSEERROR, 300, 0, 0x3f, 0);
      sprintf(str, "(%d, %d)", int(out1), int(out2));
      myLCD->printStr(0, 6, 2, 0x1f, 0x3f, 0x1f, str);
      delay(30000);
    }
    //---------- END MAIN DUTY CYCLE MANAGEMENT LOGIC ----------------------------

    // check for break conditions - only on secondary variable!
    breakCycle=0;
    if(stopType==1 && out2 > stopValue) {
      breakCycle=1;
    } 
    // also, no point to break on stopMin before the CX condition has been reached
    // do break if we spent over X minutes in this step
    if(stopType==2 && out2 < stopValue && (out1Reached==1 || (millis()-timer_step)/60000>CV_timeout) ) {
      breakCycle=1;
    }
#ifdef NiXX
    // check dV/dt and break if it is too small - stopValue is in % of pack voltage change per second
    // on a 216V nominal pack, 1E-05 stopValue corresponds to dVdt=2mV/s
    if(stopType==3 && dVdt/maxOutV < stopValue) {
      if(min_up > 5) { // ignore first few min of the charge
        breakCycle=1;        
      }
    }
#endif

    // do we REALLY need to break?
    if(breakCycle) {
      breakCnt++;
      if(breakCnt>stopCycles) {
        delay(1000);
        printClrMsg(MSG_NORMEXIT, 5000, 0, 0x3f, 0);
        break;
      }
    } else {
      breakCnt=0;
    }
    
  }; //======================================== END MAIN CHARGER LOOP ===================================

  stopPWM();
  
  return 0;
}

//============================ HELPER FUNCTIONS ====================================
// stop output in a controlled fashion. this function is called on any abnormal event
void stopPWM() {
  // ramp down only to duty_crit as that spans the entire useful current range
  // usually this is just 5-10% of the duty range, or <100 cycles
  // therefore, can afford to use reasonable delay
  for(int di=duty; di>duty_crit*0.9; di--) { // additional safety factor of 0.9
    Timer1.setPwmDuty(pin_PWM, di);
    delay(stepDelay0); // default is 4ms
  } 
  // now full reset
  duty=0; 
  Timer1.setPwmDuty(pin_PWM, 0);           
}

//============================ voltage readout functions =====================
// read output voltage
float readV() {
  return (sampleRead(pin_bV)-V_o_bV)*divider_k_bV; //  isolation opamp
}

// read mains voltage. this function will take ~15mS to complete!
float read_mV() {
  // find peak voltage
  float peakV=0, peakV_new=0;
  
  // need to sample at least 10ms - half-period of 50Hz input
  // analogRead takes 0.1uS itself
  // to be sure we cover 10mS, add a delay of 70uS to each loop
  for(int i=0; i<100; i++) {
#ifdef PC817 
    peakV_new=5-analogRead(pin_mV)*Aref/1024.; // with PC817 sensing, opto inverts the wave
    // R4 on the driver board is selected so that the 5V means ~the same input voltage as 5V in ISO124 sensing
#else    
    peakV_new=analogRead(pin_mV)*Aref/1024.; // 10-bit ADC
#endif
    if(peakV_new>peakV) peakV=peakV_new;
    delayMicroseconds(70);
  }
#ifdef PC817
  if(peakV>2.) return 240;
  return 120;
#endif

#ifdef DCinput
  return (peakV-V_o_mV0)*divider_k_mV; // peak = RMS for DC
#else 
  return (peakV-V_o_mV0)*divider_k_mV/1.414*1.2; // RMS + adjustment for RC filter on the control board
#endif
}
//============================ end voltage readout functions =====================

void setMaxC(float maxC) {
#ifdef PFCdirect
  // in PFCdirect V02 units hardware limits don't work due to the opposite direction of the sensor
  Timer1.setPwmDuty(pin_maxC, 1023); // need something more than 3 volts as zero-current output is 2.5V...
#else
  Timer1.setPwmDuty(pin_maxC, 1024./Aref*(V_o_C+k_V_C*maxC));
#endif
}

//============================ current readout functions =====================
// read output charger current
float readC() {
  float current=(sampleRead(pin_C)-V_o_C)/k_V_C;
  // read output current pin
#ifdef NEG_CSENSE
  current=-1*current;
#endif

// what if we only have input current sense (some UPP units)
// in that case, rescale the output current from input using the voltages
#ifdef INC_ASOUT
  current*=mainsV/outV;
#endif

  return current;
}

// read input charger current
float read_mC() {
  // in the PFC direct units, we are measuring average input current which is 2/pi() * peak, RMS is peak / sqrt(2)
  // therefore RMS = AVE * pi() / 2 / sqrt(2) = AVE * 1.11
  return (sampleRead(pin_mC)-V_o_mC)/k_V_mC*1.11; 
}
//================================= end current functions ========================


//====================== temperature readout functions ===========================
// compute the equivalent (normalized) charger temp
int getNormT() {
  // assume max sink T is 55, max t2 (inductor) is 85 - approx but reasonably close to reality
  // therefore need to rescale t2 by 55/85=0.65
  // BUT ONLY IF WE HAVE THIS SECONDARY MEASUREMENT
#ifdef IND_Temp
  #ifdef MCC100A
    // if using 100A inductor, hST is limited at lower temp than in regular 70A rating so need to use a different ratio here
    return max(read_T(pin_heatSinkT), read_T(pin_temp2)*0.55);
  #else
    return max(read_T(pin_heatSinkT), read_T(pin_temp2)*0.65);
  #endif
#else
  return read_T(pin_heatSinkT);
#endif
}
// master temp readout, using formulas from http://en.wikipedia.org/wiki/Thermistor
int read_T(byte pin) {
  return int(1/(log(1/(5./sampleRead(pin)-1))/4540.+1/298.)-273); 
}

float sampleRead(byte pin) {
  float sum=0.;
  
  for(int i=0; i<nReadSamples; i++) {
    sum+=analogRead(pin)*Aref/1024.; // 10-bit ADC
    if(waitReadSamples!=0) delayMicroseconds(waitReadSamples);
  }
  return sum/nReadSamples;
}

// sensitivity of the sensor
float get_k_V_C(int selection) {
  switch(selection) {
  case Tamura_50B:
    return 1.5/50; //  1.5V deviation at rated current
  case Tamura_150B:
    return 1.5/150.; //  1.5V deviation at rated current
  case Tamura_600B:
    return 1.5/600.;; // 1.5V deviation at rated current
  case Allegro_150U:
    return 0.0267; // direct from datasheet
  case Allegro_100U:
    return 0.04;  // direct from datasheet
  case Allegro_050U:
    return 0.06;  // direct from datasheet
  default: break;
  }
}
// Vout @ 0 AT
// 0.6V for Allegro X050U, X100U and 150U
float get_V_o_C(int selection) {
  switch(selection) {
  case Tamura_50B:
    return Vcc/2;  // direct from datasheet
  case Tamura_150B:
    return Vcc/2;  // direct from datasheet
  case Tamura_600B:
    return Vcc/2;  // direct from datasheet
  case Allegro_150U:
    return 0.6; // direct from datasheet
  case Allegro_100U:
    return 0.6; // direct from datasheet
  case Allegro_050U:
    return 0.6; // direct from datasheet
  default: break;
  }
}


void resetDelayParams() {
    // reset delay parameters
    stepDelay=stepDelay0;
    measCycle_len=measCycle_len0;
    nSamplesStopVar=nSamplesStopVar0;
    nReadSamples=nReadSamples0;
    
    out1Reached=0;
}


//=========================================== Communication (LCD / Serial) Functions =========================
// main parameter printing function
void printParams(float duty, float outV, float outC, int t, float curAH, float dVdt) {
  if(LCD_on) {
    char tempstr[32];
    // myLCD->setOpacity(1); // so that we override previous text
    sprintf(str, "D = %s%%  ", ftoa(tempstr, 100.*duty/PWM_res, 1)); myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x1f, str);      
    // sprintf(str, "F = %dkHz ", int(1000/period)); myLCD->printStr(0, 1, 2, 0x1f, 0x3f, 0x1f, str);      
    sprintf(str, "Out = %sA, %dV   ", ftoa(tempstr, outC, 1), int(outV)); myLCD->printStr(0, 3, 2, 0x1f, 0x3f, 0, str);      
    sprintf(str, "T = %dC ", t); myLCD->printStr(0, 5, 2, 0x1f, 0, 0, str);
    sprintf(str, "AH = %sAH", ftoa(tempstr, curAH, 1)); myLCD->printStr(0, 6, 2, 0x1f, 0x3f, 0, str);      
    sprintf(str, "t = %umin", min_up); myLCD->printStr(0, 8, 2, 0, 0, 0x1f, str);
    
#ifdef NiXX
    if(min_up>=5) {
      // print dVdt only if we are past initial settling period
      sprintf(str, "dVdt = %s     ", ftoa(tempstr, dVdt*1000., 1)); myLCD->printStr(0, 9, 2, 0, 0, 0x1f, str);
    }
#endif

  } else {
    // machine-readable
    // format: [D]uty in 0.1%, [C]urrent in 0.1A, [V]oltage in 1.0V, [T]emp in C, [O]utput AH in 0.1AH, [R]untime in min
    sprintf(str, "STATS:D%d,C%d,V%d,T%d,O%d,R%d", int(1000.*duty/PWM_res), int(outC*10), int(outV), t, int(curAH*10), min_up);
    EMWserialMsg(str);
  }
}
void printConstStr(int col, int row, int font, byte red, byte green, byte blue, const byte msg_id) {
  strcpy_P(str, (char*)pgm_read_word(&(msg_lcd_table[msg_id]))); 
  myLCD->printStr(col, row, font, red, green, blue, str);
}

void printClrMsg(const byte msg_id, const int del, const byte red, const byte green, const byte blue) {
  if(LCD_on) {
    strcpy_P(str, (char*)pgm_read_word(&(msg_long_table[msg_id]))); 
    myLCD->clrScreen();
    myLCD->printStr(0, 2, 2, red, green, blue, str);      
    delay(del);
  } else {
    strcpy_P(str, (char*)pgm_read_word(&(msg_short_table[msg_id]))); 
    EMWserialMsg(str);
  }
}
//================== serial comms ========================
// message FROM the charger via serial
void EMWserialMsg(const char *txt) {
  Serial.print("M,");
  Serial.print(txt);
  Serial.println(",E");
}
// message TO the charger via serial
// command syntax: M,xxx,yyy,E
void readSerialCmd(int *cmd_) {
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
      if(Serial.read()!='E') {
        cmd_[0]=cmd_[1]=0;
      }
    }
  }
}
//============================================== end communication functions =============================

char *ftoa(char *a, double f, int precision)
{
  long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
  
  char *ret = a;
  long heiltal = (long)f;
  itoa(heiltal, a, 10);
  while (*a != '\0') a++;
  *a++ = '.';
  long desimal = abs((long)((f - heiltal) * p[precision]));
  itoa(desimal, a, 10);
  return ret;
}


#ifdef NiXX        
  //--------------------Calculate moving Averages-------------
  void updateMovingAverages(float V) {
    float avg = V_ravg[1];
    unsigned int k = ele_counter;
    float updated_avg = avg * (k/float(k+1)) + V/float(k+1);
    if( ele_counter >= 150 /*|| abs(updated_avg - avg) < 0.00001*/) {
      // switch averages and calculate dVdt
      unsigned long now = millis();
      if(t_ms > 0) {
        float time_interval = (now - t_ms) * 0.001; 
        dVdt = (V_ravg[1] - V_ravg[0]) / time_interval;
      }
      ele_counter = 0;
      V_ravg[0] = V_ravg[1];
      t_ms = now;
    } 
    else {
      V_ravg[1] = updated_avg;
      ++ele_counter;
    }
  }
#endif

unsigned int MenuSelector2(unsigned int selection_total, const char * labels[])
{
  unsigned int selection = 0;
  unsigned int temp_selection = 1;
  
  //sprintf(str, "[%s]", labels[temp_selection-1] ); 
  //myLCD->printStr(0, 3, 2, 0x1f, 0x3f, 0x1f, str);
  printConstStr(0, 3, 2, 0x1f, 0x3f, 0x1f, MSG_LCD_BLANK);
  myLCD->printStr(1, 3, 2, 0x1f, 0x3f, 0x1f, labels[temp_selection-1]);

  while(!selection)
  {
    int step_btn = digitalRead(pin_pwrCtrlButton);
    int select_btn = digitalRead(pin_pwrCtrl2Button);
    if(step_btn == HIGH)
    {
      ++temp_selection;
      if(temp_selection > selection_total) temp_selection = 1;
      printConstStr(0, 3, 2, 0x1f, 0x3f, 0x1f, MSG_LCD_BLANK);
      myLCD->printStr(1, 3, 2, 0x1f, 0x3f, 0x1f, labels[temp_selection-1]);
      
      // ideally, this should call a StatusDisplay method and simply pass selection index
      // StatusDisplay should encapsulate all the complexities of drawing status info onto the screen
      // alternatively myLCD can be re-purposed for this
    }
    else
    if(select_btn == HIGH)
    {
      selection = temp_selection;
      printConstStr(0, 3, 2, 0x1f, 0x0, 0x0, MSG_LCD_BLANK);
      myLCD->printStr(1, 3, 2, 0x1f, 0x0, 0x0, labels[selection-1]);
      // similar to the above, should delegate display to StatusDisplay object
    } 
    delay(80);
  }

  delay(200);
  return selection - 1;
}

//-------------------------------------- check for button press -------------
// returns bitmask: bit 0 = green button, bit 1 = red button
// this takes max of 50ms if the button is pressed, and up to 100ms if BOTH buttons are pressed!
// still need to do debounce...
// in the SERIAL-CONTROLLED mode, 'STOP' command simulates red button, 'START' command - green button
byte isBtnPressed() {
  byte bmask=0;
  if(LCD_on) {
    // green button?
    if(digitalRead(pin_pwrCtrl2Button)==HIGH) {
      // check if noise
      for(int zz=0; zz<10; zz++) {
        if(digitalRead(pin_pwrCtrl2Button)==LOW) bmask&=0b10; // reset bit 0
        delay(5);
      }
      bmask|=0b01; // set bit 0
    } 
    // red button?
    if(digitalRead(pin_pwrCtrlButton)==HIGH) {
      // check if noise
      for(int zz=0; zz<10; zz++) {
        if(digitalRead(pin_pwrCtrlButton)==LOW)  bmask&=0b01; // reset bit 1
        delay(5);
      }
      bmask|=0b10; // set bit 1
    } 
  } else {
    // serial controlled mode
    // 'M,001,000,E' is STOP (or RED button emulation)
    // 'M,000,001,E' is START (or GREEN button emulation)
    cmd[0]=cmd[1]=0;
    readSerialCmd(cmd);
    if(cmd[0]<2 && cmd [1]<2) {
      bmask=cmd[0]*0b10+cmd[1];
    }
  }
  return bmask;
}
  

int BtnTimeout(int n, int line)
{
  while(n > 0)
  {
    sprintf(str, "(%d sec left) ", n); 
    myLCD->printStr(0, line, 2, 0x1f, 0x3f, 0, str);

    for(int k=0; k<100; k++) {
      if(digitalRead(pin_pwrCtrlButton)==HIGH || digitalRead(pin_pwrCtrl2Button) == HIGH) return 1;
      delay(10);
    }

    --n;
  }
  
  return -1;
}

int DecimalDigitInput3(int preset)
{
  //  myLCD->setOpacity(1);
  int d3=preset/100;
  int d2=(preset/10)%10;
  int d1=abs(preset%10);
  int digit[3] = { d3, d2, d1 };
  int x = 0; // 0-1-2-3-4
  // 0x30 ascii for "0"
  str[1] = 0x0; // eol 

  while(x < 4)
  {
    int step_btn = digitalRead(pin_pwrCtrlButton); // increments digit
    int select_btn = digitalRead(pin_pwrCtrl2Button); // moves to next digit
 
    if(step_btn == HIGH) {
      if(x > 2) x = 0;
      else {
        // increment digit
        ++digit[x];
        // wrap at 3 (for 100s) or at 9 (for 10s and 1s) 
        if(x == 0 && digit[x] > 3) digit[x] = 0;
        if(digit[x] > 9) digit[x] = 0;
      }      
    } else 
    if(select_btn == HIGH) {
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

void printDigits(int start, int * digit, int stat) {
  str[0] = 0x30+digit[0];
  printDigit(start++, stat, str);
  str[0] = 0x30+digit[1];
  printDigit(start++, stat, str);
  str[0] = 0x30+digit[2];
  printDigit(start, stat, str);
}
void printDigit(int x, int stat, char * str) {
  if(stat==0) myLCD->printStr(x, 5, 2, 0x1f, 0x3f, 0x0, str); // yellow
  if(stat==1) myLCD->printStr(x, 5, 2, 0x8, 0x8, 0x1f, str); // blue
}



