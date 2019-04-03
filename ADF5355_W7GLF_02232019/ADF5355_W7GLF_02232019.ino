//   ADF5355 and Arduino
//
//   Based on code by Alain Fort F1CJN and Dave Brink (for 64bits routines) jan 3,2018
//
//   Port F1CJN code to DUE
//
//   This code is freeware. No warranty or liability, expressed or implied, is included with this sketch. Commercial use is prohibited. 
//
//   update December  5, 2017  W7GLF Modified DFRobot shield so buttons are referenced to 3.3 volts not 5 volts.  It was causing SELECT button
//                             to not be recognized.  In any case putting 5 volts on a 3.3 volt Arduino is dangerous.  See comment below under HARDWARE...
//   update December  6, 2017  W7GLF restructured EEPROM layout to get rid of "magic" numbers like 500 and 501. 
//                             W7GLF also fixed what looked like a problem waiting for button release - 
//                             and added the ability to set and save the LEVEL. 
//   update December  6, 2017  W7GLF Added DEBUG_SETTLE options to allow viewing key press outputs.  
//   update December 22, 2017  W7GLF Added routine UPDATE_ADF5355 to solve problem with synth losing LOCK when UP/DOWN buttons pressed.  
//   update December 22, 2017  W7GLF Made some changes to get synth to lock up upon inital power up.  
//                             I also made Arduino DUE change to add 10K resistor across FET T3 as mentioned by Dan at:
//                             https://forum.arduino.cc/index.php?topic=256771.30
//   update February  8, 2018  W7GLF Allow use of matrixed digital keyboard.  
//   update February 17, 2018  W7GLF - Add alternative calculation of MOD2 as per ADF5355 spec.  
//   update February 19, 2018  W7GLF - Allow 10 MHZ to use REF double to reduce phase noise.  
//   update March     3, 2018  W7GLF - Change longs and long longs to uint32_t and uint64_t.  
//   update August   21, 2018  W7GLF - Add ability to use analog pins as digital pins for buttons for KD7TS.
//   update February 21, 2019  W7GLF - Fix bug when using UNO/NANO concerning register 9.
//   update February 22, 2019  W7GLF - Fix warnings when compiling on Linux.
//   update April     1, 2019  W7GLF - Only set INUSE flag if memory 0 is used.
//
//  This sketch supports an Arduino NANO, UNO or DUE, a standard "LCD buttons shield" from ROBOT, with buttons and the ADF5355 Chinese
//  eval board found at EBAY. The frequency can be programmed between 54 MHz and 13.6 GHz.
//  
//  Note:  the DUE requires an external I2C EEPROM (32K x 8 bits).  The one I used was an AT24C256C mounted on a small board.
//  These are available on eBay from China. 
//
//  The code so it will also run on an NANO/UNO - see #if DUE below...
//
//  Note: Be aware if the code is on the NANO or UNO then divider resistors for MOSI, SCK, and LE are required to convert the 
//  the NANO/UNO's 5 volt output levels down to the 3.3 volt level required for the ADF5355.  You can do this using either 
//  1K and 1.5K or 2K and 3K resistors.  The important thing is the resistors are in a rato to drop 5 volts to about 3.3 volts.
//
//  General description:
//
//  If one or more frequencies are stored, then at power on, memory zero is always selected.
//
//  The cursor moves with the LEFT and RIGHT buttons. Then the underlined digit can be modified with the UP and DOWN buttons, 
//    for the frequency, the memories and the frequency reference (10, 25 or 26 MHz):
//   - to change the frequency, move the cursor to the digit to be modified, then use the UP and DOWN buttons,
//   - to modify the memory number,move the cursor to the number to be modified, then use the UP and DOWN buttons,
//   - to select the reference frequence, move the cursor on lower right hand corner 10/25/26 and select with UP and DOWN.
//   - to read or write the frequency in memory, place the cursor on the lower left position and select REE (for Reading EEprom)
//    or WEE (for Writing EEprom) with UP/DOWN.
//
//    Note:  The cursor dissapears after 10 seconds and is re activated if a button is pressed.
//
//   STORING FREQUENCY AND REFERENCE VALUES 
//    - For the frequency, select WEE, then select the memory number, then push the SELECT button for a second. The word SAVED 
//      appears on the screen. This memorization works then the cursor is anywhere except on the reference 10 or 25 position.
//    - For the reference frequency, move the cursor to 10 or 25, the press SELECT for one second.  Once the reference is SAVED
//      it will be remembered on the next power up until it is manually changed.
//
//  ******************************************** HARDWARE IMPORTANT ********************************************************
//
// ************************************************ DUE INFORMATION ********************************************************
// *
// * With the Arduino DUE, resistive dividers on MOSI, SCK, and LE are not required since the DUE uses 3.3V logic.
// * The SCK, DATA outputs and ground connect to the SPI bus connector pins as opposed to pins 11 and 13 for the NANO/UNO.
// * Connect ADF5355 DATA to pin 4 (MOSI), ADF CLK to pin 3 (SCK), and GROUND to pin 6 (GND).  If you are looking at the DUE
// * with the power connector away from you the pins on the SPI connector are numbered as follows:
// *
// *   5  3  1
// *   6  4  2
// *
// * Digital pin 3 is used for the LE output to the ADF5355 and digital pin 2 (for lock detection) is connected to ADF5355 card MUXOUT.
// * The ADF5355 card is powered by at least 7 VDC from an external power supply or via the V.IN Arduino pin.
// *
// * The external EEPROM has 4 lines: VCC, GND, SCL, and SDA.  SCL connects to pin 21 (SCL) and SDA connects to pin 20 (SDA)
// * on the Due.  Connect VCC to 3.3V on button shield (adjacent to 5V pin).
// *
// * The 1602 DFRobot shield needs a minor modification so its analog buttons range from 3.3 volts to ground instead of 5 volts
// * to ground.  Gently pry up LCD and remove 2 K resistor above the gap between the
// * the RIGHT and RESET buttons.  It is the rightmost resistor when the LCD display is viewed with buttons at the bottom.
// * Take the removed resistor (or another 2K SMD resistor if you lost the removed resistor) and solder it to the bottom
// * of the board to the right side of the RIGHT button which you can see is connected to the line from A0.  The other side of
// * the resistor should be connected with a fine wire to the 3.3 volt pin that is between the RST pin and the 5 volt pin. 
// *
// ***************************************************************************************************************************
//
//
// ************************************************ NANO/UNO INFORMATION *****************************************************
// *
// * NANO/UNO connections to ADF5355 
// *   With any 5 volt unit: use a resistive divider to reduce the voltage between these pins, 
// *   MOSI   (pin D11)  to ADF5355 DAT 
// *   SCK    (pin D13)  to ADF5355 CLK 
// *   Select (pin D3 )  to ADF5355 LE 
// *
// *   D2     (pin D2)   to MUX   around +3.2 when locked, otherwise zero
// *
// *   Resistive divider 1000 Ohms in series with 1500 Ohms to ground on Arduino pins 11, 13 and 3 or
// *   resistive divider 2000 Ohms in series with 3000 Ohms to ground on Arduino pins 11, 13 and 3.
// *   Connect the junction of the series resistors to MOSI (DAT), SCK (CLK) and Select (LE) on ADF4351.
// *
// *   Arduino pin D2 (for lock detection) directly connected to ADF4351 card MUXOUT.
// *
// ***************************************************************************************************************************
//
// *********************************** Here are comments KD7TS added for his configuration using *************************
// *********************************** a NANO and discrete buttons setup - probably not **********************************
// *********************************** of interest to others unless copying his setup ************************************
// *
// *  Parallel 1602 LCD connections to NANO/UNO (not using DF Robot button board/display)
// *
// *  LCD pin RS to digital pin D8
// *  LCD pin E  to digital pin D9 (enable)
// *  LCD pin D4 to digital pin D4
// *  LCD pin D5 to digital pin D5
// *  LCD pin D6 to digital pin D6
// *  LCD pin D7 to digital pin D7
// *
// *  LCD R/W pin to ground
// *  LCD VSS pin to ground
// *  LCD VCC pin to 5V
// *  10K resistor: this (is a pot) ends to +5V and ground
// *  wiper to LCD VO pin (pin 3). Voltage on VO around 9/10ths of a volt when text is visible.
// *
//  FREQUENCY SELECT BUTTONS
//
//  5 buttons are wired from A0 - A4 to ground through the same 120 ohm resistor. The pins
//  are assigned as digital inputs with pull up resistors. 
//
//************************************************* END OF KD7TS COMMENTS *************************************************

// ************************************************* BUTTON OPERATION*******************************************************
//Touch LEFT    cursor to the left
//Touch RIGHT   cursor to the right
//Touch UP      increase frequency
//Touch DOWN    decrease frequency
//Touch SELECT  long push = frequency memorization into the EE number EEPROM / or reference memorization
//*************************************************************************************************************************
// Warning : if you are using a ROBOT Shied version 1.1, it will be necessary to rescale the
// threshold values in the read_buttons sub routine 
//
// The correction value can be used to tweak the frequency of the internal oscillator.  Let the synth run for a while
// and measure the actual frequency on a counter.  The value will be ratio of the measured divided by the expected minus 1
// times one billion.  Thus if the synth was set for 1,000,000,000 Hz and the counter said 1,000,000,625 Hz then the value
// would be 1000000625/1000000000 = 1.000000625 minus one which would be .000000625 and then times 1000000000 which would 
// be 625.

// correction = ((Measured/Expected) - 1) * 10^9
long correction = 0;

#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)  
#define __FILENAME_VER__ (strrchr(__FILE__, '_') ? strrchr(__FILE__, '_') + 1 : __FILE__)  
char version [17];

#define DUE false            // true if using DUE otherwise false for UNO or NANO

#define SINGLE_ENDED_LO true // Reference is Single Ended.  Note by default black board uses
                             // differential reference so this would be false.  Green board
                             // and my 4x4 button unit use single ended reference.

// Only one of the three following conditions should be set to true

#define ANALOG_BUTTONS true    // true for DF Robot button shield
#define DIGITAL_BUTTONS false  // true for individual buttons tied to analog inputs - KD7TS uses this
#define MATRIXED_BUTTONS false // true for 4x4 matrix keypad

// W7GLF - added some DEBUG variables.

#define DEBUG false            // GLOBAL DEBUG

#define DEBUG_BUTTONS false
#define DEBUG_SETTLE false

// Leave the following as true
#define USEMULTVCO2 true  // Multiply 10 MHZ external input by 2 to improve phase noise 

// W7GLF - useful macros for dumping out the hex value of a register
#define DebugSerialPrint(token) \
   if (DEBUG) { \
     Serial.print(token); \
   }
#define DebugSerialPrint2(token,token2) \
   if (DEBUG) { \
     Serial.print(token); \
     Serial.println(token2); \
   }
#define RegisterSerialPrint(register) \
   if (DEBUG) { \
     Serial.print("Register ["); \
     Serial.print(register); \
     Serial.print("] = "); \
     Serial.print(registers[register],HEX); \
     Serial.print("\r\n"); \
   }
#define RegisterValueSerialPrint(register,value) \
   if (DEBUG) { \
     Serial.print("Register ["); \
     Serial.print(register); \
     Serial.print("] = "); \
     Serial.print(value,HEX); \
     Serial.print("\r\n"); \
   }

#include <LiquidCrystal.h>
#if DUE
#include <Wire.h>            //To use the I2C interface (SDA & SCL) to external EEPROM
#endif
#define EEPROM_ADDRESS 0x50  //Base address of chip on the I2C interface
#include <SPI.h>
#define ADF5355_LE 3
#if !DUE
#include <EEPROM.h>
#endif

#if SINGLE_ENDED_LO
  // use these lines if single ended signal for ref - green board
  #define R04 0X30008984    // DB4=0
  #define R04_EN 0X30008994 // DB4=1
#else
  // use these lines if differential signal for ref - black board
  #define R04 0X30008B84    // DB4=0
  #define R04_EN 0X30008B94 // DB4=1
#endif

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// EEPROM Layout
#define OnboardOsc 26
#define MAXMEM 100  // Number of memories that can be saved
#define PDREF  MAXMEM*9 // 4 bytes for frequency in kHz, 4 for Hertz and 1 for level
#define MFREF PDREF+1  // save INUSE REF freq flag at startup
#define MFREQ PDREF+2  // save INUSE frequency flag at startup
#define INUSE 55  // Special Bit pattern to mark memory is in use

#if DUE
  #if MFREQ >= 32768
  #error EEPROM is too small to hold MAXMEM entries
  #endif
#else
  #if MFREQ >= 1024
  #error EEPROM is too small to hold MAXMEM entries
  #endif
#endif

byte poscursor = 0; //position cursor common 0 to 15
byte line = 0; // display line in progress on LCD in the course of 0 to 1
byte X2 = 0;
uint32_t MULTVCO2 = 0;  // For 10 MHz we are going to double VCO - it improves phase noise
byte VCO_BAND_DIV;
unsigned int ADC_CLK_DIV;

// DUE needs external EEPROM which uses 16-bit of address (memory) to address all 32K bytes.
uint16_t memory;

// I found programming synth must be tried harder after a power up.
bool first_time;

uint32_t registers[13] = {0x00200CF0, 
                          0x05C28F51, 
                          0xC28FFFF2, 
                          0x00000003, 
                          0x30008984,  // Defualt is Single Ended Ref 
                          0x00800025, 
                          0x3500A076, 
                          0x120000E7, 
                          0x102D0428, 
                          0x0B0ABCC9, 
                          0x00C00FBA, 
                          0x0061300B, 
                          0X0001041C}; //5184 (10368) MHz with 25 MHz reference

bool locked;

unsigned int address=0;
int modif = 0,WEE = 0;
int lcd_key = 0;
int adc_key_in  = 0;
int timer = 0,timer2=0; // use to measure the time a key is pressed
unsigned int i = 0;
double RFout, RFout_temp, VCOout, INTA, FRAC, FRACA, REM, Hz;
double REFin;
byte PFDRFout;
word RFoutMin = 54, RFoutMax = 13600, REFinMax = 250, PDFMax = 32;
uint32_t RFint,RFintold,RFcalc,PDRFout;
byte OutputDivider; 
byte rflevel, rflevelold;

uint32_t RFOUT,TEMP,TEST;
 int32_t TEMPLONG,RF_Divider,HERTZ,HERTZold,DELTAIF; 
uint32_t FVCO[]={0,600000000};// in deciHz (10s of Hz)
uint32_t FPFD[]={0,26}; // in MHz
uint32_t FVCOTEMP[]={0,0};// in deciHz (10s of Hz)
uint32_t FPFDTEMP[]={0,0};
  
uint32_t INT[]={0x0,0x0};
uint32_t N[]={0x0,0x0};
uint32_t NX[]={0x0,0x0};
uint32_t FRAC1[]={0x0,0x0};
uint32_t FRAC11[]={0x0,0x0};
uint32_t FRAC2[]={0x0,0x0};
uint32_t HZ[]={0x0,0x0};
uint32_t FRAC1exact[]={0x0,0x0};
uint32_t MOD1[]={0,16777216};  // 0x1000000 or 2^24

uint32_t MOD2[]={0,1000};
uint32_t MOD2MAX[]={0,16383};

byte lock=2,AMP=4;
uint32_t reg0, reg1;
uint32_t FRAC1R,FRAC2R,INTR,RF_Select,I_BLEED_N,RFB;

// Values for analog ADC - there may have to be adjusted - use DEBUG_BUTTONS to see 

#define ADC_RIGHT 45 
#define ADC_UP 240 
#define ADC_DOWN 430
#define ADC_LEFT 600
#define ADC_SELECT 1000
#define ADC_ANY ADC_SELECT

// The following definitions are for 4x4 MATRIXED KEYPAD 
#define btnM0     0
#define btnM1     1
#define btnM2     2
#define btnM3     3
#define btnM4     4
#define btnM5     5
#define btnM6     6
#define btnM7     7
#define btnM8     8
#define btnM9     9
#define btnM10    10


// Values for buttons
#define btnRIGHT  11
#define btnUP     12
#define btnDOWN   13
#define btnLEFT   14
#define btnSELECT 15
#define btnNONE   99

// Digital pin connections for 4x4 keypad - define digital pins tied to 4x4 matrix.
// Note this matrix depends on DUE which has many more digital pins than UNO or NANO.
// These pin numbers will have to be reassigned for the NANO or UNO if a 4x4 keyboard is desired.
unsigned int row [4] = {31,33,35,37};  /* Row1 D31, Row2 D33, Row3 D35, Row4 D37 */
unsigned int col [4] = {39,41,43,45};  /* Column1 D39, Column2 D41, Column3 D43, Column4 D45 */

// Map for 4x4 keypad - like phone keypad on its side
// *************************
// *  1  *  4  *  7  *  *  *  
// *************************
// *  2  *  5  *  8  *  0  *  
// *************************
// *  3  *  6  *  9  *  #  *  
// *************************
// *  A  *  B  *  C  *  D  *  
// *************************

// Map for 4x4 keypad - like phone keypad on its side

unsigned int matrix [4][4] = { {1, 4, 7, 10},
                               {2, 5, 8, 0},
                               {3, 6, 9, 11},
                               {12, 13, 14, 15} };


int digital_value;

//**************  64 bits routines written by Dave Brink *************************************
uint32_t zero64[]={0,0}; //just for comparisons sake

void init64(uint32_t  an[], uint32_t bigPart, uint32_t littlePart ){
 an[0]=bigPart;
 an[1]=littlePart;
}

//left shift 64 bit "number"
void shl64(uint32_t  an[]){
an[0] <<= 1;
if(an[1] & 0x80000000)
  an[0]++;
an[1] <<= 1;
}

//right shift 64 bit "number"
void shr64(uint32_t  an[]){
an[1] >>= 1;
if(an[0] & 0x1)
  an[1]+=0x80000000;
an[0] >>= 1;
}

//add ann to an
void add64(uint32_t  an[], uint32_t  ann[]){
 an[0]+=ann[0];
 if(an[1] + ann[1] < ann[1])
   an[0]++;
 an[1]+=ann[1];
}

//subtract ann from an
void sub64(uint32_t  an[], uint32_t  ann[]){
 an[0]-=ann[0];
 if(an[1] < ann[1]){
   an[0]--;
 }
 an[1]-= ann[1];
}

//true if an == ann
boolean eq64(uint32_t  an[], uint32_t  ann[]){
 return (an[0]==ann[0]) && (an[1]==ann[1]);
}

//true if an < ann
boolean lt64(uint32_t  an[], uint32_t  ann[]){
 if(an[0]>ann[0]) return false;
 return (an[0]<ann[0]) || (an[1]<ann[1]);
}

//divide num by den
void div64(uint32_t num[], uint32_t den[]){
 uint32_t quot[2];
 uint32_t qbit[2];
 uint32_t tmp[2];

 if (eq64(num, zero64)) {  //numerator 0, call it 0
   init64(num,0,0);
   return;            
 }

 if (eq64(den, zero64)) { //numerator not zero, denominator 0, infinity in my book.
   init64(num,0xffffffff,0xffffffff);
   return;            
 }

#if DUE
   // Speed up division on DUE.
   uint64_t value, value1, value2; 
   value1 = num[0] * 0x100000000L + num[1];
   value2 = den[0] * 0x100000000L + den[1];
   value = value1 / value2;
   num[0] = (value >> 32) & 0xFFFFFFFF;
   num[1] = value & 0xFFFFFFFF;
#else
   init64(quot,0,0);
   init64(qbit,0,1);

   init64(tmp,0x80000000,0); 
   while(lt64(den,tmp)){ // Shift den until left justified
     shl64(den);   // den *= 2;
     shl64(qbit);  // qbit is power of 2 
   }
 
   while(!eq64(qbit,zero64)){
     if(lt64(den,num) || eq64(den,num)){
       sub64(num,den);
       add64(quot,qbit);
     }
     shr64(den);
     shr64(qbit);
   }

   //remainder now in num, but using it to return quotient for now  
   init64(num,quot[0],quot[1]);
#endif
}

//multiply an by ann
void mul64(uint32_t an[], uint32_t ann[]){
 uint32_t p[2] = {0,0};
 uint32_t y[2] = {ann[0], ann[1]}; 
#if DUE
     uint64_t value, value1, value2;
     value1 = an[0] * 0x100000000L + an[1];
     value2 = ann[0] * 0x100000000L + ann[1];
     value = value1 * value2;
     an[0] = (value >> 32) & 0xFFFFFFFF;
     an[1] = value & 0xFFFFFFFF;
#else
   while(!eq64(y,zero64)) {
     if(y[1] & 1)
       add64(p,an);
     shl64(an);
     shr64(y);
   }
   init64(an,p[0],p[1]);
#endif
}

//******************************End of 64 bits routines****************************************

// ****************************Print variable f64****************************************
void DebugSerialPrint64(const char *title1, uint32_t  an[], const char *title2){

#if DEBUG
  char buffer [21];
  uint64_t value;  // long long is 64 bits

     Serial.print(title1); 
     value = an[0] * 0x100000000L + an[1];
  #if DUE
     sprintf(buffer, "%0llu", value);
     Serial.print(buffer);
  #else
    char* p = &buffer[20];      //this pointer writes into the buffer, starting at the END

    // zero to terminate a C type string
    *p = 0;

    // do digits until the number reaches zero
    do
    {
        // working on the least significant digit
        //put an ASCII digit at the front of the string
        *(--p) = '0' + (int)(value % 10);

        //knock the least significant digit off the number
        value /= 10;
    } while (value != 0);

    //print the whole string
    Serial.print(p);  
 #endif
     Serial.println(title2); 
#endif   
}

// ****************************Print variable f32****************************************
void DebugSerialPrint32(const char *title1, uint32_t  an, const char *title2){

#if DEBUG
  Serial.print(title1); 
  Serial.print(an); 
  Serial.println(title2); 
#endif 
}

//**************************** SP READ BUTTONS ********************************************
int read_buttons()
{

#if ANALOG_BUTTONS
  adc_key_in = analogRead(0);      // read the value from the buttons
  if (adc_key_in < ADC_ANY)
  {
    timer = 1;
    lcd.blink();  

    #if DEBUG_BUTTONS || DEBUG_SETTLE

      #if DEBUG_SETTLE // Test button settling time
        int adc_val_sav = adc_key_in;
        for (int i=0; i < 50; i++)
        {
          Serial.print ("Read #");
          Serial.print (i);
      #endif
      Serial.print (" Button value = ");
      Serial.print ( (3.3 / 1024) * adc_key_in);
      Serial.print (", adc_key_in = ");
      Serial.print (adc_key_in);
      Serial.print ("\n");
      #if DEBUG_SETTLE
          adc_key_in = analogRead(0);
          if (adc_key_in < adc_val_sav) adc_val_sav = adc_key_in;
        }
    adc_key_in = adc_val_sav;
    Serial.print ("-----------------------\n");
  #endif
    Serial.flush();
 #endif
  }
  
  if (adc_key_in < ADC_RIGHT) return btnRIGHT;  // for ROBOT V1.0 Display on Arduino Due 
  if (adc_key_in < ADC_UP) return btnUP;
  if (adc_key_in < ADC_DOWN) return btnDOWN;
  if (adc_key_in < ADC_LEFT) return btnLEFT;
  if (adc_key_in < ADC_SELECT) return btnSELECT; // End ROBOT 1.1 Display on Arduino Due

#elif MATRIXED_BUTTONS

  //** Scan Keyboard
    digital_value = -1;

    for (int r=0; r<4; r++)
    {
      digitalWrite(row[r], LOW);
      for (int c=0; c<4; c++)
      {
        if (digitalRead (col[c]) == LOW)
        {
          // map row and col to value
          digitalWrite(row[r], HIGH);
          digital_value = matrix [r][c];
          lcd.blink();  
          timer = 1;
          break;
        }
      }
      digitalWrite(row[r], HIGH);
    }
    if (digital_value == 2) { 
      #if DEBUG_BUTTONS
        Serial.print ("btnRIGHT");
        Serial.print ("\n");
      #endif
      return btnRIGHT;  // for ROBOT V1.0 Display on Arduino Due 
    }
    if (digital_value == 3) { 
      #if DEBUG_BUTTONS
        Serial.print ("btnUP");
        Serial.print ("\n");
      #endif
      return btnUP;
    }
    if (digital_value == 12){ 
      #if DEBUG_BUTTONS
        Serial.print ("btnDOWN");
        Serial.print ("\n");
      #endif
      return btnDOWN;
    }
    if (digital_value == 1) { 
      #if DEBUG_BUTTONS
        Serial.print ("btnLEFT");
        Serial.print ("\n");
      #endif
      return btnLEFT;
    }
    if (digital_value == 4) { 
      #if DEBUG_BUTTONS
        Serial.print ("btnSELECT");
        Serial.print ("\n");
      #endif
      return btnSELECT; // End ROBOT 1.1 Display on Arduino Due
    }
    // Here are quick memory buttons
    if (digital_value == 5) { 
      #if DEBUG_BUTTONS
        Serial.print ("btnM0");
        Serial.print ("\n");
      #endif
      return btnM0;  // for ROBOT V1.0 Display on Arduino Due 
    }
    if (digital_value == 6) { 
      #if DEBUG_BUTTONS
        Serial.print ("btnM1");
        Serial.print ("\n");
      #endif
      return btnM1;  // for ROBOT V1.0 Display on Arduino Due 
    }
    if (digital_value == 13) { 
      #if DEBUG_BUTTONS
        Serial.print ("btnM2");
        Serial.print ("\n");
      #endif
      return btnM2;  // for ROBOT V1.0 Display on Arduino Due 
    }
    if (digital_value == 7){ 
      #if DEBUG_BUTTONS
        Serial.print ("btnM3");
        Serial.print ("\n");
      #endif
      return btnM3;  // for ROBOT V1.0 Display on Arduino Due 
    }
    if (digital_value == 8){ 
      #if DEBUG_BUTTONS
        Serial.print ("btnM4");
        Serial.print ("\n");
      #endif
      return btnM4;  // for ROBOT V1.0 Display on Arduino Due 
    }
    if (digital_value == 9) { 
      #if DEBUG_BUTTONS
        Serial.print ("btnM5");
        Serial.print ("\n");
      #endif
      return btnM5;  // for ROBOT V1.0 Display on Arduino Due 
    }
    if (digital_value == 14){ 
      #if DEBUG_BUTTONS
        Serial.print ("btnM6");
        Serial.print ("\n");
      #endif
      return btnM6;  // for ROBOT V1.0 Display on Arduino Due 
    }
    if (digital_value == 10){ 
      #if DEBUG_BUTTONS
        Serial.print ("btnM7");
        Serial.print ("\n");
      #endif
      return btnM7;  // for ROBOT V1.0 Display on Arduino Due 
    }
    if (digital_value == 0){ 
      #if DEBUG_BUTTONS
        Serial.print ("btnM8");
        Serial.print ("\n");
      #endif
      return btnM8;  // for ROBOT V1.0 Display on Arduino Due 
    }
    if (digital_value == 11){ 
      #if DEBUG_BUTTONS
        Serial.print ("btnM9");
        Serial.print ("\n");
      #endif
      return btnM9;  // for ROBOT V1.0 Display on Arduino Due 
    }
    if (digital_value == 15){ 
      #if DEBUG_BUTTONS
        Serial.print ("btnM10");
        Serial.print ("\n");
      #endif
      return btnM10;  // for ROBOT V1.0 Display on Arduino Due 
    }

#elif DIGITAL_BUTTONS // KD7TS

  if  (digitalRead(A0) == LOW )
  { lcd.blink(); timer = 1; Serial.print ("RT\n");
    return btnRIGHT;
  };

  if  (digitalRead(A1) == LOW)
  { lcd.blink(); timer = 1; Serial.print ("UP\n");
    return btnUP;
  };

  if  (digitalRead(A2) == LOW)
  { lcd.blink(); timer = 1; Serial.print ("DN\n");
    return btnDOWN;
  };

  if  (digitalRead(A3) == LOW)
  { lcd.blink(); timer = 1; Serial.print ("LT\n");
    return btnLEFT;
  };

  if  (digitalRead(A4) == LOW)
  { lcd.blink(); timer = 1; Serial.print ("SE\n");
    return btnSELECT;
  };

#endif

  return btnNONE;  // no button pushed
}

//***************************** SP Display Frequency on LCD ********************************
void printAll ()
{
  int PDF;
  char digit[2] = " ";
  uint32_t RFval [2];
  uint32_t NUMERATOR [2];
  uint32_t DIVISOR [2];
  uint32_t NX [2];

  // Do formula ( (10^9 * 10^9) / (10^9 + correction) ) * freq / 10^9
  
  init64 (RFval, 0, RFint);
  init64(NX,0,1000);
  mul64 (RFval, NX);
  init64(NX,0,HERTZ);
  add64 (RFval, NX);

  init64(NUMERATOR,0, 1E9);
  init64(NX,0, 1E9);
  mul64 (NUMERATOR, NX);

  init64(DIVISOR,0, 1E9);
  if (correction < 0)
  {
    init64 (NX, 0, -correction);
    sub64 (DIVISOR, NX);
  }
  else
  {
    init64 (NX, 0, correction);
    add64 (DIVISOR, NX);
  }

  div64 (NUMERATOR, DIVISOR);
  
  mul64 (RFval, NUMERATOR);

  init64 (NX, 0, 1E9);

  div64 (RFval, NX);

  DebugSerialPrint64("Corrected Frequency is ", RFval, " Hz");
  
  lcd.setCursor(0,0); // line 1
  RFcalc=(RFint/1000000);  // RFint in kHz
  // Do GHz
  if (RFcalc<10)lcd.print(" ");
  if (RFcalc==0)lcd.print(" ");
  if (RFcalc>0)
  {
    lcd.print(RFcalc);
    lcd.print(",");
    strcpy (digit,"0");
  }
  else
  {
    lcd.print(" ");
  }
  // Do MHz
  RFcalc=RFint-((RFint/1000000)*1000000);
  if (RFcalc < 100000) lcd.print(digit);
  if (RFcalc < 10000)  lcd.print(digit);
  if (RFcalc >= 1000) 
  {
    lcd.print(RFcalc/1000);
    lcd.print(",");
    strcpy (digit,"0");
  }
  else if (RFcalc < 1000 && (strcmp(digit,"0") == 0) )
  {
    lcd.print("0");
    lcd.print(",");
  }
  // Do kHz
  RFcalc=RFint-((RFint/1000)*1000);
  if (RFcalc<100)lcd.print(digit);
  if (RFcalc<10)lcd.print(digit);
  if (RFcalc > 0) 
  {
    lcd.print(RFcalc);
    lcd.print(",");
    strcpy (digit,"0");
  }
  else if (RFcalc == 0 && (strcmp(digit,"0") == 0) )
  {
    lcd.print(RFcalc);
    lcd.print(",");
  }
  RFcalc=HERTZ-((HERTZ/1000)*1000);
  if (RFcalc<100)lcd.print(digit);
  if (RFcalc<10)lcd.print(digit);
  lcd.print(HERTZ);
  lcd.print("Hz");

  lcd.setCursor(0,1); // 2nd line
  if (WEE==0) {lcd.print("R=");}
  else {lcd.print("W=");}
  if (memory<10){lcd.print(" ");}
  lcd.print(memory,DEC);
  delay(100);
  locked = (digitalRead(2)==1);
  if  (locked)lcd.print(" LCKED ");
  else lcd.print(" NOLCK ");
  lcd.print("L");
  lcd.print(rflevel);
  lcd.print(" ");
  lcd.print(PFDRFout,DEC);
  lcd.setCursor(poscursor,line);
}

//**** display LOCK/NOLOCK on LCD **************
void printLocked ()
{
  lcd.setCursor(4, 1);
  locked = (digitalRead(2) == 1);
  if  (locked) lcd.print(" LCKED ");
  else lcd.print(" NOLCK ");
  lcd.setCursor(poscursor, line);
}

void WriteRegister32(const uint32_t value)   //Program register 32bits
{
  digitalWrite(ADF5355_LE, LOW);
  for (int i = 3; i >= 0; i--)          // loop on 4 x 8bits
    SPI.transfer((value >> (8 * i)) & 0xFF); // Offset, hiding the byte and sending via SPI
  digitalWrite(ADF5355_LE, HIGH);
  digitalWrite(ADF5355_LE, LOW);
}

//MAY NEED TO ADD A DELAY AFTER WRITING REGISTER 1 BEFORE WRITING REGISTER 0 - SEE DATA SHEET
void SetADF5355()  // Program all the registers of the ADF5355
{ 
  for (int i = 12; i >= 0; i--)  // program ADF5355 starting with R12
  {
    WriteRegister32(registers[i]);
    RegisterSerialPrint(i);
  }
}

//MAY NEED TO ADD A DELAY A END AFTER WRITING REGISTER 4 BEFORE WRITING REGISTER 0 - SEE DATA SHEET
void UpdateADF5355() {

   AMP=3;I_BLEED_N=9; // N=9
   WriteRegister32((RF_Select<<21)|(I_BLEED_N<<13)|(RFB<<10) | ((rflevel & 3) << 4) |
      (0x35000046) ); //R6      0X35000076 == R6 with RF_divider and I_BLEED
   RegisterValueSerialPrint(6,(RF_Select<<21)|(I_BLEED_N<<13)|(RFB<<10)|((rflevel & 3) << 4) |
      (0x35000046) );
   #if DEBUG
     if (RFB==1) {Serial.println ("RFB = OFF");} else {Serial.println("RFB = ON");}         
     Serial.print("RF_Select = ");Serial.println(RF_Select,DEC);// 
     Serial.print("MOD2 = ");Serial.println(MOD2[1]);
     Serial.print("FRAC2 = ");Serial.println(FRAC2R);
   #endif
   WriteRegister32(4|R04_EN|(MULTVCO2<<26));                     //R4  DB4=1
   RegisterValueSerialPrint(4,4|R04_EN|(MULTVCO2<<26));
   WriteRegister32(2|(MOD2[1]<<4)|(FRAC2R<<18));               //R2
   RegisterValueSerialPrint(2,2|(MOD2[1]<<4)|(FRAC2R<<18));
   #if DEBUG
     Serial.print("FRAC1 = ");Serial.println(FRAC1R);
   #endif
   WriteRegister32(1|(FRAC1R<<4));                             //R1
   RegisterValueSerialPrint(1,1|(FRAC1R<<4));
   #if DEBUG
     Serial.print("INT = ");Serial.println(INTR);
   #endif
   WriteRegister32(0|(INTR<<4));                                 //R0   
   RegisterValueSerialPrint(0,0|(INTR<<4));
   WriteRegister32(R04|(MULTVCO2<<26));                          //R4  DB4=0
   RegisterValueSerialPrint(4,R04|(MULTVCO2<<26));
   delay(3); //  pause 3 ms for VCO stabilization
   WriteRegister32(0|0X200000|(INTR<<4));                      //R0               
   RegisterValueSerialPrint(0,0|0X200000|(INTR<<4));
}

// The following routines were by AA5C
// Function to write a byte to EEPROM
void writeEEPROM(uint16_t address, byte data)
{
#if DUE
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)(address >>8));     //MS byte of 16-bit address
  Wire.write((int)(address & 0xFF));  //LS byte of 16-bit address
  Wire.write(data);                   //write the byte
  Wire.endTransmission();
  delay(5);
#else
  EEPROM.write(address, data);
#endif
}

// Function to write a 32-bit word to external EEPROM - code by AA5C   
void writeEEPROMlong(uint16_t address,uint32_t value) //address is address within EEPROM
  {
  byte four = (value & 0xFF);             //Least significant byte
  byte three = (value >>8 & 0xFF);
  byte two = (value >>16 & 0xFF);
  byte one = (value >>24 & 0xFF);        //Most significant byte      
  
#if DUE
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)(address >> 8));       // MS Byte of address
  Wire.write((int)(address & 0xFF));     // LSB of address
  Wire.write(four);                      //write the least significant byte first
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)((address +1) >>8));
  Wire.write((int)((address +1) & 0xFF));
  Wire.write(three);
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)((address +2) >> 8)); 
  Wire.write((int)((address +2) & 0xFF));   
  Wire.write(two);
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)((address +3) >>8));
  Wire.write((int)((address +3) & 0xFF));
  Wire.write(one);                      //write the most significant byte last
  Wire.endTransmission();
  delay(5);
#else
  EEPROM.write(address, four);
  EEPROM.write(address+1, three);
  EEPROM.write(address+2, two);
  EEPROM.write(address+3, one);
#endif
}

//Code for reading a byte from external EEPROM
byte readEEPROM(uint16_t address)  
{
#if DUE
  byte rbyte = 0xFF;
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)(address >>8));      //MSB of address
  Wire.write((int)(address & 0xFF));   //LSB of address
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_ADDRESS,1);  //read 1 byte
  if (Wire.available()) rbyte = Wire.read();
  return rbyte;
#else
  return EEPROM.read(address);
#endif
}

//Code for reading 32-bit word from external EEPROM - by AA5C
uint32_t readEEPROMlong(uint16_t address) 
{
  uint32_t four = 0x4;
  uint32_t three = 0x3;
  uint32_t two = 0x2;
  uint32_t one = 0x1;

  uint32_t rdata = 0x76543210;

#if DUE
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)(address >> 8));       // MSB of address
  Wire.write((int)(address & 0xFF));     // LSB of address
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_ADDRESS,1);    //read 1 byte
  if (Wire.available()) four = Wire.read();    //reads least significant byte first
     
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)((address +1) >> 8));    //MSB of address
  Wire.write((int)((address +1) & 0xFF));  //LSB of address
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_ADDRESS,1);   //read 1 byte
  if (Wire.available()) three = Wire.read();

  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)((address + 2) >>8));   //MSB of address
  Wire.write((int)((address + 2) & 0XFF));  //LSB of address
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_ADDRESS,1);
  if (Wire.available()) two = Wire.read();
  
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)((address +3) >>8));  //MSB of address
  Wire.write((int)((address + 3) & 0xFF));  //LSB of address
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_ADDRESS,1); 
  if (Wire.available()) one = Wire.read();  //read most significant byte 
  rdata = ((four << 0) & 0xFF) + ((three  << 8) & 0xFFFF) + ((two <<16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
  //Serial.print("rdata= ");
  //Serial.println(rdata,HEX );
  return rdata;
  delay(5);
#else
  //Read the 4 bytes from the eeprom memory.
  four = EEPROM.read(address);
  three = EEPROM.read(address + 1);
  two = EEPROM.read(address + 2);
  one = EEPROM.read(address + 3);

  //Returns a long (32bits) using the shift of 0, 8, 16 and 24 bits and masks
  rdata = ((four << 0) & 0xFF) + ((three  << 8) & 0xFFFF) + ((two <<16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
  return rdata;
#endif
}

void setCorrection(int32_t calibration = 0)
{
  correction = calibration;
}

//************************************ Setup ****************************************
void setup() {
  int i;

  rflevel = 3; rflevelold = -1;

  Serial.begin (115200); //  Serial to the PC via Arduino "Serial Monitor"  at 115200 baud
#if DUE
  Wire.begin();   // AA5C - needed to start wire function for external EEPROM
#endif

#if !ANALOG_BUTTONS
  pinMode(row[0], OUTPUT);
  pinMode(row[1], OUTPUT);
  pinMode(row[2], OUTPUT);
  pinMode(row[3], OUTPUT);
  digitalWrite(row[0],HIGH);
  digitalWrite(row[1],HIGH);
  digitalWrite(row[2],HIGH);
  digitalWrite(row[3],HIGH);
  pinMode(col[0], INPUT_PULLUP);
  pinMode(col[1], INPUT_PULLUP);
  pinMode(col[2], INPUT_PULLUP);
  pinMode(col[3], INPUT_PULLUP);

#endif
  
  lcd.begin(16, 2); // two 16 characters lines
  lcd.display();
  analogWrite(10,255); //LCD brightness
  
  lcd.print("   ADF5355   ");
  lcd.setCursor(0, 1);
  lcd.print("Signal Generator");
  poscursor = 0; line = 0; 
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.print("F1CJN/AA5C/W7GLF");
  lcd.setCursor(0, 1);
  strncpy (version, "ver ", sizeof(version));
  strncpy (&version[4], __FILENAME_VER__, sizeof(version)-4);
  for (i=strlen(version)-1; i > 0; i--) 
  {
    if (version[i] == '.')
    { 
      break;
    }
  }
  while (i < 16)
  {
     version [i++] = ' ';
  }
  lcd.print(version);
  Serial.print (__FILENAME__);
  delay(2000);

  pinMode(2, INPUT);  // PIN 2 input lock signal
  pinMode(ADF5355_LE, OUTPUT);          // Setup pins
  digitalWrite(ADF5355_LE, HIGH);
  SPI.begin();                          // Init SPI bus
  SPI.setDataMode(SPI_MODE0);           // CPHA = 0 and Clock positive
  SPI.setBitOrder(MSBFIRST);            // bit order

   if (readEEPROM(MFREF)==INUSE)
   { // if the ref is written in EEPROM read it
     PFDRFout = readEEPROM(PDREF);
   } 
   else 
   { // change to 10 for default of 10 MHz reference
     PFDRFout = OnboardOsc;
   }
   // Calc VCO_BAND_DIV
   VCO_BAND_DIV = (((PFDRFout << MULTVCO2) * 10 + 23)/24);
   registers[9] = (registers[9] & 0x00FFFFFF) +  ((uint32_t)VCO_BAND_DIV << 24);
   ADC_CLK_DIV = ((((PFDRFout << MULTVCO2) * 10 - 2) + 3) / 4);
   if (ADC_CLK_DIV > 255) ADC_CLK_DIV = 255;
   registers[10] = (registers[10] & 0xFFFFC03F) + (ADC_CLK_DIV << 6);
   
#if USEMULTVCO2
   if (PFDRFout == 10)
   {
     MULTVCO2 = 1;
   }
   else
   {
     MULTVCO2 = 0;
   }
#else
   MULTVCO2 = 0;
#endif
     
   if (readEEPROM(MFREQ)==INUSE)
   {  // if a frequency is written in EEPROM memory 0
     RFint=readEEPROMlong(0);
     HERTZ=readEEPROMlong(4);
     if (HERTZ > 1000 || HERTZ < 0) HERTZ = 0;
     rflevel=readEEPROM(8) & 0x3;
   }
   else 
   {
     RFint=13600000;
     HERTZ=0;
   }
  
  RFintold=1234;//Force RF that is different from the initial RF value  
  RFout = RFint;
  first_time = true;

  WEE=0;  address=0;
  lcd.blink();
  timer = 1;
  printAll(); 
  delay(500);
  
  SetADF5355();
  RFB=1;  // Start with RFB off

} // End setup

//*************************************Loop***********************************
void loop()
{
    
  RFOUT=RFint;  // RFint in kHz
  RFB=1; // RFB off
  X2=0; // Times 2 off
  FPFD[0] = 0;
  FPFD[1] = PFDRFout << MULTVCO2;

  if ((RFint != RFintold) || (rflevel != rflevelold) || (modif==1) || (HERTZ != HERTZold) ) {
#if DEBUG
    Serial.print("RFOUT = ");
    RFout_temp = RFOUT;
    RFout_temp = RFout_temp*1000 + HERTZ;
    RFout_temp /= 1000000;    
    Serial.print(RFout_temp,6);Serial.print(" MHz\r\n");
    Serial.print("REF = ");
    Serial.print(FPFD[1]);
    Serial.print(" MHz\r\n");
#endif

    //  Determine RF divider value and RF A or B output enable/disable based on frequency input. 
    if((53125<=RFOUT)&&(RFOUT<106250))   {RF_Divider=64;RF_Select=6;}  // RFOUT = Frequeny in KHz
    if((106250<=RFOUT)&&(RFOUT<212500))  {RF_Divider=32;RF_Select=5;}  
    if((212500<=RFOUT)&&(RFOUT<425000))  {RF_Divider=16;RF_Select=4;}
    if((425000<=RFOUT)&&(RFOUT<850000))  {RF_Divider=8;RF_Select=3;} 
    if((850000<=RFOUT)&&(RFOUT<1700000)) {RF_Divider=4;RF_Select=2;}
    if((1700000<=RFOUT)&&(RFOUT<3400000)){RF_Divider=2;RF_Select=1;}
    if((3400000<=RFOUT)&&(RFOUT<6800000)){RF_Divider=1;RF_Select=0;}
    if((6800000<=RFOUT)&&(RFOUT<=13600000)){RF_Divider=1;RF_Select=0;X2=1;RFB=0;} // set X2 to 1 //
    FVCO[1]=RFint*RF_Divider;  // FCVO now in kHz
    DebugSerialPrint64 ("FVCO at 1 is ", FVCO, " kHz");
 
    FPFDTEMP[0]=FPFD[0]; FPFDTEMP[1]=FPFD[1];
    FVCOTEMP[0]=FVCO[0]; FVCOTEMP[1]=FVCO[1];
 
    init64(N,0,1E3);
    mul64(FVCO,N);  // FVCO now in Hz

    init64(N,0,10);
    mul64(FVCO,N);  // FVCO now in deciHz

    DebugSerialPrint64 ("FVCO at 2 is ", FVCO, " 1E-1");
 
    init64(N,0,2);              // X2==1 means 6.8GHZ < RFOUT < 13.6GHz
    if (X2==1){div64(FVCO,N);}; // Divide by 2 for 6.8GHZ < RFOUT < 13.6GHz after FVCO is between 3.4 and 6.8GHz

    // multiply by ten so we can do fractional Hz for freq > 6.8
    init64(N,0,10);
    mul64(HZ,N); // HZ (in deciHz) = HERTZ*10*RF_Divider 

    init64(N,0,2);     // Also divide HZ by two
    if (X2==1){div64(HZ,N);};
    add64(FVCO,HZ);   // Add in HZ

    // At this point range is actual VCO Frequency times 10 
    DebugSerialPrint64 ("FVCO at 3 is ", FVCO, " 1E-1");

    init64(N,0,1E4);           // Why times 10^4 to handle round off from divide by 25
    mul64(FVCO,N);

    DebugSerialPrint64 ("FVCO at 4 is           ", FVCO, " 1E-5");

    div64(FVCO,FPFD);
    N[0]=FVCO[0];N[1]=FVCO[1];
    INT[0]=N[0];INT[1]=N[1];

    DebugSerialPrint64 ("INT (FVCO/FPFD) at 5 is ", INT, " 1E-5");
    DebugSerialPrint64 ("N (= INT) at 5 is ", N, " 1E-5");
 
    init64(NX,0,1E5);
    div64(INT,NX);

    DebugSerialPrint64 ("INT at 6 is ", INT, " Hz");
 
    init64(NX,0,1E6);
    div64(INT,NX);

    DebugSerialPrint64 ("INT at 7 is ", INT, " MHz");
 
    INTR=(INT[1]);

    DebugSerialPrint32 ("INTR at 8 is ", INTR, " MHz");
 
    init64(NX,0,1E6);
    mul64(INT,NX);
    init64(NX,0,1E5);
    mul64(INT,NX);

    DebugSerialPrint64 ("INT at 9 is ", INT, " 1E-5");
 
    sub64(N,INT);

    DebugSerialPrint64 ("N = N - INT at 10 is ", N, " 1E-5");
 
    mul64(N,MOD1);
    
    FRAC1exact[0]=N[0];FRAC1exact[1]=N[1];

    DebugSerialPrint64 ("MOD1 at 11 is ", MOD1, " ");
    DebugSerialPrint64 ("N *= MOD1 at 11 is ", N, " 1E-5");
    DebugSerialPrint64 ("FRAC1exact (=N) at 11 is ", FRAC1exact, " 1E-5");
 
    init64(NX,0,1E5);
    div64(N,NX);        // Now in Hz
    init64(NX,0,1E6);
    div64(N,NX);        // Now in MHz

    DebugSerialPrint64 ("N / 1E11 at 12 is ", N, " MHz");

    FRAC1R=N[1];

    DebugSerialPrint32 ("FRAC1R at 13 is ", FRAC1R, " MHz");

    FRAC11[0]=N[0];FRAC11[1]=N[1];
    init64(NX,0,1E6);
    mul64(FRAC11,NX);
    init64(NX,0,1E5);
    mul64(FRAC11,NX);

    DebugSerialPrint64 ("FRAC11 at 14 is ", FRAC11, " 1E-5");
  
    sub64(FRAC1exact,FRAC11);

    DebugSerialPrint64 ("FRAC1exact -= FRAC11 at 15 is ", FRAC1exact, " 1E-5");

    init64(NX,0,1E5);
    div64(FRAC1exact,NX);
    init64(NX,0,1E2);
    div64(FRAC1exact,NX);

    DebugSerialPrint64 ("FRAC1exact at 16 is ", FRAC1exact, " 1E2");
 
    init64(NX,0,5);// round up
    add64(FRAC1exact,NX);

    DebugSerialPrint64 ("FRAC1exact add 5 at 17 is ", FRAC1exact, " 1E2");
 
    init64(NX,0,10); 
    div64(FRAC1exact,NX);

    DebugSerialPrint64 ("FRAC1exact /= 10 at 18 is ", FRAC1exact, " 1E1");
 
    FRAC2R=FRAC1exact[1];

    DebugSerialPrint32 ("FRAC2R at 19 is ", FRAC2R, " 1E1");

    MOD2[0] = 0;
    MOD2[1] = 1000;

    FPFD[0]=FPFDTEMP[0]; FPFD[1]=FPFDTEMP[1];
    FVCO[0]=FVCOTEMP[0]; FVCO[1]=FVCOTEMP[1];
    I_BLEED_N=7;  ///
 
    if (first_time) 
    {
      // I had to add this to get synth to lock after power up - W7GLF
      SetADF5355();  // Try always program all the registers of the ADF5355
      first_time = false;
    }
   
    // I was having problems with synth locking following UP/DOWN buttons until I added 
    // the call to UpdateADF5355 below...
    UpdateADF5355();  // Update the frequency

    RFintold=RFint; rflevelold = rflevel; modif=0; HERTZold = HERTZ;
    printAll();  // Display LCD
  }

  lcd_key = read_buttons();  // read the buttons

  if (lcd_key <= btnM10)  // Fast recall keys
  {
    memory = lcd_key;
    RFint=readEEPROMlong(memory*9);   // read frequency from EEPROM and display
    HERTZ=readEEPROMlong(memory*9+4); // read frequency from EEPROM and display
    rflevel=readEEPROM(memory*9+8) & 3;   // read level from EEPROM and display
    if (RFint>13600000) RFint=13600000;
    if (HERTZ > 999) HERTZ = 999;
    if (HERTZ < 0) HERTZ = 0;
    printAll(); 
  }
  else
  {
    switch (lcd_key)               // Select action
    {
    case btnRIGHT: //RIGHT
      poscursor++; // cursor to the right
      if (line == 0) 
      {
        if (poscursor == 10 ) poscursor = 11;
        else if (poscursor == 6 ) poscursor = 7;
        else if (poscursor == 2 ) poscursor = 3;
        else if (poscursor == 14 ) 
        {
          poscursor = 0; line = 1; 
        }; 
      }
      if (line == 1) {
        if (poscursor == 1 ) poscursor = 3; //if cursor on the figure memory 
        else if (poscursor == 4 ) poscursor = 12; //if cursor on the figure memory 
        else if (poscursor == 13 ) poscursor = 15; //if cursor on the figure memory 
        else if (poscursor==16) 
        {
          poscursor=0; 
          line=0;
        };     
      }  
      //Serial.print (" RIGHT Button\r\n");
      lcd.setCursor(poscursor, line);
      break;
      
    case btnLEFT: //LEFT
      poscursor--; // offset cursor
      if (line == 0) {
        if (poscursor == 255) 
        {
          poscursor = 15;
          line=1;
        }
        else if (poscursor == 10) poscursor = 9;
        else if (poscursor == 6) poscursor = 5; 
        else if (poscursor == 2) poscursor = 1; 
      }
      if(line==1){
          if (poscursor==255) {poscursor=13; line=0;}
          else if (poscursor==2) poscursor=0;
          else if (poscursor==11) poscursor=3;
          else if (poscursor==14) poscursor=12;
      }
      //Serial.print("back  button");
      //Serial.print(poscursor,DEC);
      lcd.setCursor(poscursor, line);
      break;
      
    case btnUP: //top
      if (line == 0)
      { // RFoutfrequency
        if (poscursor == 0) RFint = RFint + 10000000 ;
        if (poscursor == 1) RFint = RFint + 1000000 ;
        if (poscursor == 3) RFint = RFint + 100000 ;
        if (poscursor == 4) RFint = RFint + 10000 ;
        if (poscursor == 5) RFint = RFint + 1000 ;
        if (poscursor == 7) RFint = RFint + 100 ;
        if (poscursor == 8) RFint = RFint + 10 ;
        if (poscursor == 9) RFint = RFint + 1 ;
        if (poscursor == 11) HERTZ = HERTZ + 100 ;
        if (poscursor == 12) HERTZ = HERTZ + 10 ;
        if (poscursor == 13) HERTZ = HERTZ + 1 ;
        if (RFint == 13600000 && HERTZ > 0) HERTZ = 0;
        if (HERTZ >= 1000) {HERTZ=HERTZ-1000; RFint=RFint+1;} 
        if (RFint > 13600000)RFint = RFintold;
      }
      else if (line == 1)
      { 
        if (poscursor == 3)
        { 
          memory++; 
          if (memory==MAXMEM)memory=0;
          if (WEE==0)
          {
            RFint=readEEPROMlong(memory*9); // read frequency from EEPROM and display
            HERTZ=readEEPROMlong(memory*9+4); // read frequency from EEPROM and display
            rflevel=readEEPROM(memory*9+8) & 3; // read level from EEPROM and display
            if (RFint>13600000) RFint=13600000;
            if (HERTZ > 999) HERTZ = 999; 
            if (HERTZ < 0) HERTZ = 0; 
          }
        }
        if (poscursor == 12 && rflevel < 3) ++rflevel ;        
        if (poscursor==15)
        { 
#if USEMULTVCO2
          if(PFDRFout==10) {PFDRFout=26;MULTVCO2=0;} //read REF and swap
          else if ( PFDRFout==26){PFDRFout=25;MULTVCO2=0;}
          else if ( PFDRFout==25){PFDRFout=20;MULTVCO2=0;}
          else if ( PFDRFout==20){PFDRFout=10;MULTVCO2=1;}
#else
          if(PFDRFout==10) {PFDRFout=26;} //read REF and swap
          else if ( PFDRFout==26){PFDRFout=25;}
          else if ( PFDRFout==25){PFDRFout=20;}
          else if ( PFDRFout==20){PFDRFout=10;}
#endif
          else PFDRFout=OnboardOsc;// In the case of PFDRF being different from 10, 20, 25 or 26
          // Recalc VCO_BAND_DIV
          VCO_BAND_DIV = (((PFDRFout << MULTVCO2) * 10 + 23)/24);
          registers[9] = (registers[9] & 0xFFFFFF) + ((uint32_t)VCO_BAND_DIV << 24);
          ADC_CLK_DIV = ((((PFDRFout << MULTVCO2) * 10 - 2) + 3) / 4);
          if (ADC_CLK_DIV > 255) ADC_CLK_DIV = 255;
          registers[10] = (registers[10] & 0xFFFFC03F) + (ADC_CLK_DIV << 6);
          first_time = true;  // Force reload of all registers
          modif=1;  
        }
                    
        if( (poscursor==0) && (WEE==1))WEE=0;
        else if ((poscursor==0) && (WEE==0))WEE=1;                  
      }
      printAll();
      break; // end button up

    case btnDOWN: //down 
      if (line == 0) {
        if (poscursor == 0) RFint = RFint - 10000000 ;
        if (poscursor == 1) RFint = RFint - 1000000 ;
        if (poscursor == 3) RFint = RFint - 100000 ;
        if (poscursor == 4) RFint = RFint - 10000 ;
        if (poscursor == 5) RFint = RFint - 1000 ;
        if (poscursor == 7) RFint = RFint - 100 ;
        if (poscursor == 8) RFint = RFint - 10 ;
        if (poscursor == 9) RFint = RFint - 1 ;
        if (poscursor == 11) {HERTZ = HERTZ - 100;}
        if (poscursor == 12) {HERTZ = HERTZ - 10 ;}
        if (poscursor == 13) {HERTZ = HERTZ - 1 ;}
        if (HERTZ < 0) {HERTZ=1000+HERTZ; RFint=RFint-1;} 
          
        if (RFint < 54000) RFint = RFintold;
        if (RFint > 13600000)  RFint = RFintold;
      }
      else if (line == 1)
      { 
        if (poscursor == 3)
        {
          memory--; 
          if (memory==0xFFFF)memory=MAXMEM-1;
          if (WEE==0)
          {
            RFint=readEEPROMlong(memory*9); // read frequency from EEPROM and display
            HERTZ=readEEPROMlong(memory*9+4); // read frequency from EEPROM and display
            rflevel=readEEPROM(memory*9+8) & 3;; // read level
            if (RFint>13600000) RFint=13600000;
            if (HERTZ > 999) HERTZ = 999; 
            if (HERTZ < 0) HERTZ = 0; 
          } 
        } // end poscursor = 4 
        if (poscursor == 12 && rflevel > 0) --rflevel ;        
        if (poscursor==15)
        { 
#if USEMULTVCO2
          if( PFDRFout==10){PFDRFout=20;MULTVCO2=0;} //swap REF from 10 to 25 or 25 to 10 with down button
          else if ( PFDRFout==20){PFDRFout=25;MULTVCO2=0;}
          else if ( PFDRFout==25){PFDRFout=26;MULTVCO2=0;}
          else if ( PFDRFout==26){PFDRFout=10;MULTVCO2=1;} // 10 MHz
#else
          if(PFDRFout==10) {PFDRFout=20;} //read REF and swap
          else if ( PFDRFout==20){PFDRFout=25;}
          else if ( PFDRFout==25){PFDRFout=26;}
          else if ( PFDRFout==26){PFDRFout=10;}
#endif
          else PFDRFout=OnboardOsc;// In the case or PFDRF different from 10, 25 or 26
          // Recalc VCO_BAND_DIV
          VCO_BAND_DIV = (((PFDRFout << MULTVCO2) * 10 + 23)/24);
          registers[9] = (registers[9] & 0xFFFFFF) + ((uint32_t)VCO_BAND_DIV << 24);
          ADC_CLK_DIV = ((((PFDRFout << MULTVCO2) * 10 - 2) + 3) / 4);
          if (ADC_CLK_DIV > 255) ADC_CLK_DIV = 255;
          registers[10] = (registers[10] & 0xFFFFC03F) + (ADC_CLK_DIV << 6);
          first_time = true;  // Force reload of all registers
          modif=1;
        }
                   
        if( (poscursor==0) && (WEE==1))WEE=0;
        else if ((poscursor==0)&&(WEE==0))WEE=1;                          
      }      
      printAll();
      // Serial.print (" DOWN Button  \r\n");
      break; // end button bottom

    case btnSELECT:
      do 
      {
        lcd_key = read_buttons();      // Test release button
        delay(1); timer2++;        // timer inc all of 1 milliseconds
        if (timer2 > 600) 
        { //waiting 600 milliseconds
          if (WEE==1 || poscursor==15)
          { 
            if (line==1 && poscursor==15) // Ref Freq
            { 
              writeEEPROM(PDREF,PFDRFout);
              writeEEPROM(MFREF,INUSE);
            } // write REF and flag
            else if (WEE==1) // Memory
            {
              writeEEPROMlong(memory*9,RFint);
              writeEEPROMlong(memory*9+4,HERTZ);
              writeEEPROM(memory*9+8,rflevel);
              if (memory == 0)
              {
                writeEEPROM(MFREQ,INUSE);
              }
            }// write RF in EEPROM at address (memory*9)
            // four bytes per RF value
            lcd.setCursor(0,1); lcd.print("    SAVED    ");
          }
          lcd.setCursor(poscursor,line);
          delay(500);timer2=0;
          printAll();
        }
      } while (lcd_key != btnNONE); // release hold
      break;  // End button select

     case btnNONE: {
        break;
      };
      break;
    }// End switch LCD keys
  }

   while (lcd_key != btnNONE) // Wait until buttons are released
   { 
     lcd_key = read_buttons(); 
     delay(1);
   }

   // No button is pushed
   delay (10);
   if (timer) // Is cursor timer running?
   {
     timer++; // inc timer
     if (timer > 1000) // See if it has been over ten seconds
     {
       lcd.noBlink();
       timer=0;
     } // cursor off
   }

   // KD7TS complained about LOCK on display not being correct so double check here
   if (locked != (digitalRead(2) == 1)) printLocked();

}   // end loop
