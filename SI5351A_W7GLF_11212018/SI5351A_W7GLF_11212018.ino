//   ADF5351 and Arduino
//
//   By W7GLF
//
//   This code is freeware. No warranty or liability, expressed or implied, is included with this sketch. Commercial use is prohibited. 
//
//   update February  8, 2018  W7GLF Allow use of matrixed digital keyboard.  
//   update August   21, 2018  W7GLF - Add ability to use analog pins as digital pins for buttons for KD7TS.
//   update November 11, 2018  W7GLF - Hack code for Si5351
//
//
//  This code runs on a UNO or NANO.
//
//  If one or more frequencies are stored, then at power on, memory zero is always selected.
//
//  The cursor moves with the LEFT and RIGHT buttons. Then the underlined digit can be modified with the UP and DOWN buttons, 
//  for the frequency, the memories and the frequency reference (10 or 25 MHz):
//  - to change the frequency, move the cursor to the digit to be modified, then use the UP and DOWN buttons,
//  - to modify the memory number,move the cursor to the number to be modified, then use the UP and DOWN buttons,
//  - to select the reference frequency, move the cursor on 10 or 25 and select with UP and DOWN.
//  - to read or write the frequency in memory, place the cursor on the more left/more down position and select REE (for Reading EEprom)
//  or WEE (for Writing EEprom).
//  The cursor dissapears after few seconds and is re activated if a button is pressed.
//
//   STORING FREQUENCY AND REFERENCE VALUES 
//    - For the frequency, select WEE, then select the memory number, then push the SELECT button for a second. The word SAVED 
//    appears on the screen. This memorization works then the cursor is anywhere except on the reference 10 or 25 position.
//    - For the reference frequency, move the cursor to 10 or 25, the press SELECT for one second. 
//
//  ******************************************** HARDWARE IMPORTANT********************************************************
//  Note that the SDA and SCL outputs connect to pins A4 and A5 on the NANO.  
//  Note that the SCL and SDA outputs connect to pins SCL and SDA on the UNO.
//
// The Adafruit SI5351A runs on 3.3 volts but it has its own regulator.  It should be connected to +5 on the UNO/NANO.
//
//  *********************************** Here are comments KD7TS added for his configuration *******************************
//  *********************************** using a NANO and his button setup - probably not **********************************
//  *********************************** of interest to others unless copying his setup ************************************
//  * 
//  *  These comments are for NANO ONLY
//  * 
//  *  1602 LCD connections to NANO (not a button board/display)
//  * 
//  *  LCD pin RS to digital pin D8
//  *  LCD pin E  to digital pin D9 (enable)
//  *  LCD pin D4 to digital pin D4
//  *  LCD pin D5 to digital pin D5
//  *  LCD pin D6 to digital pin D6
//  *  LCD pin D7 to digital pin D7
//  * 
//  *  LCD R/W pin to ground
//  *  LCD VSS pin to ground
//  *  LCD VCC pin to 5V
//  *  10K resistor: this (is a pot)
//  *  ends to +5V and ground
//  *  wiper to LCD VO pin (pin 3)voltage on VO around 9/10ths of a volt when text is visible.
//  * 
//  *   FREQUENCY SELECT BUTTONS
//  * 
//  *   5 buttons are wired from A0 - A3 and A6 to ground through the same 120 ohm resistor. The pins
//  *   are assigned as digital inputs with pull up resistors. Pins A4 and A5 are reserved for use as SDA and SCL
//  *   for I2C communication to the Si5351A board.
//  * 
//  *********************************************** END OF KD7TS COMMENTS *************************************************
//
//  ************************************************* BUTTON OPERATION*****************************************************
//
//  Touch LEFT    cursor to the left
//  Touch RIGHT   cursor to the right
//  Touch UP      increase frequency
//  Touch DOWN    decrease frequency
//  Touch SELECT  long push = frequency memorization into the EE number EEPROM / or reference memorization
//  *************************************************************************************************************************
//  Warning : if you are using a ROBOT Shield version 1.1 it will be necessary to rescale the
//  threshold values in the read_buttons sub routine 

// The correction value can be used to tweak the frequency of the internal oscillator.  Let the synth run for a while
// and measure the actual frequency on a counter.  The value will be ratio of the measured divided by the expected minus 1
// times one billion.  Thus if the synth was set for 100,000,000 Hz and the counter said 100,000,625 Hz then the value
// would be 100000625/100000000 = 1.00000625 minus one which would be .00000625 and then times 1000000000 which would 
// be 6250.

// correction = ((Measured/Expected) - 1) * 10^9

// Not needed when running from a GPS locked 10 MHz reference.
long correction = 0;

char version [17] = "v 11/21/18 18:00";

// Only one of the three following conditions should be set to true

#define ANALOG_BUTTONS true  // true for DF Robot button shield
#define DIGITAL_BUTTONS false  // true for individual buttons tied to digital inputs - KD7TS uses this
#define MATRIXED_BUTTONS false  // true for 4x4 matrix keypad

// W7GLF - added some DEBUG variables.

#define DEBUG true

#define DEBUG_BUTTONS false
#define DEBUG_SETTLE false

#include <LiquidCrystal.h>
#include <SPI.h>
#include <EEPROM.h>
#include "si5351.h"

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
Si5351 si5351;

// W7GLF - EEPROM Layout
#define OnboardOsc 10
#define MAXMEM 100  // Number of memories that can be saved
#define MSIZE 5 // 4 bytes for frequency in Hz 
                // 1 for centihertz
#define PDREF  MAXMEM*MSIZE
#define MFREF PDREF+1  // save INUSE REF freq flag at startup
#define MFREQ PDREF+2  // save INUSE frequency flag at startup
#define INUSE 55  // Special Bit pattern to mark memory is in use

#if MFREQ >= 1024
#error EEPROM is too small to hold MAXMEM entries
#endif

uint32_t ref_freq;
byte poscursor = 0; //position cursor common 0 to 15
byte line = 0; // display line in progress on LCD in the course of 0 to 1

// We define this as "uint16_t" so it will always be 16 bits which is what
// EEPROM library routine wants to see. 
uint16_t memory;

unsigned int address=0;
int WEE = 0, WEE_old = 0;
int lcd_key = 0;
int adc_key_in  = 0;
int timer_cursor_blink = 0, timer_for_select = 0; // use to measure the time a key is pressed
int timer_5351_debug = 0;
uint32_t RFint,RFintold,RFcalc,PDRFout;
byte PFDRFout;
double RFout;
double REFin;
uint32_t RFOUT;
int centihertz, centihertzold;

int old_SYS_INIT=-1, old_LOL_A=-1, old_LOL_B=-1, old_LOS=-1, old_REVID=-1;

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


// Values for analog buttons, digital buttons or 4x4 MATRIXED KEYBOARD
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

// ****************************Print variable f64****************************************
void DebugSerialPrintNative64(char *title1, uint64_t value, char *title2){
char buffer [21];

#if DEBUG
     Serial.print(title1); 

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

    Serial.println(title2); 
#endif   
}

// ****************************Print variable int32****************************************
void DebugSerialPrint32(char *title1, uint32_t  an, char *title2){

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
    timer_cursor_blink = 1;
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
          timer_cursor_blink = 1;
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
  { lcd.blink(); timer_cursor_blink = 1; Serial.print ("RT\n");
    return btnRIGHT;
  };

  if  (digitalRead(A1) == LOW)
  { lcd.blink(); timer_cursor_blink = 1; Serial.print ("UP\n");
    return btnUP;
  };

  if  (digitalRead(A2) == LOW)
  { lcd.blink(); timer_cursor_blink = 1; Serial.print ("DN\n");
    return btnDOWN;
  };

  if  (digitalRead(A3) == LOW)
  { lcd.blink(); timer_cursor_blink = 1; Serial.print ("LT\n");
    return btnLEFT;
  };

  if  (digitalRead(A6) == LOW)
  { lcd.blink(); timer_cursor_blink = 1; Serial.print ("SE\n");
    return btnSELECT;
  };

#endif

  return btnNONE;  // no button pushed
}

//***************************** SP Display Frequency on LCD ********************************
void printAll ()
{
  char digit[2] = " ";
  
  lcd.setCursor(0,0); // line 1
  // Do MHz
  RFcalc=RFint;
  if (RFcalc < 100000000) lcd.print(digit);
  if (RFcalc < 10000000)  lcd.print(digit);
  if (RFcalc >= 1000000)
  {
    lcd.print(RFcalc/1000000);
    lcd.print(",");
    strcpy (digit,"0");
  }
  else 
  { 
    lcd.print("  "); 
  } 
  
  // Do kHz
  RFcalc=RFint-((RFint/1000000)*1000000);
  if (RFcalc<100000)lcd.print(digit);
  if (RFcalc<10000)lcd.print(digit);
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
  // Do Hz
  RFcalc=RFint-((RFint/1000)*1000);
  if (RFcalc<100)lcd.print(digit);
  if (RFcalc<10)lcd.print(digit);
  lcd.print(RFcalc);
  
  // Do centihertz
  lcd.print(".");
  if (centihertz<10)lcd.print("0");
  lcd.print(centihertz);
  lcd.print("Hz");

  lcd.setCursor(0,1); // 2nd line
  if (WEE==0) {lcd.print("READ  =");}
  else {lcd.print("WRITE =");}
  if (memory<10){lcd.print(" ");}
  lcd.print(memory,DEC);
  delay(100);
  lcd.print(" REF ");
  lcd.print(PFDRFout,DEC);
  lcd.setCursor(poscursor,line);
}


// Function to write a byte to EEPROM
void writeEEPROM(uint16_t address, byte data)
{
  EEPROM.write(address, data);
}

// Function to write a 32-bit word to EEPROM  
void writeEEPROMlong(uint16_t address,uint32_t value) //address is address within EEPROM
{
  byte four = (value & 0xFF);             //Least significant byte
  byte three = (value >>8 & 0xFF);
  byte two = (value >>16 & 0xFF);
  byte one = (value >>24 & 0xFF);        //Most significant byte      
  
  EEPROM.write(address, four);
  EEPROM.write(address+1, three);
  EEPROM.write(address+2, two);
  EEPROM.write(address+3, one);
}

//Code for reading a byte from external EEPROM
byte readEEPROM(uint16_t address)  
{
  return EEPROM.read(address);
}

//Code for reading 32-bit word from EEPROM
uint32_t readEEPROMlong(uint16_t address) 
{
  uint32_t four = 0x4;
  uint32_t three = 0x3;
  uint32_t two = 0x2;
  uint32_t one = 0x1;

  uint32_t rdata = 0x76543210;

  //Read the 4 bytes from the eeprom memory.
  four = EEPROM.read(address);
  three = EEPROM.read(address + 1);
  two = EEPROM.read(address + 2);
  one = EEPROM.read(address + 3);

  //Returns a long (32bits) using the shift of 0, 8, 16 and 24 bits and masks
  rdata = ((four << 0) & 0xFF) + ((three  << 8) & 0xFFFF) + ((two <<16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
  return rdata;
}

void setCorrection(int32_t calibration = 0)
{
  correction = calibration;
}

//************************************ Setup ****************************************
void setup() {
  
  Serial.begin (9600); //  Serial to the PC via Arduino "Serial Monitor"  at 9600 baud

#if MATRIXED_BUTTONS
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
  
  lcd.print("     Si5351     ");
  lcd.setCursor(0, 1);
  lcd.print("Signal Generator");
  poscursor = 0; line = 0; 
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.print("     W7GLF      ");
  lcd.setCursor(0, 1);
  lcd.print(version);
  delay(2000);

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
     
   if (readEEPROM(MFREQ)==INUSE)
   {  // if a frequency is written in EEPROM
     RFint = readEEPROMlong(0);
     centihertz = readEEPROM(4);
   }
   else 
   {
     RFint=160000000;
     centihertz = 0;
   }
  
  RFintold=0;//Force RF that is different from the initial RF value  
  RFout = RFint;

  WEE=0;  address=0;
  lcd.blink();
  printAll(); 
  delay(500);

  // Initialize the Si5351 to use PFDRFout clock on the XO input
  // init (crystal_load, Ref Freq in MHz, Correction in .01 Hz)
  ref_freq = PFDRFout * 1000000;
  si5351.init(SI5351_CRYSTAL_LOAD_0PF, ref_freq, correction);

  timer_cursor_blink = 1;
  timer_5351_debug = 0;
} // End setup

//*************************************Loop***********************************
void loop()
{
  uint64_t frequency;
    
  RFOUT=RFint;  // RFint in Hz

  if ((RFint != RFintold) || (centihertz != centihertzold) || (WEE != WEE_old) ) 
  {
    frequency = RFint;
    frequency *= 100;
    frequency += centihertz;
 
    DebugSerialPrintNative64 ("frequency is ", frequency, " centiHz");
    // Set CLK0 to output in .01 Hz steps
    si5351.reset();
    si5351.set_freq(frequency, SI5351_CLK0);

    RFintold = RFint;
    centihertzold = centihertz;
    WEE_old = WEE;
    printAll();  // Display LCD
  }

  lcd_key = read_buttons();  // read the buttons

  if (lcd_key <= btnM10)  // Fast recall keys
  {
    memory = lcd_key;
    RFint=readEEPROMlong(memory*MSIZE);   // read frequency from EEPROM and display
    centihertz=readEEPROM(memory*MSIZE+4);   // read frequency from EEPROM and display
    if (RFint>160000000) RFint=160000000;
    if (centihertz > 99) centihertz = 99;
    if (centihertz < 0) centihertz = 0;
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
        if (poscursor == 7 ) poscursor = 8;
        else if (poscursor == 3 ) poscursor = 4;
        else if (poscursor == 11 ) poscursor = 12;
        else if (poscursor == 14 ) 
        {
          poscursor = 0; line = 1; 
        }; 
      }
      if (line == 1) {
        if (poscursor == 1 ) poscursor = 8; //if cursor on the figure memory 
        else if (poscursor == 9 ) poscursor = 15; //if cursor on the figure memory 
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
        else if (poscursor == 11) poscursor = 10;
        else if (poscursor == 7) poscursor = 6;
        else if (poscursor == 3) poscursor = 2; 
      }
      if(line==1){
          if (poscursor==255) {poscursor=13; line=0;}
          else if (poscursor==7) poscursor=0;
          else if (poscursor==14) poscursor=8;
      }
      //Serial.print("back  button");
      lcd.setCursor(poscursor, line);
      break;
      
    case btnUP: //top
      if (line == 0)
      { // RFoutfrequency
        if (poscursor == 0) RFint +=  100000000 ;
        if (poscursor == 1) RFint +=  10000000 ;
        if (poscursor == 2) RFint +=  1000000 ;
        if (poscursor == 4) RFint +=  100000 ;
        if (poscursor == 5) RFint +=  10000 ;
        if (poscursor == 6) RFint +=  1000 ;
        if (poscursor == 8) RFint +=  100 ;
        if (poscursor == 9) RFint +=  10 ;
        if (poscursor == 10) RFint += 1 ;
        if (poscursor == 12) centihertz += 10 ;
        if (poscursor == 13) centihertz += 1 ;
        if (RFint == 160000000) centihertz = 0;
        if (centihertz >= 100) 
        {
          centihertz -= 100;
          RFint += 1;
        }
        if (RFint > 160000000)RFint = RFintold;
      }
      else if (line == 1)
      { 
        if (poscursor == 8)
        { 
          memory++; 
          if (memory==MAXMEM)memory=0;
          if (WEE==0)
          {
            RFint=readEEPROMlong(memory*MSIZE); // read frequency from EEPROM and display
            centihertz=readEEPROM(memory*MSIZE+4);
            if (RFint>160000000) RFint=160000000;
            if (centihertz > 99) centihertz = 99;
            if (centihertz < 0) centihertz = 0;
          }
        }
        if (poscursor==15)
        { 
          if(PFDRFout==10) {PFDRFout=25;} //read REF and swap to 10 MHz if it was 25 and to 25 if it was 10
          else if (PFDRFout==25){PFDRFout=10;}
          else PFDRFout=OnboardOsc;// In the case of PFDRF being different from 10, 25 or 26
          ref_freq = PFDRFout * 1000000;
          si5351.init(SI5351_CRYSTAL_LOAD_0PF, ref_freq, correction);
          // After changing reference frequency we need to reload the frequency
          RFintold = 0;
        }
                    
        if( (poscursor==0) && (WEE==1))
        {
          WEE_old = WEE;
          WEE=0;
        }
        else if ((poscursor==0) && (WEE==0))
        {
          WEE_old = WEE;
          WEE=1; 
        }
      }
      break; // end button up

    case btnDOWN: //down 
      if (line == 0) {
        if (poscursor == 0) RFint -= 100000000 ;
        if (poscursor == 1) RFint -= 10000000 ;
        if (poscursor == 2) RFint -= 1000000 ;
        if (poscursor == 4) RFint -= 100000 ;
        if (poscursor == 5) RFint -= 10000 ;
        if (poscursor == 6) RFint -= 1000 ;
        if (poscursor == 8) RFint -= 100 ;
        if (poscursor == 9) RFint -= 10 ;
        if (poscursor == 10) RFint -= 1 ;
        if (poscursor == 12) centihertz -= 10 ;
        if (poscursor == 13) centihertz -= 1 ;
        if (RFint == 160000000) centihertz = 0;
        if (centihertz < 0) 
        {
          centihertz += 100;
          RFint -= 1;
        }
        if (RFint < 16000) RFint = RFintold;
        if (RFint > 160000000)RFint = RFintold;
      }
      else if (line == 1)
      { 
        if (poscursor == 8)
        {
          memory--; 
          if (memory==0xFFFF)memory=MAXMEM-1;
          if (WEE==0)
          {
            RFint=readEEPROMlong(memory*MSIZE); // read frequency from EEPROM and display
            centihertz=readEEPROM(memory*MSIZE+4); // read frequency from EEPROM and display
            if (RFint>160000000) RFint=160000000;
            if (centihertz > 99) centihertz = 99;
            if (centihertz < 0) centihertz = 0;
          } 
        } // end poscursor = 4 
        if (poscursor==15)
        { 
          if(PFDRFout==10) {PFDRFout=25;} //read REF and swap to 10 MHz if it was 25 and to 25 if it was 10
          else if ( PFDRFout==25){PFDRFout=10;}
          else PFDRFout=OnboardOsc;// In the case or PFDRF different from 10 or 25
          ref_freq = PFDRFout * 1000000;
          si5351.init(SI5351_CRYSTAL_LOAD_0PF, ref_freq, correction);
          // After changing reference frequency we need to reload the frequency
          // Force frequency to be reloaded
          RFintold = 0;
        }
                   
        if( (poscursor==0) && (WEE==1))
        {
          WEE_old = WEE;
          WEE=0;
        }
        else if ((poscursor==0) && (WEE==0))
        {
          WEE_old = WEE;
          WEE=1; 
        }
      }      
      // Serial.print (" DOWN Button  \r\n");
      break; // end button bottom

    case btnSELECT:
      do 
      {
        lcd_key = read_buttons();        // Test release button
        delay(1); timer_for_select++;    // timer inc 1 milliseconds
        if (timer_for_select > 600) 
        { //waiting 600 milliseconds
          if (WEE==1 || poscursor==15)
          { 
            if (line==1 && poscursor==15)
            { 
              writeEEPROM(PDREF,PFDRFout);
              writeEEPROM(MFREF,INUSE);
            } // write REF and flag
            else if (WEE==1) 
            {
              // write RF in EEPROM at address (memory*MSIZE)
              // five bytes per RF value
              writeEEPROMlong(memory*MSIZE,RFint);
              writeEEPROM(memory*MSIZE+4,centihertz);
              writeEEPROM(MFREQ,INUSE);
            }
            lcd.setCursor(0,1); lcd.print("    SAVED    ");
          }
          lcd.setCursor(poscursor,line);
          delay(500);timer_for_select=0;
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
   if (timer_cursor_blink) // Is cursor timer running?
   {
     timer_cursor_blink++; // inc timer
     if (timer_cursor_blink > 1000) // See if it has been over ten seconds
     {
       lcd.noBlink();
       timer_cursor_blink=0;
     } // cursor off
   }

#if DEBUG
   if (timer_5351_debug++ > 1000)
   {
     // Read the Status Register and print it every 10 seconds
     si5351.update_status();
     Serial.print("  SYS_INIT: ");
     Serial.print(si5351.dev_status.SYS_INIT);
     Serial.print("  LOL_A: ");
     Serial.print(si5351.dev_status.LOL_A);
     Serial.print("  LOL_B: ");
     Serial.print(si5351.dev_status.LOL_B);
     Serial.print("  LOS: ");
     Serial.print(si5351.dev_status.LOS);
     Serial.print("  REVID: ");
     Serial.println(si5351.dev_status.REVID);
     timer_5351_debug = 0;
   }
#endif

}   // end loop


