//   ADF4251 and Arduino mike 6 dec 2017 0725
//   By Alain Fort F1CJN feb 2,2016
//   update march 7, 2016 (ROBOT V1.1 and V1.0)
//
//  Minor fixes by Ray Cannon W7GLF
//    Fix occasional loss of cursor movement button
//    Add refresh of LOCKED/NOLOCK since KD7TS has reported problems
//    Add change of BAND SELECT CLOCK DIVIDER between 10 and 25 MHz as suggested by AD software
//    Change buttons to use digital inputs on the analog pins if ANALOG_BUTTONS is false
//
//  This sketch uses an Arduino Nano and ADF4351.
//  The frequency can be programmed between 34.5 and 4400 MHz.
//  Twenty frequencies can be saved in the Arduino EEPROM.
//  If one or more frequencies are saved, then at next power on,
//  the first memory is used.
//
//   The cursor can move with the LEFT and RIGHT buttons. Then the underlined digit can be modified with the UP and DOWN buttons, 
//    for the frequency, the memories and the frequency reference (10 or 25 MHz):
//   - to change the frequency, move the cursor to the digit to be modified, then use the UP and DOWN buttons,
//   - to modify the memory number,move the cursor to the number to be modified, then use the UP and DOWN buttons,
//   - to select the refrence frequence,move the cursor on 10 or 25 and select with UP and DOWN.
//   - to read or write the frequency in memory, place the cursor on the more left/more down position and select REE (for Reading EEprom)
//    or WEE (for Writing EEprom).
//    The cursor dissapears after few seconds and is re activated if a button is pressed.
//
//   MEMORY
//    - For the frequency, select WEE, then select the memory number, then push the SELECT button for a second. The word SAVING 
//    appears on the screen. Frequency saving works when the cursor is anywhere except on the reference 10 or 25 position.
//    - To save the reference frequency, move the cursor to 10 or 25, the press the SELECT for one second. 
//
//  ******************************************** CONNECTIONS *******************************************************
//
// 1602 LCD connections to NANO (not a button board/display)
//
// LCD pin RS to digital pin D8
// LCD pin E  to digital pin D9 (enable)
// LCD pin D4 to digital pin D4
// LCD pin D5 to digital pin D5
// LCD pin D6 to digital pin D6
// LCD pin D7 to digital pin D7
//
// LCD R/W pin to ground
// LCD VSS pin to ground
// LCD VCC pin to 5V
// 10K resistor: this (is a pot)
// ends to +5V and ground
// wiper to LCD VO pin (pin 3)voltage on VO around 9/10ths of a volt when text is visible.
//
//  NANO connections to ADF4351 
//  With any 5 volt unit: use a resistive divider to reduce the voltage between these pins, 
//  MOSI   (pin D11) to ADF4351 DAT 
//  SCK    (pin D13)  to ADF4351 CLK 
//  Select (pin D3 )  to ADF4351 LE 
//
//  D2     (pin D2)  to MUX   around +3.2 when locked, otherwise zero
//
//  Resistive divider 560 Ohms in series with 1000 Ohms to ground on Arduino pins 11, 13 and 3 or
//  resistive divider 1000 Ohms in series with 1500 Ohms to ground on Arduino pins 11, 13 and 3.
//  Connect the junction of the series resistors to MOSI, SCK and Select on ADF4351 respectively.
//
//  Arduino pin D2 (for lock detection) directly connected to ADF4351 card MUXOUT. 4
//
//  FREQUENCY SELECT BUTTONS
// 5 buttons are wired from A0 - A4 to ground through the same 120 ohm resistor. The pins
// are assigned as digital inputs with pull up resistors. 
//*********************************************************************************************************************


#define ANALOG_BUTTONS false

//  DEBUG enables the serial print
#define DEBUG false

// To evaluate buttons set the DEBUG_SETTLE true
#define DEBUG_SETTLE false

// Set desired default REF_FREQ to 10 MHz or 25 MHz
// This is only used if REF FREQ has not been stored in EEPROM
#define DEFAULT_REF_FREQ 25

#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <SPI.h>

#define ADF4351_LE 3 // Pin for Latch Enable on ADF4351

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// EEPROM Layout
#define MAXMEM 20  // Number of memories that can be saved
#define FREF  MAXMEM*5
#define MFREF FREF+5  // 5 bytes for REF
#define MFREQ FREF+6  // 5 bytes for REF
#define INUSE 55  // Special Bit pattern to mark memory is in use


byte poscursor = 0; //position cursor between 0 and 15
byte line = 0; // LCD line 0 or 1
byte memory_number, RWtemp; // memory number for EEPROM
bool locked, nreg_overflow = false;
int pushed;

uint32_t registers[6] =  {0x4580A8, 0x80080C9, 0x4E42, 0x4B3, 0xBC803C, 0x580005} ; // 437 MHz with ref 25 MHz

int address,modif=0,WEE=0,level=3,levelold=-1;
int lcd_key = 0;
int adc_key_in  = 0;
int timer = 0, timer2 = 0; // measure the key press duration
unsigned int i = 0;
unsigned long gcd;  // Only used for INT division to hold loop filter frequency


double RFout, REFin, INT, PFDRFout, OutputChannelSpacing, FRACF;
double RFoutMin = 35, RFoutMax = 4400, REFinMax = 250, PDFMax = 32;
unsigned int long RFint, RFintold, INTA, RFcalc, PDRFout, MOD, FRAC;
byte OutputDivider, lock = 2;
unsigned int long reg0, reg1;

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

//**************************** SP READING BUTTONS ********************************************
int read_buttons()
{

  //****************************  READING BUTTONS ********************************************

#if ANALOG_BUTTONS
  adc_key_in = analogRead(0);      // read the value from the buttons

#if DEBUG || DEBUG_SETTLE
  #if DEBUG_SETTLE // Test button settling time
    int adc_val_sav = adc_key_in;
    for (int i = 0; i < 50; i++)
    {
      Serial.print ("Read #");
      Serial.print (i);
  #endif
      Serial.print (" Button value = ");
      Serial.print ( (5.0 / 1024) * adc_key_in);
      Serial.print (", adc_key_in = ");
      Serial.print (adc_key_in);
      Serial.print ("\n");
#if DEBUG_SETTLE
      adc_key_in = 0;
      if (adc_key_in < adc_val_sav) adc_val_sav = adc_key_in;
  }
  adc_key_in = adc_val_sav;
  Serial.print ("-----------------------\n");
#endif
  Serial.flush();
#endif

  if (adc_key_in < 790) {
    lcd.blink();
    timer = 1;

    pushed = HIGH;
    if (adc_key_in < 50)return btnRIGHT;  // for ROBOT display V1.0
    if (adc_key_in < 195)return btnUP;
    if (adc_key_in < 380)return btnDOWN;
    if (adc_key_in < 555)return btnLEFT;
    if (adc_key_in < 790)return btnSELECT; // end display ROBOT1.1
  }
#else
  if  (digitalRead(A0) == LOW )
  { lcd.blink(); timer = 1; Serial.print ("RT"); Serial.print ("\n");
    pushed = HIGH; return btnRIGHT;
  };

  if  (digitalRead(A1) == LOW)
  { lcd.blink(); timer = 1; Serial.print ("UP"); Serial.print ("\n");
    pushed = HIGH; return btnUP;
  };

  if  (digitalRead(A2) == LOW)
  { lcd.blink(); timer = 1; Serial.print ("DN"); Serial.print ("\n");
    pushed = HIGH; return btnDOWN;
  };

  if  (digitalRead(A3) == LOW)
  { lcd.blink(); timer = 1; Serial.print ("LT"); Serial.print ("\n");
    pushed = HIGH; return btnLEFT;
  };

  if  (digitalRead(A4) == LOW)
  { lcd.blink(); timer = 1; Serial.print ("SE"); Serial.print ("\n");
    pushed = HIGH; return btnSELECT;
  };
#endif

  pushed = LOW;
  return btnNONE;                                                 //  unpressed key
}

//***************************** SP display frequency on LCD ********************************
void printAll ()
{
  //RFout=1001.10 // test
  lcd.setCursor(0, 0);
  lcd.print("RF = ");
  if (RFint < 100000) lcd.print(" ");
  if (RFint < 10000)  lcd.print(" ");
  lcd.print(RFint / 100); lcd.print(".");
  RFcalc = RFint - ((RFint / 100) * 100);
  if (RFcalc < 10)lcd.print("0");
  lcd.print(RFcalc);
  lcd.print("M L");
  lcd.print(level);
  Serial.print ("Level = ");
  Serial.print (level);
  Serial.print ("\r\n");
  lcd.setCursor(0, 1);
  if (WEE == 0) {
    lcd.print("REE=");
  }
  else {
    lcd.print("WEE=");
  }
  if (memory_number < 10)lcd.print(" ");
  lcd.print(memory_number, DEC);
  locked = (digitalRead(2) == 1);
  if  (locked) lcd.print(" LOCKED ");
  else lcd.print(" NOLOCK ");
  lcd.print(PFDRFout, DEC);
  lcd.setCursor(poscursor, line);
}

//***************************** SP display LOCK/NOLOCK on LCD ********************************
void printLocked ()
{
  lcd.setCursor(6, 1);
  locked = (digitalRead(2) == 1);
  if  (locked) lcd.print(" LOCKED ");
  else lcd.print(" NOLOCK ");
  lcd.setCursor(poscursor, line);
}

void WriteRegister32(const uint32_t value)   //program a 32 bit register
{
  digitalWrite(ADF4351_LE, LOW);
  for (int i = 3; i >= 0; i--)          // loop on 4 x 8 bits
    SPI.transfer((value >> 8 * i) & 0xFF); // offset, byte masking and sending via SPI
  digitalWrite(ADF4351_LE, HIGH);
  digitalWrite(ADF4351_LE, LOW);
}

void SetADF4351()  // Program all the registers of the ADF4351
{ for (int i = 5; i >= 0; i--)  // programming ADF4351 starting with R5
    WriteRegister32(registers[i]);
}

// *************** SP writing Long word (32 bits) in EEPROM between address and address + 3 **************
void EEPROMWritelong(int address, long value)
{
  //Decomposition of the long (32 bits) into 4 bytes
  //trois = MSB -> quatre = lsb
  byte quatre = (value & 0xFF);
  byte trois = ((value >> 8) & 0xFF);
  byte deux = ((value >> 16) & 0xFF);
  byte un = ((value >> 24) & 0xFF);


  //Writes 4 bytes to the EEPROM memory
  EEPROM.write(address, quatre);
  EEPROM.write(address + 1, trois);
  EEPROM.write(address + 2, deux);
  EEPROM.write(address + 3, un);
}

// *************** SP reading Long word (32 bits) in EEPROM between address and address + 3 **************
long EEPROMReadlong(long address)
{
  //Read the 4 bytes from the eeprom memory.
  long quatre = EEPROM.read(address);
  long trois = EEPROM.read(address + 1);
  long deux = EEPROM.read(address + 2);
  long un = EEPROM.read(address + 3);

  //Returns a long (32bits) using the shift of 0, 8, 16 and 24 bits and masks
  return ((quatre << 0) & 0xFF) + ((trois << 8) & 0xFFFF) + ((deux << 16) & 0xFFFFFF) + ((un << 24) & 0xFFFFFFFF);
}

void CalculateDivider ()
{
    if (RFout >= 2200) 
    {
      OutputDivider = 1;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
    }
    if (RFout < 2200)
    {
      OutputDivider = 2;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
    }
    if (RFout < 1100) 
    {
      OutputDivider = 4;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
    }
    if (RFout < 550)  
    {
      OutputDivider = 8;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 1);
    }
    if (RFout < 275)  
    {
      OutputDivider = 16;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
    }
    if (RFout < 137.5) 
    {
      OutputDivider = 32;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
    }
    if (RFout < 68.75) 
    {
      OutputDivider = 64;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
    }     
}

void FrequencyOrLevelHasChanged()
{
    CalculateDivider ();

    if (level != levelold)
    {  // Mask out level field and fill in with current value
      registers [4] &= 0XFFFFFFE7;
      registers [4] |= ((level & 3) << 3);
      levelold = level;
    } 

    INTA = (RFout * OutputDivider) / PFDRFout;
    MOD = (PFDRFout / OutputChannelSpacing);
    FRACF = (((RFout * OutputDivider) / PFDRFout) - INTA) * MOD;
    FRAC = round(FRACF); // The result is rounded

    registers[0] = 0;
    registers[0] = INTA << 15; // OK
    FRAC = FRAC << 3;
    registers[0] = registers[0] + FRAC;

    registers[1] = 0;
    registers[1] = MOD << 3;
    registers[1] = registers[1] + 1 ; // addition of the address "001"
    bitSet (registers[1], 27); // Prescaler on 8/9

    bitSet (registers[2], 28); // Digital lock == "110" on b28 b27 b26
    bitSet (registers[2], 27); // digital lock 
    bitClear (registers[2], 26); // digital lock
   
    SetADF4351();  // Program all the registers of the ADF4351
}

//************************************ Setup ****************************************
void setup() {
  lcd.begin(16, 2); // two 16 characters lines
  lcd.display();
  analogWrite(10, 255); //LCD Brightness

  Serial.begin (9600); //  Serial to the PC via Arduino "Serial Monitor"  at 9600
  lcd.print("   GENERATOR   ");
  lcd.setCursor(0, 1);
  lcd.print("    ADF4351     ");
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.print("    by F1CJN    ");
  lcd.setCursor(0, 1); // <=======================
  lcd.print(" 01/29/18 23:03 "); // <=======16 characters=============
  delay(3000);
  poscursor = 7; line = 0;

  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(2, INPUT);  // PIN 2 in entry for lock
  pinMode(ADF4351_LE, OUTPUT);          // Setup pins
  digitalWrite(ADF4351_LE, HIGH);
  SPI.begin();                          // Init SPI bus
  SPI.setDataMode(SPI_MODE0);           // CPHA = 0 and positive Clock
  SPI.setBitOrder(MSBFIRST);            // Big Endian

  if (EEPROM.read(MFREF) == INUSE) // if the ref is written in EEPROM, it is read
  {
    PFDRFout = EEPROM.read(FREF);
  } else {
    // Default ref
    PFDRFout = DEFAULT_REF_FREQ;
  }

  if (EEPROM.read(MFREQ) == INUSE) // if a frequency is written in EEPROM it is read
  {
    RFint = EEPROMReadlong(0);
    level=EEPROM.read(4);    
  } else {
    RFint = 7000;
  }

  RFintold = 1234; //for RFintold to be different from RFout when init
  RFout = RFint / 100 ; // output frequency
  OutputChannelSpacing = 0.01; // Frequency step = 10kHz

  WEE = 0;  address = 0;
  lcd.noBlink();
  printAll(); delay(500);
   
} // End setup

//*************************************Loop***********************************
void loop()
{
  unsigned long nreg, rreg, oldgcd;
  unsigned long freqin, pdfreqin;

  oldgcd = 0;
  RFout = RFint;
  RFout = RFout / 100;
  if ((RFint != RFintold) || (level != levelold) || (modif==1)) 
  {
    FrequencyOrLevelHasChanged();
    RFintold = RFint;
    modif = 0;
    printAll();  // LCD display
  }

  lcd_key = read_buttons();  // read the buttons

  switch (lcd_key)               // Select action
  {
    //***********************************************************case rt
    case btnRIGHT: //right
      poscursor++; // cursor to the right
      if (line == 0) // Cursor on first line
      {
        if (poscursor == 9 )
        {
          // Skip over decimal point
          poscursor = 10;
        } else if (poscursor == 12 )
        {
          // Skip to level
          poscursor = 15;
        } else if (poscursor == 16 ) 
        {
          // Cursor at end of first line - go to second line
          poscursor = 0; line = 1;
        }
      }
      if (line == 1) // Cursor on second line
      {
        if (poscursor == 1 ) {
          poscursor = 5;  // Put cursor on the memory number
        }
        else if (poscursor == 6 ) {
          poscursor = 15;  // Put cursor on the ref freq
        }
        else if (poscursor == 16) {
          poscursor = 5;
          line = 0;
        };
      }
      //    Serial.print (" RIGHT Button\r\n");  //COMMENT
      lcd.setCursor(poscursor, line);
      break;

    //************************************************************case left

    case btnLEFT: //Gauche
      poscursor--; // Backup cursor
      if (line == 0)
      {
        if (poscursor == 4) 
        {
          poscursor = 15; 
          line = 1;
        } 
        else if (poscursor == 14) 
        {   
          poscursor = 11;
        } 
        else if (poscursor == 9) 
        {
          // Skip over decimal point
          poscursor = 8;
        }
      }
      if (line == 1)
      {
        if (poscursor == 255) {
          poscursor = 15;
          line = 0;
        };
        if (poscursor == 4) {
          poscursor = 0;
        };
        if (poscursor == 14) {
          poscursor = 5;
        };
      }
      //Serial.print(poscursor,DEC);
      lcd.setCursor(poscursor, line);
      break;

    //******************************************************case up

    case btnUP: //Haut
      if (line == 0)
      { // RFoutfrequency in multiple of 10KHz
        //Serial.print(oldRFint,DEC);
        if (poscursor == 5) RFint = RFint + 100000 ;
        if (poscursor == 6) RFint = RFint + 10000 ;
        if (poscursor == 7) RFint = RFint + 1000 ;
        if (poscursor == 8) RFint = RFint + 100 ;
        if (poscursor == 10) RFint = RFint + 10 ;
        if (poscursor == 11) RFint = RFint + 1 ;
        if (poscursor == 15 && level < 3) ++level ;
        if (RFint > 440000) RFint = RFintold;
        //Serial.print(RFint,DEC);
        //Serial.print("  \r\n");
      }
      if (line == 1)
      {
        if (poscursor == 5)
        {
          memory_number++;
          if (memory_number == 20)memory_number = 0;
          if (WEE == 0)
          {
            RFint=EEPROMReadlong(memory_number*5); // Reading EEPROM and Display
            level=EEPROM.read(memory_number*5+4);
            if (level >3) level=levelold;  // Memory has not been stored yet
            if (RFint > 440000) RFint = 440000;
          }
        }
        if (poscursor == 15)
        {
          if ( PFDRFout == 10)
          {
            PFDRFout = 25;
          } else if ( PFDRFout == 25)
          {
            PFDRFout = 10;
          }
          else
          {
            PFDRFout = 25; // in the event that PFDRF differs from 10 and 25
          }
          // Calculate BAND SELECT CLOCK DIVIDER as per ADF4351 documentation
          registers[4] &= 0xFFF00FFF;
          registers[4] |= ((uint32_t) (8 * PFDRFout)) << 12;  // Shift left 12 bits
          modif = 1;
        }

        if ( (poscursor == 0) && (WEE == 1))WEE = 0;
        else if ((poscursor == 0) && (WEE == 0))WEE = 1;
      }
      printAll();
      break; // end button up

    //*********************************************************case down

    case btnDOWN: //low
      if (line == 0) {
        if (poscursor == 5) RFint = RFint - 100000 ;
        if (poscursor == 6) RFint = RFint - 10000 ;
        if (poscursor == 7) RFint = RFint - 1000 ;
        if (poscursor == 8) RFint = RFint - 100 ;
        if (poscursor == 10) RFint = RFint - 10 ;
        if (poscursor == 11) RFint = RFint - 1 ;
        if (poscursor == 15 && level > 0) --level ;
        if (RFint < 3450) RFint = RFintold;
        if (RFint > 440000)  RFint = RFintold;
      }
      else if (line == 1)
      {
        if (poscursor == 5)
        {
          memory_number--;
          if (memory_number == 255)memory_number = 19;
          if (WEE == 0)
          {
            RFint=EEPROMReadlong(memory_number*5); // Reading EEPROM and Display
            level=EEPROM.read(memory_number*5+4);
            if (level >3) level=levelold;  // Memory has not been stored yet
            if (RFint > 440000) RFint = 440000;
            // Serial.print(RFint,DEC);
          }
        } // end poscursor = 5

        if (poscursor == 15)
        {
          // Toggle REF setting between 10 and 25
          if ( PFDRFout == 10)
          {
            PFDRFout = 25;
          } else if ( PFDRFout == 25)
          {
            PFDRFout = 10;
          }
          else
          {
            PFDRFout = 25; // in the event that PFDRF differs from 10 and 25
          }
          // Calculate BAND SELECT CLOCK DIVIDER as per ADF4351 documentation
          registers[4] &= 0xFFF00FFF;
          registers[4] |= ((uint32_t) (8 * PFDRFout)) << 12;  // Shift left 12 bits
          modif = 1;
        } // end poscursor = 15

        if ( (poscursor == 0) && (WEE == 1))WEE = 0;
        else if ((poscursor == 0) && (WEE == 0))WEE = 1;
      }
      printAll();
      Serial.print (" DOWN Button  \r\n"); //COMMENT
      break; // end low button

    //***********************************************************case select

    case btnSELECT:
      do
      {
        read_buttons();
        delay(1); timer2++;        // timer in milliseconds
        if (timer2 > 600) 
        { //wait 600 milliseconds
          if (WEE==1 || (line==1 && poscursor==15))
          { 
            if (line==1 && poscursor==15)
            { // write refence frequency 
              EEPROM.write(FREF,PFDRFout);
              EEPROM.write(MFREF,INUSE);
            }
            else if (WEE == 1)
            { // write frequency and level (5 bytes) to EEPROM
              EEPROMWritelong(memory_number*5,RFint); // offset 0-3
              EEPROM.write(memory_number*5+4,level);  // offset 4
              if (memory_number==0) EEPROM.write(MFREQ,INUSE);
            }
            lcd.setCursor(0, 1); lcd.print("  MEMORIZATION  ");
          }
          lcd.setCursor(poscursor, line);
          delay(500);
          printAll();
          break;  // exit do while loop
        }; // measure
      } while (pushed == HIGH);
      timer2 = 0;
      break;  // End Select button

      //************************************************************** case NONE***********************************

      case btnNONE: {
          break;
        };
        break;
   }                       // End LCD keys

   //**************************************************guessing this is to make cursor quit blinking**************

   while (pushed == HIGH) //loop until pushed is false which means no button was pressed
   // "pushed" is set when a button is pushed so you never come here with a value of low
   // rather if pushed is low it goes to one of the other cases 
   {
      delay(1);
      read_buttons();
   } 

   // When we get here no button is pressed
   delay (10);
   if (timer) // Is cursor timer running?
   {
      timer++; // inc timer
      if (timer > 1000) 
      {
          lcd.noBlink();  // After 10 sec turn cursor off
          timer = 0;
      }
   }

   // KD7TS complained about LOCK on display not being correct so double check here
   if (locked != (digitalRead(2) == 1)) printLocked();
 
  // End loop()
}

