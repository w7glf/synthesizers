//   ADF4251 and Arduino
//   By Alain Fort F1CJN feb 2,2016
//   update march 7, 2016 (ROBOT V1.1 and V1.0)
//
//  Minor fixes by Ray Cannon W7GLF - Ver 6
//    Fix occasional loss of cursor movement button
//    Add refresh of LOCKED/NOLOCK since KD7TS has reported problems
//    Add change of BAND SELECT CLOCK DIVIDER between 10 and 25 MHz as suggested by AD software
//
//  INTDIV below was used to test the use of INT n division rather than FRAC n division.  
//  It would be interesting to compare the phase noise using each method.  I have determined INT n 
//  division does not work for all choices (for example 432.01) because N register is only 16 bits.
//  When that happens I fall back to FRACT mode and display LOCKED.  
//
#define INTDIV  false

//  DEBUG enables the serial print
#define DEBUG false

// To evaluate buttons set the DEBUG_SETTLE true
#define DEBUG_SETTLE false

// Set desired default REF_FREQ to 10 MHz or 25 MHz
// This is only used if REF FREQ has not been stored in EEPROM
#define DEFAULT_REF_FREQ 25

//
//  This sketch uses and Arduino Uno (5€), a standard "LCD buttons shield" from ROBOT (5€), with buttons and an ADF4351 chinese
//  card found at EBAY (40€). The frequency can be programmed between 34.5 and 4400 MHz.
//  Twenty frequencies can be memorized into the Arduino EEPROM.
//  If one or more frequencies are memorized, then at power on, the memory zero is always selected.
//
//   The cursor can move with le LEFT and RIGHT buttons. Then the underlined digit can be modified with the UP and DOWN buttons, 
//    for the frequency, the memories and the frequency reference (10 or 25 MHz):
//   - to change the frequency, move the cursor to the digit to be modified, then use the UP and DOWN buttons,
//   - to modify the memory number,move the cursor to the number to be modified, then use the UP and DOWN buttons,
//   - to select the refrence frequence,move the cursor on 10 or 25 and select with UP and DOWN.
//   - to read or write the frequency in memory, place the cursor on the more left/more down position and select REE (for Reading EEprom)
//    or WEE (for Writing EEprom).
//    The cursor dissapears after few seconds and is re activated if a button is pressed.
//
//   MEMORIZATION 
//    - For the frequency, select WEE, then select the memory number, then push the SELECT button for a second. The word MEMORISATION 
//    appears on the screen. This memorization works then the cursor is anywhere except on the reference 10 or 25 position.
//    - For the reference frequency, move the cursor to 10 or 25, the press the SELECT for one second. 

//  ******************************************** HARDWARE IMPORTANT********************************************************
//  With an Arduino UN0 : uses a resistive divider to reduce the voltage, MOSI (pin 11) to
//  ADF DATA, SCK (pin13) to ADF CLK, Select (PIN 3) to ADF LE
//  Resistive divider 560 Ohm with 1000 Ohm to ground on Arduino pins 11, 13 et 3 to adapt from 5V
//  to 3.3V the digital signals DATA, CLK and LE send by the Arduino.
//  Arduino pin 2 (for lock detection) directly connected to ADF4351 card MUXOUT.
//  The ADF card is 5V powered by the ARDUINO (PINs +5V and GND are closed to the Arduino LED).

//************************************************* MANUEL*****************************************************************
//Touche LEFT    curseur à gauche, cursor to the left
//Touche RIGHT   curseur à droite, cursor to the right
//Touche UP      incremente frequence ou memory_number, increase frequency
//Touche DOWN    decremente frequence ou memory_number, decrease frequency
//Touche SELECT  long push = frequency memorization into the EE number EEPROM / or reference memorization
//*************************************************************************************************************************
// Warning : if you are using a ROBOT Shield version 1.1, it is necessary to modify the read_lcd_buttons sub routine 
// you need not to comment the 1.1 version and to comment the 1.0 version. See below

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
bool locked, nreg_overflow=false;

unsigned long nreg, rreg, oldgcd;
unsigned long freqin, pdfreqin;

uint32_t registers[6] =  {0x4580A8, 0x80080C9, 0x4E42, 0x4B3, 0xBC803C, 0x580005} ; // 437 MHz with ref 25 MHz

int address,modif=0,WEE=0,level=3,levelold=-1;
int lcd_key = 0;
int adc_key_in  = 0;
int timer = 0,timer2=0; // measure the key press duration
unsigned int i = 0;
unsigned long gcd;  // Only used for INT division to hold loop filter frequency


double RFout, REFin, INT, PFDRFout, OutputChannelSpacing, FRACF;
double RFoutMin = 35, RFoutMax = 4400, REFinMax = 250, PDFMax = 32;
unsigned int long RFint,RFintold,INTA,RFcalc,PDRFout, MOD, FRAC;
byte OutputDivider, lock=2;
unsigned int long reg0, reg1;

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

//**************************** SP READING BUTTONS ********************************************
int read_LCD_buttons()
{
  adc_key_in = analogRead(0);      // read the value from the buttons

  if (adc_key_in < 790) {
    lcd.blink();
    
    // Code for Mike - his buttons take a long time to settle
    // delay (10);
    // adc_key_in = analogRead(0);
    
#if DEBUG || DEBUG_SETTLE

#if DEBUG_SETTLE // Test button settling time
    int adc_val_sav = adc_key_in;
    for (int i=0; i < 50; i++)
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
      adc_key_in = analogRead(0);
      if (adc_key_in < adc_val_sav) adc_val_sav = adc_key_in;
    }
    adc_key_in = adc_val_sav;
    Serial.print ("-----------------------\n");
#endif
    Serial.flush();
#endif
  
    if (adc_key_in < 50)return btnRIGHT;  // for ROBOT display V1.0
    if (adc_key_in < 195)return btnUP;
    if (adc_key_in < 380)return btnDOWN;
    if (adc_key_in < 555)return btnLEFT;
    if (adc_key_in < 790)return btnSELECT; // end display ROBOT1.1
  }

  return btnNONE;  // no key pressed
}

#if INTDIV
void printPDF()
{
  if (nreg_overflow)
  {
     lcd.print(" LOCKED ");
     return;
  }
  // Print out PDF
  if (gcd < 10) {
     lcd.print("    ");
     lcd.print(gcd);
     lcd.print("Hz ");
  } else if (gcd < 100) {
     lcd.print("   ");
     lcd.print(gcd);
     lcd.print("Hz ");
  } else if (gcd < 1000) {
     lcd.print("  ");
     lcd.print(gcd);
     lcd.print("Hz ");
  } else if (gcd < 10000) {
     lcd.print("   ");
     lcd.print(gcd/1000);
     lcd.print("KHz ");
  } else if (gcd < 100000) {
     lcd.print("  ");
     lcd.print(gcd/1000);
     lcd.print("KHz ");
  } else if (gcd < 1000000) {
     lcd.print(" ");
     lcd.print(gcd/1000);
     lcd.print("KHz ");
  } else if (gcd < 10000000) {
     lcd.print("   ");
     lcd.print(gcd/1000000);
     lcd.print("MHz ");
  } else if (gcd < 100000000) {
     lcd.print("  ");
     lcd.print(gcd/1000000);
     lcd.print("MHz ");
  } else {
     // Bogus value
     lcd.print(" ?????? ");
  }          
}
#endif

//***************************** SP display frequency on LCD ********************************
void printAll ()
{
  //RFout=1001.10 // test
  lcd.setCursor(0, 0);
  lcd.print("RF = ");
  if (RFint < 100000) lcd.print(" ");
  if (RFint < 10000)  lcd.print(" ");
  lcd.print(RFint/100);lcd.print(".");
  RFcalc=RFint-((RFint/100)*100);
  if (RFcalc<10)lcd.print("0");
  lcd.print(RFcalc);
  lcd.print("M L");
  lcd.print(level);
  lcd.setCursor(0,1);
  if (WEE==0) {lcd.print("REE=");}
  else {lcd.print("WEE=");}
  if (memory_number<10)lcd.print(" ");
  lcd.print(memory_number,DEC);
  locked = (digitalRead(2)==1);
#if INTDIV
// Print out PDF
  if  (locked) {
    printPDF();
  }
  else {
    lcd.print(" NOLOCK ");
  }
#else
  if  (locked) lcd.print(" LOCKED ");
     else lcd.print(" NOLOCK ");
#endif
  lcd.print(PFDRFout,DEC);
  lcd.setCursor(poscursor,line);
}

//***************************** SP display LOCK/NOLOCK on LCD ********************************
void printLocked ()
{
  lcd.setCursor(6,1);
  locked = (digitalRead(2)==1);
#if INTDIV
// Print out PDF
  if  (locked) {
    printPDF();          
  }
  else {
    lcd.print(" NOLOCK ");
  }
#else
  if  (locked) lcd.print(" LOCKED ");
     else lcd.print(" NOLOCK ");
#endif
  lcd.setCursor(poscursor,line);
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
{ for (int i = 5; i >= 0; i--)
  {  // programming ADF4351 starting with R5
    WriteRegister32(registers[i]);
//#if DEBUG
    Serial.print ("registers [");   
    Serial.print (i);   
    Serial.print ("] = ");   
    Serial.print (registers [i], HEX);
    Serial.print ("\r\n");   
//#endif       
  }
}

// *************** SP writing Long word (32 bits) in EEPROM between address and address + 3 **************
void EEPROMWritelong(int address, long value)
      {
      //Decomposition of the long (32 bits) into 4 bytes
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
      return ((quatre << 0) & 0xFF) + 
         ((trois << 8) & 0xFFFF) + 
         ((deux << 16) & 0xFFFFFF) + 
         ((un << 24) & 0xFFFFFFFF);
      }

#if INTDIV
// Calculate Greatest Common Divisor of desired freq and ref which gives us the
// highest Phase Detection Frequency we can use for integer divisor.
// Credit for algorithm goes to Euclid.
long GCD(unsigned long freq, unsigned long ref )
{
  while (freq != ref)
  {
    if (freq > ref)
    {
      freq -= ref;
    }
    else
    {
      ref -= freq;
    }
  }
  return freq;
}
#endif

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
    {  // fill in with current value
      registers [4] &= 0XFFFFFFE7;
      registers [4] |= ((level & 3) << 3);
    } 
    
#if INTDIV
    // User INTEGER Division
    // Calculate best integer Phase Detector Frequency
    freqin = 10000 * RFint * OutputDivider;
    pdfreqin  = 1000000 * PFDRFout;
    gcd = GCD(freqin, pdfreqin );
    nreg = freqin / gcd;
    rreg = pdfreqin / gcd;

  #if DEBUG    
    if (gcd != oldgcd) 
    {
      Serial.print ("GCD of ");
      Serial.print (freqin);
      Serial.print (" and ");
      Serial.print (pdfreqin);
      Serial.print (" is ");
      Serial.print (gcd);
      Serial.print (", n = ");
      Serial.print (nreg);
      Serial.print (", r = ");
      Serial.print (rreg);
      Serial.print ("\n");
    }
  #endif 

    if (nreg > 65535) 
    {
  #if DEBUG
      Serial.print ("N too large for INT mode so using FRACT mode\n");
  #endif
      nreg_overflow = true;
      // We cannot use INT so go back to FRACT
      INTA = (RFout * OutputDivider) / PFDRFout;
      MOD = (PFDRFout / OutputChannelSpacing);
      if (MOD < 2) MOD = 2;
      FRACF = (((RFout * OutputDivider) / PFDRFout) - INTA) * MOD;
      FRAC = round(FRACF); // The result is rounded
      registers[2] = 0x4E42;
    }
    else
    {
      // No Overflow
      nreg_overflow = false;
      INTA = nreg;
      MOD = 2;
      FRAC = 0; // The result is rounded
      registers[2] = (( rreg & 0x3ff ) << 14) | 0xF42;  // (0x100) set means use INT Division
    } 
#else //  FRACT DIVISION
    INTA = (RFout * OutputDivider) / PFDRFout;
    MOD = (PFDRFout / OutputChannelSpacing);
    if (MOD < 2) MOD = 2;
    FRACF = (((RFout * OutputDivider) / PFDRFout) - INTA) * MOD;
    FRAC = round(FRACF); // The result is rounded
#endif // #if INTDIV

    registers[0] = 0;
    registers[0] = INTA << 15; // OK
    FRAC = FRAC << 3;
    registers[0] = registers[0] + FRAC;

    registers[1] = 0;
    registers[1] = MOD << 3;
    registers[1] = registers[1] + 1 ; // addition of the address "001"
    bitSet (registers[1], 27); // Prescaler on 8/9
    bitSet (registers[1], 15); // Phase to 1

    bitSet (registers[2], 28); // Digital lock == "110" on b28 b27 b26
    bitSet (registers[2], 27); // digital lock 
    bitClear (registers[2], 26); // digital lock
   
    SetADF4351();  // Program all the registers of the ADF4351
}
      
//************************************ Setup ****************************************
void setup() {
  lcd.begin(16, 2); // two 16 characters lines
  lcd.display();
  analogWrite(10,255); //LCD Brightness

  Serial.begin (9600); //  Serial to the PC via Arduino "Serial Monitor"  at 9600
  lcd.print("   GENERATOR   ");
  lcd.setCursor(0, 1);
  lcd.print("  ADF4351 ver6  ");
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.print("F1CJN revs W7GLF");
  delay(1000);
  poscursor = 5; line = 0; 

  pinMode(2, INPUT);  // PIN 2 in entry for lock
  pinMode(ADF4351_LE, OUTPUT);          // Setup pins
  digitalWrite(ADF4351_LE, HIGH);
  SPI.begin();                          // Init SPI bus
  SPI.setDataMode(SPI_MODE0);           // CPHA = 0 and positive Clock
  SPI.setBitOrder(MSBFIRST);            // Big Endian

  if (EEPROM.read(MFREF)==INUSE) // if the ref is written in EEPROM, it is read
  {
    PFDRFout=EEPROM.read(FREF);
  } else {
    // Default ref
    PFDRFout=DEFAULT_REF_FREQ;
  }

  if (EEPROM.read(MFREQ)==INUSE) // if a frequency is written in EEPROM 0 it is read
  {
     RFint=EEPROMReadlong(0);
     level=EEPROM.read(4);
  } else {
    RFint=7000;
  }
  
  RFintold=0;//for RFintold to be different from RFout when init
  RFout = RFint/100 ; // output frequency
  OutputChannelSpacing = 0.01; // Frequency step = 10kHz

  WEE=0;  address=0;
  lcd.noBlink();
  printAll(); delay(500);


} // End setup

//*************************************Loop***********************************
void loop()
{
  oldgcd = 0;
  RFout=RFint;
  RFout=RFout/100;
  if ((RFint != RFintold) || (level != levelold) || (modif==1)) 
  {
    FrequencyOrLevelHasChanged();
    RFintold=RFint;
    levelold=level;
    modif=0;
    printAll();  // LCD display
  }

  lcd_key = read_LCD_buttons();  // read the buttons

  switch (lcd_key)               // Select action
  {
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
        if (poscursor == 1 ) {poscursor = 5; line = 1; } // Put cursor on the memory number 
        else if (poscursor == 6 ) {poscursor = 15; line = 1; } // Put cursor on the ref freq 
        else if (poscursor==16) {poscursor=5; line=0;};     
      }  
      // Serial.print (" RIGHT Button\r\n");
      lcd.setCursor(poscursor, line);
      break;
      
    case btnLEFT: //Gauche
      poscursor--; // Backup cursor
      if (line == 0) {
        if (poscursor == 4) 
        {
          poscursor = 15; line = 1;
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
       if(line==1){
          if (poscursor==255) 
          {
            poscursor=15; line=0;
          } 
          else if (poscursor==4) 
            poscursor=0;
          else if (poscursor==14) 
            poscursor=5;
      }
      //Serial.print(poscursor,DEC);  
      lcd.setCursor(poscursor, line);
      break;
      
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
          if (memory_number==20)memory_number=0;
          if (WEE==0)
          {
            RFint=EEPROMReadlong(memory_number*5); // Reading EEPROM and Display
            level=EEPROM.read(memory_number*5+4);
            if (level >3) level=levelold;  // Memory has not been stored yet
            if (RFint>440000) RFint=440000; 
          }
        }  
        if (poscursor==15)
        { 
          if( PFDRFout==10)
          {
            PFDRFout=25;
          } else if ( PFDRFout==25)
          {
            PFDRFout=10;
          }
          else 
          {
            PFDRFout=25;// in the event that PFDRF differs from 10 and 25
          }
          // Calculate BAND SELECT CLOCK DIVIDER as per ADF4351 documentation
          registers[4] &= 0xFFF00FFF;
          registers[4] |= ((uint32_t) (8 * PFDRFout)) << 12;  // Shift left 12 bits
          modif=1;  
        }
                    
        if( (poscursor==0) && (WEE==1))WEE=0;
        else if ((poscursor==0) && (WEE==0))WEE=1;                  
      }
      printAll();
      // Serial.print (" UP Button  \r\n");
      break; // end button up

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
          if (memory_number==255)memory_number=19;
          if (WEE==0)
          {
            RFint=EEPROMReadlong(memory_number*5); // Reading EEPROM and Display
            level=EEPROM.read(memory_number*5+4);
            if (level >3) level=levelold;  // Memory has not been stored yet
            if (RFint>440000) RFint=440000;
            // Serial.print(RFint,DEC);  
          } 
        } // end poscursor = 5 

        if (poscursor==15)
        {  
          // Toggle REF setting between 10 and 25
          if( PFDRFout==10) 
          {
             PFDRFout=25;
          } else if ( PFDRFout==25) 
          {
             PFDRFout=10;
          } 
          else 
          {
             PFDRFout=25;// in the event that PFDRF differs from 10 and 25
          }
          // Calculate BAND SELECT CLOCK DIVIDER as per ADF4351 documentation
          registers[4] &= 0xFFF00FFF;
          registers[4] |= ((uint32_t) (8 * PFDRFout)) << 12;  // Shift left 12 bits
          modif=1;
        } // end poscursor = 15
                   
        if( (poscursor==0) && (WEE==1))WEE=0;
        else if ((poscursor==0)&&(WEE==0))WEE=1;                          
      }
      printAll();
      // Serial.print (" DOWN Button  \r\n");
      break; // end low button

    case btnSELECT:
      do 
      {
        adc_key_in = analogRead(0);      // Test release button
        delay(1); timer2++;        // timer in milliseconds
        if (timer2 > 600) { //wait 600 milliseconds
          if (WEE==1 || (line==1 && poscursor==15))
          { 
            if (line==1 && poscursor==15)
            { // write refence frequency 
              EEPROM.write(FREF,PFDRFout);
              EEPROM.write(MFREF,INUSE);
            }
            else if (WEE==1) 
            { // write frequency and level (5 bytes) to EEPROM
              EEPROMWritelong(memory_number*5,RFint); // offset 0-3
              EEPROM.write(memory_number*5+4,level);  // offset 4
              if (memory_number==0) EEPROM.write(MFREQ,INUSE);
            }
            lcd.setCursor(0,1); lcd.print("  MEMORIZATION  ");
          }
          lcd.setCursor(poscursor,line);
          delay(500);timer2=0;
          printAll();
        }; // measure
      } while (adc_key_in < 900); // wait for SELECT button release
      break;  // End Select button

     case btnNONE: {
        break;
      };
      break;
  }// End LCD keys

  if (lcd_key != btnNONE) // A button was pressed
  {
    do { 
      adc_key_in = analogRead(0);
      delay(1);
    } while (adc_key_in < 900); // Wait for button release
    timer = 1;  // Whenever key pressed restart cursor timer
  }
   
  delay (10);
  if (timer) // Is cursor timer running?
  {
    timer++; // inc timer
    if (timer>1000){lcd.noBlink();timer=0;} // After 10 sec turn cursor off
  }
  
  // KD7TS complained about LOCK on display not being correct so double check here
  if (locked != (digitalRead(2)==1)) printLocked();

// End loop()
}   

