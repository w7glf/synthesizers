//   ADF435x and Arduino
//   By Alain Fort F1CJN feb 2,2016
//   update march 7, 2016 (ROBOT V1.1 and V1.0)
//
//  Lots of changes by Ray Cannon W7GLF
//   Update July 17, 2018 (Remove INTDIV - it does not work)
//
//

#define ADF4351 false   // Are we ADF4350 or ADF4351 ?

//  DEBUG enables the serial print
#define DEBUG true

// Set desired default REF_FREQ to 10 MHz or 25 MHz
#define DEFAULT_REF_FREQ 10

// Set desired default level (0 to 3)
#define DEFAULT_LEVEL 0

double frequency_table [16] =
  // I made these double so if on board oscillator is too low these can be
  // tweaked so call frequency is within the ham band.  Otherwise many radios
  // will not allow you to transmit.  This means that using integer division is
  // likely not possible.  If you lock to GPS then these numbers will end up 
  // being integers and you can use integer divison to improve phase noise.
          {
            1135.95,  // 0  - 1136*9 => 10224 + 144 => 10368
            1104.00,  // 1  - 1104*9 => 9936 + 432 => 10368
             875.00,  // 2  - 875 + 28 => 903
            1152.00,  // 3  - 1152 + 28 => 1296 
            2160.00,  // 4  - 2160 + 144 => 2304
            1872.00,  // 5  - 1872 + 432 => 2304
            3312.00,  // 6  - 3312 + 144 => 3456 
            3024.00,  // 7  - 3024 + 432 => 3456
            1872.00,  // 8  - 1872*3 => 5616 + 144 => 5760
            1776.00,  // 9  - 1776*3 => 5328 + 432 => 5760
               0.00,  // 10 - 
               0.00,  // 11 - 
               0.00,  // 12 - 
               0.00,  // 13 - 
               0.00,  // 14 - 
            1152.00   // 15 - For testing - good harmonics to 10 GHz at 2304, 3456, 5760, 10368
          };
//
//  This sketch uses and Arduino Nano ($2) and an ADF435x chinese
//  card found at EBAY ($16). The frequency can be programmed between 137.5 and 4400 MHz.
//  Sixteen frequencies can be selected using pins D7 (most significant bit) through D4 (least significant bit).
//  The bit is considered one if the pin is low and zero if the pin is high.  This means if nothing is connected
//  the Aduino will select frequency zero. This is because of the pullups.
//  To select frequency 3 D7 high, D6 high, D5 low, D4 low.
//  Pin D8 when LOW tells Arduino to loop rather than halt.
//
//  ******************************************** HARDWARE IMPORTANT********************************************************
//  With an Arduino UNO (NANO) : uses a resistive divider to reduce the voltage, MOSI (pin 11) to
//  ADF DATA, SCK (pin13) to ADF CLK, Select (PIN 3) to ADF LE
//  Resistive divider 1000 Ohm with 1300 Ohm to ground on Arduino pins D11, D13 and D3 to adapt from 5V
//  to 3.3V the digital signals DATA, CLK and LE send by the Arduino.
//  Arduino pin D2 (for lock detection) directly connected to ADF435x card MUXOUT.
//  The ADF card is 5V powered by the ARDUINO (PINs Vin and GND).

#include <SPI.h>
#include <avr/sleep.h>

#define ADF435x_LE 3 // Pin for Latch Enable on ADF435x

bool locked, nreg_overflow=false;

uint32_t registers[6] =  {0x718008, 0x8008029, 0x4E42, 0x4B3, 0x95003C, 0x580005} ; // 1136 MHz with ref 10 MHz

int level=DEFAULT_LEVEL, levelold=-1;  // output level is 0 through 3 see ADF435x documentation Register 4
unsigned int i = 0;
unsigned long gcd;  // Only used for INT division to hold loop filter frequency

double RFout, REFin, INT, PFDRFout, OutputChannelSpacing, FRACF;

#if ADF4351
double RFoutMin = 35;
#else
double RFoutMin = 137.5;
#endif

double RFoutMax = 4400, REFinMax = 250, PDFMax = 32;
unsigned int long RFint,RFintold,INTA,RFcalc,PDRFout, MOD, FRAC;
byte OutputDivider, lock=2;
unsigned int long reg0, reg1;

unsigned long nreg, rreg, oldgcd;
unsigned long freqin, pdfreqin;

void WriteRegister32(const uint32_t value)   //program a 32 bit register
{
  digitalWrite(ADF435x_LE, LOW);
  for (int i = 3; i >= 0; i--)          // loop on 4 x 8 bits
    SPI.transfer((value >> 8 * i) & 0xFF); // offset, byte masking and sending via SPI
  digitalWrite(ADF435x_LE, HIGH);
}

void SetADF435x()  // Program all the registers of the ADF435x
{ 
  for (int i = 5; i >= 0; i--)
  {  // programming ADF435x starting with R5
    WriteRegister32(registers[i]);
#if DEBUG
    Serial.print ("registers [");   
    Serial.print (i);   
    Serial.print ("] = ");   
    Serial.print (registers [i], HEX);
    Serial.print ("\r\n");
#endif       
  }
#if DEBUG
    Serial.flush ();
#endif       
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
#if ADF4351
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
#endif
}

void FrequencyOrLevelHasChanged()
{
    CalculateDivider ();
    
    if (level != levelold)  // Output level has changed
    {  // fill in with current value
      registers [4] &= 0XFFFFFFE7;
      registers [4] |= ((level & 3) << 3);
    } 
    
    INTA = (RFout * OutputDivider) / PFDRFout;
    MOD = (PFDRFout / OutputChannelSpacing);
    if (MOD < 2) MOD = 2;
    FRACF = (((RFout * OutputDivider) / PFDRFout) - INTA) * MOD;
    FRAC = round(FRACF); // The result is rounded

#if DEBUG    
      Serial.print ("INT = ");
      Serial.print (INTA);
      Serial.print ("\nR = 1");
      Serial.print ("\nPDF = 10 MHz");
      Serial.print ("\nFRAC = ");
      Serial.print (FRAC);
      Serial.print ("\nMOD = ");
      Serial.print (MOD);
      Serial.print ("\n");
#endif 

    registers[0] = 0;
    registers[0] = INTA << 15; // OK
    FRAC = FRAC << 3;
    registers[0] = registers[0] + FRAC;

    registers[1] = 0;
    registers[1] = MOD << 3;
    registers[1] = registers[1] + 1 ; // addition of the address "001"
    bitSet (registers[1], 27); // Prescaler on 8/9
    bitSet (registers[1], 15); // Set Phase to 1

    bitSet (registers[2], 28); // Digital lock == "110" on b28 b27 b26
    bitSet (registers[2], 27); // digital lock 
    bitClear (registers[2], 26); // digital lock
   
    SetADF435x();  // Program all the registers of the ADF435x
}
      
//************************************ Setup ****************************************
void setup() {

  Serial.begin (9600); //  Serial to the PC via Arduino "Serial Monitor"  at 9600

  pinMode(2, INPUT);  // PIN 2 in entry for lock
  pinMode(ADF435x_LE, OUTPUT);          // Setup pins
  digitalWrite(ADF435x_LE, HIGH);
  
  SPI.begin();                          // Init SPI bus
  SPI.setDataMode(SPI_MODE0);           // CPHA = 0 and positive Clock
  SPI.setBitOrder(MSBFIRST);            // Big Endian

  pinMode(4, INPUT);          // Setup pins
  pinMode(5, INPUT);          // Setup pins
  pinMode(6, INPUT);          // Setup pins
  pinMode(7, INPUT);          // Setup pins
  pinMode(8, INPUT);          // Setup pins
  digitalWrite(4, HIGH);     // Turn on the internal pull-up resistor, default state is HIGH
  digitalWrite(5, HIGH);     // Turn on the internal pull-up resistor, default state is HIGH
  digitalWrite(6, HIGH);     // Turn on the internal pull-up resistor, default state is HIGH
  digitalWrite(7, HIGH);     // Turn on the internal pull-up resistor, default state is HIGH
  digitalWrite(8, HIGH);     // Turn on the internal pull-up resistor, default state is HIGH

  // Default ref
  PFDRFout=DEFAULT_REF_FREQ;

  RFint = 0;  // RFint is in tens of KHz
  OutputChannelSpacing = 0.01; // Frequency step = 10kHz
  
  RFintold=0;//for RFintold to be different from RFout when init
  RFout = RFint/100 ; // output frequency in MHz

  delay (500);

} // End setup

//*************************************Loop***********************************
void loop()
{
  byte freq_select;

  // Read D7 through D4 to find RF Frequency

  freq_select = 0;

  // Invert sense of input pins.  We do this because we want our default value
  // (when pins 7 through 4 are not connected and are thus pulled high) to use register 0.
  // There is no choice to have the pins pulled low when there is no connection.
  // When using external switches to set which frequency to use mount the switches so they
  // are closed when the paddle is down.  This way the binary pattern of the switches will
  // directly indicate which choice is being made.
   
  freq_select |= (digitalRead (7) == 0) << 3;
  freq_select |= (digitalRead (6) == 0) << 2;
  freq_select |= (digitalRead (5) == 0) << 1;
  freq_select |= (digitalRead (4) == 0);

  RFint = frequency_table [freq_select] * 100;  // Convert MHz to tens of KHz.
  
  if (RFint > 440000) RFint = RFintold;
  
  oldgcd = 0;
  RFout=RFint;
  RFout=RFout/100;
  if ( (RFint != RFintold) || (level != levelold) ) 
  {
    FrequencyOrLevelHasChanged();
    RFintold=RFint;
    levelold=level;
  }
   
  delay (10);

  if (digitalRead (8) == HIGH)
  {
     // When we have programmed the synth HALT the processor to reduce 
     // switching transients and power draw.  This means that if the frequency
     // is changed on the fly the Arduino Nano will need to be reset to pick up
     // the new value.  This makes sense if using as a fixed LO.  If you wish to
     // be able to shange the frequency on the fly remove the next three lines.
     cli();
     sleep_enable();
     sleep_cpu();
  }

  delay (1000);

// End loop()
}   

