//   ADF435x and Arduino
//   By Alain Fort F1CJN feb 2,2016
//   update march 7, 2016 (ROBOT V1.1 and V1.0)
//
//  Lots of changes by Ray Cannon W7GLF
//
//   Update Aug 26, 2018 (Allow users to redefine DAT and CLK pins basically get rid of SPI_LIB)
//   Update Feb 2, 2019 (Fix round off error)
//

// Warning pins D2 through D9 are reserved for the following use
//
// D2 - input pin for Lock
// D4 - input frequency table index bit (2^0) - ones digit complement
// D5 - input frequency table index bit (2^1) - twos digit complement
// D6 - input frequency table index bit (2^2) - fours digit complement
// D7 - input frequency table index bit (2^3) - eights digit complement
// D8 - input frequency table index bit (2^4) - sixteens digit complement
// D9 - input - HIGH (default) means halt processor after programming ADF4350

#define ADF435x_LE 3 // Pin for Latch Enable on ADF435x
#define ADF435x_CLK 13 // Pin for CLK on ADF435x
#define ADF435x_DAT 11 // Pin for DAT on ADF435x

//  INTDIV below was used to test the use of INT n division rather than FRAC n division.  
//  It would be interesting to compare the phase noise using each method.  I have determined INT n 
//  division does not work for all choices (for example 432.01) because N register is only 16 bits.
//  When that happens I fall back to FRACT mode and display LOCKED.  
//
#define INTDIV  true

#define ADF4351 true   // Are we ADF4350 or ADF4351 ?

//  DEBUG enables the serial print
#define DEBUG true

// Set desired default REF_FREQ to 10 MHz or 25 MHz
#define DEFAULT_REF_FREQ 10

// Set desired default level (0 to 3)
#define DEFAULT_LEVEL 3

double frequency_table [32] =
  // I made these double so if on board oscillator is too low these can be
  // tweaked so call frequency is within the ham band.  Otherwise many radios
  // will not allow you to transmit.  This means that using integer division is
  // likely not possible.  If you lock to GPS then these numbers will end up 
  // being integers and you can use integer divison to improve phase noise.
          {
            // The first value is the default value when Arduino has no jumpers. 
            2556.00,    // 0  - 2556*4 => 10224 + 144.1 => 10368.1
//            1136.00,  // 0  - 1136*9 => 10224 + 144.1 => 10368.1

            1135.80,  // 1  - 1135.8*9 => 10222.2 + 145.9 => 10368.1
            1104.00,  // 2  - 1104*9 => 9936 + 432 => 10368
             875.00,  // 3  - 875 + 28 => 903
            1152.00,  // 4  - 1152 + 28 => 1296 
            2160.00,  // 5  - 2160 + 144 => 2304
            1872.00,  // 6  - 1872 + 432 => 2304
            3312.00,  // 7  - 3312 + 144 => 3456 
            3024.00,  // 8  - 3024 + 432 => 3456
            1872.00,  // 9  - 1872*3 => 5616 + 144 => 5760
            1776.00,  // 10 - 1776*3 => 5328 + 432 => 5760
               0.00,  // 11 - 
               0.00,  // 12 - 
               0.00,  // 13 - 
               0.00,  // 14 - 
               0.00,  // 15 - 
             144.20,  // 16 - For testing 
               0.00,  // 17 - 
               0.00,  // 18 - 
               0.00,  // 19 - 
               0.00,  // 20 -
               0.00,  // 21 - 
               0.00,  // 22 - 
               0.00,  // 23 - 
               0.00,  // 24 - 
               0.00,  // 25 - 
               0.00,  // 26 - 
               0.00,  // 27 - 
               0.00,  // 28 - 
               0.00,  // 29 - 
               0.00,  // 30 - 
            1152.00   // 31 - For testing - good harmonics at 2304, 3456, 5760, 10368
          };
//
//  This sketch uses and Arduino Nano ($2) and an ADF435x chinese
//  card found at EBAY ($16). The frequency can be programmed between 137.5 and 4400 MHz.
//  Thirty two frequencies can be selected using pins D8 (most significant bit) through D4 (least significant bit).
//  The bit is considered one if the pin is low and zero if the pin is high.  This means if nothing is connected
//  the Aduino will select frequency zero. This is because of the pullups.
//  To select frequency 3 D8 high, D7 high, D6 high, D5 low, D4 low.
//  Pin D9 when LOW tells Arduino to loop rather than halt.  This way it can change frequencies on the fly.
//
//  ******************************************** HARDWARE IMPORTANT********************************************************
//  With an Arduino UNO (NANO) : uses a resistive divider to reduce the voltage, MOSI (pin 11) to
//  ADF DATA, SCK (pin13) to ADF CLK, Select (PIN 3) to ADF LE
//  Resistive divider 1000 Ohm with 1800 Ohm to ground on Arduino pins D11, D13 and D3 to adapt from 5V
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
unsigned long gcd1; // Used for FRAC division to simply FRAC/MOD

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
  for (int i = 31; i >= 0; i--)          // loop on 32 bits MSB to LSB
  {
    digitalWrite (ADF435x_DAT, (value >> i) & 1);
    digitalWrite (ADF435x_CLK, HIGH);
    digitalWrite (ADF435x_CLK, LOW);  
  }
  digitalWrite(ADF435x_LE, HIGH);
}

void SetADF435x()  // Program all the registers of the ADF435x
{ 
  char output [10];

  int val;
  for (int i = 5; i >= 0; i--)
  {  // programming ADF435x starting with R5
    WriteRegister32(registers[i]);
#if DEBUG
    Serial.print ("registers [");   
    Serial.print (i);   
    Serial.print ("] = ");
    sprintf (output, "%04X", (registers [i])>>16 & 0xffff);
    Serial.print (output);
    sprintf (output, "%04X", (registers [i]) & 0xFFFF);
    Serial.print (output);
    Serial.print ("\r\n");
#endif       
  }
#if DEBUG
    Serial.flush ();
#endif       
}

// Calculate Greatest Common Divisor of desired freq and ref which gives us the
// highest Phase Detection Frequency we can use for integer divisor.
// Credit for algorithm goes to Euclid.
unsigned long GCD(unsigned long freq, unsigned long ref )
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
    
#if INTDIV
    // User INTEGER Division
    // Calculate best integer Phase Detector Frequency

    // Convert frequencies to Hertz
    freqin = 10000 * RFint * OutputDivider;
    pdfreqin  = 1000000 * PFDRFout;
    gcd = GCD(freqin, pdfreqin );
    nreg = freqin / gcd;
    rreg = pdfreqin / gcd;

  #if DEBUG    
    if (gcd != oldgcd) 
    {
      Serial.print ("RFint = ");
      Serial.println (RFint);
      Serial.print ("OutputDivider = ");
      Serial.println (OutputDivider);
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

    if (nreg > 65535 || gcd < 500000) // If Loop freq is below .5 MHz
    {
  #if DEBUG
    if (nreg > 65536)
      Serial.print ("N too large for INT mode so using FRACT mode\n");
    if (gcd < 500000)
      Serial.print ("Loop Comparison frequency < 500 kHz so using FRACT mode\n");
  #endif
      nreg_overflow = true;
      // We cannot use INT so go back to FRACT
      INTA = (RFout * OutputDivider) / PFDRFout;
      MOD = (PFDRFout / OutputChannelSpacing);
      if (MOD < 2) MOD = 2;
      FRACF = (((RFout * OutputDivider) / PFDRFout) - INTA) * MOD;
      FRAC = round(FRACF); // The result is rounded
      registers[2] = 0x4E42;
      #if ADF4351
      bitClear (registers[3], 21); // Reset for Fractional Division
      bitClear (registers[3], 22); // Reset for Fractional Division
      #endif
    }
    else
    {
      // No Overflow - we can do INT Division
      nreg_overflow = false;
      INTA = nreg;
      MOD = 2;
      FRAC = 0; // The result is rounded
      registers[2] = (( rreg & 0x3ff ) << 14) | 0xF42;  // (0x100) set means use INT Division
      #if ADF4351
      bitSet (registers[3], 21); // Set for INT Division
      bitSet (registers[3], 22); // Set for INT Division
      #endif
    } 
#else //  INTDIV false always use FRACT DIVISION
    INTA = (RFout * OutputDivider) / PFDRFout;
    MOD = (PFDRFout / OutputChannelSpacing);
    if (MOD < 2) MOD = 2;
    FRACF = (((RFout * OutputDivider) / PFDRFout) - INTA) * MOD;
    FRAC = round(FRACF); // The result is rounded
#endif // #if INTDIV

//  We will simply FRAC/MOD if appropriate
    if (FRAC > 0)
    {
      gcd1 = GCD(FRAC,MOD);
      FRAC /= gcd1;
      MOD  /= gcd1;
    }
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

  pinMode(2, INPUT);  // PIN 2 input entry for lock
  pinMode(ADF435x_LE, OUTPUT);          // Setup pins
  digitalWrite(ADF435x_LE, HIGH);
  
  pinMode(ADF435x_CLK, OUTPUT);          // Setup pins
  pinMode(ADF435x_DAT, OUTPUT);          // Setup pins
  digitalWrite (ADF435x_CLK, LOW);  
  digitalWrite (ADF435x_DAT, LOW);  

  pinMode(4, INPUT);          // Setup pins
  pinMode(5, INPUT);          // Setup pins
  pinMode(6, INPUT);          // Setup pins
  pinMode(7, INPUT);          // Setup pins
  pinMode(8, INPUT);          // Setup pins
  pinMode(9, INPUT);          // Setup pins
  digitalWrite(4, HIGH);     // Turn on the internal pull-up resistor, default state is HIGH
  digitalWrite(5, HIGH);     // Turn on the internal pull-up resistor, default state is HIGH
  digitalWrite(6, HIGH);     // Turn on the internal pull-up resistor, default state is HIGH
  digitalWrite(7, HIGH);     // Turn on the internal pull-up resistor, default state is HIGH
  digitalWrite(8, HIGH);     // Turn on the internal pull-up resistor, default state is HIGH
  digitalWrite(9, HIGH);     // Turn on the internal pull-up resistor, default state is HIGH

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

  // Read D8 through D4 to find RF Frequency

  freq_select = 0;

  // Invert sense of input pins.  We do this because we want our default value
  // (when pins 8 through 4 are not connected and are thus pulled high) to use register 0.
  // There is no choice to have the pins pulled low when there is no connection.
  // When using external switches to set which frequency to use mount the switches so they
  // are closed when the paddle is down.  This way the binary pattern of the switches will
  // directly indicate which choice is being made.
   
  freq_select |= (digitalRead (8) == 0) << 4;
  freq_select |= (digitalRead (7) == 0) << 3;
  freq_select |= (digitalRead (6) == 0) << 2;
  freq_select |= (digitalRead (5) == 0) << 1;
  freq_select |= (digitalRead (4) == 0);

  RFint = frequency_table [freq_select] * 100.0 + .5;  // Convert MHz to tens of KHz.
                                                       // Add .5 to handle any roundoff

#if DEBUG    
      Serial.print ("freq_table = ");
      Serial.print (frequency_table [freq_select]);
      Serial.print ("\n");
      Serial.print ("RFint = ");
      Serial.print (RFint);
      Serial.print ("\n");
#endif  
  
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

  if (digitalRead (9) == HIGH)
  {
     // When we have programmed the synth HALT the processor to reduce 
     // switching transients and power draw.  This means that if the frequency
     // is changed on the fly the Arduino Nano will need to be reset to pick up
     // the new value.  This makes sense if using as a fixed LO.  If you wish to
     // be able to change the frequency on the fly remove the next three lines.
     cli();
     sleep_enable();
     sleep_cpu();
  }

  delay (1000);

// End loop()
}   

