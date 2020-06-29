/*
 * "Sandwich Digital VFO" control software for CRKits KN-Q7A SSB transceiver kit 40 meter version and later on versions
 * 
 * For hardware based on Arduino Pro Mini 3.3V/8M board, with Ctrl Board and Osc Board
 * 
 * Author: Adam Rong, CRKits http://crkits.com
 * 
 * Created: Feb 26, 2017
 * ------------------------------------------------------------------------------------
 * Thanks to the following authors for their reference code:
 * 
 * si5351example.ino - Simple example of using Si5351Arduino library
 *
 * Copyright (C) 2015 Jason Milldrum <milldrum@gmail.com>
 * ------------------------------------------------------------------------------------
 * /* Simple calibration routine for the Si5351 breakout board.
 *
 * Copyright 2015 Paul Warren <pwarren@pwarren.id.au>
 * ------------------------------------------------------------------------------------
 * Rotary encoder example.
 * Author: Nick Gammon
 * Date:   25th May 2011
 *
 * Thanks for jraskell for helpful suggestions.
 * ------------------------------------------------------------------------------------
 * Debounce example
 *  created 21 November 2006
 *  by David A. Mellis
 *  modified 30 Aug 2011
 *  by Limor Fried
 *  modified 28 Dec 2012
 *  by Mike Walters
 * ------------------------------------------------------------------------------------
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 

#include "si5351.h"
#include "Wire.h"
#include "EEPROM.h"

Si5351 si5351;

// Wiring: Connect common pin of encoder to ground.
// Connect pin A (one of the outer ones) to a pin that can generate interrupts (eg. D2)
// Connect pin B (the other outer one) to another free pin (eg. D4)

volatile boolean fired;
volatile boolean up;
volatile boolean khz = true;  //tuning step in 1k Hz or 100 Hz in normal mode. calibration mode is not affected by this.
volatile boolean mode = true; //true = normal mode; false = calibration mode. To enable the calibration mode, press and hold the encoder switch and power on
volatile boolean CALIBMODE = true;  //true = bfo adjustment in 100 Hz step in calibration mode; false = crystal calibration in 20 Hz step in calibration mode

const byte encoderPinA = 2;
const byte encoderPinB = 4;
const byte buttonPin = 3; //button on encoder connected to D3
const byte LED_R = 10;  //red portion on a dual color LED
const byte LED_G = 11;  //green portion on a dual color LED
const byte JP = 7;  //ITU region select: high - region 1&3, low - region 2

byte eeprom_freq[7], cal_steps;  //first three bytes for op_freq, later three bytes for bfo_freq storage in eeprom, last byte for crystal calibration correction data (-127 ~ +127 times 20 Hz step)
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned int debounceDelay = 50;    // the debounce time; increase if the output flickers

unsigned long op_freq, band_low_limit = 7000000, band_high_limit, bfo_freq, bfo_low_limit, bfo_high_limit, temp_op_freq, temp_bfo_freq, target_freq = 10000000, rx_freq = 10000000;
unsigned int current_step;
int32_t crystal_cal;

// Interrupt Service Routine for a change to encoder pin A
void isr ()
{
  if (digitalRead (encoderPinA))
    up = !digitalRead (encoderPinB);
  else
    up = digitalRead (encoderPinB);
  fired = true;
}  // end of isr


void sDebug(char *str) {
  Serial.println(str) ;
}

void sDebug(int sval) {
  Serial.println(sval) ;
}


void setup ()
{
  Serial.begin(115200) ;
  
  pinMode (encoderPinA, INPUT_PULLUP);     // enable pull-ups
  pinMode (encoderPinB, INPUT_PULLUP);
  pinMode (buttonPin, INPUT_PULLUP); 
  pinMode (JP, INPUT_PULLUP);
  pinMode (LED_R, OUTPUT);
  pinMode (LED_G, OUTPUT);
  
  testLEDs() ;
  
  attachInterrupt (digitalPinToInterrupt (encoderPinA), isr, FALLING);   // interrupt 0 is pin 2

  if (digitalRead(JP))  //ITU region select: HIGH - region 1&3, LOW - region 2
  {
    sDebug("Region 1&3") ;
    op_freq = 7100000;  //set op_freq to the mid of the band
    band_high_limit = 7200000;  //set band high limit to 7.2 MHz for region 1&3, like Europe and Asia
    bfo_freq = 8465300; //IF freq = 8467.2 kHz, BFO freq is around IF freq depending on LSB or USB, default to LSB
    bfo_low_limit = 8464700;
    bfo_high_limit = 8468700;
  }
  else 
  {
    sDebug("Region 2") ;
    
    op_freq = 7200000;  //set op_freq to higher portion of the band
    band_high_limit = 7300000;  //set band high limit to 7.3 MHz for region 2, like America
    bfo_freq = 8190100;     //historical reason: for KN-Q7A kits covering 7145-7165, 7200-7220, 7280-7300, IF freq = 8192 kHz
    bfo_low_limit = 8189500;
    bfo_high_limit = 8193500;
  }
  
  for (int i=0;i<7;i++)
    eeprom_freq[i] = EEPROM.read(i);

  debugEEPROM() ;
  


temp_op_freq = 0;
for (int i=0;i<eeprom_freq[0];i++)    //read out op_freq from eeprom and assemble in an ugly way
    temp_op_freq = temp_op_freq + 1000000;
for (int i=0;i<eeprom_freq[1];i++)
    temp_op_freq = temp_op_freq + 10000;
for (int i=0;i<eeprom_freq[2];i++)
    temp_op_freq = temp_op_freq + 100;

temp_bfo_freq = 0;
for (int i=0;i<eeprom_freq[3];i++)    //read out bfo_freq from eeprom and assemble in an ugly way
    temp_bfo_freq = temp_bfo_freq + 1000000;
for (int i=0;i<eeprom_freq[4];i++)
    temp_bfo_freq = temp_bfo_freq + 10000;
for (int i=0;i<eeprom_freq[5];i++)
    temp_bfo_freq = temp_bfo_freq + 100;

  if ((temp_op_freq >= band_low_limit) && (temp_op_freq <= band_high_limit))  //validate EEPROM data
//    op_freq = temp_op_freq;
  if ((temp_bfo_freq >= bfo_low_limit) && (temp_bfo_freq <= bfo_high_limit))  //validate EEPROM data
    bfo_freq = temp_bfo_freq;

cal_steps = eeprom_freq[6];
if (cal_steps < 128) {
  for (int i=0;i<cal_steps;i++)
  crystal_cal -= 2000; //range = (-128 ~ 127) * 20 Hz
}
else if (cal_steps >= 128) {
  for (int i=cal_steps;i<256;i++)
  crystal_cal += 2000;
}

  // Start serial and initialize the Si5351
  //Serial.begin(57600);
  sDebug("Init si5351") ;
  
  si5351.init(SI5351_CRYSTAL_LOAD_10PF, 27000000); // Crystal = 27MHz, load capacitor 20 pF, set max value to be as close as possible
  
  si5351.set_correction(crystal_cal); // Frequency Correction 

  // Set CLK0 to output vfo freq
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.set_freq(uint64_t((bfo_freq + op_freq)*100), SI5351_PLL_FIXED, SI5351_CLK0);
  
  si5351.output_enable(SI5351_CLK1,0); //disable output of CLK1

  // Set CLK1 to output 10 MHz for crystal calibration when needed
  // si5351.set_freq(uint64_t(target_freq * 100), 0ULL, SI5351_CLK1);
  
  // Set CLK2 to output bfo freq
  si5351.set_freq(uint64_t(bfo_freq*100), SI5351_PLL_FIXED, SI5351_CLK2); 
  
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA); //SET Output_0 Current Drive (2mA=3dBm; 8mA=10dBm)
  
   si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA); //SET Output_1 Current Drive (2mA=3dBm; 8mA=10dBm)
  
   si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); //SET Output_2 Current Drive (2mA=3dBm; 8mA=10dBm)(Delete // to enable)


 // Available values: 2, 4, 6 or 8mA
  sDebug("si5351 completed") ;

 if (!digitalRead(buttonPin)) {
  mode = false;  //if button is pressed and hold at power on, check button status and switch mode to calibration mode
  sDebug("CALIBRATE MODE") ;
  
 } else {
  mode = true; //otherwise stay in normal mode
  sDebug("NORMAL MODE ") ;
 }

}



void debugEEPROM() {
  sDebug("Reading EEPROM") ;
  for (int i = 0 ; i < 7 ; i++) {
    sDebug(eeprom_freq[i]) ;
  
  }
}

void testLEDs() {
  sDebug("testLEDs") ;
  digitalWrite (LED_R, HIGH); // LED self test at power on
  delay (250);
  digitalWrite (LED_R, LOW);
  delay (250);
  
  digitalWrite (LED_G, HIGH);
  delay (250);
  digitalWrite (LED_G, LOW);
  delay (250);
 

  digitalWrite (LED_R, HIGH); // LED self test at power on
  digitalWrite (LED_G, HIGH);
  delay (250);
  digitalWrite (LED_R, LOW); // LED self test at power on
  digitalWrite (LED_G, LOW);
  delay(250) ;
  
}
void loop()
{
int reading = digitalRead(buttonPin);

  // check to see if you just pressed the button
  // (i.e. the input went from HIGH to LOW),  and you've waited
  // long enough since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();

  }

if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) //button pressed
      {
      if (mode)  //normal mode
      {

          
        khz = !khz; //toggle kHz and 100 Hz step
        if (khz == true) {
          op_freq = (op_freq / 1000) * 1000;  //switch to 1 kHz mode and remove 100 Hz portion if it was not exact kHz
          if ((op_freq > band_low_limit) && (op_freq < band_high_limit))  //if op_freq is in band, set frequency
              si5351.set_freq(uint64_t((bfo_freq + op_freq)*100), SI5351_PLL_FIXED, SI5351_CLK0);
        }
          eeprom_freq[0] = op_freq / 1000000;  //write operation frequency to EEPROM at each toggle
          eeprom_freq[1] = (op_freq - eeprom_freq[0] * 1000000) / 10000;  
          eeprom_freq[2] = ((op_freq - eeprom_freq[0] * 1000000) % 10000) / 100;
          for (int i=0;i<3;i++)
          EEPROM.write(i,eeprom_freq[i]);

      }
      else {  //calibration mode
          CALIBMODE = !CALIBMODE;  //toggle bfo or crystal adjustment in calibration

          eeprom_freq[3] = bfo_freq / 1000000;  //write BFO frequency to EEPROM at each toggle
          eeprom_freq[4] = (bfo_freq - eeprom_freq[3] * 1000000) / 10000;  
          eeprom_freq[5] = ((bfo_freq - eeprom_freq[3] * 1000000) % 10000) / 100;
          eeprom_freq[6] = cal_steps; 
          for (int i=3;i<7;i++)
          EEPROM.write(i,eeprom_freq[i]);
          if (!CALIBMODE) { //crystal calibration mode
            rx_freq = target_freq;
            crystal_cal = 0;
            if (cal_steps < 128) {
              for (int i=0;i<cal_steps;i++) {
              rx_freq += 20; //range = (-128 ~ 127) * 20 Hz
              crystal_cal -= 2000;
              }
            }
            else if (cal_steps >= 128) {
              for (int i=cal_steps;i<256;i++)
              rx_freq -= 20;
              crystal_cal += 2000;
            }
            si5351.set_correction(0);
            si5351.output_enable(SI5351_CLK1,1);
            si5351.set_freq(uint64_t(rx_freq * 100), 0ULL, SI5351_CLK1);
          }
          else if (CALIBMODE)  {
              si5351.output_enable(SI5351_CLK1,0);
              si5351.set_correction(crystal_cal);
              si5351.set_freq(uint64_t(bfo_freq*100), SI5351_PLL_FIXED, SI5351_CLK2);
              si5351.set_freq(uint64_t((bfo_freq + op_freq)*100), SI5351_PLL_FIXED, SI5351_CLK0);
          }

      }

      }

      }
    }

  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonState = reading;


//LED frequency display in NORMAL mode
if (mode) {    //normal mode
    if ((op_freq <= band_low_limit) || (op_freq >= band_high_limit))
    {
      digitalWrite (LED_R, HIGH); //red - out of band
      digitalWrite (LED_G, LOW);
    }
    else if (op_freq % 100000 == 0)
    {
      digitalWrite (LED_R, HIGH); //red - 100 kHz indication
      digitalWrite (LED_G, LOW);   
    }
    else if (op_freq % 10000 == 0)
    {
      digitalWrite (LED_R, HIGH); //orange - 10 kHz indication
      digitalWrite (LED_G, HIGH);
    }
    else if (op_freq % 2000 == 1000)
    {
      digitalWrite (LED_R, LOW); //green - every 2 kHz indication at 1, 3, 5, 7, 9 kHz
      digitalWrite (LED_G, HIGH);
    }
    else if (!khz && ((op_freq % 1000) == 0))  //if in 100 Hz step mode
      {
        digitalWrite (LED_R, LOW);  //green - for each kHz, including 2, 4, 6, 8, 0 kHz
        digitalWrite (LED_G, HIGH);
      } 
/*    else if (!khz && ((op_freq % 500) == 0))
    {
      digitalWrite (LED_R, LOW); //green blinking - every 500 Hz in 100 Hz step mode
      digitalWrite (LED_G, HIGH);
      delay(100);
      digitalWrite (LED_G, LOW); 
      delay(100);
    }*/
    else {
      digitalWrite (LED_R, LOW);  //off - all other scenario
      digitalWrite (LED_G, LOW);
    }
}
else {  //calibration mode
        if (CALIBMODE && (bfo_freq > bfo_low_limit) && (bfo_freq < bfo_high_limit)) {
          digitalWrite (LED_R, HIGH);  //orange for bfo mode
          digitalWrite (LED_G, HIGH);
        }
        else if (!CALIBMODE) {
          digitalWrite (LED_R, LOW);
          digitalWrite (LED_G, HIGH);  //green for crystal mode
        }
        else {  //red: out of range
          digitalWrite (LED_R, HIGH);
          digitalWrite (LED_G, LOW);
        }
}

//encoder portion
  if (fired)
    {
      if (mode&&khz) 
      current_step = 1000;  //1 kHz step in normal mode
      else if ((mode&&(!khz)) || (!mode&&CALIBMODE))
      current_step = 100;  //100 Hz step in normal mode and bfo calibration mode
      else current_step = 20; //20 Hz step for crystal calibration
      
    if (up) {
      if (mode) {  //NORMAL mode
      op_freq=op_freq + current_step; 
      if (op_freq >= band_high_limit) op_freq = band_high_limit;  //band high limit control
      si5351.set_correction(crystal_cal);
          si5351.set_freq(uint64_t((bfo_freq + op_freq)*100), SI5351_PLL_FIXED, SI5351_CLK0);
      //    si5351.set_freq(uint64_t(bfo_freq*100), SI5351_PLL_FIXED, SI5351_CLK2);
      }
      else if ((!mode) && (CALIBMODE)) { //bfo mode in calibration mode
        bfo_freq=bfo_freq + current_step;
        if (bfo_freq >= bfo_high_limit) bfo_freq = bfo_high_limit; //bfo high limit control
        si5351.set_correction(crystal_cal);
        si5351.set_freq(uint64_t((bfo_freq + op_freq)*100), SI5351_PLL_FIXED, SI5351_CLK0);
        si5351.set_freq(uint64_t(bfo_freq*100), SI5351_PLL_FIXED, SI5351_CLK2);
      }
      else {
        cal_steps++;  //crystal mode in calibration mode
        rx_freq += 20;
        si5351.set_correction(0);
        si5351.set_freq(uint64_t(rx_freq * 100), 0ULL, SI5351_CLK1);
      }
    }
      
    else {
      if (mode) { //NORMAL mode
       op_freq=op_freq - current_step;
       if (op_freq <= band_low_limit) op_freq = band_low_limit;  //band low limit control
      si5351.set_correction(crystal_cal);
          si5351.set_freq(uint64_t((bfo_freq + op_freq)*100), SI5351_PLL_FIXED, SI5351_CLK0);
      //    si5351.set_freq(uint64_t(bfo_freq*100), SI5351_PLL_FIXED, SI5351_CLK2);
      }
      else if ((!mode) && (CALIBMODE)) { //bfo mode in calibration mode
        bfo_freq=bfo_freq - current_step;
        if (bfo_freq <= bfo_low_limit) bfo_freq = bfo_low_limit;  //bfo low limit control
        si5351.set_correction(crystal_cal);
        si5351.set_freq(uint64_t((bfo_freq + op_freq)*100), SI5351_PLL_FIXED, SI5351_CLK0);
        si5351.set_freq(uint64_t(bfo_freq*100), SI5351_PLL_FIXED, SI5351_CLK2);
      }
      else {
        cal_steps--;  //crystal mode in calibration mode
        rx_freq -= 20;
        si5351.set_correction(0);
        si5351.set_freq(uint64_t(rx_freq * 100), 0ULL, SI5351_CLK1);
      }
      
    }
    


     
 //   Serial.println (op_freq);
 //   Serial.println (bfo_freq);
 //   Serial.println (crystal_cal);
 //   Serial.println (cal_steps);


   
    fired = false;

    }  // end if fired

}
