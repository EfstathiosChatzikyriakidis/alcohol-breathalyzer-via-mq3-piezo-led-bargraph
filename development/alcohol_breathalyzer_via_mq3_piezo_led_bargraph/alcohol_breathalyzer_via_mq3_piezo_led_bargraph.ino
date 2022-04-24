/*
 *  Alcohol Breathalyzer Via MQ-3, Piezo & Led Bargraph.
 *
 *  Copyright (C) 2010 Efstathios Chatzikyriakidis (stathis.chatzikyriakidis@gmail.com)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

// include for portable "non-local" jumps.
#include <setjmp.h>

// include notes' frequencies.
#include "pitches.h"

const uint8_t buttonOnPin = 2;  // the pin number of the button "on" element.
const uint8_t buttonOnIRQ = 0;  // the IRQ number of the button "on" pin.

const uint8_t buttonOffPin = 3; // the pin number of the button "off" element.
const uint8_t buttonOffIRQ = 1; // the IRQ number of the button "off" pin.

const uint8_t ledPin = 7;    // the led pin of the system.
const uint8_t piezoPin = 6;  // the piezo pin of the system.
const uint8_t sensorPin = 0; // the sensor pin of the system.

// set 74HC595 8-bit shift register control pin numbers.
const uint8_t data = 12;
const uint8_t clock = 9; // PWM pin.
const uint8_t latch = 4;

// notes in the system melody.
const uint16_t notesMelody[] = {
  NOTE_G4, NOTE_C4, NOTE_G3, NOTE_G3, NOTE_C4, NOTE_G3, NOTE_C4
};

// calculate the number of the notes in the melody in the array.
const uint8_t NUM_NOTES = (uint8_t) (sizeof (notesMelody) / sizeof (const uint16_t));

// note durations: 4 = quarter note, 8 = eighth note, etc.
const uint8_t noteDurations[] = {
  4, 8, 8, 2, 4, 2, 4
};

// number of leds to control.
const uint8_t NUM_LEDS = 8;

// threshold value for the music to play.
const uint8_t ledsThreshold = NUM_LEDS-1;

// default value for leds bargraph.
int16_t ledsRange = -1;

// used for single LED manipulation.
uint8_t ledState = LOW;

// the following are used in the bitwise math that we use to change individual LEDs.

// each specific bit set to 1.
const uint8_t bits[] = { B00000001, B00000010,
                         B00000100, B00001000,
                         B00010000, B00100000,
                         B01000000, B10000000 };

// each specific bit set to 0.
const uint8_t masks[] = { B11111110, B11111101,
                          B11111011, B11110111,
                          B11101111, B11011111,
                          B10111111, B01111111 };

// the two bounds of the input sensor (analog input).
const uint16_t sensorMin = 500;
const uint16_t sensorMax = 900;

// define a delay time for the next sensor sample.
const uint16_t SENSOR_DELAY = 300;

// define a bounce time for a button.
const uint16_t BOUNCE_DURATION = 200;

// ms count to debounce a pressed button.
volatile uint32_t bounceTime = 0;

// the state of the system (on = true, off = false).
volatile bool systemState = false;

// information to restore calling environment.
jmp_buf buf;

// startup point entry (runs once).
void
setup() {
  // set buttons "on/off" elements as an input.
  pinMode(buttonOnPin, INPUT);
  pinMode(buttonOffPin, INPUT);

  // set the input sensor as input.
  pinMode(sensorPin, INPUT);

  // set the led and piezo as outputs.
  pinMode(ledPin, OUTPUT);
  pinMode(piezoPin, OUTPUT);

  // set 74HC595 control pins as output.
  pinMode (data, OUTPUT);
  pinMode (clock, OUTPUT);  
  pinMode (latch, OUTPUT);  

  // attach the ISRs for the IRQs (for buttons).
  attachInterrupt(buttonOnIRQ, buttonOnISR, RISING);
  attachInterrupt(buttonOffIRQ, buttonOffISR, RISING);

  // dark all system leds.
  darkAllLeds();
}

// loop the main sketch.
void
loop() {
  // save the environment of the calling function.
  setjmp(buf);

  // if the system is on start working.
  if(systemState) {
    // get the value from the input sensor.
    ledsRange = analogRead(sensorPin);
    
    // map the value for the leds bargraph.
    ledsRange = map(ledsRange, sensorMin, sensorMax, -1, NUM_LEDS-1);

    // light or dark the appropriate leds from the bargraph.
    for (int thisLed = 0; thisLed < NUM_LEDS; thisLed++) {
      if (thisLed <= ledsRange)
        // light the led.
        changeLED (thisLed, HIGH);
      else
        // dark the led.
        changeLED (thisLed, LOW);
    }

    // if the bargraph leds is completed play a melody.
    if(ledsRange >= ledsThreshold)
      // trigger the system melody.
      playMelody ();

    delay(SENSOR_DELAY);
  }
}

// ISR for the button "on" IRQ (is called on button presses).
void
buttonOnISR() {
  // it ignores presses intervals less than the bounce time.
  if (abs(millis() - bounceTime) > BOUNCE_DURATION) {
    // on the system device.
    systemState = true;

    // first light the led.
    digitalWrite(ledPin, HIGH);
    
    // set whatever bounce time in ms is appropriate.
    bounceTime = millis(); 
  }
}

// ISR for the button "off" IRQ (is called on button presses).
void
buttonOffISR() {
  // it ignores presses intervals less than the bounce time.
  if (abs(millis() - bounceTime) > BOUNCE_DURATION) {
    // off the system device.
    systemState = false;

    // dark all system leds.
    darkAllLeds();
    
    // set whatever bounce time in ms is appropriate.
    bounceTime = millis(); 

    // go to the main loop and start again.
    longjmp(buf, 0);
  }
}

// dark all system leds.
void
darkAllLeds() {
  // dark the led and do nothing.
  digitalWrite(ledPin, LOW);

  // dark all the bargraph leds.
  for (uint8_t i = 0; i < NUM_LEDS; i++)
    changeLED (i, LOW);
}

// play a melody and return immediately.
void
playMelody() {
  // iterate over the notes of the melody.
  for (uint8_t thisNote = 0; thisNote < NUM_NOTES; thisNote++) {
    // to calculate the note duration, take one second divided by the note type.
    // e.g. quarter note = 1000/4, eighth note = 1000/8, etc.
    uint16_t noteDuration = 1000 / noteDurations[thisNote];

    // play the tone.
    tone(piezoPin, notesMelody[thisNote], noteDuration);

    // to distinguish notes, set a minimum time between them.
    // the note's duration plus 30% seems to work well enough.
    uint16_t pauseBetweenNotes = noteDuration * 1.30;

    // delay some time.
    delay(pauseBetweenNotes);
  }
}

// sends the LED states set in ledStates to the 74HC595 sequence.
void
updateLEDs(int value) {
  // pulls the chips latch low.
  digitalWrite (latch, LOW);

  // shifts out the 8 bits to the shift register.
  shiftOut (data, clock, MSBFIRST, value);

  // pulls the latch high displaying the data.
  digitalWrite (latch, HIGH);
}

// changes an individual LED.
void
changeLED(int led, int state) {
  // clears ledState of the bit we are addressing.
  ledState = ledState & masks[led];

  // if the bit is on we will add it to ledState.
  if (state == HIGH) {
    ledState = ledState | bits[led];
  }

  // send the new LED state to the shift register.
  updateLEDs(ledState);
}
