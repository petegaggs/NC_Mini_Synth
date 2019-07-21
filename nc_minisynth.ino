/*
 * Note this is work in progress, not finished yet
 * Numerically Controlled Mini-Synth, by Peter Gaggs
 * Based on the midi controlled oscillator project, this version is intended to 
 * be used in a self contained mini-synth design
 * 16 bit timer is used to generate a square wave at the required frequency
 * this is used to reset the integrator (ramp generator)
 * runs on arduino 328p
 * note we use the timer directly, hence only use on 328p or 168p
 * needs arduino MIDI library
 * Also features Noise souce, LFO (not done yet)
 * 
 * MIT License
 * Copyright (c) 2019 petegaggs
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <MIDI.h>
#include <midi_Defs.h>
#include <midi_Message.h>
#include <midi_Namespace.h>
#include <midi_Settings.h> 
MIDI_CREATE_DEFAULT_INSTANCE();
#include <avr/io.h> 
#include <avr/interrupt.h>
#define TIMER_PIN 9 //OC1A output
#define INTEGRATOR_PWM_PIN 3 //integrator drive by timer2 PWM OC2B, for mk2 h/w
#define GATE_PIN 8 //gate control
#define NOISE_PIN 7 //psuedo random noise output
#define MIDI_BASE_NOTE 21 //lowest midi note
#define BASE_NOTE_FREQ 27.5
#define VOLTS_PER_SEMITONE 1.0 / 12.0
#define PWM_DAC_MULTIPLIER 31.312 // note this gives a 16 bit value for pwm
#define PITCH_BEND_FACTOR 1.0 / 32768.0; //adjust for desired pitch bend operation
//MIDI variables
int currentMidiNote; //the note currently being played
int keysPressedArray[128] = {0}; //to keep track of which keys are pressed
float midiControlVoltage; // represents midi note
uint16_t pwmVal; //value for PWM with 16 bit precision, dither will be used to get average value close to this
float bendControlVoltage = 0; // represents midi pitch bend
uint32_t lfsr = 1; //32 bit LFSR, must be non-zero to start

void setTimer1(uint16_t val) {
  OCR1AH = val >> 8;
  OCR1AL = val;
}

void setup() {
  //MIDI stuff
  // Initiate MIDI communications, listen to all channels
  MIDI.begin(MIDI_CHANNEL_OMNI);      
  // Connect the HandleNoteOn function to the library, so it is called upon reception of a NoteOn.
  MIDI.setHandleNoteOn(handleNoteOn);  // Put only the name of the function
  // Do the same for NoteOffs
  MIDI.setHandleNoteOff(handleNoteOff);
  // and also pitchbend, we can do that now we calculate the value on the fly
  MIDI.setHandlePitchBend(handlePitchBend); 
  pinMode(TIMER_PIN,OUTPUT);// OC1A output
  pinMode(GATE_PIN,OUTPUT);
  pinMode(NOISE_PIN,OUTPUT);
  digitalWrite(GATE_PIN,LOW);
  // timer 1 fast PWM mode 15, x8 prescaling
  TCCR1A = _BV(COM1A0) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
 
  pinMode(INTEGRATOR_PWM_PIN, OUTPUT); // OC2B output
  // timer 2 phase accurate PWM, no prescaling, non inverting mode channel B
  TCCR2A = _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  // timer 2 interrupts (used for noise)
  TIMSK2 = _BV(TOIE2);

  setNotePitch(60); //middle C for test
}

void loop() {
  MIDI.read();
}

SIGNAL(TIMER2_OVF_vect) {
  // noise output on digital pin 7 = PD7
  unsigned lsb = lfsr & 1;
  if (lsb) {
    PORTD |= 0x80;
  }
  else {
    PORTD &= ~0x80;
  }
  // advance LFSR
  lfsr >>= 1;
  if (lsb) {
    lfsr ^= 0xA3000000u; // 32 bit version
  }
  // set PWM using dither for better precision with low notes
  uint8_t pwmSet = pwmVal >> 8; // whole part
  uint8_t ditherByte = lfsr & 0xFF; // random dither
  uint8_t pwmFrac = pwmVal & 0xFF; // fractional part of 16 bit value
  if (pwmFrac > ditherByte) {
    pwmSet += 1; // increase, ensure no overflow
  }
  OCR2B = pwmSet; // write to pwm register
}

void handleNoteOn(byte channel, byte pitch, byte velocity) { 
  // this function is called automatically when a note on message is received 
  keysPressedArray[pitch] = 1;
  synthNoteOn(pitch);
}

void handleNoteOff(byte channel, byte pitch, byte velocity)
{
  keysPressedArray[pitch] = 0; //update the array holding the keys pressed 
  if (pitch == currentMidiNote) {
    //only act if the note released is the one currently playing, otherwise ignore it
    int highestKeyPressed = findHighestKeyPressed(); //search the array to find the highest key pressed, will return -1 if no keys pressed
    if (highestKeyPressed != -1) { 
      //there is another key pressed somewhere, so the note off becomes a note on for the highest note pressed
      synthNoteOn(highestKeyPressed);
    }    
    else  {
      //there are no other keys pressed so proper note off
      synthNoteOff();
    }
  }  
}

void handlePitchBend (byte channel, int bend) {
  // respond to MIDI pitch bend messages
  bendControlVoltage = (float)bend * PITCH_BEND_FACTOR;
  updateNotePitch();
}

void updateNotePitch() {
  // update note pitch, taking into account midi note, midi pitchbend, analogue control
  float controlVoltage = midiControlVoltage + bendControlVoltage;
  float freqHz = BASE_NOTE_FREQ * pow(2.0, controlVoltage);  
  uint16_t timerSetting = round((1000000.0 / freqHz)-1.0);
  setTimer1(timerSetting);
  pwmVal = int(PWM_DAC_MULTIPLIER * freqHz) - 256;
}

void setNotePitch(int note) {
  //receive a midi note number and set both integrator drive and timer accordingly
  midiControlVoltage = ((float)(note - MIDI_BASE_NOTE) * VOLTS_PER_SEMITONE);
  updateNotePitch();
}

int findHighestKeyPressed(void) {
  //search the array to find the highest key pressed. Return -1 if no keys are pressed
  int highestKeyPressed = -1; 
  for (int count = 0; count < 127; count++) {
    //go through the array holding the keys pressed to find which is the highest (highest note has priority), and to find out if no keys are pressed
    if (keysPressedArray[count] == 1) {
      highestKeyPressed = count; //find the highest one
    }
  }
  return(highestKeyPressed);
}

void synthNoteOn(int note) {
  //starts playback of a note
  setNotePitch(note); //set the oscillator pitch
  digitalWrite(GATE_PIN,HIGH); //turn gate on
  currentMidiNote = note; //store the current note
}

void synthNoteOff(void) {
  digitalWrite(GATE_PIN,LOW); //turn gate off
}


