/*
 * Numerically Controlled Mini-Synth, by Peter Gaggs
 * Based on the midi controlled oscillator project, this version is intended to 
 * be used in a self contained mini-synth design
 * 16 bit timer is used to generate a square wave at the required frequency
 * this is used to reset the integrator (ramp generator)
 * runs on arduino 328p
 * note we use the timer directly, hence only use on 328p or 168p
 * needs arduino MIDI library
 * Also features Noise souce, LFO with 5 wave types, others to follow
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
#define INTEGRATOR_PWM_PIN 3 //integrator drive by timer2 PWM OC2B
#define INTEGRATOR_PWM OCR2B
#define LFO_PWM_PIN 11 //LFO by timer2 PWM OC2A
#define LFO_PWM OCR2A
#define LFO_FREQ_PIN A1
#define LFO_WAVE_PIN A0
#define GATE_HIGH_PIN 8 //gate control, uses separate pins for attack/decay control
#define GATE_LOW_PIN 10
#define NOISE_PIN 7 //psuedo random noise output
#define MIDI_BASE_NOTE 21 //lowest midi note
#define BASE_NOTE_FREQ 27.5
#define VOLTS_PER_SEMITONE 1.0 / 12.0
#define PWM_DAC_MULTIPLIER 31.312 // note this gives a 16 bit value for pwm
#define PWM_BODGE 350 // why is this needed?
#define PITCH_BEND_FACTOR 1.0 / 32768.0; //adjust for desired pitch bend operation
//MIDI variables
int currentMidiNote; //the note currently being played
int keysPressedArray[128] = {0}; //to keep track of which keys are pressed
float midiControlVoltage; // represents midi note
uint16_t pwmVal; //value for PWM with 16 bit precision, dither will be used to get average value close to this
float bendControlVoltage = 0; // represents midi pitch bend
uint32_t lfsr = 1; //32 bit LFSR, must be non-zero to start

// table of 256 sine values / one sine period / stored in flash memory
const char sineTable[] PROGMEM = {
  127,130,133,136,139,143,146,149,152,155,158,161,164,167,170,173,176,178,181,184,187,190,192,195,198,200,203,205,208,210,212,215,217,219,221,223,225,227,229,231,233,234,236,238,239,240,
  242,243,244,245,247,248,249,249,250,251,252,252,253,253,253,254,254,254,254,254,254,254,253,253,253,252,252,251,250,249,249,248,247,245,244,243,242,240,239,238,236,234,233,231,229,227,225,223,
  221,219,217,215,212,210,208,205,203,200,198,195,192,190,187,184,181,178,176,173,170,167,164,161,158,155,152,149,146,143,139,136,133,130,127,124,121,118,115,111,108,105,102,99,96,93,90,87,84,81,78,
  76,73,70,67,64,62,59,56,54,51,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,16,15,14,12,11,10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,16,18,20,21,23,25,27,29,31,
  33,35,37,39,42,44,46,49,51,54,56,59,62,64,67,70,73,76,78,81,84,87,90,93,96,99,102,105,108,111,115,118,121,124
};

// LFO stuff
uint32_t lfoPhaccu;   // phase accumulator
uint32_t lfoTword_m;  // dds tuning word m
uint8_t lfoCnt;      // top 8 bits of accum is index into table

float lfoControlVoltage;
enum lfoWaveTypes {
  RAMP,
  SAW,
  TRI,
  SINE,
  SQR
};
lfoWaveTypes lfoWaveform;
bool lfoSyncMode = false;
bool lfoReset = false;
void setTimer1(uint16_t val) {
  OCR1AH = val >> 8;
  OCR1AL = val;
}

void setGate(bool onOff) {
  // set gate on/off
  //GATE_HIGH pin is driven high for note on, and tri-state (set to input) for note off
  //GATE_LOW pin is tri-state for note on and driven low for not off
  if (onOff) {
    // note on
    digitalWrite(GATE_HIGH_PIN, HIGH);
    pinMode(GATE_LOW_PIN, INPUT); // tri-state
    pinMode(GATE_HIGH_PIN, OUTPUT);
  } else {
    // note off
    digitalWrite(GATE_LOW_PIN, LOW);
    pinMode(GATE_HIGH_PIN, INPUT); // tri-state
    digitalWrite(GATE_HIGH_PIN, LOW); //ensure internal pull up is off
    pinMode(GATE_LOW_PIN, OUTPUT);
  }
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
  pinMode(NOISE_PIN,OUTPUT);
  // timer 1 fast PWM mode 15, x8 prescaling
  TCCR1A = _BV(COM1A0) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
 
  pinMode(INTEGRATOR_PWM_PIN, OUTPUT); // OC2B output
  pinMode(LFO_PWM_PIN, OUTPUT); //OC2A output
  // timer 2 phase accurate PWM, no prescaling, non inverting mode channel A+B
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  // timer 2 interrupts (used for noise)
  TIMSK2 = _BV(TOIE2);

  setNotePitch(60); //middle C for test
  setGate(false); //start with gate off
}

void getLfoParams() {
  // read ADC to calculate the required DDS tuning word, log scale between 0.01Hz and 10Hz approx
  float lfoControlVoltage = float(analogRead(LFO_FREQ_PIN)) * float(10)/float(1024); //gives 10 octaves range 0.01Hz to 10Hz
  lfoTword_m = float(1369) * pow(2.0, lfoControlVoltage); //1369 sets the lowest frequency to 0.01Hz
  // read ADC to get the LFO wave type
  uint8_t adcVal = (analogRead(LFO_WAVE_PIN) >> 7) & 0x7; //top 3 bits of 10 bit value
  switch (adcVal) {
    case 0:
      lfoWaveform = RAMP;
      break;
    case 1:
      lfoWaveform = SAW;
      break;
    case 2:
      lfoWaveform = TRI;
      break;
    case 3:
      lfoWaveform = SINE;
      break;
    case 4:
      lfoWaveform = SQR;
      break;
    case 5:
      lfoWaveform = RAMP; // reserved for future use
      break;
    case 6:
      lfoWaveform = SAW; // reserved for future use
      break;
    case 7:
      lfoWaveform = TRI; // reserved for future use
      break;
    default:
      lfoWaveform = RAMP;
      break;
  }
}

void loop() {
  MIDI.read();
  getLfoParams();
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
  INTEGRATOR_PWM = pwmSet; // write to pwm register
  // handle LFO DDS
  if (lfoReset) {
    lfoPhaccu = 0; // reset the lfo
    lfoReset = false;
  } else {
    lfoPhaccu += lfoTword_m; // increment phase accumulator  
  }
  lfoCnt = lfoPhaccu >> 24;  // use upper 8 bits for phase accu as frequency information
  switch (lfoWaveform) {
    case RAMP:
      LFO_PWM = lfoCnt;
      break;
    case SAW:
      LFO_PWM = 255 - lfoCnt;
      break;
    case TRI:
      if (lfoCnt & 0x80) {
        LFO_PWM = 254 - ((lfoCnt & 0x7F) << 1); //ramp down
      } else {
        LFO_PWM = lfoCnt << 1; //ramp up
      }
      break;
    case SINE:
      // sine wave from table
      LFO_PWM = pgm_read_byte_near(sineTable + lfoCnt);
      break;
    case SQR:
      if (lfoCnt & 0x80) {
        LFO_PWM = 255;
      } else {
        LFO_PWM = 0;
      }
      break;
    default:
      break;
  }
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
  pwmVal = int(PWM_DAC_MULTIPLIER * freqHz) - PWM_BODGE;
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
  setGate(true); // turn gate on
  currentMidiNote = note; //store the current note
}

void synthNoteOff(void) {
  setGate(false);
}


