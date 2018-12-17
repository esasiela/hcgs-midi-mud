/*
 * hc-midi-mud-firmware
 * 
 * This is the Arduino Nano firmware for the Hedge Court MIDI Mud step sequencer.  The device is a MIDI controller stepping through 8 beats and two instruments.
 * User specifies what beats each instrument is to play on using buttons (simultaneously press the button for the instrument and the beat).
 * Tempo is controlled by potentiometer.
 * 
 * 24 LEDs, organized as 3 groups of 8
 *  - Group 0 is the beat indicator, only one lights at a time
 *  - Group 1 is instrument 1, shows what beats will play this instrument
 *  - Group 2 is instrument 2, shows what beats will play this instrument
 *  
 * Group of 8 buttons (beat selectors) and group of 2 buttons (instrument selectors)
 * - Hold down the instrument selector, and press the beat selectors to toggle play/no-play of the selected instrument on the selected beat.
 * 
 * Potentiometer 1 specifies the tempo
 * Potentiometer 2 is unused as of this writing
 * 
 * Group of 2 buttons is unused as of this writing
 * 
 * Pin Usage:
 * 
 * 2,3,4 - LED column register (serial out)
 * 5,6,7 - LED row transistors
 * 8,9 - MIDI software serial
 * 10,11,12 - Beat Select buttons (serial in)
 * 13 - Button 0 (select instrument 1)
 * A0 - Button 1 (select instrument 2)
 * A1 - Button 2 (unassigned function)
 * A2 - Button 3 (unassigned function)
 * A3 - unused
 * A4 - unused
 * A5 - unused
 * A6 - Potentiometer 1 (tempo)
 * A7 - Potentiometer 2 (unassigned function)
 */




// libraries

#include <SoftwareSerial.h>

// We will use the SoftwareSerial library instead of the Serial library, as this will let us control which pins our MIDI interface is connected to.
SoftwareSerial mySerial(8, 9); // RX, TX

#define PIN_LED_1 7
#define PIN_LED_2 6

#define PIN_LED_REG_DATA 4
#define PIN_LED_REG_CLOCK 3
#define PIN_LED_REG_LATCH 2

// constants
const byte midiNoteOn = 144;
const byte midiNoteOff = 128;

const int midiSendDelay = 100; // give MIDI-device a short time to "digest" MIDI messages

// note lengths
const int bpm = 128; // tempo, BPM (beats per minute), a value that makes t16 an integer
const int t1 = 1024 * (bpm/64); // 1 whole beat = 512
const int t2 = t1/2; // 256
const int t4 = t2/2; // 128
const int t8 = t4/2; // 64
const int t16 = t8/2; // 32

const int voiceMidiChannel = 0;
const int voiceMidiPatch = 86;
const int voiceMidiVolume = 80;
const int voiceMidiPan = 100;
const int midiVelocity = 100;

//int notes[] = {36, 36, 53, 55 };
//int notes[] = {36, 36, 36, 55, 36, 36, 36, 55 };
int notes1[] = {0, 0, 0, 40, 0, 0, 0, 40 };
int notes2[] = {36, 36, 36, 36, 36, 36, 36, 36 };

//byte lightsR[] = {0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F};
byte lightsR[] = {0xBF, 0xDF, 0xEF, 0xC7, 0xF7, 0xFB, 0xFD, 0xFC};

//byte lightsG[] = {0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F };
byte lightsG[] = {0xFE, 0xFD, 0xFB, 0xE3, 0xEF, 0xDF, 0xBF, 0x3F };

boolean evenLoop=true;


#define NOTE_COUNT 8
// initialize to NOTE_COUNT, first loop will reset to zero
byte noteIndex = NOTE_COUNT;

unsigned long noteTimestamp=0;

boolean notePlaying1=false;
boolean notePlaying2=false;

//  Play a MIDI note

void midiNote(int aMidiCommand, int aMidiPitch, int aMidiVelocity) {
  mySerial.write(aMidiCommand);
  mySerial.write(aMidiPitch);
  mySerial.write(aMidiVelocity);
}

// Send MIDI command with 1 data byte

void midiData1(int aMidiCommand, int aData1) {
  mySerial.write(aMidiCommand);
  mySerial.write(aData1);
}

// Send MIDI command with 2 data bytes

void midiData2(int aMidiCommand, int aData1, int aData2) {
  mySerial.write(aMidiCommand);
  mySerial.write(aData1);
  mySerial.write(aData2);
}

void updateDisplay() {
    // disable both rows
    digitalWrite(PIN_LED_1, HIGH);
    digitalWrite(PIN_LED_2, HIGH);
  
    // write the new strand data to the register
    digitalWrite(PIN_LED_REG_LATCH, LOW);
    shiftOut(PIN_LED_REG_DATA, PIN_LED_REG_CLOCK, MSBFIRST, lightsG[noteIndex]);
    digitalWrite(PIN_LED_REG_LATCH, HIGH);

    // enable row 1
    digitalWrite(PIN_LED_1, LOW);
    digitalWrite(PIN_LED_2, HIGH);
    // different delay for Green than for Red
    delay(2);

    // disable both rows
    digitalWrite(PIN_LED_1, HIGH);
    digitalWrite(PIN_LED_2, HIGH);

    // write the new strand data to the register
    digitalWrite(PIN_LED_REG_LATCH, LOW);
    shiftOut(PIN_LED_REG_DATA, PIN_LED_REG_CLOCK, MSBFIRST, lightsR[noteIndex]);
    digitalWrite(PIN_LED_REG_LATCH, HIGH);

    // enable row 2
    digitalWrite(PIN_LED_1, HIGH);
    digitalWrite(PIN_LED_2, LOW);
    // different delay for Green than for Red
    delay(1);

    // disable both rows
    digitalWrite(PIN_LED_1, HIGH);
    digitalWrite(PIN_LED_2, HIGH);
}


void setup() {

  // configure and disable LED row drivers
  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  digitalWrite(PIN_LED_1, HIGH);
  digitalWrite(PIN_LED_2, HIGH);


  // setup SoftSerial for MIDI control
  mySerial.begin(31250);
  delay(midiSendDelay);


  // configure LED column driver register
  pinMode(PIN_LED_REG_DATA, OUTPUT);
  pinMode(PIN_LED_REG_CLOCK, OUTPUT);
  pinMode(PIN_LED_REG_LATCH, OUTPUT);


  // volume
  midiData2((0xB0 | voiceMidiChannel), 07, voiceMidiVolume);
  delay(midiSendDelay);

  // sound/patch
  midiData1((0xC0 | voiceMidiChannel), voiceMidiPatch);
  delay(midiSendDelay);

  // pan
  midiData2((0xB0 | voiceMidiChannel), 10, voiceMidiPan);
  delay(midiSendDelay);

}



void loop() {
  if ((millis()-noteTimestamp)>t8) {
    // it is time to play the next note
    noteTimestamp = millis();

    // if we are playng a note, turn it off
    if (notePlaying1) {
      midiNote(midiNoteOff + voiceMidiChannel, notes1[noteIndex], midiVelocity);
    }
    if (notePlaying2) {
      midiNote(midiNoteOff + voiceMidiChannel, notes2[noteIndex], midiVelocity);
    }

    noteIndex++;
    if (noteIndex>=NOTE_COUNT) {
      noteIndex=0;
    }

    // turn the next note on
    if (notes1[noteIndex]>0) {
      notePlaying1=true;
      midiNote(midiNoteOn + voiceMidiChannel, notes1[noteIndex], midiVelocity);
    }
    if (notes2[noteIndex]>0) {
      notePlaying2=true;
      midiNote(midiNoteOn + voiceMidiChannel, notes2[noteIndex], midiVelocity);
    }
  }

  // every time through the loop, update the LED display for all columns
  updateDisplay();

/*  
  for (int i=0; i<8; i++) {

    if (notes1[i]!=0) {
      midiNote(midiNoteOn + voiceMidiChannel, notes1[i], midiVelocity);
    }
    if (notes2[i]!=0) {
      midiNote(midiNoteOn + voiceMidiChannel, notes2[i], midiVelocity);      
    }

    delay(t8);

    if (notes1[i]!=0) {
      midiNote(midiNoteOff + voiceMidiChannel, notes1[i], midiVelocity);
    }
    if (notes2[i]!=0) {
      midiNote(midiNoteOff + voiceMidiChannel, notes2[i], midiVelocity);      
    }

    
  }

  evenLoop = !evenLoop;
  */


/*  
  // 48
  midiNote(midiNoteOn + voiceMidiChannel, 36, midiVelocity);
  delay(t8);
  midiNote(midiNoteOff + voiceMidiChannel, 36, midiVelocity); 

  //51
  midiNote(midiNoteOn + voiceMidiChannel, 36, midiVelocity);
  delay(t8);
  midiNote(midiNoteOff + voiceMidiChannel, 36, midiVelocity); 
  
  midiNote(midiNoteOn + voiceMidiChannel, 53, midiVelocity);
  delay(t8);
  midiNote(midiNoteOff + voiceMidiChannel, 53, midiVelocity); 

  midiNote(midiNoteOn + voiceMidiChannel, 55, midiVelocity);
  delay(t8);
  midiNote(midiNoteOff + voiceMidiChannel, 55, midiVelocity); 
*/
}
