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
 * 13 *** UNUSED, see A3 - manually adjusted circuit to move from 13 to A3, no input pullup on 13
 * A0 - Button 1 (select instrument 2)
 * A1 - Button 2 (unassigned function)
 * A2 - Button 3 (unassigned function)
 * A3 - Button 0 (select instrument 1)
 * A4 - unused
 * A5 - unused
 * A6 - Potentiometer 1 (tempo)
 * A7 - Potentiometer 2 (unassigned function)
 */

#include <SoftwareSerial.h>
#include <HC_BouncyButton.h>

#define PIN_POT_TEMPO A6
#define PIN_POT_2 A7

#define PIN_LED_COL_DATA 4  // SER
#define PIN_LED_COL_CLOCK 2 // RCK (RCLK)
#define PIN_LED_COL_LATCH 3 // SCK (SRCLK)

#define PIN_LED_ROW_R 5
#define PIN_LED_ROW_B 6
#define PIN_LED_ROW_G 7

#define PIN_BTN_R A3 // PCB maps to pin 13, but useless internal pullup on 13, so handsoldered to A3
#define PIN_BTN_B A0
#define PIN_BTN_G A1
#define PIN_BTN_Z A2

#define PIN_BEAT_DATA 10
#define PIN_BEAT_CLOCK 12
#define PIN_BEAT_LATCH 11

#define PIN_MIDI_OUT_RX 8
#define PIN_MIDI_OUT_TX 9

SoftwareSerial midiSerial(PIN_MIDI_OUT_RX, PIN_MIDI_OUT_TX);

BouncyButton btnR = BouncyButton(PIN_BTN_R);
BouncyButton btnB = BouncyButton(PIN_BTN_B);
BouncyButton btnG = BouncyButton(PIN_BTN_G);
BouncyButton btnZ = BouncyButton(PIN_BTN_Z);


// true on any change (press or release)
boolean isBeatRegisterChange=false;
// only true for press, not for release
boolean isBeatButtonPress=false;

byte bouncingCurrentBeatRegister;
byte bouncingPreviousBeatRegister;
byte beatRegister;
byte previousBeatRegister;

unsigned long lastDebounceTime;
#define DEBOUNCE_DELAY_MILLIS 13

// use nowMillis when possible, avoids a million calls to millis() in loop()
unsigned long nowMillis;

unsigned long noteTimestamp;
boolean notePlaying1=false;
boolean notePlaying2=false;

int notes1[] = {40, 40, 40, 40, 40, 40, 40, 40 };
int notes2[] = {36, 36, 36, 36, 36, 36, 36, 36 };

// stores the bit pattern for for the red LED's indicating the beat
byte redBits;

// register for the beats on/off for the instrument
byte instrument1=0xff, instrument2=0xff;


#define NOTE_COUNT 8
// initialize to NOTE_COUNT, first loop will reset to zero
byte noteIndex = NOTE_COUNT;


// constants
const byte MIDI_NOTE_ON = 144;
const byte MIDI_NOTE_OFF = 128;

// was 100, seems lengthy, eh?
const int MIDI_SEND_DELAY = 1; // give MIDI-device a short time to "digest" MIDI messages

// note lengths (bpm=128)
int bpm = 128; // tempo, BPM (beats per minute)
int t8 = 60000/bpm; // fractional part of millisec will be truncated, some ears might be accurate enough to detect?

const int voiceMidiChannel = 0;
const int voiceMidiPatch = 86;
const int voiceMidiVolume = 80;
const int voiceMidiPan = 100;
const int midiVelocity = 100;


// newTempo expected in range 0-1023 (i.e. analogRead)
void updateTempo(int newTempo) {
  // map tempo value to BPM, then generate the rest
  bpm = map(newTempo, 0, 1023, 10, 1023);
  t8 = 60000/bpm; // fractional part of millisec will be truncated, some ears might be accurate enough to detect?
  /*
  Serial.print("updateTempo(");
  Serial.print(newTempo);
  Serial.print(") bpm=");
  Serial.print(bpm);
  Serial.print(" t8=");
  Serial.print(t8);
  Serial.println("");
  */

}


//  Play a MIDI note
void midiNote(int aMidiCommand, int aMidiPitch, int aMidiVelocity) {
  midiSerial.write(aMidiCommand);
  midiSerial.write(aMidiPitch);
  midiSerial.write(aMidiVelocity);
}

// Send MIDI command with 1 data byte
void midiData1(int aMidiCommand, int aData1) {
  midiSerial.write(aMidiCommand);
  midiSerial.write(aData1);
}

// Send MIDI command with 2 data bytes
void midiData2(int aMidiCommand, int aData1, int aData2) {
  midiSerial.write(aMidiCommand);
  midiSerial.write(aData1);
  midiSerial.write(aData2);
}



void updateDisplay() {

  /*
   * RED
   */
  
  // disable all rows
  digitalWrite(PIN_LED_ROW_R, HIGH);
  digitalWrite(PIN_LED_ROW_B, HIGH);
  digitalWrite(PIN_LED_ROW_G, HIGH);

  // write the columns for the red row
  digitalWrite(PIN_LED_COL_LATCH, LOW);


  switch (noteIndex) {
    case 0:
      redBits = B01111111;
      break;
    case 1:
      redBits = B10111111;
      break;
    case 2:
      redBits = B11011111;
      break;
    case 3:
      redBits = B11101111;
      break;
    case 4:
      redBits = B11110111;
      break;
    case 5:
      redBits = B11111011;
      break;
    case 6:
      redBits = B11111101;
      break;
    case 7:
      redBits = B11111110;
      break;
    default :
      // all off at boot
      redBits = B11111111;
      break;
  }


  
  shiftOut(PIN_LED_COL_DATA, PIN_LED_COL_CLOCK, MSBFIRST, redBits);
  digitalWrite(PIN_LED_COL_LATCH, HIGH);

  // enable red for a bit
  digitalWrite(PIN_LED_ROW_R, LOW);
  delayMicroseconds(80);
  digitalWrite(PIN_LED_ROW_R, HIGH);
  
  /*
   * BLUE
   */
  digitalWrite(PIN_LED_COL_LATCH, LOW);
  shiftOut(PIN_LED_COL_DATA, PIN_LED_COL_CLOCK, MSBFIRST, instrument1);
  digitalWrite(PIN_LED_COL_LATCH, HIGH);

  // enable blue for a bit
  if (instrument1==0xFF) {
    // all bits off, dont even show it
    delayMicroseconds(40);
  } else {
    digitalWrite(PIN_LED_ROW_B, LOW);
    delayMicroseconds(40);
    digitalWrite(PIN_LED_ROW_B, HIGH);
  }
  
  /*
   * GREEN
   */
  digitalWrite(PIN_LED_COL_LATCH, LOW);
  shiftOut(PIN_LED_COL_DATA, PIN_LED_COL_CLOCK, MSBFIRST, instrument2);
  digitalWrite(PIN_LED_COL_LATCH, HIGH);

  // enable green for a bit
  if (instrument2==0xFF) {
    // all bits off, dont even show it
    delayMicroseconds(160);
  } else {
    digitalWrite(PIN_LED_ROW_G, LOW);
    delayMicroseconds(160);
    digitalWrite(PIN_LED_ROW_G, HIGH);
  }
}



void setup() {
  // LED row transistor pins (HIGH to shut them off at boot)
  pinMode(PIN_LED_ROW_R, OUTPUT);
  digitalWrite(PIN_LED_ROW_R, HIGH);
  
  pinMode(PIN_LED_ROW_B, OUTPUT);
  digitalWrite(PIN_LED_ROW_B, HIGH);
  
  pinMode(PIN_LED_ROW_G, OUTPUT);
  digitalWrite(PIN_LED_ROW_G, HIGH);


  
  Serial.begin(115200);
  delay(10);
  Serial.println("HC MIDI Mud");

  // initialize MIDI standard baud rate 31250
  midiSerial.begin(31250);
  delay(MIDI_SEND_DELAY);
  

  // LED column shift register pins
  pinMode(PIN_LED_COL_DATA, OUTPUT);
  pinMode(PIN_LED_COL_CLOCK, OUTPUT);
  pinMode(PIN_LED_COL_LATCH, OUTPUT);  


  // 4 buttons on the right
  pinMode(PIN_BTN_R, INPUT_PULLUP);
  pinMode(PIN_BTN_B, INPUT_PULLUP);
  pinMode(PIN_BTN_G, INPUT_PULLUP);
  pinMode(PIN_BTN_Z, INPUT_PULLUP);

  btnR.init();
  btnB.init();
  btnG.init();
  btnZ.init();


  // beat button multiplexer
  pinMode(PIN_BEAT_DATA, INPUT);
  pinMode(PIN_BEAT_CLOCK, OUTPUT);
  pinMode(PIN_BEAT_LATCH, OUTPUT);

  digitalWrite(PIN_BEAT_LATCH, LOW);
  digitalWrite(PIN_BEAT_CLOCK, HIGH);


  updateDisplay();


  // volume
  midiData2((0xB0 | voiceMidiChannel), 07, voiceMidiVolume);
  delay(MIDI_SEND_DELAY);

  // sound/patch
  midiData1((0xC0 | voiceMidiChannel), voiceMidiPatch);
  delay(MIDI_SEND_DELAY);

  // pan
  midiData2((0xB0 | voiceMidiChannel), 10, voiceMidiPan);
  delay(MIDI_SEND_DELAY);


}



void loop() {

  nowMillis = millis();

  // PIN_POT_TEMPO / A6 is wired backwards, so subtract from 1023
  updateTempo(1023-analogRead(PIN_POT_TEMPO));
  
  if ((nowMillis-noteTimestamp)>t8) {
    // time to play the next note
    noteTimestamp = nowMillis;

    // if we are playng a note, turn it off
    if (notePlaying1) {
      midiNote(MIDI_NOTE_OFF + voiceMidiChannel, notes1[noteIndex], midiVelocity);
      notePlaying1=false;
    }
    if (notePlaying2) {
      midiNote(MIDI_NOTE_OFF + voiceMidiChannel, notes2[noteIndex], midiVelocity);
      notePlaying2=false;
    }

    noteIndex++;
    if (noteIndex>=NOTE_COUNT) {
      noteIndex=0;
    }

    // kind of a reverse-endian thing, my mind thinks of bit zero on the left, but math/everybody/computers think of bit zero on the right
    byte bitIndex = 7 - noteIndex;
    
    // turn the next note on
    if (!bit_is_set(instrument1, bitIndex)) {
      notePlaying1=true;
      midiNote(MIDI_NOTE_ON + voiceMidiChannel, notes1[noteIndex], midiVelocity);
    }

    if (!bit_is_set(instrument2, bitIndex)) {
      /*
      Serial.print("ins2 noteOn noteIndex=");
      Serial.print(noteIndex);
      Serial.print(" bitIndex=");
      Serial.print(bitIndex);
      Serial.print(" ins2=");
      printBinaryByte(instrument2);
      Serial.print(" redBits=");
      printBinaryByte(redBits);
      Serial.println("");
      */
      notePlaying2=true;
      midiNote(MIDI_NOTE_ON + voiceMidiChannel, notes2[noteIndex], midiVelocity);
    }
  }


  // read the multiplexed buttons

  // latch HIGH and delay, allowing us to provide the LOW transition to begin reading
  digitalWrite(PIN_BEAT_LATCH, HIGH);
  digitalWrite(PIN_BEAT_CLOCK, HIGH);
  delayMicroseconds(20);
  // latch LOW tells the multiplexer to latch the button states so we can begin reading
  digitalWrite(PIN_BEAT_LATCH, LOW);

  isBeatRegisterChange=false;
  bouncingCurrentBeatRegister = shiftIn(PIN_BEAT_DATA, PIN_BEAT_CLOCK, LSBFIRST);

  if (bouncingCurrentBeatRegister != bouncingPreviousBeatRegister) {
    // something is bouncing
    lastDebounceTime = millis();
  }


  isBeatRegisterChange = bouncingCurrentBeatRegister!=beatRegister;

  if ((millis()-lastDebounceTime)> DEBOUNCE_DELAY_MILLIS && isBeatRegisterChange) {
    // bouncing is done and there is a state change, make it live
    previousBeatRegister = beatRegister;
    beatRegister = bouncingCurrentBeatRegister;

    // determine if it is a PRESS, we dont care about release
    for (byte bitIdx=0; bitIdx<8; bitIdx++) {
      boolean prevBit = previousBeatRegister & (1<<bitIdx);
      boolean curBit = beatRegister & (1<<bitIdx);

      // not equals means either press or relase....
      if (prevBit != curBit) {

        // we want only press eveents....curBit is LOW on a press event
        if (!curBit) {

          // toggle the value in this bit position in the instrumentN byte
          if (!btnB.getState()) {
            instrument1 ^= 1UL << bitIdx;
          }
          if (!btnG.getState()) {
            instrument2 ^= 1UL << bitIdx;
            //Serial.print("instrument2 changed - ");
            //printBinaryByte(instrument2);
            //Serial.println("");
          }
        }

        
      }
    }
  }

  // update the "previous" value for the next iteration
  bouncingPreviousBeatRegister = bouncingCurrentBeatRegister;
  

  

  if (btnR.update() && !btnR.getState()) {
    // make sure to call update() every loop(), even if "then" clause is blank
    //Serial.println(F("red down"));
  }

  if (btnB.update() && !btnB.getState()) {
    // make sure to call update() every loop(), even if "then" clause is blank
    //Serial.println(F("blue down"));
  }

  if (btnG.update() && !btnG.getState()) {
    // make sure to call update() every loop(), even if "then" clause is blank
    //Serial.println(F("green down"));
  }

  if (btnZ.update()) {
    // make sure to call update() every loop(), even if "then" clause is blank
    if (!btnZ.getState()) {
//      Serial.println(F("zed button update"));

    }
  }


  // when don't you update the display? same as when you don't eat the cat food.
  updateDisplay();
}

void printBinaryByte(byte b) {
  int x;
  for (x=8; x>0; x--) {
    if ((b>>(x-1))&1) {
      Serial.print("1");
    } else {
      Serial.print("0");
    }
  }
}
