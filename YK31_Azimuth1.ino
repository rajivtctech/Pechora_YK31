/*
   This C++ code is for the YK-31 azimuth capture unit
   comprising an ATMEGA328PB processor running at 16 MHz.
   The processor captures azimuth with a 1 degree resolution,
   from a magnetic encoder yielding 90 quadrature cycles per
   revolution, fully decoded to 360 pulses per revolution,
   plus one index pulse per revolution. All captured data
   is sent serially at 115200 bps to a Processing app on a PC.

   There is a 1:15 gearing ratio between the Missile Guidance
   radar (MGR)and the azimuth encoder. One revolution of the MGR
   equals 15 revolutions of the azimuth encoder.

   (c)2017-2019 Rajiv Tyagi
   (c)2017-2019 T&C Technology (India) Pvt. Ltd.
   ALL RIGHTS RESERVED

   This code is perpetually licensed to the Indian Air Force
   for use exclusively in the YK-31 Missile Guidance Radar Console
   of the Pechora Surface to Air Guided Missile system.
*/

#include <EEPROMex.h>
#include <EEPROMVar.h>
//#include <EEPROM.h>
#include<Encoder.h>
#include <PinChangeInt.h>
//#include <PinChangeIntConfig.h>
#include<VSync.h>
#include "Arduino.h"

ValueSender<2> sender;
#define indexIntPin 12
#define toggleSw 4
#define powerFailPin 5
#define ledPin 6
#define buttonPin 7
#define TSV 9
#define LR_Pin 10 //Long Range mode 

Encoder myEnc(2, 3);

volatile boolean isIndex = false;
volatile boolean rawIndex = true;
volatile boolean toggle;
volatile boolean powerFail;
volatile boolean buttonPressed = HIGH;
volatile int indexCount = 0;
boolean testBool = HIGH;
boolean isRawOffsetRecorded = true;
int beamPos;
int startPos;
int indexPos;
int counter = 0;
int addressBeamPos = 0; //EEPROM storing address for beamPos
int addressOffset = 10; //EEPROM storage address of offset
int rawOffset = 0;
int offset1 = 0;
int Mode;
int LR;

void setup() {
  pinMode(2, INPUT); // Encoder chanel 1
  pinMode(3, INPUT); // Encoder channel 2
  pinMode(toggleSw, INPUT_PULLUP); // toggle switch to change beam rotation direction
  pinMode(powerFailPin, INPUT); //when system turns off, save position
  pinMode(ledPin, OUTPUT); //LED
  //LED should blink when button is pressed
  pinMode(buttonPin, INPUT_PULLUP); //button
  pinMode(TSV, INPUT);
  pinMode(LR_Pin,INPUT);
  //to know antenna is at north
  attachPinChangeInterrupt(indexIntPin, indexISR, RISING);
  Serial.begin(115200);
  toggle = digitalRead(4);
  startPos = readIntEEPROM(addressBeamPos); // starPos is in degrees
  rawOffset = readIntEEPROM(addressOffset); //raw offset position
  myEnc.write(startPos * 15);
  beamPos = myEnc.read();
  sender.sync();

  sender.observe(beamPos);
  sender.observe(Mode);
  sender.observe(LR);
}

void loop()
{
  powerFail = digitalRead(5);
  digitalWrite(ledPin, HIGH);
  Mode = digitalRead(TSV);
  LR = digitalRead(LR_Pin); // Long Range Mode

  if(rawIndex){
    digitalWrite(ledPin, !testBool);
    rawIndex = false;
  }

  if (powerFail == LOW) {
    //save position
    writeIntEEPROM(addressBeamPos, myEnc.read());
    writeIntEEPROM(addressOffset, rawOffset);
    powerFail = HIGH;
  }

  //buttonpress
  if (digitalRead(7) == LOW) {
    delay(10);
    if (digitalRead(7) == LOW) {
      isRawOffsetRecorded = false; // Raw offset is offset not divided by 15
      myEnc.write(0);
      beamPos = 0;
      for (int i = 0; i <= 5; i++) {
        digitalWrite(ledPin, HIGH);
        delay(100);
        digitalWrite(ledPin, LOW);
        delay(100);
      }
    }

  }

  if (isIndex && !isRawOffsetRecorded) {
    rawOffset = myEnc.read();
    isRawOffsetRecorded = true;
    writeIntEEPROM(addressOffset, rawOffset);
    digitalWrite(ledPin, HIGH);
    isIndex = false;
  }

  if (isIndex){
    myEnc.write(0);
    beamPos = 0;
    isIndex = false;
  }

  if ((myEnc.read() / 15) == -1) {
    myEnc.write(359 * 15);
    //isIndex = false;
  }
  if ((myEnc.read() / 15) >= 360) {
    myEnc.write(0);    // Read the encoder while interrupts are enabled.
    //isIndex = false;
  }
  if (toggle == HIGH) {
    beamPos = abs((myEnc.read() / 15) - 360); // to rotate in opposite direction
  }
  if (toggle == LOW) {
    beamPos = myEnc.read() / 15;
  }
  //beamPos=beamPos/15;
  sender.sync();
}

void indexISR() {
  indexCount++;
  rawIndex = true;
  if (indexCount == 15) {
    isIndex = true;
    indexCount = 0; 
  }

}


int readIntEEPROM(int addressInt) {
  int data = EEPROM.readInt(addressInt);
  return data;
}

/********************************************************/

void writeIntEEPROM(int addressInt, int data) {
  EEPROM.writeInt(addressInt, data);
}
