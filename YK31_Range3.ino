/*
   This C++ code is for the YK-31 range capture unit
   comprising an ESP32 processor running at 240 MHz
   with an instruction cycle of 4.1666 nanoseconds.

   (c)2017-2019 Rajiv Tyagi
   (c)2017-2019 T&C Technology (India) Pvt. Ltd.
   ALL RIGHTS RESERVED

   This code is perpetually licensed to the Indian Air Force
   for use exclusively in the YK-31 Missile Guidance Radar Console
   of the Pechora Surface to Air Guided Missile system.
*/

#include<WiFi.h>
#include<VSync.h>
#include "esp32-hal-cpu.h"

#define RO1 15
#define RO2 2
#define RHT 4
#define RHIP 5
#define FB_NB 18
#define echo 19
#define led 21

unsigned long startCount;
unsigned long endCount;
unsigned long RHTCount;
unsigned long RHIPCount;
unsigned long NBCount;
unsigned long FBCount;
unsigned long range;
unsigned long echoCount;
unsigned long RHTTimeCount;
unsigned long RHIPTimeCount;
unsigned long NBTimeCount;
unsigned long FBTimeCount;



int echo1;
int echo2;
int echo3;
int echo0;
int FarBCount;
//int FarB1;
//int FarB2;
//int FarB3;
//int FarB0;
int NearBCount;
//int NearB1;
//int NearB2;
//int NearB3;
//int NearB0;
int RHTCount;
//int RHT1;
//int RHT2;
//int RHT3;
//int RHT0;
int RHIPCount;
//int RHIP1;
//int RHIP2;
//int RHIP3;
//int RHIP0;

byte b0;
byte b1;
byte b2;
byte b3;

unsigned long timeStart;
unsigned long timeNow;

volatile boolean getStartTime_search = false;
volatile boolean getStartTime_track = false;
volatile boolean getEchoTime = false;
volatile boolean getRHTTime = false;
volatile boolean getRHIPTime = false;
volatile boolean get_FB_NB = false;
volatile boolean sendDataToPC = false;
boolean ledStatus = false;

int Mode = 0; //0-tracking 1-search
int counter = 1;
int echo_true = 0;
int dataPeriod = 250; // milliseconds after which to update PC with data

volatile int interruptCounter;
int totalInterruptCounter;

//hw_timer_t * timer = NULL;
//portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
//portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

ValueSender<9> sender;
ValueReceiver<1> receiver;

void setup() {
  //WiFi.forceSleepBegin();
  WiFi.mode(WIFI_OFF); // Turn OFF WiFi
  btStop(); // Turn OFF Bluetooth
  delay(1);
  //system_update_cpu_freq(240);

  pinMode(RO1, INPUT);
  pinMode(RO2, INPUT);
  pinMode(echo, INPUT);
  pinMode(FB_NB, INPUT);
  pinMode(RHT, INPUT);
  pinMode(RHIP, INPUT);
  pinMode(led, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(RO1), ISR_sync_track, RISING);
  attachInterrupt(digitalPinToInterrupt(RO2), ISR_sync_search, RISING);
  attachInterrupt(digitalPinToInterrupt(echo), ISR_echo, RISING);
  attachInterrupt(digitalPinToInterrupt(FB_NB), ISR_FB_NB, RISING);
  attachInterrupt(digitalPinToInterrupt(RHT), ISR_RHT, RISING);
  attachInterrupt(digitalPinToInterrupt(RHIP), ISR_RHIP, RISING);

  Serial.begin(115200);

  sender.observe(RHTCount);
  //sender.observe(RHT0);
  //sender.observe(RHT1);
  //sender.observe(RHT2);
  //sender.observe(RHT3);

  sender.observe(RHIPCount);
  //sender.observe(RHIP0);
  //sender.observe(RHIP1);
  //sender.observe(RHIP2);
  //sender.observe(RHIP3);

  sender.observe(FarBCount);
  //sender.observe(FarB0);
  //sender.observe(FarB1);
  //sender.observe(FarB2);
  //sender.observe(FarB3);

  sender.observe(NearBCount);
  //sender.observe(NearB0);
  //sender.observe(NearB1);
  //sender.observe(NearB2);
  //sender.observe(NearB3);

  sender.observe(echo0);
  sender.observe(echo1);
  sender.observe(echo2);
  sender.observe(echo3);

  //sender.observe(echo_true);
  receiver.observe(Mode);

  //timer = timerBegin(0, 80, true); //Prescaler 80 will divide 80MHz to yield a 1 us granularity
  //timerAttachInterrupt(timer, &ISR_onTimer, true);
  //timerAlarmWrite(timer, 500000, true); //500 milliseconds, the reqd interrupt period, is 500000 * 1usec
  //timerAlarmEnable(timer);


  timeStart = millis();
}
// /65536 and %

//******************************* Main Loop ********************************
void loop() {

  /*
    

    Serial.println();
  */

  if (getStartTime_search) {
    startCount = ESP.getCycleCount();
    getStartTime_search = false;
  }
  if (getStartTime_track) {
    startCount = ESP.getCycleCount();
    getStartTime_track = false;
  }

/****************************** Get Echo Time *******************/
  if (getEchoTime) {
    endCount = ESP.getCycleCount();
    getEchoTime = false;
    echo_true = 0;
  }
  if (endCount >= startCount) {
    echoCount = endCount - startCount;
    byteConverter(echoCount);

    echo0 = int(b0);
    echo1 = int(b1);
    echo2 = int(b2);
    echo3 = int(b3);
  }
  else if (endCount < startCount) {
    echoCount = (4294967296 - endCount) + startCount;
    byteConverter(echoCount);


    echo0 = int(b0);
    echo1 = int(b1);
    echo2 = int(b2);
    echo3 = int(b3);
  }
  //tracking mode-range-32.5km
  //if(digitalRead(TSV)==HIGH)
  if (Mode == 0) {
    range = 32.5;
    //Mode=0;

/************************* Get NB_FB Count **************************/
    if (get_FB_NB) {

      if (counter == 1) {
        NBCount = ESP.getCycleCount();
        counter += 1;
        if (NBCount > startCount) {
          NBTimeCount = NBCount - startCount;
          //byteConverter(NBTimeCount);
          NearBCount = int(NBTimeCount);

          //NearB0 = int(b0);
          //NearB1 = int(b1);
          //NearB2 = int(b2);
          //NearB3 = int(b3);

        }
        else if (startCount > NBCount) {
          NBTimeCount = (4294967296 - NBCount) + startCount;
          //byteConverter(NBTimeCount);
          NearBCount = int(NBTimeCount);

          //NearB0 = int(b0);
          //NearB1 = int(b1);
          //NearB2 = int(b2);
          //NearB3 = int(b3);
        }

      }

/************************* FB Count **************************/
      if (counter == 2) {
        FBCount = ESP.getCycleCount();
        if (FBCount > startCount) {
          FBTimeCount = FBCount - startCount;
          //byteConverter(FBTimeCount);
          FarBCount = int(FBTimeCount);

          //FarB0 = int(b0);
          //FarB1 = int(b1);
          //FarB2 = int(b2);
          //FarB3 = int(b3);
        }
        else if (startCount > FBCount) {
          FBTimeCount = (4294967296 - FBCount) + startCount;
          //byteConverter(FBTimeCount);
          FarBCount = int(FBTimeCount);

          //FarB0 = int(b0);
          //FarB1 = int(b1);
          //FarB2 = int(b2);
          //FarB3 = int(b3);
        }
        counter = 1;
      }
      get_FB_NB = false;
    }

/************************ RHT ********************************/
    if (getRHTTime) {
      RHTCount = ESP.getCycleCount();
      if (RHTCount > startCount) {
        RHTTimeCount = RHTCount - startCount;
        //byteConverter(RHTTimeCount);
        RHTCount = int(RHTTimeCount); // convert to a single int

        //RHT0 = int(b0);
        //RHT1 = int(b1);
        //RHT2 = int(b2);
        //RHT3 = int(b3);

      }
      else if (RHTCount < startCount) {
        RHTTimeCount = (4294967296 - RHTCount) + startCount;
        //byteConverter(RHTTimeCount); // changed from Range2
        RHTCount = int(RHTTimeCount); // convert to a single int

        //RHT0 = int(b0); //value will be accommodated in a single int
        //RHT1 = int(b1);
        //RHT2 = int(b2);
        //RHT3 = int(b3);
      }
      getRHTTime = false;
    }

/************************** RHIP *******************************/
    if (getRHIPTime) {
      RHIPCount = ESP.getCycleCount();
      if (RHIPCount > startCount) {
        RHIPTimeCount = RHIPCount - startCount;
        //byteConverter(RHIPTimeCount);
        RHIPCount = int(RHIPTimeCount);

        //RHIP0 = int(b0);
        //RHIP1 = int(b1);
        //RHIP2 = int(b2);
        //RHIP3 = int(b3);
      }
      else if (RHIPCount < startCount) {
        RHIPTimeCount = (4294967296 - RHIPCount) - startCount;
        //byteConverter(RHIPTimeCount);
        RHIPCount = int(RHIPTimeCount);

        //RHIP0 = int(b0);
        //RHIP1 = int(b1);
        //RHIP2 = int(b2);
        //RHIP3 = int(b3);
      }
      getRHIPTime = false;
    }
  }

  //search mode-range-82.5km
  if (Mode == 1) {

    range = 82.5;
  }

  timeNow = millis();
  if ((timeNow - timeStart) >= dataPeriod) {
    timeStart = millis();
    noInterrupts();
    //ledStatus = !ledStatus;
    //digitalWrite(led, ledStatus);
    sender.sync();
    receiver.sync();
    interrupts();
  }


}

void ISR_sync_search() {
  //portENTER_CRITICAL(&mux);
  getStartTime_search = true;
  //portENTER_CRITICAL(&mux);
  //Serial.println("RO2");
}

void ISR_sync_track() {
  //portENTER_CRITICAL(&mux);
  getStartTime_track = true;
  //portENTER_CRITICAL(&mux);
  //Serial.println("RO1");
}

void ISR_echo() {
  //portENTER_CRITICAL(&mux);
  getEchoTime = true;
  echo_true = 1;
  //portENTER_CRITICAL(&mux);
  //Serial.println("echo");
}

void ISR_FB_NB() {
  //portENTER_CRITICAL(&mux);
  get_FB_NB = true;
  //portENTER_CRITICAL(&mux);
  //Serial.println("FB/NB");
}

void ISR_RHT() {
  //portENTER_CRITICAL(&mux);
  getRHTTime = true;
  //portENTER_CRITICAL(&mux);
  //Serial.println("RHT");
}

void ISR_RHIP() {
  //portENTER_CRITICAL(&mux);
  getRHIPTime = true;
  //portEXIT_CRITICAL(&mux);
  //Serial.println("RHIP");
}

void ISR_onTimer() {
  //portENTER_CRITICAL_ISR(&timerMux);
  sendDataToPC = true;
  //portEXIT_CRITICAL_ISR(&timerMux);

}

void byteConverter(unsigned long Count) {
  b3 = (int)((Count >> 24) & 0xFFFFFFFF);
  b2 = (int)((Count >> 16) & 0xFFFFFF);
  b1 = (int)((Count >> 8) & 0xFFFF);
  b0 = (int)((Count) & 0xFF);
}
