/*
 * This Java code, written in the Processing IDE,
 * is for the YK-31 MGR Console display of the 
 * Pechora SAGW system, running on an Ubuntu Linux PC
 * with 2GiB RAM. This app receives range, near boundary,
 * far boundary, RHT, RHIP and azimuth data from an ESP8266
 * and an ATMEGA328PB Nano processor, to display it on a PC
 * monitor display.
 * 
 * (c)2017-2019 Rajiv Tyagi
 * (c)2017-2019 T&C Technology (India) Pvt. Ltd.
 * ALL RIGHTS RESERVED
 * 
 * This code is perpetually licensed to the Indian Air Force 
 * for use exclusively in the YK-31 Missile Guidance Radar Console 
 * of the Pechora SAGW system.
 */


PFont font;

import processing.serial.*;
import vsync.*;

//PrintWriter output;
//String fileName;

ValueReceiver ESP;
ValueReceiver ENCODER;
ValueSender fromEncoderToESP;

public int Mode;
public int beamPos;
public int LR;

public int FBCount;
//public int FB0;
//public int FB1;
//public int FB2;
//public int FB3;

public int NBCount;
//public int NB0;
//public int NB1;
//public int NB2;
//public int NB3;

public int echo0;
public int echo1;
public int echo2;
public int echo3;

public int RHTCount;
//public int RHT0;
//public int RHT1;
//public int RHT2;
//public int RHT3;

public int RHIPCount;
//public int RHIP0;
//public int RHIP1;
//public int RHIP2;
//public int RHIP3;
int RHIPcounter=0;
int FBcounter=0;
int NBcounter=0;

int gridBrightness=100;
int radialBrightness=100;
int beamBrightness=100;
int screenSizeX=1800;
int screenSizeY=1000;
int radarSize=750;
int echo_true=0;
int tgtCounter=0;
int roundCounter=0;
int prevCounter=0;
int rowCounter=0;

float degreeIntervals=radians(30);
float theta;
float beamX;
float beamY;
float radialX;
float radialY;
float Azimuth=0.0;
float Range;
float beam;
float radial;
public float FB;
public float filteredFB;
public float NB;
public float filteredNB;
public float RHT;
public float RHIP;
public float filteredRHIP;
public float echo;

public long echoCount;
//public long RHTCount;
//public long RHIPCount;
//public long NBCount;
//public long FBCount;

// ****************************** S E T U P 
void setup() {
  size (1020,720); 
  //  fileName=str(year())+str(month())+str(day())+str(hour())+str(minute())+str(second());
 // output = createWriter(fileName+".txt");
  //fullScreen();
  frameRate(36);
  beam=abs(radarSize/2/50)*50;
  radial=beam;
  noStroke();
  
  
  Serial serial1 = new Serial(this, "/dev/ttyUSB0", 115200); //Nano azimuth capture unit
  Serial serial2 = new Serial(this, "/dev/ttyUSB1", 115200); //ESP32 range capture unit
  ENCODER=new ValueReceiver(this,serial1);
  ESP = new ValueReceiver(this, serial2);   
  fromEncoderToESP = new ValueSender(this, serial2);
  
  ENCODER.observe("beamPos");
  ENCODER.observe("Mode");
  ENCODER.observe("LR");
  
  Azimuth = radians(Azimuth);

  ESP.observe("RHTCount");
  //ESP.observe("RHT0");
  //ESP.observe("RHT1");
  //ESP.observe("RHT2");
  //ESP.observe("RHT3");
  
  ESP.observe("RHIPCount");
  //ESP.observe("RHIP0");
  //ESP.observe("RHIP1");
  //ESP.observe("RHIP2");
  //ESP.observe("RHIP3");
  
  ESP.observe("FBCount");
  //ESP.observe("FB0");
  //ESP.observe("FB1");
  //ESP.observe("FB2");
  //ESP.observe("FB3");

  ESP.observe("NBCount");
  //ESP.observe("NB0");
  //ESP.observe("NB1");
  //ESP.observe("NB2");
  //ESP.observe("NB3");
  
  ESP.observe("echo0");
  ESP.observe("echo1");
  ESP.observe("echo2");
  ESP.observe("echo3");
  
  //ESP.observe("echo_true");
  fromEncoderToESP.observe("Mode");
  

  ellipseMode(CENTER);
  
  //Mode = 0; //Test purpose
  //LR = 0;
  
  
}

// *********************************** M A I N ***************************/

void draw() {
  int i=0;
  int j=0;
  int q=45;
  
  background(0);
  //Search Mode at Long Range
  if ((Mode==0) && (LR == 1)) {
    stroke(255, 255, 255);
    textSize(26);
    text("Search Long Range", 750, 30);
    textSize(12);
    calculateTime();
    drawCircleGrid();
    drawRadials(10);
    radialValues(10);
    distanceValuesLongRange(10);
    drawMarks();
    drawBlip(Range,(Azimuth-radians(90)));
    makeBlipStay();
    pushMatrix();
    drawBeam(radians(beamPos)-radians(90));
    popMatrix();
    
    //println("Search Long Range");
    //print("Mode: ");
    //println(Mode);
    //print("LR: ");
    //println(LR);
  }
  
  //Search Mode at Short Range
  if ((Mode==0) && (LR==0)) {
    stroke(255, 255, 255);
    textSize(26);
    text("Search Short Range", 750, 30);
    textSize(12);
    calculateTime();
    drawCircleGrid();
    drawRadials(10);
    radialValues(10);
    distanceValuesShortRange(10);
    drawMarks();
    drawBlip(Range,(Azimuth-radians(90)));
    makeBlipStay();
    pushMatrix();
    drawBeam(radians(beamPos)-radians(90));
    popMatrix();
    
    print("Search Short Range");
    print("Mode: ");
    println(Mode);
    print("LR: ");
    println(LR);
  }
  
  // Tracking mode
  if (Mode==1) {
      //clearButton();
    //buttonPressed();
    calculateTime();
    /*println("FB");
    println();
    println(FB0);println(FB1);println(FB2);println(FB3);
   println("NB");
    println();
    println(NB0);println(NB1);println(NB2);println(NB3);
   println("RHT");
    println();
    println(RHT0);println(RHT1);println(RHT2);println(RHT3);
   println("RHIP");
    println();
    println(RHIP0);println(RHIP1);println(RHIP2);println(RHIP3);
   */
    //println();
    
    drawLabels();
    //createTable(10,3);
    //tableTest();
    drawCircleGrid();
    //drawFBCircle(medianFB); //FB
    drawFBCircle(FB); //FB
    //drawNBCircle(medianNB);//NB
    drawNBCircle(NB);//NB
    //drawBrokenCircle(medianRHIP);//RHIP
    drawBrokenCircle(RHIP);//RHIP
    drawRadials(10);
    radialValues(10);
    distanceValuesTrack(10);
    drawMarks();
    drawBlip(RHT, (Azimuth-radians(90)));
    if(q>0)
    q-=1;
    else
    q=45;
    makeBlipStay();
    pushMatrix();
    //drawBeam(radians(j)-radians(90));
    popMatrix();
    
    //println("Tracking");
    //print("Mode: ");
    //println(Mode);
    //print("LR: ");
    //println(LR);
    
    if (j<=358) {
      j+=1;
    } else{
      j=0;
    }
  }
}


// ************************ Store Values in a file *************************/
/*void fileStoreValues(){
  output.println(Range + "\t" + Azimuth);
}*/


int z=1;
float[] x=new float[500];
float[] y=new float[500];
float[] storeAzimuth=new float[500];
float[] storeRange=new float[500];
// ************************ Draw Blip ****************************/
void drawBlip(float Range, float Azimuth) {
  
  radialX = Range*cos(Azimuth);
  radialY = Range*sin(Azimuth);
  x[z]=map(radialX,-45,45,0,600);
  y[z]=map(radialY,-45,45,0,600);
  storeAzimuth[z]=Azimuth;
  storeRange[z]=Range;
  if(x[z]!=x[z-1] || y[z]!=y[z-1]){
    //tableValues();
    stroke(255, 0, 0);
    fill(255, 0, 0);
    ellipse(x[z]+55,y[z]+60, 3, 3);
    stroke(255, 255, 255);
    fill(255, 2555, 255);
    z+=1;
    if(z==499)
    z=1;
  }
}

//*********************** Make blip stay *****************************/
void makeBlipStay(){
  stroke(255,0,0);
  fill(255,0,0);
  for(int i=1;i<z;i++){
    //fill(137,50,50);
    //stroke(137,50,50);
    ellipse(x[i]+55,y[i]+60,3,3);
  }
    stroke(255,255,255);
    fill(255,255,255);
}


//*********************** Create Clear Button *************************/
void clearButton(){
  fill(0);
  rect(930,210,60,15);
  fill(255);
  text("CLEAR",940,222);
}

//********************** Create Close Button *************************/
/*void closeFile(){
  stroke(255);
  rect(940,25,60,15);
  fill(0);
  text("CLOSE",947,37);
}*/

//*********************** Button Functions (Clear table, Close File)
void buttonPressed() {
  if(mouseX>=930 && mouseX<=990 && mouseY>=210 && mouseY<=235){// clear table
    if (mousePressed) {
      createTable(10,3);
    }
  }
  /*
  if(mouseX>=940 && mouseX<=1000 &&mouseY>=25 && mouseY<=40){
    if(mousePressed){
      output.flush(); // Write the remaining data
      output.close(); // Finish the file
      exit(); // Stop the program
    }
  }*/
}

//*********************** Create Table **************************/
void createTable(int rows,int columns) {
  strokeWeight(1);
  int cellSizeX=75;
  int cellSizeY=25;
  int x=750;
  int y=250;
  for(int i=0;i<rows;i++){
    for(int j=0;j<columns;j++){
      rect(x,y,cellSizeX,cellSizeY);
         x+=cellSizeX;
    }
    x=750;
   y+=cellSizeY;
  }
}

  int cellx=750;
  int celly=250;
  
  void tableTest(){
    for(int i=0;i<=10;i++){
      text("1",cellx+10,celly+20);
      celly+=25;
    }
    
    rowCounter+=1;
  //celly+=25;
  if(rowCounter==10){
    cellx=750;
    celly=250;
    rowCounter=0;
    fill(0);
    createTable(10,3);
    
  }
  }
//******************* Table Values **********************
void tableValues(){

for(int i=0;i<=z;i++){
  text(int(storeAzimuth[i]),cellx+10,celly+20);
  text(int(storeRange[i]),cellx+40,celly+20);
  delay(100);
  celly+=25;
  //rowCounter=0;
}
  rowCounter+=1;
  celly+=25;
  if(rowCounter==10){
    cellx=750;
    celly=250;
    rowCounter=0;
    fill(0);
    createTable(10,3);
    delay(1000);
  }
}

//*********************** Draw circular grids **********************
void drawCircleGrid() {
  for (int i=abs(radarSize/200)*200; i>=100; i=i-200) {
    stroke(0, gridBrightness, 0);
    strokeWeight(2);
    fill(0);
    ellipse(width/2-radarSize/2+220, height/2, i, i);
  }
}

//************************* Draw Broken Circle **********************
void drawBrokenCircle(float RHIP) {
  //float rhip=map(RHIP, 0, 45, 0, 600);
  float rhip=map(RHIP, 0, 30, 0, 750);
  stroke(0, 0, 255);
  strokeWeight(2);
  noFill();
  ellipse(width/2-radarSize/2+220, height/2, rhip, rhip );
}

//************************* Draw NB Circle **************************
void drawNBCircle(float NB) {
  //float nb=map(NB, 0, 45, 0, 600); // original
  float nb=map(NB, 0, 30, 0, 750); // changed
  stroke(245, 90, 0);
  strokeWeight(2);
  fill(245, 90, 0);
  ellipse(width/2-radarSize/2+220, height/2, nb, nb ); 
}

//************************* Draw FB Circle
void drawFBCircle(float FB) {
  //float fb=map(FB, 0, 45, 0, 600); // original
  float fb=map(FB, 0, 30, 0, 750); // changed
  stroke(224, 165, 0);
  strokeWeight(2);
  noFill();
  ellipse(width/2-radarSize/2+220, height/2, fb, fb );
}
//************************* Draw Labels
void drawLabels() {
  noFill();
  stroke(255, 255, 255);
  strokeWeight(1);
  //rect(850, 100, 100, 20);
  rect(width/2-radarSize/2+220+495, height/2-260,100,20);
  text("RHT", 860, 140);
  text(RHT, 860, 115);

  rect(850, 160, 100, 20);
  text("RHIP", 860, 200);
  text(RHIP, 860, 175);
  textSize(26);
  text("Tracking Mode", 750, 40);
  textSize(12);
  stroke(224, 165, 0);
  strokeWeight(3);
  line(770, 530, 820, 530);
  stroke(245, 90, 0);
  line(770, 570, 820, 570);
  stroke(0, 0, 255);
  line(770, 610, 820, 610);
  stroke(255, 255, 255);
  textSize(14);
  text("Far Boundary Circle", 830, 530);
  text("Near Boundary Circle", 830, 570);
  text("RHIP Circle", 830, 610);
  textSize(12);
}


// ************************ Draw Radials
void drawRadials(float degreeIntervals) {
  for (theta=0; theta<radians(360); theta=theta+radians(degreeIntervals)) {
    radialX = radial*cos(theta)*0.85;
    radialY = radial*sin(theta)*0.85;
    stroke(radialBrightness);
    strokeWeight(1);
    strokeCap(ROUND);
    if ((int)degrees(theta)==0 || (int)degrees(theta)==89 || (int)degrees(theta)==179 || (int)degrees(theta)==269)
    {
      stroke(0, 255, 0);
      strokeWeight(2);
      strokeCap(ROUND);
      line(width/2-radarSize/2+220, height/2, width/2-radarSize/2+220 + radialX, height/2 + radialY);
    } else
      line(width/2-radarSize/2+220, height/2, width/2-radarSize/2+220+ radialX, height/2 + radialY);
  }
}

//******************************Draw  10 degree marks
void drawMarks() {
  stroke(255, 255, 255);
  strokeWeight(2);
  //north
  line(width/2-radarSize/2+220-5, height/2-264,width/2-radarSize/2+220+5, height/2-264);
  line(width/2-radarSize/2+220-5, height/2-231,width/2-radarSize/2+220+5, height/2-231);
  line(width/2-radarSize/2+220-5, height/2-171,width/2-radarSize/2+220+5, height/2-171);
  line(width/2-radarSize/2+220-5, height/2-136,width/2-radarSize/2+220+5, height/2-136);
  line(width/2-radarSize/2+220-5, height/2-71,width/2-radarSize/2+220+5, height/2-71);
  line(width/2-radarSize/2+220-5, height/2-36,width/2-radarSize/2+220+5, height/2-36);
  //south
  line(width/2-radarSize/2+220-5, height/2+30,width/2-radarSize/2+220+5, height/2+30);
  line(width/2-radarSize/2+220-5, height/2+66,width/2-radarSize/2+220+5, height/2+66);
  line(width/2-radarSize/2+220-5, height/2+135,width/2-radarSize/2+220+5, height/2+135);
  line(width/2-radarSize/2+220-5, height/2+170,width/2-radarSize/2+220+5, height/2+170);
  line(width/2-radarSize/2+220-5, height/2+235,width/2-radarSize/2+220+5, height/2+235);
  line(width/2-radarSize/2+220-5, height/2+270,width/2-radarSize/2+220+5, height/2+270);
  //east
  line(width/2-radarSize/2+220+30, height/2+5,width/2-radarSize/2+220+30, height/2-5);
  line(width/2-radarSize/2+220+65, height/2+5,width/2-radarSize/2+220+65, height/2-5);
  line(width/2-radarSize/2+220+130, height/2+5,width/2-radarSize/2+220+130, height/2-5);
  line(width/2-radarSize/2+220+165, height/2+5,width/2-radarSize/2+220+165, height/2-5);
  line(width/2-radarSize/2+220+230, height/2+5,width/2-radarSize/2+220+230, height/2-5);
  line(width/2-radarSize/2+220+265, height/2+5,width/2-radarSize/2+220+265, height/2-5);
  //west
  line(width/2-radarSize/2+220-30, height/2+5,width/2-radarSize/2+220-30, height/2-5);
  line(width/2-radarSize/2+220-65, height/2+5,width/2-radarSize/2+220-65, height/2-5);
  line(width/2-radarSize/2+220-130, height/2+5,width/2-radarSize/2+220-130, height/2-5);
  line(width/2-radarSize/2+220-165, height/2+5,width/2-radarSize/2+220-165, height/2-5);
  line(width/2-radarSize/2+220-235, height/2+5,width/2-radarSize/2+220-235, height/2-5);
  line(width/2-radarSize/2+220-270, height/2+5,width/2-radarSize/2+220-270, height/2-5);
}

// ********************************* Draw Beam **********************************/
void drawBeam(float phi) {
  beamX = beam*cos(phi)*0.85;
  beamY = beam*sin(phi)*0.85;
  stroke(255, 0, 0);
  strokeWeight(4);
  strokeCap(ROUND);
  line(width/2-radarSize/2+220, height/2, width/2-radarSize/2+220 + beamX, height/2 + beamY);
}


// ************************ Write Radials Values ***************************/
void radialValues(float degreeIntervals) {
  for (theta=0; theta<=radians(360); theta=theta+radians(degreeIntervals)) {
    float radialX1 = (radial-20)*cos(theta);
    float radialY1 = (radial-20)*sin(theta);
    float radialX2 = (radial-30)*cos(theta);
    float radialY2 = (radial-25)*sin(theta);
    fill(255);
    float textval=degrees(theta)+90;
    if (textval>=90 && textval<165)
      text((int)textval, width/2-radarSize/2+100 + radialX1+100, height/2 + radialY1);
    else if (textval>=165 && textval<180)
      text((int)textval+1, width/2-radarSize/2+100 + 100+radialX1, height/2 + radialY1);
    else if (textval>=180 && textval<359)
      text((int)textval+1, width/2-radarSize/2+100+100 + radialX2, height/2 + radialY2);
    else if ((int)textval==359)
      text(0, width/2-radarSize/2+100 + radialX1+100, height/2 + radialY1);
    else
    {
      textval=degrees(theta)-270;
      text((int)textval+1, width/2-radarSize/2+100 +100+ radialX1, height/2 + radialY1);
    }
  }
}



// ************************ Write distance Values Long Range ******************************/
void distanceValuesLongRange(float degreeIntervals) {
  for (theta=0; theta<=radians(360); theta=theta+radians(degreeIntervals)) {
    float radialX1 = (radial-15)*cos(theta);
    float radialY1 = (radial-15)*sin(theta);
    float radialX2 = (radial-70)*cos(theta);
    float radialY2 = (radial-95)*sin(theta);
    float radialX3 = (radial-195)*cos(theta);
    float radialY3 = (radial-195)*sin(theta);
    float radialX4 = (radial-295)*cos(theta);
    float radialY4 = (radial-295)*sin(theta);

    fill(255);
    float textval=degrees(theta)+90;
    if ((int)textval==359)
    {
      text(90, width/2-radarSize/2+220 + radialX1+5, height/2 + radialY1+50);
      text(60, width/2-radarSize/2+220+ radialX2+5, height/2 + radialY2+70);
      text(30, width/2-radarSize/2+220 + radialX3+5, height/2 + radialY3+70);
      //text(20, width/2-radarSize/2+100+300 + radialX4, height/2 + radialY4+20);
    }

    if ((int)textval==449)
    {
      //text(80, width/2-radarSize/2+100 + radialX1-10, height/2-25 + radialY1+15);
      text(30, width/2-radarSize/2+120 + radialX2-75-200, height/2-25 + radialY2+20);
      text(60, width/2-radarSize/2+120 + radialX3-50-200, height/2-25 + radialY3+19);
      text(90, width/2-radarSize/2+120 + radialX4-50-200, height/2-25 + radialY4+18);
    }

    if ((int)textval==179)
    {
      // text(80, width/2-radarSize/2+100 + radialX1-20, height/2-25 + radialY1+5);
      text(90, width/2-radarSize/2+100 +120+ radialX2-20, height/2 + radialY2+40);
      text(60, width/2-radarSize/2+100 +120+ radialX3-20, height/2 + radialY3+40);
      text(30, width/2-radarSize/2+100 +120+ radialX4-17, height/2 + radialY4+40);
    }

    if ((int)textval==269)
    {
      //text(80, width/2-radarSize/2+100 + radialX1-10, height/2-25 + radialY1-5);
      text(90, width/2-radarSize/2+120+800 + radialX2+60-200, height/2 + radialY2-5);
      text(60, width/2-radarSize/2+120+600 + radialX3+35-200, height/2 + radialY3-5);
      text(30, width/2-radarSize/2+120+450 + radialX4-20-200, height/2 + radialY4-2);
    }
  }
}

// ************************ Write distance Values Short Range ******************************/
void distanceValuesShortRange(float degreeIntervals) {
  for (theta=0; theta<=radians(360); theta=theta+radians(degreeIntervals)) {
    float radialX1 = (radial-15)*cos(theta);
    float radialY1 = (radial-15)*sin(theta);
    float radialX2 = (radial-70)*cos(theta);
    float radialY2 = (radial-95)*sin(theta);
    float radialX3 = (radial-195)*cos(theta);
    float radialY3 = (radial-195)*sin(theta);
    float radialX4 = (radial-295)*cos(theta);
    float radialY4 = (radial-295)*sin(theta);

    fill(255);
    float textval=degrees(theta)+90;
    if ((int)textval==359)
    {
      text(45, width/2-radarSize/2+220 + radialX1+5, height/2 + radialY1+50);
      text(30, width/2-radarSize/2+220+ radialX2+5, height/2 + radialY2+70);
      text(15, width/2-radarSize/2+220 + radialX3+5, height/2 + radialY3+70);
      //text(20, width/2-radarSize/2+100+300 + radialX4, height/2 + radialY4+20);
    }

    if ((int)textval==449)
    {
      //text(80, width/2-radarSize/2+100 + radialX1-10, height/2-25 + radialY1+15);
      text(15, width/2-radarSize/2+120 + radialX2-75-200, height/2-25 + radialY2+20);
      text(30, width/2-radarSize/2+120 + radialX3-50-200, height/2-25 + radialY3+19);
      text(45, width/2-radarSize/2+120 + radialX4-50-200, height/2-25 + radialY4+18);
    }

    if ((int)textval==179)
    {
      // text(80, width/2-radarSize/2+100 + radialX1-20, height/2-25 + radialY1+5);
      text(45, width/2-radarSize/2+100 +120+ radialX2-20, height/2 + radialY2+40);
      text(30, width/2-radarSize/2+100 +120+ radialX3-20, height/2 + radialY3+40);
      text(15, width/2-radarSize/2+100 +120+ radialX4-17, height/2 + radialY4+40);
    }

    if ((int)textval==269)
    {
      //text(80, width/2-radarSize/2+100 + radialX1-10, height/2-25 + radialY1-5);
      text(45, width/2-radarSize/2+120+800 + radialX2+60-200, height/2 + radialY2-5);
      text(30, width/2-radarSize/2+120+600 + radialX3+35-200, height/2 + radialY3-5);
      text(15, width/2-radarSize/2+120+450 + radialX4-20-200, height/2 + radialY4-2);
    }
  }
}

// ************************ Write Tracking distance Values ***************************/
void distanceValuesTrack(float degreeIntervals) {
  for (theta=0; theta<=radians(360); theta=theta+radians(degreeIntervals)) {
    float radialX1 = (radial-15)*cos(theta);
    float radialY1 = (radial-15)*sin(theta);
    float radialX2 = (radial-70)*cos(theta);
    float radialY2 = (radial-95)*sin(theta);
    float radialX3 = (radial-195)*cos(theta);
    float radialY3 = (radial-195)*sin(theta);
    float radialX4 = (radial-295)*cos(theta);
    float radialY4 = (radial-295)*sin(theta);

    fill(255);
    float textval=degrees(theta)+90;
    if ((int)textval==359)
    {
      text(45, width/2-radarSize/2+220 + radialX1+5, height/2 + radialY1+50);
      text(30, width/2-radarSize/2+220+ radialX2+5, height/2 + radialY2+70);
      text(15, width/2-radarSize/2+220 + radialX3+5, height/2 + radialY3+70);
      //text(20, width/2-radarSize/2+100+300 + radialX4, height/2 + radialY4+20);
    }

    if ((int)textval==449)
    {
      //text(80, width/2-radarSize/2+100 + radialX1-10, height/2-25 + radialY1+15);
      text(15, width/2-radarSize/2+120 + radialX2-75-200, height/2-25 + radialY2+20);
      text(30, width/2-radarSize/2+120 + radialX3-50-200, height/2-25 + radialY3+19);
      text(45, width/2-radarSize/2+120 + radialX4-50-200, height/2-25 + radialY4+18);
    }

    if ((int)textval==179)
    {
      // text(80, width/2-radarSize/2+100 + radialX1-20, height/2-25 + radialY1+5);
      text(45, width/2-radarSize/2+100 +120+ radialX2-20, height/2 + radialY2+40);
      text(30, width/2-radarSize/2+100 +120+ radialX3-20, height/2 + radialY3+40);
      text(15, width/2-radarSize/2+100 +120+ radialX4-17, height/2 + radialY4+40);
    }

    if ((int)textval==269)
    {
      //text(80, width/2-radarSize/2+100 + radialX1-10, height/2-25 + radialY1-5);
      text(45, width/2-radarSize/2+120+800 + radialX2+60-200, height/2 + radialY2-5);
      text(30, width/2-radarSize/2+120+600 + radialX3+35-200, height/2 + radialY3-5);
      text(15, width/2-radarSize/2+120+450 + radialX4-20-200, height/2 + radialY4-2);
    }
  }
}

//float[] RHIPbuffer=new float[5];


//************************* Calculate pulse Time **************************/
void calculateTime() {
  //FBCount=FB0+(FB1*256)+(FB2*65536)+(FB3*16777216);
  //NBCount=NB0+(NB1*256)+(NB2*65536)+(NB3*16777216);
  //RHTCount=RHT0+(RHT1*256)+(RHT2*65536)+(RHT3*16777216);
  //RHIPCount=RHIP0+(RHIP1*256)+(RHIP2*65536)+(RHIP3*16777216);
  echoCount=echo0+(echo1*256)+(echo2*65536)+(echo3*16777216);

  //FB=FBCount*6.25*0.00299709; //each count is 6.25 nanoseconds for ESP8266 @160 MHz
  FB=FBCount*4.1666*0.00299792*3.0; //each count is 4.1666 nanoseconds for ESP32 @240 MHz
  //FBbuf[FBcounter]=FB;
  //FBcounter+=1;
  //NB=NBCount*6.25*0.000299709; //each count is 6.25 nanoseconds for ESP8266 @160 MHz
  NB=NBCount*4.1666*0.00299792*2.0; //each count is 4.1666 nanoseconds for ESP32 @240 MHz
  //NBbuf[NBcounter]=NB;
  //NBcounter+=1;
  //RHIP=RHIPCount*6.25*0.00299709; //each count is 6.25 nanoseconds for ESP8266 @160 MHz
  RHIP=RHIPCount*4.1666*0.00299792*3.0; //each count is 4.1666 nanoseconds for ESP32 @240 MHz
  //RHIPbuf[RHIPcounter]=RHIP;
  //RHIPcounter+=1;
  //RHT=RHTCount*6.25*0.000299709; //each count is 6.25 nanoseconds for ESP8266 @160 MHz
  RHT=RHTCount*4.1666*0.00299792*3.0; //each count is 4.1666 nanoseconds for ESP32 @240 MHz. 0.000299792 km per ns
  
  //if(FBcounter%100==0){
    //FBfilter();
    //FBcounter = 0;
  //}
  //if(NBcounter%100==0){
    //NBfilter();
    //NBcounter = 0;
  //}
  //if(RHIPcounter%100==0){
    //RHIPfilter();
    //RHIPcounter = 0;
  //}
    
  //print("filteredRHIP       ");
  //print(medianRHIP);
  print("RHIP: ");
  println(RHIP);
  //print("Mode: ");
  //println(Mode);
  //print("LR: ");
  //println(LR);
  
  //echo=(echoCount/2)*6.25*0.000299709; //each count is 6.25 nanoseconds for ESP8266 @160 MHz
  echo=(echoCount/2)*4.1666*0.000299792*2; ////each count is 4.1666 nanoseconds for ESP32 @240 MHz
  Range=echo;
  /*print("NB   ");
  println(NB);
  print("FB    ");
  println(FB);
  print("RHT     ");
  println(RHT);
  print("RHIP     ");
  println(RHIP);*/

    if(echo_true==1){
      Azimuth=beamPos;
     //println("echo true");
  }
}

/*
float[] RHIPbuf=new float[100000];
int indexPosRHIP=0;
float maxPosRHIP=0;
float minPosRHIP=0;
float medianRHIP=0;
int windowWidthRHIP=100;

void RHIPfilter(){
  for(int i=indexPosRHIP;i<(indexPosRHIP+windowWidthRHIP);i++){
    for(int j=i;j<(indexPosRHIP+windowWidthRHIP);j++){
      if(RHIPbuf[j]>RHIPbuf[i])
      maxPosRHIP=j;
      if(RHIPbuf[j]<RHIPbuf[i])
      minPosRHIP=j;
    }
  }
  
  for(int i=indexPosRHIP;i<(indexPosRHIP+windowWidthRHIP);i++){
    if(i!=maxPosRHIP && i!=minPosRHIP){
      medianRHIP+=RHIPbuf[i];
    }
  }
  medianRHIP=medianRHIP/(windowWidthRHIP-2);
  if(indexPosRHIP<100000)
  indexPosRHIP+=1;
  else
  indexPosRHIP=0;
}
*/

/*
float[] FBbuf=new float[100000];
int indexPosFB=0;
float maxPosFB=0;
float minPosFB=0;
float medianFB=0;
int windowWidthFB=100;
*/

/*void FBfilter(){
  for(int i=indexPosFB;i<(indexPosFB+windowWidthFB);i++){
    for(int j=i;j<(indexPosFB+windowWidthFB);j++){
      if(FBbuf[j]>FBbuf[i])
      maxPosFB=j;
      if(FBbuf[j]<FBbuf[i])
      minPosFB=j;
    }
  }
  
  for(int i=indexPosFB;i<(indexPosFB+windowWidthFB);i++){
    if(i!=maxPosFB && i!=minPosFB){
      medianFB+=FBbuf[i];
    }
  }
  medianFB=medianFB/(windowWidthFB-2);
  if(indexPosFB<100000)
  indexPosFB+=1;
  else
  indexPosFB=0;
}
*/

/*float[] NBbuf=new float[100000];
int indexPosNB=0;
float maxPosNB=0;
float minPosNB=0;
float medianNB=0;
int windowWidthNB=10;
*/

/*void NBfilter(){
  for(int i=indexPosNB;i<(indexPosNB+windowWidthNB);i++){
    for(int j=i;j<(indexPosNB+windowWidthNB);j++){
      if(NBbuf[j]>NBbuf[i])
      maxPosNB=j;
      if(NBbuf[j]<NBbuf[i])
      minPosNB=j;
    }
  }
  
  for(int i=indexPosNB;i<(indexPosNB+windowWidthNB);i++){
    if(i!=maxPosNB && i!=minPosNB){
      medianNB+=NBbuf[i];
    }
  }
  medianNB=medianNB/(windowWidthNB-2);
  if(indexPosNB<100000)
  indexPosNB+=1;
  else
  indexPosNB=0;
}
*/
