/*
  Big ass giant LED clock.  Kevin Moore 2021

    MIT License

Copyright (c) 2021 Kevin Moore

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), 
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <ezButton.h>
#include <Encoder.h>
#include <FastLED.h>
#include "SparkFunCCS811.h"
#include "SparkFunBME280.h"
#include <SparkFun_RV8803.h>

//LED pin defs
#define NUM_LEDS 130
#define DATA_PIN 6

#define CCS811_ADDR 0x5B //Default I2C Address

BME280 myEnv;
CCS811 myGas(CCS811_ADDR);
ezButton button(9);  // create ezButton object that attach to pin9
Encoder myEnc(7, 8);
int hue = 0;

const int SHORT_PRESS_TIME = 1000; // 1000 milliseconds
const int LONG_PRESS_TIME  = 1000; // 1000 milliseconds

//LED offsets used in the 7 segment display.  3 LEDs per segment, 21 total LEDs per digit plus 4 dots for 2 semicolons
static int d0_offset = 0; 
static int d1_offset = 21;
static int d2_offset = 42;
static int d3_offset = 63;
static int d4_offset = 84;
static int d5_offset = 105;
static int c0_offset = 126;
static int c1_offset = 127;
static int c2_offset = 128;
static int c3_offset = 129;

// Rotary encoder declarations

volatile byte encoderPos = 0;
volatile byte oldPosition  = 0;

int main_menu  = 0;
int sub_menu  = 0;

bool isPressing = false;
bool isLongDetected = false;
bool isShortDetected = false;
bool menuActive = false;

unsigned long pressedTime  = 0;
unsigned long releasedTime = 0;

unsigned long CO2;
unsigned long VOC;

float humidity;
float pressure;
float temp;

int hours;
int minutes;
int seconds;
int day = 1;
int month = 1;
int year = 2021;


byte Mode = 0;   // This is which menu mode we are in at any given time (top level or one of the submenus)
const byte modeMax = 4; // This is the number of submenus/settings you want
byte param_brightness = 128;  // a variable which holds the value we set 
byte param_color = 0;  // a variable which holds the value we set 
byte param_scrolltime = 3;  // a variable which holds the value we set
byte param_corf = 0;//0 = celcius 1 = farenheit

RV8803 rtc;
CRGB leds[NUM_LEDS];

const int8_t digit[25][21] = {
//    c        d        e        g        b        a        f
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1}, //0
  {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0}, //1
  {0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0}, //2
  {1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0}, //3
  {1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1}, //4
  {1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1}, //5
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1}, //6
  {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0}, //7
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, //8
  {1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, //9
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, //degree symbol-10
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1}, //'b'-11
  {0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1}, //'C'-12
  {0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1}, //'E'-13
  {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1}, //'F'-14
  {1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1}, //'h'-15
  {0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //'i'-16
  {0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1}, //'1'-17
  {1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //'n'-18
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //'o'-19
  {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, //'P'-20
  {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //'r'-21
  {1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1}, //'S'-22
  {0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1}, //'t'-23
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}  //all off-24
};

void setup() {

  Wire.begin();

  Serial.begin(115200);
  Serial.println("Read Time from RTC Example");

  if (myGas.begin() == false)
  {
    Serial.print("CCS811 error. Please check wiring. Freezing...");
    while (1)
      ;
  }
  if (myEnv.begin() == false)
  {
    Serial.print("BME280 error. Please check wiring. Freezing...");
    while (1)
      ;
  }
  if (rtc.begin() == false)
  {
    Serial.print("RTC error. Please check wiring. Freezing...");
    while (1)
      ;
  }

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed

  rtc.set24Hour();

}

void loop(){
  
  buttonCheck(); //check status of buttons for presses
  readEncoder(); //read encoder value for menu operations
  
  rotaryMenu(); //display and handle menu displays / parameter updates
  if(main_menu != 3) //don't update time from RTC while setting the time
    getTime(); //get the time
  updateClock(); //update the main display
}

void getEnv(){ //read CO2 and VOC from i2c board
  if (myGas.dataAvailable())
  {
    //If so, have the sensor read and calculate the results.
    //Get them later
    myGas.readAlgorithmResults();
    CO2 = myGas.getCO2();    
    VOC = myGas.getTVOC();
    delay(10);
  }
}
void getTemp(){ //read temp and pressure from i2c board
  
  humidity = myEnv.readFloatHumidity();
  pressure = myEnv.readFloatPressure();
  if(param_corf)
    temp = myEnv.readTempF();
  else
    temp = myEnv.readTempC();
  Serial.println(temp);
  delay(10);  
}

void getTime(){ //read time from i2c board
  if (rtc.updateTime() == false)
    Serial.print("RTC failed to update");

  hours = rtc.getHours();
  minutes = rtc.getMinutes();
  seconds = rtc.getSeconds();
}

void buttonCheck(){
  button.loop();
  if(button.isPressed()){
    pressedTime = millis();
    isPressing = true;
    isLongDetected = false;
    isShortDetected = false;
  }

  if(button.isReleased()) {
    isPressing = false;
    releasedTime = millis();

    long pressDuration = releasedTime - pressedTime;

    if( pressDuration < SHORT_PRESS_TIME ){
      Serial.println("A short press is detected");
      isShortDetected = true;      
    }
  }

  if(isPressing == true && isLongDetected == false) {
    long pressDuration = millis() - pressedTime;

    if( pressDuration > LONG_PRESS_TIME ) {
      Serial.println("A long press is detected");
      isLongDetected = true;
    }
  }  
}

void readEncoder(){
  encoderPos = myEnc.read()/4;
  if (encoderPos != oldPosition) {
    oldPosition = encoderPos;
    Serial.println(encoderPos);
  }
  if(main_menu == 1){
    if (encoderPos > (modeMax)){
    //Serial.println("exceeded max, setting to max");
    encoderPos = modeMax; // check we haven't gone out of bounds below 0 and correct if we have
    myEnc.write(modeMax);
    }
  }else if(main_menu == 3){
    if (sub_menu == 10 && encoderPos > 23){ //reset to 0 if beyond 23 hours
      encoderPos = 0; 
      myEnc.write(0);
    }
    else if ((sub_menu == 11 || sub_menu == 12) && encoderPos > 59){ //reset to 0 if beyond 59 min/sec
      encoderPos = 0; 
      myEnc.write(0);
    }
  }
}

void updateClock(){
  if(main_menu  == 0){ //main clock update if not in a menu
    writeDigit(1, d0_offset, (seconds % 10));
    writeDigit(1, d1_offset, (seconds / 10));
    writeDigit(1, d2_offset, (minutes % 10));
    writeDigit(1, d3_offset, (minutes / 10));
    writeDigit(1, d4_offset, (hours % 10));
    if((hours / 10) == 0)
      writeDigit(1, d5_offset, 24); //send all off to tens hour digit if == 0
    else
      writeDigit(1, d5_offset, (hours / 10));
    FastLED.show();
    }
  if(main_menu  == 3){ //display time so that it can be edited (updateTime is not being called)
    if(sub_menu == 10){ //display time so hours can be udpated
      writeDigit(1, d0_offset, (seconds % 10));
      writeDigit(1, d1_offset, (seconds / 10));
      writeDigit(1, d2_offset, (minutes % 10));
      writeDigit(1, d3_offset, (minutes / 10));
      writeDigit(1, d4_offset, (encoderPos % 10));
      if((encoderPos / 10) == 0)
        writeDigit(1, d5_offset, 24); //send all off to tens hour digit if == 0
      else
        writeDigit(1, d5_offset, (encoderPos / 10));
      FastLED.show();
    }
    else if(sub_menu == 11){ //display time so minutes can be udpated
      writeDigit(1, d0_offset, (seconds % 10));
      writeDigit(1, d1_offset, (seconds / 10));
      writeDigit(1, d2_offset, (encoderPos % 10));
      writeDigit(1, d3_offset, (encoderPos / 10));
      writeDigit(1, d4_offset, (hours % 10));
      if((hours / 10) == 0)
        writeDigit(1, d5_offset, 24); //send all off to tens hour digit if == 0
      else
        writeDigit(1, d5_offset, (hours / 10));
      FastLED.show();
    }
    else if(sub_menu == 12){ //display time so seconds can be udpated
      writeDigit(1, d0_offset, (encoderPos % 10));
      writeDigit(1, d1_offset, (encoderPos / 10));
      writeDigit(1, d2_offset, (minutes % 10));
      writeDigit(1, d3_offset, (minutes / 10));
      writeDigit(1, d4_offset, (hours % 10));
      if((hours / 10) == 0)
        writeDigit(1, d5_offset, 24); //send all off to tens hour digit if == 0
      else
        writeDigit(1, d5_offset, (hours / 10));
      FastLED.show();
    }    
  }    
}


void writeDigit(int type, int offset, int value){
  for( int i = 0 ; i < 21; i++ ){
    if( type == 1 ){
      if(digit[value][i]){ //main if statement to check against digit array for whether or not to turn on or off an LED
        if(sub_menu == 1){ //display LEDs to adjust color
          leds[offset + i] = CHSV(param_color, 255, encoderPos);
          leds[c0_offset] = CHSV(param_color, 255, encoderPos);
          leds[c1_offset] = CHSV(param_color, 255, encoderPos);
          leds[c2_offset] = CHSV(param_color, 255, encoderPos);
          leds[c3_offset] = CHSV(param_color, 255, encoderPos);        
        }
        else if(sub_menu == 2){ //display LEDs to adjust brightness
          leds[offset + i] = CHSV(encoderPos, 255, param_brightness);
          leds[c0_offset] = CHSV(encoderPos, 255, param_brightness);
          leds[c1_offset] = CHSV(encoderPos, 255, param_brightness);
          leds[c2_offset] = CHSV(encoderPos, 255, param_brightness);
          leds[c3_offset] = CHSV(encoderPos, 255, param_brightness);
        }
        else{  //MAIN display to show clock when not in a menu
          leds[offset + i] = CHSV(param_color, 255, param_brightness);
          leds[c0_offset] = CHSV(param_color, 255, param_brightness);
          leds[c1_offset] = CHSV(param_color, 255, param_brightness);
          leds[c2_offset] = CHSV(param_color, 255, param_brightness);
          leds[c3_offset] = CHSV(param_color, 255, param_brightness);
        }
      }
      else {
        leds[offset + i] = CRGB::Black;  //set LED to black/off if digit array has a value of 0 for a particluar LED
      }
    }
  }
}

void rotaryMenu() { 

  //if main menu isn't being displayed and short press is detected, call displayEnv to scroll through environment readings
  if(main_menu == 0 && isShortDetected){
    displayEnv();
    isShortDetected = false;
  }
  
  //if a long press is detected, go to main menu
  if (isLongDetected) {
    main_menu = 1;
    isLongDetected = false;
    }
  
  //scroll through text of main menu after long press   
  if(main_menu == 1){
    
    //display "SET" when menu is active
    if(encoderPos == 0){ 
      writeDigit(1, d0_offset, 24);
      writeDigit(1, d1_offset, 24);
      writeDigit(1, d2_offset, 24);
      writeDigit(1, d3_offset, 23);
      writeDigit(1, d4_offset, 13);
      writeDigit(1, d5_offset, 22);
      leds[c0_offset] = CRGB::Black;
      leds[c1_offset] = CRGB::Black;
      leds[c2_offset] = CRGB::Black;
      leds[c3_offset] = CRGB::Black;
      //Serial.println("turn off colons!");
      //Serial.println("display set");
      FastLED.show();
    }
    
    //display "brite" when menu is active
    else if (encoderPos == 1){
      writeDigit(1, d0_offset, 24);
      writeDigit(1, d1_offset, 13);
      writeDigit(1, d2_offset, 23);
      writeDigit(1, d3_offset, 16);
      writeDigit(1, d4_offset, 21);
      writeDigit(1, d5_offset, 11);
      leds[c0_offset] = CRGB::Black;
      leds[c1_offset] = CRGB::Black;
      leds[c2_offset] = CRGB::Black;
      leds[c3_offset] = CRGB::Black;
      FastLED.show();      
    }
    
    //display "color" when menu is active
    else if (encoderPos == 2){
      writeDigit(1, d0_offset, 24);
      writeDigit(1, d1_offset, 21);
      writeDigit(1, d2_offset, 19);
      writeDigit(1, d3_offset, 17);
      writeDigit(1, d4_offset, 19);
      writeDigit(1, d5_offset, 12);
      leds[c0_offset] = CRGB::Black;
      leds[c1_offset] = CRGB::Black;
      leds[c2_offset] = CRGB::Black;
      leds[c3_offset] = CRGB::Black;
      FastLED.show();      
    }
    
    //display "temp" when menu is active
    else if (encoderPos == 3){
      writeDigit(1, d0_offset, 24);
      writeDigit(1, d1_offset, 20);
      writeDigit(1, d2_offset, 18);
      writeDigit(1, d3_offset, 18);
      writeDigit(1, d4_offset, 13);
      writeDigit(1, d5_offset, 23);
      leds[c0_offset] = CRGB::Black;
      leds[c1_offset] = CRGB::Black;
      leds[c2_offset] = CRGB::Black;
      leds[c3_offset] = CRGB::Black;
      FastLED.show();      
    }
    
    //display "scroll" when menu is active
    else if (encoderPos == 4){
      writeDigit(1, d0_offset, 17);
      writeDigit(1, d1_offset, 17);
      writeDigit(1, d2_offset, 19);
      writeDigit(1, d3_offset, 21);
      writeDigit(1, d4_offset, 12);
      writeDigit(1, d5_offset, 22);
      leds[c0_offset] = CRGB::Black;
      leds[c1_offset] = CRGB::Black;
      leds[c2_offset] = CRGB::Black;
      leds[c3_offset] = CRGB::Black;
      FastLED.show();      
    }
  }
  if(main_menu == 1 && isShortDetected){
    main_menu = 2;
    isShortDetected = false;
    if(encoderPos == 0){      //enter set time menu
      main_menu = 3;
      sub_menu = 10;
      myEnc.write(hours*4); //set encoder to hours
    }
    else if(encoderPos == 1){ //enter brightness parameter edit 
      sub_menu = encoderPos;     
      myEnc.write(param_brightness*4); //set encoder to brightness value
    }
    else if(encoderPos == 2){ //enter color parameter edit 
      sub_menu = encoderPos;        
      myEnc.write(param_color*4);  //set encoder to color value
    }
    else if(encoderPos == 3){  //enter temp mode edit 
      sub_menu = encoderPos;      
      myEnc.write(param_corf*4); //set encoder to temp mode value
    }
    else if(encoderPos == 4){ //enter scroll time mode edit 
      sub_menu = encoderPos;      
      myEnc.write(param_scrolltime*4); //set encoder to scroll time value
    } 
        
  }else if(main_menu == 2){
    if(sub_menu == 1 && isShortDetected){ //save brightness parameter
      param_brightness = encoderPos;
      isShortDetected = false;
      sub_menu = 0;
      main_menu = 0;
      myEnc.write(0);      
    }
    else if(sub_menu == 2 && isShortDetected){ //save color parameter
      param_color = encoderPos;
      isShortDetected = false;
      sub_menu = 0;
      main_menu = 0;
      myEnc.write(0);
    }
    else if(sub_menu == 3 && isShortDetected){ //save temp mode parameter
      if(encoderPos % 2 == 0)
        param_corf = 0;
      else
        param_corf = 1;      
      isShortDetected = false;
      sub_menu = 0;
      main_menu = 0;
      myEnc.write(0);         
    }
    else if(sub_menu == 4 && isShortDetected){ //save scroll time parameter
      param_scrolltime = encoderPos;
      
      isShortDetected = false;
      sub_menu = 0;
      main_menu = 0;
      myEnc.write(0);
    }
    
  }else if(main_menu == 3){ //setting time menu
    
    if(sub_menu == 10 && isShortDetected){ //confirm hours
      isShortDetected = false;
      hours = encoderPos;
      sub_menu = 11;      
      myEnc.write(minutes*4);
    }
    else if(sub_menu == 11 && isShortDetected){ //confirm minutes
      isShortDetected = false;
      minutes = encoderPos;
      sub_menu = 12;      
      myEnc.write(seconds*4);
    }
    else if(sub_menu == 12 && isShortDetected){ //confirm seconds
      isShortDetected = false;
      seconds = encoderPos;
      sub_menu = 0;      
      main_menu = 0;
      myEnc.write(0);
      rtc.setTime(seconds, minutes, hours, 1, day, month, year);           
    }    
  }
  
  if (main_menu == 2 && sub_menu == 1){
    writeDigit(1, d0_offset, parse_int(encoderPos, 1));
    writeDigit(1, d1_offset, parse_int(encoderPos, 10));
    writeDigit(1, d2_offset, parse_int(encoderPos, 100));
    writeDigit(1, d3_offset, 24);
    writeDigit(1, d4_offset, 21);
    writeDigit(1, d5_offset, 11);
    leds[c0_offset] = CRGB::Black;
    leds[c1_offset] = CRGB::Black;
    leds[c2_offset] = CRGB::Black;
    leds[c3_offset] = CRGB::Black;
    FastLED.show();   
  }
  else if (main_menu == 2 && sub_menu == 2){
    writeDigit(1, d0_offset, parse_int(encoderPos, 1));
    writeDigit(1, d1_offset, parse_int(encoderPos, 10));
    writeDigit(1, d2_offset, parse_int(encoderPos, 100));
    writeDigit(1, d3_offset, 21);
    writeDigit(1, d4_offset, 17);
    writeDigit(1, d5_offset, 12);
    leds[c0_offset] = CRGB::Black;
    leds[c1_offset] = CRGB::Black;
    leds[c2_offset] = CRGB::Black;
    leds[c3_offset] = CRGB::Black;
    FastLED.show();
  }
  else if (main_menu == 2 && sub_menu == 3){
    if(encoderPos % 2 == 1)
      writeDigit(1, d0_offset, 14); //display deg F
    else
      writeDigit(1, d0_offset, 12); //display deg C
    writeDigit(1, d1_offset, 20);
    writeDigit(1, d2_offset, 18);
    writeDigit(1, d3_offset, 18);
    writeDigit(1, d4_offset, 13);
    writeDigit(1, d5_offset, 23);
    leds[c0_offset] = CRGB::Black;
    leds[c1_offset] = CRGB::Black;
    leds[c2_offset] = CRGB::Black;
    leds[c3_offset] = CRGB::Black;
    FastLED.show();
  }
  else if (main_menu == 2 && sub_menu == 4){
    writeDigit(1, d0_offset, parse_int(encoderPos, 1));
    writeDigit(1, d1_offset, parse_int(encoderPos, 10));
    writeDigit(1, d2_offset, 24);
    writeDigit(1, d3_offset, 21);
    writeDigit(1, d4_offset, 12);
    writeDigit(1, d5_offset, 22);
    leds[c0_offset] = CRGB::Black;
    leds[c1_offset] = CRGB::Black;
    leds[c2_offset] = CRGB::Black;
    leds[c3_offset] = CRGB::Black;
    FastLED.show();
  }
} 

//display temp, humidity, pressure, CO2 and VOC
void displayEnv(){
  
  getEnv();
  delay(50);
  getTemp();
  delay(50);
   
  if(param_corf){
    writeDigit(1, d0_offset, 14); //display deg F
    writeDigit(1, d1_offset, 10);
    writeDigit(1, d2_offset, 24);
  }else{
    writeDigit(1, d0_offset, 12); //display deg C
    writeDigit(1, d1_offset, 10);
    writeDigit(1, d2_offset, 24);
  }
  writeDigit(1, d3_offset, parse_int(temp*10, 1)); //gets (hopefully) the first decimal value
  writeDigit(1, d4_offset, parse_int(temp, 1));
  writeDigit(1, d5_offset, parse_int(temp, 10));
  leds[c0_offset] = CHSV(param_color, 255, param_brightness);
  leds[c1_offset] = CRGB::Black;
  leds[c2_offset] = CRGB::Black;
  leds[c3_offset] = CRGB::Black;
  FastLED.show();
  delay(param_scrolltime*1000);

  writeDigit(1, d0_offset, 15);
  writeDigit(1, d1_offset, 21);
  writeDigit(1, d2_offset, parse_int(humidity, 1));
  writeDigit(1, d3_offset, parse_int(humidity, 10));
  writeDigit(1, d4_offset, parse_int(humidity, 100));
  writeDigit(1, d5_offset, 24);
  leds[c0_offset] = CRGB::Black;
  leds[c1_offset] = CRGB::Black;
  leds[c2_offset] = CRGB::Black;
  leds[c3_offset] = CRGB::Black;
  FastLED.show();
  delay(param_scrolltime*1000);
  
  writeDigit(1, d0_offset, 24);
  writeDigit(1, d1_offset, parse_int(pressure/10, 1)); //gets the first decimal value
  writeDigit(1, d2_offset, parse_int(pressure/100, 1));
  writeDigit(1, d3_offset, parse_int(pressure/100, 10));
  writeDigit(1, d4_offset, parse_int(pressure/100, 100));
  writeDigit(1, d5_offset, parse_int(pressure/100, 1000));
  leds[c0_offset] = CRGB::Black;
  leds[c1_offset] = CRGB::Black;
  leds[c2_offset] = CHSV(param_color, 255, param_brightness);
  leds[c3_offset] = CRGB::Black;
  FastLED.show();
  Serial.println(pressure);
  delay(param_scrolltime*1000);

  writeDigit(1, d0_offset, parse_int(CO2, 1));
  writeDigit(1, d1_offset, parse_int(CO2, 10));
  writeDigit(1, d2_offset, parse_int(CO2, 100));
  writeDigit(1, d3_offset, parse_int(CO2, 1000));
  writeDigit(1, d4_offset, parse_int(CO2, 10000));
  writeDigit(1, d5_offset, 24);
  leds[c0_offset] = CRGB::Black;
  leds[c1_offset] = CRGB::Black;
  leds[c2_offset] = CRGB::Black;
  leds[c3_offset] = CRGB::Black;
  FastLED.show();
  delay(param_scrolltime*1000);

  writeDigit(1, d0_offset, parse_int(VOC, 1));
  writeDigit(1, d1_offset, parse_int(VOC, 10));
  writeDigit(1, d2_offset, parse_int(VOC, 100));
  writeDigit(1, d3_offset, parse_int(VOC, 1000));
  writeDigit(1, d4_offset, parse_int(VOC, 10000));
  writeDigit(1, d5_offset, 24);
  leds[c0_offset] = CRGB::Black;
  leds[c1_offset] = CRGB::Black;
  leds[c2_offset] = CRGB::Black;
  leds[c3_offset] = CRGB::Black;
  FastLED.show();
  delay(param_scrolltime*1000);
}

int parse_int(int val, int target){
  int a;
  switch (target) {
    case 1:
      a = (val%10);
      break;
    case 10:
      a = ((val/10)%10);
      if(a == 0 && val < 100) //make sure a 0 is displayed and not blanked out if not == 0 and less than 100
        a = 24;
      break;
    case 100:
      a = ((val/100)%10);
      if(a == 0)
        a = 24;
      break;
    case 1000:
      a = ((val/1000)%10);
      if(a == 0)
        a = 24;
      break;
    case 10000:
      a = ((val/10000)%10);
      if(a == 0)
        a = 24;
      break;
    default:
      a = 24;
      break;
  }
  return a;
}
