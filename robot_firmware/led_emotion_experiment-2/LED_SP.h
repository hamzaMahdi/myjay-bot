//Shahed Saleh 
//Nov 19th, 2020
 
#ifndef LED_SP_
#define LED_SP_

//Libraries required for interfacing with the LED strips
//These are compatible with the Arduino IDE
//https://github.com/adafruit/Adafruit_NeoPixel
//https://github.com/FastLED/FastLED
#include <Adafruit_NeoPixel.h>
#include <FastLED.h>

//BEFORE YOU BEGIN, CHECK WHICH STRIP IS BIG USED AND COUNT HOW MANY LEDS ARE ON THE STRIP

/////////////////////////////////////////////////////////////////////////////////////
//Definitions 


#define RIGHT_PIN 6        //which microcontroller pin is sending the signal 
#define LEFT_PIN 7
#define NUM_LEDS  40   //total number of LEDs
//define the strip for the Adafruit_NeoPixel library
Adafruit_NeoPixel right_strip = Adafruit_NeoPixel(NUM_LEDS, RIGHT_PIN, NEO_GRB + NEO_KHZ800); 
Adafruit_NeoPixel left_strip = Adafruit_NeoPixel(NUM_LEDS, LEFT_PIN, NEO_GRB + NEO_KHZ800);

//create an array holding each led in the strip
CRGB leds[NUM_LEDS];   



/////////////////////////////////////////////////////////////////////////////////////
//The following functions are implementation functions that style functions use to execute the pattern 

//This function will assign a colour to an led based on which library functions are calling it 
void setPixel(int Pixel, byte red, byte green, byte blue) 
{
#ifdef ADAFRUIT_NEOPIXEL_H
  // NeoPixel
  left_strip.setPixelColor(Pixel, left_strip.Color(red, green, blue));
  right_strip.setPixelColor(Pixel, right_strip.Color(red, green, blue));
#endif

#ifndef ADAFRUIT_NEOPIXEL_H
  // FastLED
  leds[Pixel].r = red;
  leds[Pixel].g = green;
  leds[Pixel].b = blue;
#endif
}

//This function will send the signal to the strip based on which libary functions are calling it 
void showStrip() 
{
#ifdef ADAFRUIT_NEOPIXEL_H
  //NeoPixel
  right_strip.show();
  left_strip.show();
#endif

#ifndef ADAFRUIT_NEOPIXEL_H
  //FastLED
  FastLED.show();
#endif
}

//This function sets all leds in the strip to the same colour 
void setAll(byte red, byte green, byte blue){
  for (int point = 0; point < NUM_LEDS; point++){
    setPixel(point, red, green, blue);
  }
  showStrip();
}



/////////////////////////////////////////////////////////////////////////////////////
//The following functions create different patterns on the strip



// sends a wave down and back the strip
void pattern(float evaluation, float activation){
  
  
  int EyeSize, SpeedDelay, ReturnDelay;
  byte red, green, blue;

  if (evaluation < 0){
    red = 255;
    green = blue = 255 / 2.41 * (evaluation + 2.41);
  }
  
  else if (evaluation == 0){
    red = green = blue = 255;
  }
  
  else{
    green = 255;
    red = blue = 255 / 3.45 * (3.45 - evaluation);
  }
  
  SpeedDelay = int(200 / 5.58 * (2.79 - activation));
  ReturnDelay = SpeedDelay;

  EyeSize = int(NUM_LEDS/ 2 / 5.58 * (activation + 2.79));


  
  for(int i = 0; i < NUM_LEDS - EyeSize -2; i++){
    setAll(0,0,0);
    setPixel(i, red / 10, green / 10, blue / 10);
    
    for(int j = 1; j <= EyeSize; j++){
      setPixel(i + j, red, green, blue);
    }
    
    setPixel(i + EyeSize + 1, red / 10, green / 10, blue / 10);
    showStrip();
    delay(SpeedDelay);
  }

  delay(ReturnDelay);

  for(int i = NUM_LEDS - EyeSize - 2; i > 0; i--){
    setAll(0,0,0);
    setPixel(i, red / 10, green / 10, blue / 10);
    
    for(int j = 1; j <= EyeSize; j++){
      setPixel(i + j, red, green, blue);
    }
    
    setPixel(i + EyeSize + 1, red / 10, green / 10, blue / 10);
    showStrip();
    delay(SpeedDelay);
  }
 
  delay(ReturnDelay);
}
  
#endif
