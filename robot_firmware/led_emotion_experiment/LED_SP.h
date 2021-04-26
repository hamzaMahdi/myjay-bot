//Shahed Saleh 
//Oct 26th, 2020


/* happy(delay number)
 * 
 * sad(red, green, blue, delay number, looping count)
 * 
 * fear(looping count, delay number) {
 * 
 * annoyed(cooling number, spark number, delay number)
 * 
 * tired(red, green, blue, delay number)
 * 
 * excited(number of balls)
 * 
 * calm(red, green, blue, center size, delay number, return delay number)
 * 
 * surprise(delay number, looping count)
 */
 
//should make this into header and cpp file later
#ifndef LED_H_
#define LED_H_
//Libraries required for interfacing with the LED strips
//These are compatible with the Arduino IDE
//https://github.com/adafruit/Adafruit_NeoPixel
//https://github.com/FastLED/FastLED
#include <Adafruit_NeoPixel.h>
#include <FastLED.h>



/////////////////////////////////////////////////////////////////////////////////////
//Definitions

#define NUM_LEDS 40   //total number of LEDs
#define RIGHT_PIN 6      //which microcontroller pin is sending the signal
#define LEFT_PIN 7


//define the strip for the Adafruit_NeoPixel library
Adafruit_NeoPixel right_strip = Adafruit_NeoPixel(NUM_LEDS, RIGHT_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel left_strip = Adafruit_NeoPixel(NUM_LEDS, LEFT_PIN, NEO_GRB + NEO_KHZ800);
CRGB leds[NUM_LEDS];  //create an array holding each led in the strip



/////////////////////////////////////////////////////////////////////////////////////
//The following functions are implementation functions that style functions use to execute the pattern

//This function will assign a colour to an led based on which library functions are calling it
void setPixel(int Pixel, byte red, byte green, byte blue)
{
#ifdef ADAFRUIT_NEOPIXEL_H
  // NeoPixel
  right_strip.setPixelColor(Pixel, right_strip.Color(red, green, blue));
  left_strip.setPixelColor(Pixel, left_strip.Color(red, green, blue));
#endif
#ifndef ADAFRUIT_NEOPIXEL_H
  // FastLED
  leds[Pixel].r = red;
  leds[Pixel].g = green;
  leds[Pixel].b = blue;
#endif
}

//This function will send the signal to the strip based on which libary functions are calling it
void showStrip(byte strip_choice)
{
#ifdef ADAFRUIT_NEOPIXEL_H
  //NeoPixel
  switch (strip_choice){
    case 0:
      right_strip.show();
      break;
    case 1:
      left_strip.show();
      break;
    default:
      right_strip.show();
      left_strip.show();
      break;
     
  }
  
#endif
#ifndef ADAFRUIT_NEOPIXEL_H
  //FastLED
  FastLED.show();
#endif
}

//This function sets all leds in the strip to the same colour
void setAll(byte red, byte green, byte blue, byte led_side)
{
  for (int point = 0; point < NUM_LEDS; point++)
  {
    setPixel(point, red, green, blue);
  }
  showStrip(led_side);
}



//These functions cycle a blended rainbow over the length of the strip
//with assistance from Tweaking4All Neopixel blog post)
byte* Wheel(byte WheelPos) //This function creates the hue each pixle should take
{
  static byte c[3];

  if (WheelPos < 85)
  {
    c[0] = 174+WheelPos;
    c[1] = 100 - WheelPos;
    c[2] = 0;
  }
  else if (WheelPos < 170)
  {
    WheelPos -= 85;
    c[0] = 255 - WheelPos;
    c[1] = 150+WheelPos-4;
    c[2] = 174+WheelPos-4;
  }
  else
  {
    
    WheelPos -= 170;
    c[0] = 255;
    c[1] = 150;
    c[2] = 255 - WheelPos;
  }

  return c;
}

void setPixelHeatColor (int pixle, byte temp) {
  
  // Scale heat down from 0-255 to 0-191
  byte t192 = round((temp / 255.0) * 191);
 
  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252
 
  //based on which third of the strip:

  //fire end 
  if( t192 > 0x80){                    
    setPixel(pixle, 255, 255, heatramp);
  } 
  //middle
  else if( t192 > 0x40 ){            
    setPixel(pixle, 255, heatramp, 0);
  } 
  //cool end 
  else{                      
    setPixel(pixle, heatramp, 0, 0);
  }
}



/////////////////////////////////////////////////////////////////////////////////////
//The following functions create different patterns on the strip

//HAPPY ------------------------------------------------------------------------------- NEEDS TESTING
//cycles blended orange, yellow and green
void happy(int SpeedDelay){
//SpeedDelay is similar to wave delay
  byte* blend;
  uint16_t point, total;

  for (total = 0; total < 256; total++){ 
    for (point = 0; point < NUM_LEDS; point++){
      blend = Wheel(((point * 256 / NUM_LEDS) + total) & 255);
      setPixel(point, *(blend)/2, *(blend + 1), 0);
    }
    showStrip(2);
    delay(SpeedDelay);
  }
}


//SAD ------------------------------------------------------------------------------------------- DONE
//cycles a fade in and out of a colour 
void sad(byte red, byte green, byte blue, int SpeedDelay, int count){
  
  float r, g, b;

  for(int i = 0; i < count; i++){
    
    for(int k = 0; k < 256; k = k+1){
      r = (k / 256.0) * red;
      g = (k / 256.0) * green;
      b = (k / 256.0) * blue;
      
      setAll(r,g,b,2);
      showStrip(2);
      delay(SpeedDelay);
    }
       
    for(int k = 255; k >= 0; k = k-1){
      r = (k / 256.0) * red;
      g = (k / 256.0) * green;
      b = (k / 256.0) * blue;
      
      setAll(r,g,b,2);
      showStrip(2);
      delay(SpeedDelay);
    }
  }
}




//FEAR ----------------------------------------------------------------------------------------- DONE
//random spots of violet of various hues  
void fear(int count, int SpeedDelay) {
  
  setAll(0,0,0,2);

  for(int last = 0; last < count; last++){
    for (int i = 0; i < NUM_LEDS; i++) {
       setPixel(i,random(75, 200),0,random(50,255));
    }
    showStrip(2);
    delay(SpeedDelay);
  }
  
}




//ANNOYED -------------------------------------------------------------------------------------- DONE
//flickers a fire-like pattern from one end of the strip
void annoyed(int cool, int spark,uint8_t loop_count, int SpeedDelay) {
  for (int i=0;i<loop_count;i++){
  
  static byte heat[NUM_LEDS];
  int cooled;
 
  //this loop cools each point 
  for(int point = 0; point < NUM_LEDS; point++){
    
    cooled = random (0, ((cool * 10) / NUM_LEDS) + 2);
   
    if(cooled > heat[point]){
      heat[point] = 0;
    } 
    else{
      heat[point] = heat[point] - cooled;
    }
  }
 
  //diffuses heated points along the strip
  for(int point = NUM_LEDS - 1; point >= 2; point--){
    heat[point] = (heat[point - 1] + heat[point - 2] + heat[point - 2]) / 3;
  }
   
  //Create sparks at end of strip
  if( random(255) < spark ){
    int num = random(7);
    heat[num] = heat[num] + random(160, 255);
    //heat[num] = random(160, 255);
  }

  //make heat colours
  for(int point = 0; point < NUM_LEDS; point++){
    setPixelHeatColor(point, heat[point]);
  }

  showStrip(2);
  delay(SpeedDelay);
  }
}






//TIRED ----------------------------------------------------------------------------------------- DONE
//waves of grey
void tired(byte red, byte green, byte blue, int WaveDelay){
  int pos = 0;
  for (int total = 0; total < NUM_LEDS; total++){
    pos++; //move the wave along the strip 
    for (int point = 0; point < NUM_LEDS; point++){
      setPixel(point, ((sin(point + pos) * 127 + 128) / 255) * red, 
              ((sin(point + pos) * 127 + 128) / 255) * green, 
              ((sin(point + pos) * 127 + 128) / 255) * blue);
    }
    showStrip(2);
    delay(WaveDelay);
  }
}


//EXCITED ------------------------------------------------------------------------------------- 
// bouncing balls of red, orange and yellow
void excited(int BallCount){

  byte colors[3][3] = {{255, 93, 43}, {255, 205, 41}, {234, 255, 41}};
  
  float Gravity = -3;
  int StartHeight = 1;
 
  float Height[BallCount];
  float ImpactVelocityStart = sqrt( -2 * Gravity * StartHeight );
  float ImpactVelocity[BallCount];
  float TimeSinceLastBounce[BallCount];
  int   Position[BallCount];
  long  ClockTimeSinceLastBounce[BallCount];
  float Dampening[BallCount];
 
  for (int i = 0 ; i < BallCount ; i++){  
    ClockTimeSinceLastBounce[i] = millis();
    Height[i] = StartHeight;
    Position[i] = 0;
    ImpactVelocity[i] = ImpactVelocityStart;
    TimeSinceLastBounce[i] = 0;
    Dampening[i] = 0.90 - float(i)/pow(BallCount,2);
  }

  for (int i=0;i<1000;i++){
    for (int i = 0 ; i < BallCount ; i++) {
      TimeSinceLastBounce[i] =  millis() - ClockTimeSinceLastBounce[i];
      Height[i] = 0.5 * Gravity * pow( TimeSinceLastBounce[i]/1000 , 2.0 ) + ImpactVelocity[i] * TimeSinceLastBounce[i]/1000;
 
      if ( Height[i] < 0 ) {                      
        Height[i] = 0;
        ImpactVelocity[i] = Dampening[i] * ImpactVelocity[i];
        ClockTimeSinceLastBounce[i] = millis();
 
        if ( ImpactVelocity[i] < 0.01 ) {
          ImpactVelocity[i] = ImpactVelocityStart;
        }
      }
      Position[i] = round( Height[i] * (NUM_LEDS - 1) / StartHeight);
    }
 
    for (int i = 0 ; i < BallCount ; i++) {
      setPixel(Position[i],colors[i][0],colors[i][1],colors[i][2]);
    }
   
    showStrip(2);
    setAll(0,0,0,2);
  }
}



//CALM ---------------------------------------------------------------------------------------- DONE
// sends a wave down and back the strip
void calm(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay){

  for(int i = 0; i < NUM_LEDS - EyeSize -2; i++){
    setAll(0,0,0,2);
    setPixel(i, red / 10, green / 10, blue / 10);
    
    for(int j = 1; j <= EyeSize; j++){
      setPixel(i + j, red, green, blue);
    }
    
    setPixel(i + EyeSize + 1, red / 10, green / 10, blue / 10);
    showStrip(2);
    delay(SpeedDelay);
  }

  delay(ReturnDelay);

  for(int i = NUM_LEDS - EyeSize - 2; i > 0; i--){
    setAll(0,0,0,2);
    setPixel(i, red / 10, green / 10, blue / 10);
    
    for(int j = 1; j <= EyeSize; j++){
      setPixel(i + j, red, green, blue);
    }
    
    setPixel(i + EyeSize + 1, red / 10, green / 10, blue / 10);
    showStrip(2);
    delay(SpeedDelay);
  }
 
  delay(ReturnDelay);
}





//SURPRISE 
// flash white, pink, orange, green ------------------------------------------------------------ DONE
void surprise(int SpeedDelay, int count){
    setAll(255,255,255,2);
    showStrip(2);
    delay(SpeedDelay);
    
    setAll(255,128,195,2);
      
    showStrip(2);
    delay(SpeedDelay);
    
    setAll(255,149,56,2);
      
    showStrip(2);
    delay(SpeedDelay);

    setAll(116,217,43,2);
      
    showStrip(2);
    delay(SpeedDelay);

  
}
  
#endif
