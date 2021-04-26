#include <Adafruit_NeoPixel.h>
#include <FastLED.h>
#include <Arduino.h>
#include "WiFi.h"
#include <WiFiUdp.h>

#define NUM_LEDS 50
#define PIN 25
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

CRGB leds[NUM_LEDS];

const int udpPort = 8080;
//create UDP instance
WiFiUDP udp;
String message;
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < NUM_LEDS; i++ ) {
    setPixel(i, red, green, blue);
  }
  showStrip();
}

void rainbowCycle(int SpeedDelay) {
  byte *c;
  uint16_t i, j;

  for(j = 0; j < 256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< NUM_LEDS; i++) {
      c = Wheel(((i * 256 / NUM_LEDS) + j) & 255);
      setPixel(i, *c, *(c+1), *(c+2));
    }
    showStrip();
    delay(SpeedDelay);
  }
}

byte * Wheel(byte WheelPos) {
  static byte c[3];
 
  if(WheelPos < 85) {
   c[0]=WheelPos * 3;
   c[1]=255 - WheelPos * 3;
   c[2]=0;
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   c[0]=255 - WheelPos * 3;
   c[1]=0;
   c[2]=WheelPos * 3;
  } else {
   WheelPos -= 170;
   c[0]=0;
   c[1]=WheelPos * 3;
   c[2]=255 - WheelPos * 3;
  }

  return c;
}

void showStrip() {
 #ifdef ADAFRUIT_NEOPIXEL_H
   //NeoPixel
   strip.show();
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   //FastLED
   FastLED.show();
 #endif
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
 #ifdef ADAFRUIT_NEOPIXEL_H
   // NeoPixel
   strip.setPixelColor(Pixel, strip.Color(red, green, blue));
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   leds[Pixel].r = red;
   leds[Pixel].g = green;
   leds[Pixel].b = blue;
 #endif
}


void startup(){
    for(int k = 0; k < 256; k++) {
      
      setAll(0,0,k); 
      showStrip();
      delay(40);
    }
}

void oneColour(int red, int green, int blue)
{
  setAll(red, green, blue);
}



void RunningLightsForward(byte red, byte green, byte blue, int WaveDelay) {
  int Position = 0;
  for(int j=0; j<NUM_LEDS*2; j++)
  {
      Position++; // = 0; //Position + Rate;
      for(int i=0; i<NUM_LEDS; i++) {
        setPixel(i,((sin(i+Position) * 127 + 128)/255)*red,
                   ((sin(i+Position) * 127 + 128)/255)*green,
                   ((sin(i+Position) * 127 + 128)/255)*blue);
      }
      showStrip();
      delay(WaveDelay);
  }
}


void RunningLightsBackward(byte red, byte green, byte blue, int WaveDelay) {
  int Position=0;
 
  for(int j=NUM_LEDS*2; j>0; j--)
  {
      Position--; // = 0; //Position + Rate;
      for(int i=0; i<NUM_LEDS; i++) {
        // sine wave, 3 offset waves make a rainbow!
        //float level = sin(i+Position) * 127 + 128;
        //setPixel(i,level,0,0);
        //float level = sin(i+Position) * 127 + 128;
        setPixel(i,((sin(i+Position) * 127 + 128)/255)*red,
                   ((sin(i+Position) * 127 + 128)/255)*green,
                   ((sin(i+Position) * 127 + 128)/255)*blue);
      }
     
      showStrip();
      delay(WaveDelay);
  }
}
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||


void setup() { 
  strip.begin();
  strip.show(); 
  FastLED.addLeds<WS2812B, PIN, GRB>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );

  Serial.begin(115200);

  //Resets the wifi connection, solved a bug where no communication took place between two wemos's
  WiFi.disconnect(true);
  delay(1000);
  const char* ssid = "Robot";
  const char* pass = "something";

  WiFi.softAP(ssid, pass);
  Serial.println(WiFi.softAPIP());
  Serial.println(WiFi.localIP());
  //This initializes udp and transfer buffer
  udp.begin(udpPort);
}

void loop() {


  uint8_t buffer[50];
  int packet = udp.parsePacket();
  if(udp.read(buffer, 50) > 0){
    Serial.print("recieved: ");
    Serial.println((char *)buffer);

    String message((char *)buffer);
    Serial.print(message.substring(0).toInt());
    
  if (message.substring(0).toInt() == 1){
    startup();
    delay(50);}
  else if (message.substring(0).toInt() == 2){
    rainbowCycle(3);
    delay(50); }
  else if (message.substring(0).toInt() == 3){
    RunningLightsForward(0xff,0xff,0x00, 50);
    delay(50); }
  else if (message.substring(0).toInt() == 4){
    RunningLightsBackward(0xff,0xff,0x00, 50);
    delay(50);}
  else if (message.substring(0).toInt() == 5){
    oneColour(255,0,0);
     delay(2000);
}
  
   
    }
}
