
//Libraries required for interfacing with the LED strips
//These are compatible with the Arduino IDE
//https://github.com/adafruit/Adafruit_NeoPixel
//https://github.com/FastLED/FastLED
#include <Adafruit_NeoPixel.h>
#include <FastLED.h>



/////////////////////////////////////////////////////////////////////////////////////
//Definitions 

#define NUM_LEDS 50   //total number of LEDs
#define PIN 25		  //which microcontroller pin is sending the signal 
//define the strip for the Adafruit_NeoPixel library
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);    
CRGB leds[NUM_LEDS];  //create an array holding each led in the strip 



/////////////////////////////////////////////////////////////////////////////////////
//The following functions are implementation functions that style functions use to execute the pattern 

//This function will assign a colour to an led based on which library functions are calling it 
void setPixel(int Pixel, byte red, byte green, byte blue) 
{
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

//This function will send the signal to the strip based on which libary functions are calling it 
void showStrip() 
{
#ifdef ADAFRUIT_NEOPIXEL_H
	//NeoPixel
	strip.show();
#endif
#ifndef ADAFRUIT_NEOPIXEL_H
	//FastLED
	FastLED.show();
#endif
}

//This function sets all leds in the strip to the same colour 
void setAll(byte red, byte green, byte blue) 
{
	for (int point = 0; point < NUM_LEDS; point++) 
	{
		setPixel(point, red, green, blue);
	}
	showStrip();
}



/////////////////////////////////////////////////////////////////////////////////////
//The following functions create different patterns on the strip

//This function fades from no light to full-intensity green in about 2.5 seconds 
void starting() 
{
	for (int k = 0; k < 256; k++) 
	{
		setAll(0, k, 0);
		showStrip();
		delay(10);
	}
}

//This function fades from full-intensity green to no light in about 2.5 seconds 
void shutting() 
{
	for (int k = 256; k > 0; k--)
	{
		setAll(0, k, 0);
		showStrip();
		delay(10);
	}
}

//This function creates a propogating sinusodal wave of one colour to indicate the robot 
//is driving forwards; the speed of propogation is WaveDelay  
void driveForward(byte red, byte green, byte blue, int WaveDelay) 
//WaveDelay is the milliseconds between each new wave position and it determines how
//fast the wave will move along the strip
{
	int pos = 0;
	for (int total = 0; total < NUM_LEDS; total++) //for one full cycle of the strip
	{
		pos++; //move the wave along the strip 
		for (int point = 0; point < NUM_LEDS; point++) 
		{
			setPixel(point, ((sin(point + pos) * 127 + 128) / 255) * red, 
							((sin(point + pos) * 127 + 128) / 255) * green, 
							((sin(point + pos) * 127 + 128) / 255) * blue);
		}
		showStrip();
		delay(WaveDelay);
	}
}

//This function creates a propogating sinusodal wave of one colour to indicate the robot 
//is driving forwards; the speed of propogation is WaveDelay  
void driveBackward(byte red, byte green, byte blue, int WaveDelay)
{
	int pos = 0;
	for (int total = NUM_LEDS; total > 0; total--) //for one full cycle of the strip
	{
		pos--; //move the wave along the strip
		for (int point = 0; point < NUM_LEDS; point++)
		{
			setPixel(point, ((sin(point + pos) * 127 + 128) / 255) * red, 
							((sin(point + pos) * 127 + 128) / 255) * green, 
							((sin(point + pos) * 127 + 128) / 255) * blue);
		}
		showStrip();
		delay(WaveDelay);
	}
}

//These functions cycle a blended rainbow over the length of the strip 
//with assistance from Tweaking4All Neopixel blog post)
void rainbow(int SpeedDelay) 
//SpeedDelay is similar to wave delay 
{
	byte* blend;
	uint16_t point, total;

	for (total = 0; total < 256; total++) 
	{ 
		for (point = 0; point < NUM_LEDS; point++) 
		{
			blend = Wheel(((point * 256 / NUM_LEDS) + total) & 255);
			setPixel(point, *blend, *(blend + 1), *(blend + 2));
		}
		showStrip();
		delay(SpeedDelay);
	}
}
byte* Wheel(byte WheelPos) //This function creates the hue each pixle should take 
{
	static byte c[3];

	if (WheelPos < 85) 
	{
		c[0] = WheelPos * 3;
		c[1] = 255 - WheelPos * 3;
		c[2] = 0;
	}
	else if (WheelPos < 170) 
	{
		WheelPos -= 85;
		c[0] = 255 - WheelPos * 3;
		c[1] = 0;
		c[2] = WheelPos * 3;
	}
	else 
	{
		WheelPos -= 170;
		c[0] = 0;
		c[1] = WheelPos * 3;
		c[2] = 255 - WheelPos * 3;
	}

	return c;
}



/////////////////////////////////////////////////////////////////////////////////////
//The rest of the code is the implementation of the above functions 

//Required steps to start sending signals to the LED strip 
void setup() 
{
	strip.begin();
	strip.show();
	//specify type of strip below
	FastLED.addLeds<WS2812B, PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip); 
}

void loop() {
	/*
	The following functions may be called to create the associated patterns below:

	- Solid, unchanging colour: setAll(byte red, byte green, byte blue)
	- Startup the robot led strips to full colour: starting()
	- Shutdown the robot led strips to no light: shutting()
	- Show the robot is driving forwards: driveForward(byte red, byte green, byte blue, int WaveDelay)
	- Show the robot is driving backwards: driveBackward(byte red, byte green, byte blue, int WaveDelay)
	- Create a cycling rainbow along the strip: rainbow(int SpeedDelay) 
    */
	

}