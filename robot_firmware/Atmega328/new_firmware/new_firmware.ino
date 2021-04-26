#include "led.h"
#include <Servo.h>

// The flywheel uses a brushless motor and needs PWM
byte servoPin = 11;
Servo servo;


void setup() {
 

  //Neo pixel LED array initialization
  right_strip.begin();
  left_strip.begin();
  right_strip.show();
  left_strip.show();
  //specify type of strip below. There are two robot sides with identical LEDs
  FastLED.addLeds<WS2812B, RIGHT_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812B, LEFT_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  starting(); //initiate starting sequency for LED

   //Setup Flywheel
  servo.attach(servoPin);
  servo.writeMicroseconds(1500); // send "stop" signal to ESC.
  delay(7000); // delay to allow the ESC to recognize the stopped signal
    
  // Begin Serial
  // Serial expects int,int,int,int,int,int
  //                pwm,func,r,g,b,wave_delay
  //                1600,0,100,0,0,10
  Serial.begin(9600);

}

void loop() {
  bool recieved = false; // Receive data
  int pwm = 1500; // Set signal value, which should be between 1100 and 1900
                // Neutral is 1500
  //0: set all
  //1: starting
  //2: shutting
  //3: driveForward
  //4: driveBackward
  //5: rainbow
  int chosen_led_func = 0;
  // currently not optimized, rgb can be bytes
  int r,g,b;// RGB values
  r=g=b=0;
  int wave_delay=0;// Step in ms
  if (Serial.available()){
    //expected data format is int, int, int
    int input0 = Serial.parseInt();
    int input1 = Serial.parseInt();
    int input2 = Serial.parseInt();
    int input3 = Serial.parseInt();
    int input4 = Serial.parseInt();
    int input5 = Serial.parseInt();
    if (Serial.read() == '\n'){//check for new line character
      pwm=input0;
      chosen_led_func = input1;
      r = input2;
      g = input3;
      b = input4;
      wave_delay = input5;
//      Serial.print(pwm);
//      Serial.print(input1);
//      Serial.print(input2);
      recieved = true;
    }
   }
   if (recieved){
    servo.writeMicroseconds(pwm); // Send signal to ESC.
    delay(200);
    switch (chosen_led_func){// Send signal to LED
      case 0:
        setAll(r,g,b);
        break;
      case 1:
        starting();
        break;
      case 2:
        shutting();
      case 3:
        driveForward(r,g,b,wave_delay);
        break;
      case 4:
        driveBackward(r,g,b,wave_delay);
        break;
      case 5:
        rainbow(wave_delay);
        break;
      default:
        break;
        
    }
   }
  // put your main code here, to run repeatedly:

}
