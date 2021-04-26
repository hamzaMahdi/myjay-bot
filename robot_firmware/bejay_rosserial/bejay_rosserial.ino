
/*
 * Controls LEDs and Flywheel on BeJay using ros topics
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Int16.h>
ros::NodeHandle  nh;
//#include<Servo.h>
#include <Adafruit_TiCoServo.h>
#include "led.h"


// The flywheel uses a brushless motor and needs PWM
byte servoPin = 9;
#define SERVO_MIN 1000 // 1 ms pulse
#define SERVO_MAX 2000 // 2 ms pulse
Adafruit_TiCoServo servo;



void led( const std_msgs::UInt8MultiArray& led_msg){
//  uint8_t chosen_led_func = led_msg.data[0];
//  uint8_t r = led_msg.data[1];
//  uint8_t g = led_msg.data[2];
//  uint8_t b = led_msg.data[3];
//  uint8_t wave_delay = led_msg.data[5];
  // message structure: func,r,g,b,wave_delay
  switch (led_msg.data[0]){// Send signal to LED
      case 0:
        setAll(led_msg.data[1],led_msg.data[2],led_msg.data[3],2);
        break;
      case 1:
        starting();
        break;
      case 2:
        shutting();
      case 3:
        driveForward(led_msg.data[1],led_msg.data[2],led_msg.data[3],led_msg.data[4]);
        break;
      case 4:
        driveBackward(led_msg.data[1],led_msg.data[2],led_msg.data[3],led_msg.data[4]);
        break;
      case 5:
        // 0 right, 1 left
        setAll(led_msg.data[1],led_msg.data[2],led_msg.data[3],0);
        break;
      case 6:
        setAll(led_msg.data[1],led_msg.data[2],led_msg.data[3],1);
        break;
      case 7:
        rainbow(led_msg.data[4]);
        break;
      default:
        break;
        
    }

  

}

void flywheel( const std_msgs::Int16& vel){
//  servo.writeMicroseconds(vel.data); // Send signal to ESC.
servo.write(vel.data); // Send signal to ESC.
}

ros::Subscriber<std_msgs::UInt8MultiArray> led_sub("led", led );
ros::Subscriber<std_msgs::Int16> flywheel_sub("flywheel", flywheel );



void setup()
{

  //ros init stuff
  nh.initNode();
  nh.subscribe(flywheel_sub);
  nh.subscribe(led_sub);
  //Neo pixel LED array initialization
  right_strip.begin();
  left_strip.begin();
  right_strip.show();
  left_strip.show();
//  //specify type of strip below. There are two robot sides with identical LEDs
//  FastLED.addLeds<WS2812B, RIGHT_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
//  FastLED.addLeds<WS2812B, LEFT_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  starting(); //initiate starting sequency for LED


  //Setup Flywheel
  servo.attach(servoPin, SERVO_MIN, SERVO_MAX);
  //servo.writeMicroseconds(1500); // send "stop" signal to ESC.
  servo.write(1500); // Send signal to ESC.
  delay(7000); // delay to allow the ESC to recognize the stopped signal

}

void loop()
{
  nh.spinOnce();
  delay(5);
}
