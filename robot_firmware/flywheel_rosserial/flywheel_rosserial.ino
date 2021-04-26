
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
//#include "led.h"


// The flywheel uses a brushless motor and needs PWM
byte servoPin = 9;
#define SERVO_MIN 1000 // 1 ms pulse
#define SERVO_MAX 2000 // 2 ms pulse
Adafruit_TiCoServo servo;



void flywheel( const std_msgs::Int16& vel){
//  servo.writeMicroseconds(vel.data); // Send signal to ESC.
servo.write(vel.data); // Send signal to ESC.
}
ros::Subscriber<std_msgs::Int16> flywheel_sub("flywheel", flywheel );



void setup()
{

  //ros init stuff
  nh.initNode();
  nh.subscribe(flywheel_sub);
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
