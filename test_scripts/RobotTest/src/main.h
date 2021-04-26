#include <Arduino.h>
#include "WiFi.h"
#include <WiFiUdp.h>
#define intake_motor_pin 13
#define intake_dir_pin 12
#define right_motor_pin 4
#define right_dir_pin 16
#define left_motor_pin 15
#define left_dir_pin 2
#define H_motor_pin 17
#define H_dir_pin 5
#define puncher_motor_pin 18
#define puncher_dir_pin 19
// setting PWM properties
const int freq = 5000;
const int resolution = 8;//8 bit resolution
String pwmListStr [3];
int pwmListInt [3];
const int udpPort = 8080;
//create UDP instance
WiFiUDP udp;
void runDrive(int dir,int speed){
  if(dir>0){
    digitalWrite(left_dir_pin,LOW);
    digitalWrite(right_dir_pin,LOW);
  }else{
    digitalWrite(left_dir_pin,HIGH);
    digitalWrite(right_dir_pin,HIGH);
  }
  ledcWrite(1, speed);
  ledcWrite(2, speed);
}
void sideWays(int dir,int speed){
  if(dir>0){
    digitalWrite(H_dir_pin,HIGH);
  }else{
    digitalWrite(H_dir_pin,LOW);
  }
  ledcWrite(3, speed);
}
