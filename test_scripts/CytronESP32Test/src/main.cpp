#include <Arduino.h>


// TODO : have a header for definitions
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
const int Channel = 0;
const int resolution = 8;//8 bit resolution
int dutyCycle  = 40;//test max speed
void setup() {
  for(int i=0;i<5;i++){
  // configure Motor PWM functionalitites
  ledcSetup(i, freq, resolution);
}

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(intake_motor_pin, 0);
  ledcAttachPin(left_motor_pin, 1);
  ledcAttachPin(right_motor_pin, 2);
  ledcAttachPin(H_motor_pin, 3);
  ledcAttachPin(puncher_motor_pin, 4);

  pinMode(intake_dir_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(H_dir_pin,OUTPUT);
  pinMode(puncher_dir_pin,OUTPUT);
}
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
void loop() {
  runDrive(1, 40);
  sideWays(1,0);
  digitalWrite(intake_dir_pin,HIGH);
  ledcWrite(Channel, dutyCycle);
  delay(1000);
  runDrive(1, 0);
  sideWays(-1,40);
  digitalWrite(intake_dir_pin,LOW);
  ledcWrite(Channel, dutyCycle);
  delay(1000);


}
