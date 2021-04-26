/*#include <RoboClaw.h>
#include <SoftwareSerial.h>


SoftwareSerial serial(7,8); 
RoboClaw roboclaw(&serial,10000);

#define address 0x80

void setup() {
  //Open roboclaw serial ports
  roboclaw.begin(38400);
}

void loop() {
  roboclaw.ForwardM1(address,64); //start Motor1 forward at half speed
  roboclaw.BackwardM2(address,64); //start Motor2 backward at half speed
  delay(2000);

  roboclaw.BackwardM1(address,64);
  roboclaw.ForwardM2(address,64);
  delay(2000);

  roboclaw.ForwardBackwardM1(address,96); //start Motor1 forward at half speed
  roboclaw.ForwardBackwardM2(address,32); //start Motor2 backward at half speed
  delay(2000);

  roboclaw.ForwardBackwardM1(address,32);
  roboclaw.ForwardBackwardM2(address,96);
  delay(2000);
}
*/

#include <RoboClaw.h>

RoboClaw roboclaw(&Serial2, 10000); 

void setup() {
  roboclaw.begin(38400); //baud rate
}

void loop() {
  roboclaw.ForwardM1(0x80, 120); //the motor at a secific roboclaw is asseigned a speed
  roboclaw.ForwardM2(0x80, 120);
  roboclaw.ForwardM1(0x81, 120);
  roboclaw.ForwardM2(0x81, 120);
  roboclaw.ForwardM1(0x82, 120);
  roboclaw.ForwardM2(0x82, 120);
  delay(2000);
  roboclaw.ForwardM1(0x80, 0);
  roboclaw.ForwardM2(0x80, 0);
  roboclaw.ForwardM1(0x81, 0);
  roboclaw.ForwardM2(0x81, 0);
  roboclaw.ForwardM1(0x82, 0);
  roboclaw.ForwardM2(0x82, 0);
  delay(2000);
}
