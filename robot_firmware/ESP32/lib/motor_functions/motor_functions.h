//should make this into header and cpp file later
#ifndef MOTOR_FUNCTIONS
#define MOTOR_FUNCTIONS
#include "RoboClaw.h"
#include "WiFi.h"
#include <WiFiUdp.h>

//some initializations
//the addrsses for the roboclaws are set in motion studio
#define right_drive 0x80
#define intake 0x81
#define left_drive 0x82
RoboClaw roboclaw(&Serial2,10000);

//udp stuff
const int udpPort = 8080;
WiFiUDP udp;

//there are 4 messages being transmitted currently
const int message_size = 6;
String pwmListStr [message_size];
int pwmListInt [message_size];


void drive(int l1,int l2,int r1,int r2){
  roboclaw.ForwardBackwardM1(left_drive, l1);
  roboclaw.ForwardBackwardM2(left_drive, l2);
  roboclaw.ForwardBackwardM2(right_drive, r1);
  roboclaw.ForwardBackwardM1(right_drive, r2);
}

void run_intake(int speed){
  roboclaw.ForwardBackwardM1(intake, speed);
}

void run_elevator(int speed){
  roboclaw.ForwardBackwardM2(intake, speed);
}

void stop_drive(){
  roboclaw.ForwardM1(left_drive, 0);
  roboclaw.ForwardM2(left_drive, 0);
  roboclaw.ForwardM1(right_drive, 0);
  roboclaw.ForwardM2(right_drive, 0);
}


#endif
