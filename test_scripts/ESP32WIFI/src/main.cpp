#include <Arduino.h>

#include "WiFi.h"
#include <WiFiUdp.h>
String pwmListStr [3];
//int pwmListInt [3];
const int udpPort = 8080;
//create UDP instance
WiFiUDP udp;
void setup(){
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

void loop(){
  uint8_t buffer[50];
  int packet  = udp.parsePacket();
  if(udp.read(buffer, 50) > 0){
    Serial.print("recieved: ");
    Serial.println((char *)buffer);

  String message((char *)buffer);
  //Isolates command from number value, ex:"DA111222333444555666|23" => "111"
  //String messageTemp = message;
  for (int i = 0; i < 3; i++) {
    pwmListStr [i] = message.substring(i * 3 + 1, 4 + i * 3);
  }
  for (int j = 0; j <= 3; j++) {
    //pwmListInt[j] = pwmListStr[j].toInt();
    //Serial.println(pwmListInt[j]);
  }
}
  //Wait for 1 second
  delay(1000);
}
//comment
