#include "main.h"

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
  int packet  = udp.parsePacket();
  if(udp.read(buffer, 50) > 0){
    Serial.print("recieved: ");
    Serial.println((char *)buffer);

    String message((char *)buffer);
    //Isolates command from number value, ex:"DA111222333444555666|23" => "111"
    String messageTemp = message;
    for (int i = 0; i < 3; i++) {
      pwmListStr [i] = message.substring(i * 3 + 1, 4 + i * 3);
    }
    for (int j = 0; j <3; j++) {
      pwmListInt[j] = pwmListStr[j].toInt()-255;
      Serial.println(pwmListInt[j]);
    }
    if(pwmListInt[0]>=0)
    runDrive(1, abs(pwmListInt[0]));
    else
    runDrive(-1, abs(pwmListInt[0]));
    if(pwmListInt[1]>=0)
    sideWays(1,abs(pwmListInt[1]));
    else
    sideWays(-1,abs(pwmListInt[1]));
    digitalWrite(intake_dir_pin,HIGH);
    ledcWrite(0, abs(pwmListInt[2]));

  }

}
