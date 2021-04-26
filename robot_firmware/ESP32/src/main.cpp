#include <Arduino.h>

//Includes required to use Roboclaw library
#include <motor_functions.h>
#include <led.h>
int last_seen = 0;
void setup() {
  //Communciate with roboclaw at 38400bp
  roboclaw.begin(38400);


  // wifi setup
  WiFi.disconnect(true);
  delay(1000);
  const char* ssid = "Robot";
  const char* pass = "something";
  WiFi.softAP(ssid, pass);
  //debugging
  //Serial.begin(9600);
  //Serial.println(WiFi.softAPIP());
  //Serial.println(WiFi.localIP());
  //This initializes udp and transfer buffer
  udp.begin(udpPort);



  //Neo pixel LED array initialization
  strip.begin();
	strip.show();
	//specify type of strip below
	FastLED.addLeds<WS2812B, PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  starting(); //initiate starting sequency for LED
}

void loop() {
  uint8_t buffer[50];
  int packet  = udp.parsePacket();

  if(udp.read(buffer, 50) > 0){
    last_seen= millis();
    //Serial.print("recieved: ");
    //Serial.println((char *)buffer); //debugging purposes only
    String message((char *)buffer);

    //Isolates command from number value, ex:"D111222333444555666|23" => "111"
    //String messageTemp = message;

    // parse values
    for (int i = 0; i < message_size; i++) {
      pwmListStr [i] = message.substring(i * 3 + 1, 4 + i * 3);
      pwmListInt[i] = pwmListStr[i].toInt();
    }

    // send pwm values to functions
    drive(pwmListInt[0], pwmListInt[1], pwmListInt[2], pwmListInt[3]);
    run_intake(pwmListInt[4]);
    run_elevator(pwmListInt[5]);
    }

    // stop the robot in case of a connection loss
    if(millis()-last_seen > 2000){
      stop_drive();
      run_intake(127/2);
      run_elevator(127/2);
    }


}
