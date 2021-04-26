/*
 * Controls LEDs and Flywheel on BeJay using ros topics
 */

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
ros::NodeHandle  nh;
#include "LED_SP.h"




void led( const std_msgs::Float32MultiArray& led_msg){

  // message structure: Evaluation, Activity, (small,big)
  pattern(led_msg.data[0], led_msg.data[1]);  

}

ros::Subscriber<std_msgs::Float32MultiArray> led_sub("led", led );



void setup()
{

  //ros init stuff
  nh.initNode();
  nh.subscribe(led_sub);
  //Neo pixel LED array initialization
  right_strip.begin();
  left_strip.begin();
  right_strip.show();
  left_strip.show();
//  //specify type of strip below. There are two robot sides with identical LEDs
 // FastLED.addLeds<WS2812B, RIGHT_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
 // FastLED.addLeds<WS2812B, LEFT_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  //starting(); //initiate starting sequency for LED
  delay(2000);

}

void loop()
{
  nh.spinOnce();
  delay(20);
}
