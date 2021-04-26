/*
 * Controls LEDs and Flywheel on BeJay using ros topics
 */

#include <ros.h>
#include <std_msgs/UInt8MultiArray.h>
ros::NodeHandle  nh;
#include "LED_SP.h"




void led( const std_msgs::UInt8MultiArray& led_msg){
//  uint8_t chosen_led_func = led_msg.data[0];
//  uint8_t r = led_msg.data[1];
//  uint8_t g = led_msg.data[2];
//  uint8_t b = led_msg.data[3];
//  uint8_t wave_delay = led_msg.data[5];
  // message structure: func,r,g,b,wave_delay, loop count, cooling_number, spark_number
  switch (led_msg.data[0]){// Send signal to LED
      case 0:
        // last argument in setAll:  0 right, 1 left, 2 for both
        setAll(led_msg.data[1],led_msg.data[2],led_msg.data[3],2);
        break;
      case 1:
        //ex: 1,45,139,186,5,3
        sad(led_msg.data[1],led_msg.data[2],led_msg.data[3], led_msg.data[4], led_msg.data[5]);
        break;
      case 2:
        //ex: 2,0,0,0,255,3
        fear(led_msg.data[5], led_msg.data[4]);
      case 3:
        //annoyed(cooling number, spark number, led_msg.data[4])
        //3,0,0,0,10,200,50,50 
        annoyed(led_msg.data[6], led_msg.data[7], led_msg.data[5], led_msg.data[4]);
        break;
      case 4:
        //ex:4,163,175,181,120,0
        tired(led_msg.data[1],led_msg.data[2],led_msg.data[3], led_msg.data[4]);
        break;
      case 5:
        //ex: 5,0,0,0,0,0
        excited(3);
        break;
      case 6:
        //center size is the argument after rgb 
        //ex: 6,66,185,245,100,10
        calm(led_msg.data[1],led_msg.data[2],led_msg.data[3],5, led_msg.data[4], led_msg.data[4]);
        break;
      case 7:
        //ex: 7,0,0,0,255,1
        surprise(led_msg.data[4], led_msg.data[5]);
        break;
      case 8:
        //ex: 8,0,0,0,5,0
        happy(led_msg.data[4]);
        break;
      default:
        break;
        
    }

  

}

ros::Subscriber<std_msgs::UInt8MultiArray> led_sub("led", led );



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
//  FastLED.addLeds<WS2812B, RIGHT_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
//  FastLED.addLeds<WS2812B, LEFT_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  //starting(); //initiate starting sequency for LED


}

void loop()
{
  nh.spinOnce();
  delay(5);
}
