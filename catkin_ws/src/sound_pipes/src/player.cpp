#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <memory>
#include <unistd.h>

void playerCallback(const std_msgs::String::ConstPtr& msg) {
  FILE *gp;
  // const std::string filename = "warning1.mp3"; // filename of mp3 file
  std::string message;
  if (msg->data.find(".mp3") != std::string::npos) {
    message = "mpg123 -q " + msg->data;
  } else if (msg->data.find(".wav") != std::string::npos) {
    message = "aplay " + msg->data;
  }else{
    ROS_INFO("%s isn't a sound file.", msg->data.c_str());
  }

  if (!message.empty()) {
    char *cstr = new char[message.size() + 1]; // get memory
    std::strcpy(cstr, message.c_str());        // copy
    gp = popen(cstr, "w");
    pclose(gp); // パイプを閉じる

    ROS_INFO("playing %s", msg->data.c_str());
  }
}


int main(int argc, char **argv) {

  // establish ros node
  ros::init(argc, argv, "sound_player");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("sound_pipes/sound",
                                     1000,
                                     playerCallback);
  ROS_INFO("sound player is ready.");

  ros::spin();

}