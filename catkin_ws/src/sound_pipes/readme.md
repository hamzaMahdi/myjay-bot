# sound_pipes

ROS package to play mp3 and wav files. 

*sound_pipes* create pipe for [mpg123](https://www.mpg123.de/) and [aplay](https://linux.die.net/man/1/aplay). 

This package is tested with ROS-Kinetic and Raspberry Pi 3B. 


## Install

```
cd ~/catkin_ws/src
git clone git@github.com:botamochi6277/sound_pipes.git
cd ~/catkin_ws/
catkin_make
```

## Sample Usage

Terminal1:

Execute a music player node. 

```
rosrun sound_pipes player
```

Terminal2:

Publish a full path of a music file, for exsample "/home/ubuntu/Music/music01.mp3"

```
rostopic pub -1 sound_pipes/sound std_msgs/String "<full path of sound>"
```