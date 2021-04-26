# MyJay by SIRRL

This is the software/mechanical design repository for MyJay (formerly known as BeJay).

This is an ongoing project and is being updated regularly. Code will be refactored and cleaned up eventually.

Full CAD of the robot can be found under the *3D* folder.

The robot currently uses a Jetson Nano to do vision tasks, localization, recieve teloperation commands over WIFI and control motors. The robot is ROS compatible and all packages can be found under the *catkin_ws* folder. Packages were developed for ROS Melodic, and may not be compatible with future versions especially because it uses Python 2.x 

Teleoperation scripts can run on any PC with WIFI. Scripts can be found in the *teleop_scripts* folder. Dependency documentation will be added.

*bejay_dev* is a folder for prototype scripts, which are later integrated into MyJay's ROS packages.

*test_scripts* houses random firmware written for the ESP32 with a prototype UDP protocol. This was for an earlier version of the robot which did not use the Jetson Nano compute module and ran exclusively on the ESP32 microcontroller.

**Remote Host Connection:**\
local ethernet: ssh bejay0@192.168.0.70\
local wifi: ssh bejay0@10.42.0.1
