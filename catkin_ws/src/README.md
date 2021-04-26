Packages in this folder are designed to run the basic functionalities of MyJay. 

MyJay is able to track balls based on color and uses a simple proportional controller to follow them. It can also stream its video feed over the network using the [`web_video_server`](http://wiki.ros.org/web_video_server) package. To start the tracker and the video streamer, simple run: `roslaunch myjay_cameras start_camera.launch`

The *robot_controller* package takes care of actuating and localizing the robot. By running `roslaunch myjay_core start_robot.launch`, the robot will listen to `/cmd_vel` messages for the drivetrain, `/intake` for the intake motor and `/elevator` for the upper elevator stage leading up to the flywheel. Additionally, the messages `/led` and `/flywheel` will be sent to the microcontroller via the `rosserial` node (please see `bejay_rosserial.ino` in the *robot_firmware* folder for more details).\
The *robot_controller* package is also responsible for localization using odometry. The robot needs a IMU for reliable localization (TODO).

The *myjay_teleop* package is responsible for receiving joystick teleoperation commands. It has two modes:
1. Local UDP mode: this recieves UDP messages from a local computer connected to the network. The scripts running on the local computer are `udp_adaptive.py` and `udp_joy_teleop.py` which can be found in the *teleop_scripts* folder.
2. Remote teleoperation over the internet using the firebase realtime database. 