#!/usr/bin/env python

# tutorial from: https://maker.pro/nvidia-jetson/tutorial/streaming-real-time-video-from-rpi-camera-to-browser-on-jetson-nano-with-flask
import cv2
import time
from collections import deque
import imutils
import numpy as np
import sys
from platform import python_version
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# GStreamer Pipeline to access the Raspberry Pi camera
# GSTREAMER_PIPELINE = 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=3280, height=2464, format=(string)NV12, framerate=21/1 ! nvvidconv flip-method=0 ! video/x-raw, width=960, height=616, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink wait-on-eos=false max-buffers=1 drop=True'




# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
tracker_pnts = 4
pts = deque(maxlen=tracker_pnts)

# publisher with respect to camera coordinates 
collect_data = rospy.get_param('/collect_data', default=True)

drive_train_debug = rospy.Publisher("/camera_cmd_vel", Twist, queue_size =1)
if not collect_data:
    drive_train = rospy.Publisher("/cmd_vel", Twist, queue_size =1) 

last_seen = False # check if the ball was seen in the previous frame



# Currently there are setting frame rate on CSI Camera on Nano through gstreamer
# Here we directly select sensor_mode 3 (1280x720, 59.9999 fps)
def gstreamer_pipeline(
    sensor_id=0,
    sensor_mode=3,
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d sensor-mode=%d ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            sensor_mode,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )



def filter_image(frame):
    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    return frame, mask


def track_balls(center, height, width):
    turn = width/2-center[0]
    forward = height-center[1]

    vel = Twist()

    turn_kp = 0.007
    forward_kp = 0.002#0.005
    vel.angular.z = turn*turn_kp
    vel.linear.x = forward*forward_kp
    if not collect_data:
        drive_train.publish(vel)
    drive_train_debug.publish(vel)



def find_balls(frame, mask):
    global last_seen
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # only proceed if the radius meets a minimum size
        if radius > 15:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
                (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            height, width = frame.shape[:2]
            track_balls(center, height, width)
            last_seen = True
        else:
            if last_seen and not collect_data:
                drive_train.publish(Twist())# send a stop command
                last_seen = False
    # update the points queue
    pts.appendleft(center)
    # loop over the set of tracked points
    for i in range(1, len(pts)):
        # if either of the tracked points are None, ignore
        # them
        if pts[i - 1] is None or pts[i] is None:
            continue
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thickness = int(np.sqrt(tracker_pnts/ float(i + 1)) * 2.5)
        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

    return frame




def captureFrames():
    # Video capturing from OpenCV

    '''old_settings =cv2.VideoCapture(gstreamer_pipeline(
            sensor_id=0,
            sensor_mode=3,
            flip_method=0,
            # capture_width=640,
            # capture_height=480,
            framerate=10,
            display_height=240,
            display_width=320,
        ), cv2.CAP_GSTREAMER)
    '''

    width=640
    height=480
    dispW=width
    dispH=height
    flip=0
    top_tech_boy_settings = 'nvarguscamerasrc sensor-mode=3 !  video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=60/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! videobalance  contrast=1.5 brightness=-.3 saturation=1.2 ! appsink drop=true'
    test_settings = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"
    video_capture = cv2.VideoCapture(top_tech_boy_settings)


    VideoRaw = rospy.Publisher('VideoRaw', Image, queue_size=10)
    VideoBall = rospy.Publisher('VideoBall', Image, queue_size=10)




    bridge = CvBridge()
    while not rospy.is_shutdown():# and video_capture.isOpened():
        return_key, frame = video_capture.read()
        if not return_key:
            break
        filtered_frame, mask = filter_image(frame)
        drawn_frame = find_balls(filtered_frame,mask)
        # VideoRaw.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        VideoBall.publish(bridge.cv2_to_imgmsg(drawn_frame, "bgr8"))
    video_capture.release()



# check to see if this is the main thread of execution
if __name__ == '__main__':

    rospy.init_node('camera_capture')
    captureFrames()
