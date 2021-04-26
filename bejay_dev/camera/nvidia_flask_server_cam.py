import jetson.inference
import jetson.utils
import time
import cv2
import numpy as np

import threading
from flask import Response, Flask





# Image frame sent to the Flask object
global video_frame
video_frame = None


# Use locks for thread-safe viewing of frames in multiple browsers
global thread_lock 
thread_lock = threading.Lock()


# Create the Flask object for the application
app = Flask(__name__)



def captureFrames():
    global video_frame, thread_lock

    # Video capturing from OpenCV
    width=640
    height=480
    #cam=jetson.utils.gstCamera(width,height,'/dev/video1')
    cam=jetson.utils.gstCamera(width,height,'0')
    timeMark=time.time()
    fpsFilter=0
    timeMark=time.time()
    font=cv2.FONT_HERSHEY_SIMPLEX
    while True:
        frame, width, height = cam.CaptureRGBA(zeroCopy=1)
        dt=time.time()-timeMark
        fps=1/dt
        fpsFilter=.95*fpsFilter+.05*fps
        timeMark=time.time()
        frame=jetson.utils.cudaToNumpy(frame,width,height,4)
        frame=cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR).astype(np.uint8)
        # with thread safe access
        with thread_lock:
            video_frame = frame.copy()
        if cv2.waitKey(1)==ord('q'):
            break
            cam.release()        # Create a copy of the frame and store it in the global variable,
        
def encodeFrame():
    global thread_lock
    while True:
        # Acquire thread_lock to access the global video_frame object
        with thread_lock:
            global video_frame
            if video_frame is None:
                continue
            return_key, encoded_image = cv2.imencode(".jpeg", video_frame)
            if not return_key:
                continue

        # Output image as a byte array
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
            bytearray(encoded_image) + b'\r\n')

@app.route("/")
def streamFrames():
    return Response(encodeFrame(), mimetype = "multipart/x-mixed-replace; boundary=frame")

# check to see if this is the main thread of execution
if __name__ == '__main__':

    # Create a thread and attach the method that captures the image frames, to it
    process_thread = threading.Thread(target=captureFrames)
    process_thread.daemon = True

    # Start the thread
    process_thread.start()

    # start the Flask Web Application
    # While it can be run on any feasible IP, IP = 0.0.0.0 renders the web app on
    # the host machine's localhost and is discoverable by other machines on the same network 
    app.run("0.0.0.0", port="80")
