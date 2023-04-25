from flask import Flask, render_template, Response,request,redirect,jsonify
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import sys
import time
import os 
from commands import ReadingCommands

app = Flask(__name__)

def set_vel(vx, vy, vz, avx=0, avy=0, avz=0):
    """
    Send comand velocities. Must be in GUIDED mode. Assumes angular
    velocities are zero by default.
    """
    os.system("ros2 topic pub --once /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist \"{linear: {x: "+str(vx)+", y: "+str(vy)+", z: "+str(vz)+"}, angular: {x: "+str(avx)+", y: "+str(avy)+", z: "+str(avz)+"}}\"")

@app.route('/', methods=["GET"])
def index():
    #settings = get_settings()
    #print(settings)
    return render_template('index.html')

@app.route('/video_feed/<int:w>/<int:h>', methods=["GET"])
def video_feed(w,h):
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen_frames(w,h),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/command/<int:a>', methods=["GET"])
def Test(a):
    ReadingCommands.__init__
    print("Executing command: "+str(a))
    if a == 0: # Forward
        ReadingCommands.GetCommand(ReadingCommands,1.0, 0.0, 0.0, avx=0.0, avy=0.0, avz=0.0)
        #set_vel(1.0, 0.0, 0.0, 0.0, 0.0,0.0)
    if a == 2: #Left
        ReadingCommands.GetCommand(0.0, 0.0, 0.0, 0.0, 0.0,1)
    if a == 3: #Right
        ReadingCommands.GetCommand(0.0, 0.0, 0.0, 0.0, 0.0,-1)
    if a == 6: #Stop
        ReadingCommands.GetCommand(0.0, 0.0, 0.0, 0.0, 0.0,0)
    if a == 1: #Reverse
        ReadingCommands.GetCommand(-1.0, 0.0, 0.0, 0.0, 0.0,0)

    return "OK"

def gen_frames(w,h):
    firstFrame = None
    #cam = "rtsp://admin:pjlxdv12@192.168.2.5/cam/realmonitor?channel=1&subtype=1"
    cap = cv2.VideoCapture(0)
    while True:
        # for cap in caps:
        # # Capture frame-by-frame
        success, frame = cap.read()  # read the camera frame

        if not success: #Display No Video image
            #print("ERROR Laoding video")
            frame = cv2.imread("static/images/NoVideo.jpg")
        resize = cv2.resize(frame, (w, h))
        font = cv2.FONT_HERSHEY_PLAIN
        cv2.putText(resize,"MapperBot",(120,10),font,0.9,(0,255,0))
        #cv2.putText(resize,"Lat: "+str(lat),(10,150),font,1,(0,0,255))
        #cv2.putText(resize,"Lon: "+str(lon),(10,170),font,1,(0,0,255))
        #cv2.putText(resize,"Elv: "+str(elv),(10,190),font,1,(0,0,255))
        #cv2.putText(resize,"Speed: "+str(spd),(140,190),font,1,(0,0,255))
        ret, buffer = cv2.imencode('.jpg', resize)
        frame = buffer.tobytes()

        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# main driver function
if __name__ == '__main__':
 
    # run() method of Flask class runs the application
    # on the local development server.
    app.run()