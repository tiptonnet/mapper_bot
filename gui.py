from flask import Flask, render_template, Response,request,redirect,jsonify
import cv2
 
# Flask constructor takes the name of
# current module (__name__) as argument.
app = Flask(__name__)
 
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
        #cv2.putText(resize,"Buster 1",(120,10),font,0.9,(0,0,255))
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