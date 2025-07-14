from flask import Flask, Response
from picamera2 import Picamera2
import cv2
import numpy as np
import threading
import time

app = Flask(__name__)

FPS = 24 # specifies frames per second 

# Initialize the camera with low-latency preview stream
picam2 = Picamera2()
video_config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(video_config)
picam2.start()
time.sleep(1)  # allow camera to warm up

# Global frame variable and lock
output_frame = None
frame_lock = threading.Lock()

def detect_red_ball(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 180, 130])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 180, 130])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    mask = cv2.GaussianBlur(mask, (9, 9), 2)
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 300:
            continue

        (x, y, w, h) = cv2.boundingRect(contour)
        if not 0.8 < float(w) / h < 1.2:
            continue

        (cx, cy), radius = cv2.minEnclosingCircle(contour)
        if radius > 10:
            center = (int(cx), int(cy))
            radius = int(radius)
            cv2.circle(frame, center, radius, (0, 0, 255), 2)
            cv2.putText(frame, "Red Ball", (center[0] - 20, center[1] - radius - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    return frame

def camera_loop():
    global output_frame
    while True:
        frame = picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        frame = detect_red_ball(frame)

        with frame_lock:
            output_frame = frame

        time.sleep(FPS/1000)  # sets fps here

@app.route('/')
def video_feed():
    def generate():
        while True:
            with frame_lock:
                if output_frame is None:
                    continue
                ret, buffer = cv2.imencode('.jpg', output_frame)
                if not ret:
                    continue
                frame_bytes = buffer.tobytes()

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(FPS/1000)  # Stream at ~30 fps

    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    t = threading.Thread(target=camera_loop)
    t.daemon = True
    t.start()

    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False, threaded=True)
