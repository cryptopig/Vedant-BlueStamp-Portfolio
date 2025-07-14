from flask import Flask, Response
import RPi.GPIO as GPIO
from gpiozero import DistanceSensor
from picamera2 import Picamera2
import cv2
import numpy as np
import threading
import time
from motorcode import move_forward, turn_left, turn_right, stop, init, set_speed

app = Flask(__name__)
sensor = DistanceSensor(echo=26, trigger=19)

FPS = 24  # frames per second
ball_pos = [0, 0]

# tuned this value; the camera is offset to the right of the bot, so I needed to correct
# for this offset in code. Tuned this value based on trial-and-error.
CAMERA_OFFSET = 140

init()
# Initialize camera
picam2 = Picamera2()
video_config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(video_config)
picam2.start()
time.sleep(1)

output_frame = None
frame_lock = threading.Lock()
# motor pins are already specified in imported functions; don't need to be used directly here

# main detection function
def detect_red_ball(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # sets color mode 

# color bounds set here
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
 #contour area for bounding box 
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 300:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        if not 0.8 < float(w) / h < 1.2:
            continue

        # rdraw bounding box around the red ball
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        ball_pos[0] = (x + w)/2 # gets center of the ball instead of bottom/sides
        ball_pos[1] = (y + h)/2 # gets center of the ball like above
        break  # only detect one ball

    return frame

# main camera loop; cycles through frames specified by FPS variable and detects whether the
# red ball is in frame
def camera_loop():
    global output_frame
    while True:
        frame = picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        frame = detect_red_ball(frame)

        with frame_lock:
            output_frame = frame
                        

        time.sleep(1.0 / FPS)

def control_loop():
    try:
        init()
        while True:
            track_center()
            
    except KeyboardInterrupt:
        stop()
    
# determine center 
# turn left/right until the bounding box is in the center of the screen
# move forwards when the ball is in the center of the screen
# stop based on ultrasonic sensor distance

# all movement functios are imported from motorcode so they don't need to be specified
def track_center(frame_width=640, threshold=50, distance = 0.075):
    global ball_pos
    set_speed(0.65,0.65)


    if ball_pos[0] == 0:
        print("Ball not detected.")
        stop()
        return

    frame_center = (frame_width // 2) - CAMERA_OFFSET
    delta = ball_pos[0] - frame_center

    # print(f"Ball X: {ball_pos[0]}, Center: {frame_center}, Delta: {delta}")

    # stop condition
    if (sensor.distance < distance):
        print(f'Bot is too close: distance is {sensor.distance}')
        stop()
        exit()

    # forward
    if abs(delta) <= threshold:
        move_forward()
        print(f"Ball X: {ball_pos[0]}, Center: {frame_center}, Delta: {delta}; FORWARD\nDistance: {sensor.distance}")

    # left turn
    elif delta < -threshold:
        turn_left()
        print(f"Ball X: {ball_pos[0]}, Center: {frame_center}, Delta: {delta}; LEFT\nDistance: {sensor.distance}")
        
    
    # right turn
    elif delta > threshold:
        turn_right()
        print(f"Ball X: {ball_pos[0]}, Center: {frame_center}, Delta: {delta}; RIGHT\nDistance: {sensor.distance}")

@app.route('/')
def video_feed():
    def generate():
        last_frame_time = time.time()

        while True:
            with frame_lock:
                if output_frame is None:
                    continue
                current_time = time.time()
                if current_time - last_frame_time < (1.0 / FPS):
                    continue  # skip if too soon
                last_frame_time = current_time

                ret, buffer = cv2.imencode('.jpg', output_frame)
                if not ret:
                    continue
                frame_bytes = buffer.tobytes()

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


# host on web server to easily see what the cam. is actually detecting; for debugging purposes
if __name__ == '__main__':
    t1 = threading.Thread(target=camera_loop)
    t1.daemon = True
    t1.start()

    t2 = threading.Thread(target=control_loop)
    t2.daemon = True
    t2.start()

    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False, threaded=True)
