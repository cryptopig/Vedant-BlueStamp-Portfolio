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
import cv2

net = cv2.dnn.readNetFromCaffe('MobileNetSSD_deploy.prototxt', 'MobileNetSSD_deploy.caffemodel')
# imports some arbitrar objects from the predownloaded model 
classes = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant",
           "sheep", "sofa", "train", "tvmonitor"]

trash_classes = {"bottle"}  # You can expand this as needed
detected_trash_center = [0, 0]

# main deetction function
def detect_trash(frame):
    global detected_trash_center
    h, w = frame.shape[:2]
    
    # small resolution to increase performance (probably will run ~8 FPS)
    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
                                 0.007843, (300, 300), 127.5)
    net.setInput(blob)
    detections = net.forward()

    detected_trash_center = [0, 0]  # Reset every frame

    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.5:
            idx = int(detections[0, 0, i, 1])
            label = classes[idx]
            if label in trash_classes:
                box = detections[0, 0, i, 3:7] * [w, h, w, h]
                (startX, startY, endX, endY) = box.astype("int")
                centerX = (startX + endX) // 2
                centerY = (startY + endY) // 2

                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
                cv2.putText(frame, f"{label}: {confidence:.2f}", (startX, startY - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                detected_trash_center = [centerX, centerY]
                break  # Only track the first detected trash item

    return frame


# main camera loop; cycles through frames specified by FPS variable and detects whether the
# red ball is in frame
def camera_loop():
    global output_frame
    while True:
        frame = picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        frame = detect_trash(frame)

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
    global detected_trash_center
    set_speed(0.65,0.65)


    if detected_trash_center[0] == 0:
        print("Ball not detected.")
        stop()
        return

    frame_center = (frame_width // 2) - CAMERA_OFFSET
    delta = detected_trash_center[0] - frame_center

    # print(f"Ball X: {ball_pos[0]}, Center: {frame_center}, Delta: {delta}")

    # stop condition
    if (sensor.distance < distance):
        print(f'Bot is too close: distance is {sensor.distance}')
        stop()
        exit()

    # forward
    if abs(delta) <= threshold:
        move_forward()
        print(f"Ball X: {detected_trash_center[0]}, Center: {frame_center}, Delta: {delta}; FORWARD\nDistance: {sensor.distance}")

    # left turn
    elif delta < -threshold:
        turn_left()
        print(f"Ball X: {detected_trash_center[0]}, Center: {frame_center}, Delta: {delta}; LEFT\nDistance: {sensor.distance}")
        
    
    # right turn
    elif delta > threshold:
        turn_right()
        print(f"Ball X: {detected_trash_center[0]}, Center: {frame_center}, Delta: {delta}; RIGHT\nDistance: {sensor.distance}")

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
