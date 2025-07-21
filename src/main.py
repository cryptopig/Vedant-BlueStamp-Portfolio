from flask import Flask, Response
from gpiozero import DistanceSensor
from picamera2 import Picamera2
import cv2
import numpy as np
import threading
import time
import RPi.GPIO as GPIO

from motorcode import robot, left_motor, right_motor
from servo_code import set_servo
print("end of imports")

app = Flask(__name__)
# sensor = DistanceSensor(echo=26, trigger=19)

FPS = 10  # Optimized for Raspberry Pi performance
CAMERA_OFFSET = 0  # Camera alignment offset

# function for approximating distance based on width of bottle
REAL_OBJECT_WIDTH_CM = 7.0  # real-world width of bottle; assumes that the bottle is ~7 cm. in width
FOCAL_LENGTH = 238  # tuned based on formula: (percieved pixel width * known cm. distance away from camera) / real cm. width
# Initialize camera

SPEED = 1.0 # from 0.0 to 1.0

servo_pos = "open" # keeps track of servo position

ENA = 3
ENB = 13

GPIO.setmode(GPIO.BCM)
GPIO.setup([ENA, ENB], GPIO.OUT)
GPIO.output(ENA, GPIO.HIGH)
GPIO.output(ENB, GPIO.HIGH)
print("motors initialized")

picam2 = Picamera2()
video_config = picam2.create_video_configuration(main={"size": (320, 240), "format": "RGB888"})
print("video initialized")
picam2.configure(video_config)
picam2.start()
time.sleep(1)

output_frame = None
frame_lock = threading.Lock()

# Load MobileNetSSD model
net = cv2.dnn.readNetFromCaffe('MobileNetSSD_deploy.prototxt', 'MobileNetSSD_deploy.caffemodel')
print("model loaded")
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

classes = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant",
           "sheep", "sofa", "train", "tvmonitor"]

trash_classes = {"bottle"}
detected_trash_center = [0, 0]
print("model initialized")



set_servo("half")
def detect_trash(frame):
    global detected_trash_center, estimated_distance_cm

    h, w = frame.shape[:2]

    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (400, 400)), 0.007843, (400, 400), 127.5)
    net.setInput(blob)
    detections = net.forward()

    detected_trash_center = [0, 0]
    estimated_distance_cm = float('inf')

    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.3:
            idx = int(detections[0, 0, i, 1])
            label = classes[idx]
            if label in trash_classes:
                box = detections[0, 0, i, 3:7] * [w, h, w, h]
                (startX, startY, endX, endY) = box.astype("int")
                centerX = (startX + endX) // 2
                centerY = (startY + endY) // 2

                box_width = endX - startX  # width in pixels based on bounding box

                # distance estimation (focal length has to be calibrated)
                if box_width > 0:
                    estimated_distance_cm = (REAL_OBJECT_WIDTH_CM * FOCAL_LENGTH) / box_width

                detected_trash_center = [centerX, centerY]

                # annotate Frame
                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
                cv2.putText(frame, f"{label}: {confidence:.2f}", (startX, startY - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f"Dist: {estimated_distance_cm:.1f} cm", (centerX, centerY),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                break  # only process first detected object

    return frame



def camera_loop():
    global output_frame
    frame_count = 0
    while True:
        frame = picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        if frame_count % 2 == 0:  # process every other frame
            frame = detect_trash(frame)

        with frame_lock:
            output_frame = frame

        frame_count += 1
        time.sleep(1.0 / FPS)


def control_loop():
    last_detected = False
    try:
        while True:
            if detected_trash_center[0] != 0:
                if not last_detected:
                    print("✅ Trash detected.", flush=True)
                    last_detected = True
                track_center()
            else:
                if last_detected:
                    print("❌ Trash not detected.", flush=True)
                    last_detected = False
            time.sleep(1.0 / FPS)
    except KeyboardInterrupt:
        pass


# main function to track the center of the object; actually moves the bot to track the bottle
def track_center(frame_width=320, threshold=70):
    global detected_trash_center, estimated_distance_cm, servo_pos

    frame_center = (frame_width // 2) - CAMERA_OFFSET
    delta = detected_trash_center[0] - frame_center

    if  estimated_distance_cm <= 11:
        print(f'🛑 Bot is too close: distance is {estimated_distance_cm} cm', flush=True)
        robot.stop()
        if (servo_pos != "close"):
            set_servo("close")
            servo_pos="close" # makes sure the claw only closes once, so it doesn't try to close repeatedly
        return

    if abs(delta) <= threshold:
        robot.forward(SPEED)
        if (servo_pos != "open"):
            set_servo("open")
            servo_pos = "open"
        time.sleep(0.5)
        robot.stop()
        print(f"⬆️  Moving forward — Trash X: {detected_trash_center[0]}, Center: {frame_center}, Delta: {delta}; Distance: {estimated_distance_cm:.3f} cm", flush=True)

    elif delta < -threshold:
        robot.left(SPEED)
        time.sleep(0.3)
        robot.stop()
        print(f"⬅  Turning left — Trash X: {detected_trash_center[0]}, Center: {frame_center}, Delta: {delta}; Distance: {estimated_distance_cm:.3f} cm", flush=True)

    elif delta > threshold:
        robot.right(SPEED)
        time.sleep(0.3)
        robot.stop()
        print(f"➡️> Turning right — Trash X: {detected_trash_center[0]}, Center: {frame_center}, Delta: {delta}; Distance: {estimated_distance_cm:.3f} cm", flush=True)

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
                    continue
                last_frame_time = current_time

                ret, buffer = cv2.imencode('.jpg', output_frame)
                if not ret:
                    continue
                frame_bytes = buffer.tobytes()

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    t1 = threading.Thread(target=camera_loop)
    t1.daemon = True
    t1.start()

    t2 = threading.Thread(target=control_loop)
    t2.daemon = True
    t2.start()

    # live video feed server hosted on RPi
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False, threaded=True)
    print("server started")
