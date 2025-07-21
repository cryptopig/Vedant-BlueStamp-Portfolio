from flask import Flask, Response
from gpiozero import DistanceSensor
from picamera2 import Picamera2
import cv2
import numpy as np
import threading
import time

from motorcode import move_forward, turn_left, turn_right, stop
print("end of imports")

app = Flask(__name__)
# sensor = DistanceSensor(echo=26, trigger=19)

FPS = 10  # Optimized for Raspberry Pi performance
CAMERA_OFFSET = 0  # Camera alignment offset

x_y = 3
# Initialize camera
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


def detect_trash(frame):
    global detected_trash_center
    h, w = frame.shape[:2]

    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)
    net.setInput(blob)
    detections = net.forward()

    detected_trash_center = [0, 0]

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

                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
                cv2.putText(frame, f"{label}: {confidence:.2f}", (startX, startY - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                detected_trash_center = [centerX, centerY]
                break

    return frame


def camera_loop():
    global output_frame
    frame_count = 0
    while True:
        frame = picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        if frame_count % 2 == 0:  # Process every other frame
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


def track_center(frame_width=320, threshold=30, distance=0.075):
    global detected_trash_center

    frame_center = (frame_width // 2) - CAMERA_OFFSET
    delta = detected_trash_center[0] - frame_center

    # if x_y < distance:
    #     print(f'🛑 Bot is too close: distance is {x_y:.3f} m', flush=True)
    #     time.sleep(1)
    #     return

    if abs(delta) <= threshold:
        move_forward()
        print(f"⬆️  Moving forward — Trash X: {detected_trash_center[0]}, Center: {frame_center}, Delta: {delta}; Distance: {x_y:.3f} m", flush=True)
    elif delta < -threshold:
        turn_left()
        print(f"⬅  Turning left — Trash X: {detected_trash_center[0]}, Center: {frame_center}, Delta: {delta}; Distance: {x_y:.3f} m", flush=True)
    elif delta > threshold:
        turn_right()
        print(f"➡️> Turning right — Trash X: {detected_trash_center[0]}, Center: {frame_center}, Delta: {delta}; Distance: {x_y:.3f} m", flush=True)

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

    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False, threaded=True)
    print("server started")
