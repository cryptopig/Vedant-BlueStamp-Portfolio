import RPi.GPIO as GPIO
from gpiozero import DistanceSensor
from motorcode import IN1, IN2, IN3, IN4, ENA, ENB, move_forward, turn_left, turn_right, stop, init
import time

sensor = DistanceSensor(echo=26, trigger=19)

try:
    init()
    while sensor.distance > 0.09:
        print(f'Distance: {sensor.distance}')
        move_forward()
        time.sleep(0.01)

    stop()
    GPIO.cleanup()

except KeyboardInterrupt:
    stop()
    GPIO.cleanup()
