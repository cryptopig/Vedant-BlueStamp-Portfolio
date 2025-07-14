from gpiozero import PWMOutputDevice
import RPi.GPIO as GPIO
from time import sleep

IN1 = 22
IN2 = 27
IN3 = 17
IN4 = 4
ENA = 2
ENB = 3  


# TODO: Reinstate Old PWM Code
# def init():
#     global left_pwm, right_pwm
#     GPIO.setmode(GPIO.BCM)
#     GPIO.setwarnings(False)
#     GPIO.setup([IN1, IN2, IN3, IN4], GPIO.OUT)
#     if left_pwm is None:
#         left_pwm = PWMOutputDevice(ENA)
    
#     if right_pwm is None:
#         right_pwm = PWMOutputDevice(ENB)

#     left_pwm.value = 1.0
#     right_pwm.value = 1.0

# # sets the motor speed based on a float value from 0.0-1.0
# def set_speed(left=1.0, right=1.0):
#     left_pwm.value = left
#     right_pwm.value = right

def init():
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    
    # set all pins as outputs
    GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)

    # enable both sides at full power (HIGH)
    GPIO.output(ENA, GPIO.HIGH)
    GPIO.output(ENB, GPIO.HIGH)

def move_forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def turn_right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def turn_left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

# separate functinos for stop and clean-up (final stop vs. stop)
def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def cleanup():
    stop()
    # left_pwm.stop()
    # right_pwm.stop()
    GPIO.cleanup()

init()
move_forward()
sleep(1)
cleanup()
# sleep(3)
# print("cleaning up")
# cleanup()
# print("cleaned up")
# init()
# set_speed(0.57,0.57)
# move_forward()
# sleep(1)
# stop()
# cleanup()
