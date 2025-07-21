# from gpiozero import PWMOutputDevice
# import RPi.GPIO as GPIO
# from time import sleep
from gpiozero import Motor, Robot
from time import sleep
import RPi.GPIO as GPIO

IN1 = 4
IN2 = 17
IN3 = 27
IN4 = 22
ENA = 3
ENB = 13  


GPIO.setmode(GPIO.BCM)
GPIO.setup([ENA, ENB], GPIO.OUT)
GPIO.output(ENA, GPIO.HIGH)
GPIO.output(ENB, GPIO.HIGH)

right_motor = Motor(IN1, IN2, pwm=True)
left_motor = Motor(IN3, IN4, pwm=True)


robot = Robot(left=left_motor, right=right_motor)

# commented code is uncessary; made redundant by robot
# def move_forward(SPEED):
#     print(f"moving forwards. Speed {SPEED}")
#     right_motor.forward(SPEED)
#     left_motor.forward(SPEED)


# def turn_right(SPEED):
#     right_motor.forward(SPEED)
     
# def turn_left(SPEED):
#     left_motor.forward(SPEED)
robot.forward(1)
sleep(0.3)
robot.stop()
GPIO.cleanup()
