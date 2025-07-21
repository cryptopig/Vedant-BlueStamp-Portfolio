import RPi.GPIO as GPIO
from time import sleep

claw = 14
wrist = 18
OFFSET = 2.5
GPIO.setmode(GPIO.BCM)
GPIO.setup(claw, GPIO.OUT)
GPIO.setup(wrist, GPIO.OUT)

claw_pwm = GPIO.PWM(claw, 50)  # 50Hz PWM frequency
claw_pwm.start(0)

wrist_pwm = GPIO.PWM(wrist, 50)
wrist_pwm.start(0)


def set_angle(angle, servo="claw"):
    duty = angle / 18 + OFFSET  # maps 0–180 degrees to ~2.5%–12.5% duty cycle
    if (servo=="claw"):
        claw_pwm.ChangeDutyCycle(duty)
        sleep(1)
        claw_pwm.ChangeDutyCycle(0)
    else:
        wrist_pwm.ChangeDutyCycle(duty)
        sleep(1)
        wrist_pwm.ChangeDutyCycle(0)
      # prevent servo jitter

# kind of an enum; makes it easier to map servo positions to numbers for tuning
# values based on trial and error
class ServoPositions():
    CLOSE = 55
    HALF = 30
    OPEN = 15
    TOP = 20
    BOTTOM = 60

# main function to set position
def set_servo(position="half"):
    if (position == "close"):
        set_angle(ServoPositions.CLOSE)
    elif (position == "complete_open"):
        set_angle(ServoPositions.OPEN)
    elif (position == "half"):
        set_angle(ServoPositions.HALF)
    elif (position == "top"):
        set_angle(ServoPositions.TOP, "wrist")
    else:
        set_angle(ServoPositions.BOTTOM, "wrist")

# test code for tuning/testing the servos
try:  
    while True:
        motor = input("Enter motor (claw or wrist): ")
        pos = input("Enter desired position (open, close, half; ): ")
        
        # claw code
        if (motor == "claw:"):
            if(pos == "open" ):
                set_servo("complete_open")
            elif (pos == "half"):
                set_servo("half")
            else:
                set_servo("close")

        # wrist code
        else:
            if (pos == "top"):
                set_servo("top")
            else:
                set_servo("bottom")

except KeyboardInterrupt:
    GPIO.cleanup()
    exit()
