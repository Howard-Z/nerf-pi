import RPi.GPIO as GPIO
from time import sleep

# BCM Numbering for 28byj
in1 = 25
in2 = 8
in3 = 7
in4 = 1
step_sleep = 0.002
step_count = int(4096 * ( 5.0 / 360.0 )) # 5.625*(1/64) per step, 4096 steps is 360°
direction = True # True for clockwise, False for counter-clockwise
                    # False for down, True for up
# defining stepper motor sequence (found in documentation http://www.4tronix.co.uk/arduino/Stepper-Motors.php)
step_sequence = [[1,0,0,1],
                 [1,0,0,0],
                 [1,1,0,0],
                 [0,1,0,0],
                 [0,1,1,0],
                 [0,0,1,0],
                 [0,0,1,1],
                 [0,0,0,1]]


# Use BCM GPIO numbering
DIR = 27     # Direction pin
STEP = 17    # Step pin
STEPS_PER_REV = 200  # 1.8° per step (typical for NEMA 17)

GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)

# 28byj setting up
GPIO.setmode( GPIO.BCM )
GPIO.setup( in1, GPIO.OUT )
GPIO.setup( in2, GPIO.OUT )
GPIO.setup( in3, GPIO.OUT )
GPIO.setup( in4, GPIO.OUT )

# 28byj initializing
GPIO.output( in1, GPIO.LOW )
GPIO.output( in2, GPIO.LOW )
GPIO.output( in3, GPIO.LOW )
GPIO.output( in4, GPIO.LOW )

motor_pins = [in1,in2,in3,in4]
motor_step_counter = 0

def step_motor(revolutions, direction=True, delay=0.0008): #0004 is fastest
    GPIO.output(DIR, direction)
    steps = int(STEPS_PER_REV * revolutions)
    for _ in range(steps):
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)

try:
    print("Rotate CW 1 revolution")
    step_motor(10, direction=True)

    sleep(1)

    print("Rotate CCW 1 revolution")
    step_motor(10, direction=False)

except KeyboardInterrupt:
    print("Interrupted")

finally:
    GPIO.output( in1, GPIO.LOW )
    GPIO.output( in2, GPIO.LOW )
    GPIO.output( in3, GPIO.LOW )
    GPIO.output( in4, GPIO.LOW )
    GPIO.cleanup()
