from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep

factory = PiGPIOFactory()

PIN = 14
# Use wider pulse width range to fully reach ±90°
# SG90 typically responds from 0.5ms to 2.5ms
servo = Servo(PIN, pin_factory=factory,
              min_pulse_width=0.0005,  # 0.5ms
              max_pulse_width=0.0025)  # 2.5ms

try:

    print("Rotating to +90°")
    servo.max()
    sleep(1)

    # print("Rotating to -90°")
    # servo.min()
    # sleep(1)



    

    print("Center (0°)")
    servo.mid()
    sleep(1)

    print("Rotating to +90°")
    servo.max()
    sleep(1)

    

except KeyboardInterrupt:
    print("Interrupted")

finally:
    servo.detach()
