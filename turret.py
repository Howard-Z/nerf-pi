import paho.mqtt.client as mqtt
from pid import PID
import RPi.GPIO as GPIO
from time import sleep
import time
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

TOPIC_MESSAGES = [
    "cam/coord/x",
    "cam/coord/y",
    "cam/coord/z",
    "cam/ang/hor",
    "cam/ang/vert",
    "manual/override",
    "manual/hor",
    "manual/vert",
    "manual/trigger",
    "manual/level",
    "manual/center",
    "manual/arm",
    "manual/sentry"
]

BROKER = "192.168.1.158"  # Replace with your broker address
PORT = 1884                   # Standard MQTT port

factory = PiGPIOFactory()
PIN = 14
servo = Servo(PIN, pin_factory=factory,
              min_pulse_width=0.0005,  # 0.5ms
              max_pulse_width=0.0025)  # 2.5ms


# Use BCM GPIO numbering
DIR = 27     # Direction pin
STEP = 17    # Step pin
STEPS_PER_REV = 200  # 1.8° per step (typical for NEMA 17)
hor_pos = 0 # position in steps (aka 1.8 degrees)
vert_pos = 0
# 28byj pins (BCM)
in1 = 25
in2 = 8
in3 = 7
in4 = 1

# careful lowering this, at some point you run into the mechanical limitation of how quick your motor can move
step_sleep = 0.002

step_count = int(4096 * ( 5.0 / 360.0 )) # 5.625*(1/64) per step, 4096 steps is 360°

# direction = True # True for clockwise, False for counter-clockwise
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

# setting up
GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)

# GPIO.setmode( GPIO.BCM )
GPIO.setup( in1, GPIO.OUT )
GPIO.setup( in2, GPIO.OUT )
GPIO.setup( in3, GPIO.OUT )
GPIO.setup( in4, GPIO.OUT )

# initializing
GPIO.output( in1, GPIO.LOW )
GPIO.output( in2, GPIO.LOW )
GPIO.output( in3, GPIO.LOW )
GPIO.output( in4, GPIO.LOW )

motor_pins = [in1,in2,in3,in4]
motor_step_counter = 0

manual_override = True

hor_target = 0.0
ver_target = 0.0

armed = False
sentry = False
data_flag = False

def step_motor_rev(revolutions, direction=True, delay=0.0008): #0004 is fastest
    GPIO.output(DIR, direction)
    steps = int(STEPS_PER_REV * revolutions)
    for _ in range(steps):
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)
        
def step_motor_deg(degrees, direction=True, delay=0.0008): #0004 is fastest
    GPIO.output(DIR, direction)
    steps = (degrees * 85.0 / 15.0) / 1.8
    for _ in range(int(steps)):
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)

def step_motor(steps, direction=True, delay=0.0008): #0004 is fastest
    GPIO.output(DIR, direction)
    global hor_pos
    # positive is clockwise
    # True is clockwise
    # print("attempt to step motor: ", steps)
    # print("hor_pos: ", hor_pos)
    # print("direction: ", direction)
    if abs(steps) < 2:
        return
    if hor_pos >= 720 and direction: #1440 is for 8th step +- 180 deg
        return
    if hor_pos <= -720 and not direction:
        return
    for _ in range(int(steps)):
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)
    if direction:
        hor_pos += int(steps * 1.8 * 15.0 / 85.0)
    else:
        hor_pos -= int(steps * 1.8 * 15.0 / 85.0)

def goto_tilt(degrees):
    global vert_pos
    global motor_step_counter
    global motor_pins
    global step_sequence
    global in1, in2, in3, in4

    limit = 10
    # target_angle = max(-limit, min(limit, degrees - vert_pos))
    # print("target_angle: ", target_angle, "vert_pos: ", vert_pos)
    # diff = target_angle - vert_pos
    if degrees > limit:
        degrees = limit
    elif degrees < -limit:
        degrees = -limit
    target_angle = degrees - vert_pos
    diff = target_angle
    steps = int(4096 * (abs(diff) / 360.0))
    direction = diff > 0
    steps = abs(steps)
    # print("current position: ", vert_pos, "deg requested: ", degrees, "target_angle: ", target_angle)
    # print("stepping: ", steps, " direction: ", direction, " diff: ", diff)
    vert_pos = vert_pos + diff
    # the meat
    i = 0
    for i in range(steps):
        for pin in range(0, len(motor_pins)):
            GPIO.output( motor_pins[pin], step_sequence[motor_step_counter][pin] )
        if direction==False:
            motor_step_counter = (motor_step_counter - 1) % 8
        elif direction==True:
            motor_step_counter = (motor_step_counter + 1) % 8
        else: # defensive programming
            print( "uh oh... direction should *always* be either True or False" )
            GPIO.cleanup()
            exit( 1 )
        time.sleep( step_sleep )
    GPIO.output( in1, GPIO.LOW )
    GPIO.output( in2, GPIO.LOW )
    GPIO.output( in3, GPIO.LOW )
    GPIO.output( in4, GPIO.LOW )

def trigger():
    global servo
    print("FIREEE")
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



# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Connected with result code {reason_code}")
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    for topic in TOPIC_MESSAGES:
        client.subscribe(topic)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global manual_override
    global hor_pos
    global hor_pid
    global vert_pid
    global vert_pos
    global hor_target
    global ver_target
    global armed
    global sentry
    global data_flag
    # print(msg.topic+" "+msg.payload.decode())
    if msg.topic == "manual/override":
        if msg.payload.decode() == "1":
            print("Manual mode")
            manual_override = True
            hor_pid.reset([0.0])
            vert_pid.reset([0.0])
        elif msg.payload.decode() == "0":
            print("Automated mode")
            manual_override = False
        else:
            print("Uknown command, defualt to manual")
            manual_override = True
    elif msg.topic == "manual/hor":
        if manual_override:
            deg_rot = float(msg.payload.decode())
            if deg_rot > 0:
                print(f"Rotate horizontal CW by {deg_rot} degrees")
                step_motor_deg(deg_rot, direction=True)
                hor_pos += int(deg_rot)
            elif deg_rot < 0:
                print(f"Rotate horizontal CCW by {deg_rot} degrees")
                step_motor_deg(-1 * deg_rot, direction=False)
                hor_pos += int(deg_rot)
            else:
                print("Unknown command for horizontal rotation")
    elif msg.topic == "manual/vert":
        if manual_override:
            deg_rot = float(msg.payload.decode())
            print("tilting to: ", deg_rot)
            goto_tilt(deg_rot)
            vert_pos = deg_rot
    elif msg.topic == "manual/level":
        if manual_override:
            if msg.payload.decode() == "1":
                print("setting current position as level")
                vert_pos = 0
                vert_pid.reset([0.0])
    elif msg.topic == "manual/center":
            if msg.payload.decode() == "1":
                print("setting current position as the center")
                hor_pos = 0
                hor_pid.reset([0.0])
    elif msg.topic == "manual/trigger":
        if msg.payload.decode() == "1":
            print("Firing!")
            trigger()
    elif msg.topic == "cam/ang/hor":
        if not manual_override:
            deg_target = float(msg.payload.decode())
            print("deg_target: ", deg_target)
            # hor_pid.reset([deg_target]) # maybe use set_target instead?
            hor_pid.set_target([deg_target])
            hor_target = deg_target
            data_flag = True
    elif msg.topic == "cam/ang/vert":
        if not manual_override:
            deg_target = float(msg.payload.decode())
            # vert_pid.reset([deg_target]) # maybe use set_target instead?
            vert_pid.set_target([deg_target])
            ver_target = deg_target
    elif msg.topic == "manual/arm":
        if msg.payload.decode() == "1":
            print("Arming turret")
            armed = True
        elif msg.payload.decode() == "0":
            print("Disarming turret")
            armed = False
    elif msg.topic == "manual/sentry":
        if msg.payload.decode() == "1":
            print("Sentry mode enabled")
            sentry = True
        elif msg.payload.decode() == "0":
            print("Sentry mode disabled")
            sentry = False
        
        
        
mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_connect = on_connect
mqttc.on_message = on_message

mqttc.connect("192.168.1.158", 1884, 60)

hor_pid = PID(kp=0.6, ki=0.1, kd=1.0, target=[0.0])
vert_pid = PID(kp=0.6, ki=0.1, kd=1.0, target=[0.0])

loop_dt = .005
hor_dt = .01
vert_dt = .01

if __name__ == "__main__":
    print("Connecting to MQTT broker...")
    mqttc.connect("192.168.1.158", 1884, 60)
    print("Connected! Starting loop")
    mqttc.loop_start()
    prev_time = 0


    try:
        while True:

            # mqttc.publish("status/theta_pos", hor_pos)
            sleep(loop_dt)
            # hor_update = hor_pid.update(hor_pos, hor_dt)[0] #update hor_pos to 0
            # vert_update = vert_pid.update(vert_pos, vert_dt)[0] #update vert_pos to 0
            hor_update = hor_pid.update(0, hor_dt)[0]
            vert_update = vert_pid.update(vert_pos, vert_dt)[0]
            # print("looping")
            if not manual_override:
                if hor_update > 0:
                    # print(f"PID horizontal control: Rotate CW by {hor_update} degrees")
                    # step_motor(abs(hor_update), direction=True)
                    # step_motor(5, True)
                    # hor_pos += int(hor_update)
                    pass
                elif hor_update < 0:
                    # print(f"PID horizontal control: Rotate CCW by {hor_update} degrees")
                    # step_motor(abs(hor_update), direction=False)
                    # step_motor(5, False)
                    # hor_pos += int(hor_update)
                    pass
                else:
                    # print("No horizontal PID control needed")
                    pass
                goto_tilt(ver_target)
                if hor_target > 0:
                    step_motor_deg(hor_target, direction=True)
                elif hor_target < 0:
                    step_motor_deg(-hor_target, direction=False)

                # print("hor_target: ", hor_target, "armed: ", armed, "sentry: ", sentry, "data flag: ", data_flag)
                if abs(hor_target) < 2 and armed and sentry and data_flag:
                    trigger()
                    armed = False
                    sentry = False
                    mqttc.publish("manual/sentry", "0")
                    mqttc.publish("manual/arm", "0")


                hor_target = 0

                
                # Vertical control logic can be added here
                # if vert_update > 0 or vert_update < 0:
                #     step_motor_deg(vert_update / 360, direction=(vert_update > 0))
            else:
                # print("Manual override active, skipping PID control")
                pass
            if manual_override:
                pass
            cur_time = time.time()
            if cur_time - prev_time >= 1:
                prev_time = cur_time
                # Publish status to MQTT broker
                mqttc.publish("status/hor_pos", hor_pos)
                mqttc.publish("status/vert_pos", vert_pos)
            data_flag = False
            
                
    except KeyboardInterrupt:
        print("Disconnecting from MQTT broker...")
        mqttc.loop_stop()
        mqttc.disconnect()
        servo.detach()
        GPIO.output( in1, GPIO.LOW )
        GPIO.output( in2, GPIO.LOW )
        GPIO.output( in3, GPIO.LOW )
        GPIO.output( in4, GPIO.LOW )
        GPIO.cleanup()
        print("Disconnected.")