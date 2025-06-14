# nerf-pi
UCLA Com Sci 188 - Introduction to Robotics, final project.

Our goal is to build a nerf "turret", consisting of a nerf gun on top of a rotating base and two cameras.  Binocular vision is implemented for the cameras to perceive depth and calculate rotation angles.  Two stepper motors are used to move the turret to the desired location, and shoot.

### Hardware
Hardware used in this include:
1. Nema 17 stepper motor
    - A4988 driver board
2. 28BYJ-48 stepper motor
    - ULN2003 driver board
3. SG90 servo motor

The pins for each of the hardware components are defined inside of `turret.py`

An external 12v and 5v power supply are requred for eletronic device safety and smoother operation.

### Running
**Camera side**: run `binocular.py`.  This will start up the cameras, and any poses recorded will have the chest coordinate, as well as angle of rotations sent to the MQTT server.
- Environment: should be using python 3.10
```
mediapipe==0.10.21
opencv-python==4.6.0.66
```

**Turret side**: run `turret.py`.  This will receive input from the MQTT server, and pass the rotation angles into a PID controller, which will move the turret.
- Environment: Should be using python 3.12 or anything compatible with the following libraries:
```python
paho.mqtt
pid
RPi.GPIO
time
gpiozero
```

### Camera Calibration
To calibrate the two cameras, print out the checkerboard in `camera_calibration_checkerboard.pdf`, and take around 10-20 pictures with the checkerboard at different locations and angles within the camera frame.  Add those images to the calibration paths specified in `constants.py`.  Repeat for both cameras.  
* The positions of the checkerboard need not be the same between images for the two cameras.
* It is recommended to mount the checkerboard on a rigid surface (e.g., a picture frame, or cardboard) to ensure it is flat.
* Make sure the checkerboard is clearly and completely visible in each picture; not obscured, dark, washed out, or covered by shadows.  Smooth lighting is recommended.

### MQTT
The camera sends information to our turret platform using MQTT. We specify an MQTT broker within our 2 python files that allow data to be sent to the turret.

The topics that the turret is subscribed to are:
```python
TOPIC_MESSAGES = [
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
```

Their descriptions are as follows:
- `manual/override`: Enable or disable manual control (disable or enable the robot policy) (1 or 0)
- `manual/hor`: manual horizontal movement of the base (integers pos/neg)
- `manual/vert`: manual vertical movement of the gun tilt (integers pos/neg)
- `manual/trigger`: manual actuation of the trigger servo (1)
- `manual/level`: Set the home level of the gun tilt stepper (1)
- `manual/center`: Set the home rotation of the base (1)
- `manual/arm`: Arm or disarm the robot (1 or 0)
    - signifies that the gun is either loaded (1) or unloaded (0)
- `manual/sentry`: Enables the policy to pull the trigger (1 or 0)
    - signifies if the trigger can be pulled by the automatic policy (1 for yes, 0 for no)
- `cam/ang/hor`: The angle relative to the camera that the targeted person is in the horizontal axis
- `cam/ang/vert`: The angle relative to the camera tht the targed person is in the vertical axis