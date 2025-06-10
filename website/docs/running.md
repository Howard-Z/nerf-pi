# Nerf-Pi   
Nerf-Pi is the repository storing the code for our nerf gun turret.  The turret features a rotating base with a mounted nerf gun and two webcams, and powered by a Raspberry Pi and Windows desktop.  The cameras scan the room, use Google Mediapipe for pose tracking and landmarking, and the gun rotates and shoots at nearby people.

## Running
**Camera side**: run `binocular.py`.  This will start up the cameras, and any poses recorded will have the chest coordinate, as well as angle of rotations sent to the MQTT server.

**Turret side**: run `[insert here]`.  This will receive input from the MQTT server, and pass the rotation angles into a PID controller, which will move the turret. 

### Camera Calibration
To calibrate the two cameras, print out the checkerboard in `camera_calibration_checkerboard.pdf`, and take around 10-20 pictures with the checkerboard at different locations and angles within the camera frame.  Add those images to the calibration paths specified in `constants.py`.  Repeat for both cameras.  
* The positions of the checkerboard need not be the same between images for the two cameras.
* It is recommended to mount the checkerboard on a rigid surface (e.g., a picture frame, or cardboard) to ensure it is flat.
* Make sure the checkerboard is clearly and completely visible in each picture; not obscured, dark, washed out, or covered by shadows.  Smooth lighting is recommended.
