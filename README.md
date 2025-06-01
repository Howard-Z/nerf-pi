# nerf-pi

### Running
Run `binocular.py`.  This is the main file.  (for now)

### Camera Calibration
To calibrate the two cameras, print out the checkerboard in `camera_calibration_checkerboard.pdf`, and take around 10-20 pictures with the checkerboard at different locations and angles within the camera frame.  Add those images to the calibration paths specified in `constants.py`.  Repeat for both cameras.  
* The positions of the checkerboard need not be the same between images for the two cameras.
* It is recommended to mount the checkerboard on a rigid surface (e.g., a picture frame, or cardboard) to ensure it is flat.
* Make sure the checkerboard is clearly and completely visible in each picture; not obscured, dark, washed out, or covered by shadows.  Smooth lighting is recommended.