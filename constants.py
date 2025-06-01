# Used for mediapipe (pose_landmarker.py)
MODEL_PATH = "pose_landmarker_lite.task"
IMAGE_HEIGHT = 1080
IMAGE_WIDTH = 1920

# used for binocular vision calculations (binocular.py)
CAMERA_DIST = 3.90  # distance between the two cameras, in inches

# used for camera calibration (camera_calibration.py)
CHECKERBOARD = (9, 6)  # number of internal corners
SQUARE_SIZE = 1.0  # 1 inch irl
CALIBRATION_PATH_RIGHT = r"C:\Users\adnja\Pictures\camera_calibration\right\*.jpg"
CALIBRATION_PATH_LEFT = r"C:\Users\adnja\Pictures\camera_calibration\left\*.jpg"

