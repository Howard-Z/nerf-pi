import cv2
import numpy as np
import glob

def calibrate(path):
    # Checkerboard pattern dimensions
    CHECKERBOARD = (9, 6)  # number of internal corners
    square_size = 1.0  # 1 inch irl

    # Prepare object points
    objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= square_size

    objpoints = []  # 3d points
    imgpoints = []  # 2d image points

    # Load all checkerboard images
    images = glob.glob(path)

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)

    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    return ret, K, dist, rvecs, tvecs, objpoints, imgpoints

def get_intrinsic_components(K):
    # K is the intrinsic matrix
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]
    return fx, fy, cx, cy

def get_field_of_view(fx, fy):
    fov_x = 2 * np.arctan(1920 / (2 * fx)) * 180 / np.pi
    fov_y = 2 * np.arctan(1080 / (2 * fy)) * 180 / np.pi

    return fov_x, fov_y
