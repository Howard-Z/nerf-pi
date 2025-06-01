import cv2
import numpy as np
import glob
from constants import CHECKERBOARD, SQUARE_SIZE, CALIBRATION_PATH_RIGHT, CALIBRATION_PATH_LEFT


def calibrate(path):
    # Prepare object points
    objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

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

if __name__ == '__main__':
    # run this file to calibrate cameras with images in the provided filepath, and
    # regenerate the calibration.npy file.
    calibration_path_right = CALIBRATION_PATH_RIGHT
    calibration_path_left = CALIBRATION_PATH_LEFT

    ret_L, K_L, dist_L, rvecs_L, tvecs_L, objpoints_L, imgpoints_L = calibrate(calibration_path_left)
    ret_R, K_R, dist_R, rvecs_R, tvecs_R, objpoints_R, imgpoints_R = calibrate(calibration_path_right)

    d = {"ret_L": ret_L,
         "K_L": K_L,
         "dist_L": dist_L,
         "rvecs_L": rvecs_L,
         "tvecs_L": tvecs_L,
         "objpoints_L": objpoints_L,
         "imgpoints_L": imgpoints_L,
         "ret_R": ret_R,
         "K_R": K_R,
         "dist_R": dist_R,
         "rvecs_R": rvecs_R,
         "tvecs_R": tvecs_R,
         "objpoints_R": objpoints_R,
         "imgpoints_R": imgpoints_R}

    np.save("calibration.npy", d)
    print("Camera calibration matrices saved to file.")