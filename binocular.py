from pose_landmarker import initialize_landmarker, data_lock, shared_data
from constants import CAMERA_DIST
from numpy import load
import threading
import time


def start_cameras():
    thread1 = threading.Thread(target=initialize_landmarker, args=(0, True))
    thread2 = threading.Thread(target=initialize_landmarker, args=(1, False))

    thread1.start()
    thread2.start()

def calculate_depth(K, point_L, point_R, dist_between_cams):
    x_disparity = point_L[0] - point_R[0]
    f_x = K[0, 0]

    depth = (f_x * dist_between_cams) / x_disparity
    return abs(depth)      # positive is in front of the cameras.

def calculate_world(K_L, K_R, point_L, point_R, dist_between_cams):
    _ZL = calculate_depth(K_L, point_L, point_R, dist_between_cams)
    _XL = _ZL * (point_L[0] - K_L[0, 2]) / K_L[0, 0] - (dist_between_cams / 2)
    _YL = _ZL * (point_L[1] - K_L[1, 2]) / K_L[1, 1]

    _ZR = calculate_depth(K_R, point_L, point_R, dist_between_cams)
    _XR = _ZR * (point_R[0] - K_R[0, 2]) / K_R[0, 0] - (dist_between_cams / 2)
    _YR = _ZR * (point_R[1] - K_R[1, 2]) / K_R[1, 1]

    # print(_XL, _YL, _ZL)
    # print(_XR, _YR, _ZR)

    return [(_XL + _XR) / 2, (_YL + _YR) / 2, (_ZL + _ZR) / 2]

if __name__ == '__main__':
    # load camera calibration file.  (Generated with camera_calibration.py)
    # Fields: "ret_L", "K_L", "dist_L", "rvecs_L", "tvecs_L", "objpoints_L", "imgpoints_L"
    #         "ret_R", "K_R", "dist_R", "rvecs_R", "tvecs_R", "objpoints_R", "imgpoints_R"
    calibration = load("./calibration.npy", allow_pickle=True).item()

    calculate_world_curry = lambda lp, rp: calculate_world(
        calibration["K_L"], calibration["K_R"], lp, rp, CAMERA_DIST)

    start_cameras()
    started = False
    while True:
        with data_lock:
            left = shared_data[0]
            right = shared_data[1]

        if left and right and left[1] and right[1]:
            points = list(map(calculate_world_curry, left[1], right[1]))
            started = True
        else:
            points = []

        if not started:
            time.sleep(1)
            continue

        if not points:
            print("No pose detected.")
            time.sleep(1)
            continue

        # TODO: do rest of processing here.  (PID?, etc)
        # points: list(list(float, float, float)) contains the (estimate) 3D coordinate of all 33 landmarks.
        print("------------------------------------------")
        for n, i in enumerate(points):
            print(f"{n}: ({i[0]}, {i[1]}, {i[2]})")

        time.sleep(1)