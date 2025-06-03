from pose_landmarker import *
from camera_calibration import *
from binocular import *
from constants import *

IMAGE_SIZE = (IMAGE_WIDTH, IMAGE_HEIGHT)

def initialize_landmarker_with_image(image_path: str):
    base_options = python.BaseOptions(model_asset_path=MODEL_PATH)
    options = vision.PoseLandmarkerOptions(
        base_options=base_options,
        running_mode=VisionTaskRunningMode.IMAGE,
        output_segmentation_masks=False,
        num_poses=1
    )

    # Create landmarker
    landmarker = vision.PoseLandmarker.create_from_options(options)

    # Read image with OpenCV and convert to RGB
    frame_bgr = cv2.imread(image_path)
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)

    # Run pose detection
    result = landmarker.detect(mp_image)

    # Print landmarks (reuse your callback logic here)
    landmarks = []
    if not result.pose_landmarks:
        print("No pose detected.")
    else:
        for i, landmark in enumerate(result.pose_landmarks[0]):
            pixel_x = int(landmark.x * frame_rgb.shape[1])
            pixel_y = int(landmark.y * frame_rgb.shape[0])
            # print(f"Landmark {i}: x={pixel_x}, y={pixel_y}, z={landmark.z:.4f}, visibility={landmark.visibility:.4f}")
            landmarks.append((pixel_x, pixel_y, landmark.z, landmark.visibility))
    return landmarks

if __name__ == '__main__':
    left_points = initialize_landmarker_with_image(r"C:\Users\adnja\Pictures\Camera Roll\left.jpg")
    right_points = initialize_landmarker_with_image(r"C:\Users\adnja\Pictures\Camera Roll\right.jpg")

    print(left_points)
    print(right_points)

    # calibrations (K_L, K_R should be in pixels)
    ret_L, K_L, dist_L, rvecs_L, tvecs_L, objpoints, imgpoints_L = calibrate(r"C:\Users\adnja\Pictures\camera_calibration\left\*.jpg")
    ret_R, K_R, dist_R, rvecs_R, tvecs_R, _, imgpoints_R = calibrate(r"C:\Users\adnja\Pictures\camera_calibration\right\*.jpg")

    print(K_L, K_R)

    distances = []

    for i in range(len(left_points)):
        distances.append(calculate_world(K_L, K_R, left_points[i], right_points[i], 3.90))

    print(distances)

    # stereo rectification
    # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 100, 1e-5)
    #
    # retval, K_L, dist_L, K_R, dist_R, R, T, E, F = cv2.stereoCalibrate(
    #     objpoints,
    #     imgpoints_L,
    #     imgpoints_R,
    #     K_L,
    #     dist_L,
    #     K_R,
    #     dist_R,
    #     IMAGE_SIZE,
    #     criteria=criteria,
    #     flags=cv2.CALIB_FIX_INTRINSIC
    # )

    # print(T)

    # # Left camera projection matrix
    # P1 = K_L @ np.hstack((np.eye(3), np.zeros((3, 1))))
    #
    # # Right camera projection matrix
    # P2 = K_R @ np.array([[1, 0, 0, 3.90], [0, 1, 0, 0], [0, 0, 1, 0]])
    #
    # # triangulate points
    # points_3d = []
    #
    # for i in range(len(left_points)):
    #     pt_L = np.array((left_points[i][0], left_points[i][1]))
    #     pt_R = np.array((right_points[i][0], right_points[i][1]))
    #
    #     point_4d = cv2.triangulatePoints(P1, P2, pt_L, pt_R)
    #     # print(point_4d)
    #     point_3d = point_4d[:3] / point_4d[3]
    #     points_3d.append(point_3d.ravel())
    #
    # # print(points_3d)

