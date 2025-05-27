import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.tasks.python.vision.core.vision_task_running_mode import VisionTaskRunningMode

# Constants
MODEL_PATH = "pose_landmarker_lite.task"  # Update if needed
IMAGE_HEIGHT = 1080
IMAGE_WIDTH = 1920

# Callback to handle results
def print_landmarks(result, output_image, timestamp_ms):
    if not result.pose_landmarks:
        print(f"[{timestamp_ms} ms] No pose detected.")
        return


    print(f"[{timestamp_ms} ms] Detected {len(result.pose_landmarks)} pose(s).")

    landmark = result.pose_landmarks[0][17]
    pixel_x = int(landmark.x * IMAGE_WIDTH)
    pixel_y = int(landmark.y * IMAGE_HEIGHT)
    print(f"Landmark 17: x={pixel_x}, y={pixel_y}, z={landmark.z:.4f}, visibility={landmark.visibility:.4f}")

    # for i, landmark in enumerate(result.pose_landmarks[0]):
    #     pixel_x = int(landmark.x * IMAGE_WIDTH)
    #     pixel_y = int(landmark.y * IMAGE_HEIGHT)
    #     print(f"Landmark {i}: x={pixel_x}, y={pixel_y}, z={landmark.z:.4f}, visibility={landmark.visibility:.4f}")


# Set up landmarker options
base_options = python.BaseOptions(model_asset_path=MODEL_PATH)
options = vision.PoseLandmarkerOptions(
    base_options=base_options,
    output_segmentation_masks=False,
    running_mode=VisionTaskRunningMode.LIVE_STREAM,
    num_poses=1,
    result_callback=print_landmarks
)

# Create the landmarker
pose_landmarker = vision.PoseLandmarker.create_from_options(options)

# Start webcam capture
cap = cv2.VideoCapture(0)

try:
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            print("Ignoring empty frame.")
            continue

        # Convert to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)

        # Use current time for timestamp
        timestamp = int(cap.get(cv2.CAP_PROP_POS_MSEC))
        pose_landmarker.detect_async(mp_image, timestamp)

        # Display frame
        cv2.imshow("Pose Landmarker", frame)
        if cv2.waitKey(5) & 0xFF == 27:
            break
finally:
    cap.release()
    cv2.destroyAllWindows()