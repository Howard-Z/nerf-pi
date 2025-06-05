import signal
import sys

from pose_landmarker import initialize_landmarker, data_lock, shared_data
from constants import CAMERA_DIST, FUDGE
from numpy import load, array
import threading
import time
import math

import paho.mqtt.client as mqtt
from paho.mqtt.client import Client, CallbackAPIVersion

# MQTT broker configuration
BROKER = "192.168.1.158"  # Replace with your broker address
PORT = 1884                   # Standard MQTT port
# Dictionary of topics and their messages
TOPIC_MESSAGES = [
    "cam/coord/x",
    "cam/coord/y",
    "cam/coord/z",
    "cam/ang/hor",
    "cam/ang/vert"
]
CLIENT_ID = "camera-node"

# Callback when the client connects to the broker
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Connected with result code {reason_code}")


# https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker


shutdown_event = threading.Event()

def start_cameras():
    thread1 = threading.Thread(target=initialize_landmarker, args=(0, True, shutdown_event))
    thread2 = threading.Thread(target=initialize_landmarker, args=(1, False, shutdown_event))
    thread1.start()
    thread2.start()
    return [thread1, thread2]


def calculate_depth(K, point_L, point_R, dist_between_cams):
    x_disparity = point_L[0] - point_R[0]
    f_x = K[0, 0]

    depth = (f_x * dist_between_cams) / x_disparity
    return abs(depth) * FUDGE(abs(depth))     # positive is in front of the cameras.


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
    print("Connecting to MQTT Broker: ", BROKER)
    print("Using port: ", PORT)
    
    mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    mqttc.on_connect = on_connect
    # mqttc.on_message = on_message
    # Connect and start loop
    mqttc.connect(BROKER, PORT, 60)
    mqttc.loop_start()
    time.sleep(1)  # Allow time for connection
    print("Publishing to: ", TOPIC_MESSAGES)

    # Publish to each topic
    # for topic, message in TOPIC_MESSAGES.items():
    #     result = mqttc.publish(topic, message)
    #     status = result[0]
    #     if status == 0:
    #         print(f"📤 Sent `{message}` to `{topic}`")
    #     else:
    #         print(f"⚠️ Failed to send to {topic}")

    calibration = load("./calibration.npy", allow_pickle=True).item()

    calculate_world_curry = lambda lp, rp: calculate_world(
        calibration["K_L"], calibration["K_R"], lp, rp, CAMERA_DIST)
    
    started = False
    try:
        camera_threads = start_cameras()
        while True:
            if shutdown_event.is_set():
                break
            with data_lock:
                left = shared_data[0]
                right = shared_data[1]

            if left and right and left[1] and right[1]:
                left[1].append(array([left[1][11], left[1][12], left[1][23], left[1][24]]).mean(axis=0))
                right[1].append(array([right[1][11], right[1][12], right[1][23], right[1][24]]).mean(axis=0))
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

            # print("------------------------------------------")
            # for n, i in enumerate(points):
            #     print(f"{n}: ({i[0]}, {i[1]}, {i[2]})")
            
            result = mqttc.publish(TOPIC_MESSAGES[0], points[33][0])
            result = mqttc.publish(TOPIC_MESSAGES[1], points[33][1])
            result = mqttc.publish(TOPIC_MESSAGES[2], points[33][2])

            theta = math.atan(points[33][0]/points[33][2])
            phi = math.atan(points[33][1]/points[33][2])

            result = mqttc.publish(TOPIC_MESSAGES[3], theta * 180 / math.pi)
            result = mqttc.publish(TOPIC_MESSAGES[4], phi * 180 / math.pi)
            
            time.sleep(1)

    except KeyboardInterrupt:
        shutdown_event.set()
        print("\n🛑 Ctrl+C detected. Shutting down gracefully...")

    finally:
        for t in camera_threads:
            t.join()
        mqttc.loop_stop()
        mqttc.disconnect()
        print("🔌 MQTT client disconnected. Exiting.")
        # sys.exit(0)