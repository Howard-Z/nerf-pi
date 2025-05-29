from pose_landmarker import initialize_landmarker, data_lock, shared_data
import threading
import time

def start_cameras():
    thread1 = threading.Thread(target=initialize_landmarker, args=(0, False))
    thread2 = threading.Thread(target=initialize_landmarker, args=(1, False))

    thread1.start()
    thread2.start()

if __name__ == '__main__':
    start_cameras()
    while True:
        with data_lock:
            left = shared_data[0]
            right = shared_data[1]
        if left and right and left[1] and right[1]:
            # print(f"Left[{left[0]}]: {left[1][17]}")
            # print(f"Right[{right[0]}]: {right[1][17]}")
            print(f"Diff t: {abs(right[0] - left[0])}, Diff x: {abs(right[1][17][0] - left[1][17][0])}, Diff y: {abs(right[1][17][1] - left[1][17][1])}")

        time.sleep(1)