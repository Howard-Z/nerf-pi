import numpy as np
import matplotlib.pyplot as plt
from time import sleep
from pid import PID

# Paste your PID class here if not imported
# from your_pid_module import PID

# Test function for PID controller
def test_pid_1d():
    print("Testing 1D PID Controller")

    pid = PID(kp=0.6, ki=0.1, kd=0.05, target=[10.0])
    pos = np.array([0.0])
    dt = 0.1
    history = []

    for _ in range(100):
        control = pid.update(pos, dt)
        pos += control * dt
        history.append(pos[0])

    # Plot the trajectory
    plt.figure()
    plt.title("1D PID Convergence to Target")
    plt.plot(history, label="Position")
    plt.axhline(10.0, color='r', linestyle='--', label="Target")
    plt.xlabel("Time step")
    plt.ylabel("Position")
    plt.legend()
    plt.grid(True)
    plt.show()

def test_pid_2d():
    print("Testing 2D PID Controller")

    pid = PID(kp=[0.6, 0.4], ki=[0.1, 0.1], kd=[0.05, 0.05], target=[5.0, -5.0])
    pos = np.array([0.0, 0.0])
    dt = 0.1
    history = []

    for _ in range(100):
        control = pid.update(pos, dt)
        pos += control * dt
        history.append(pos.copy())

    history = np.array(history)

    # Plot the trajectory
    plt.figure()
    plt.title("2D PID Trajectory")
    plt.plot(history[:, 0], history[:, 1], label="Trajectory")
    plt.plot([5.0], [-5.0], 'rx', label="Target")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

if __name__ == "__main__":
    test_pid_1d()
    test_pid_2d()
