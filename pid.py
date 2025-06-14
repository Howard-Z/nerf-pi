import numpy as np

class PID:
    def __init__(self, kp, ki, kd, target):
        """
        Initialize a variable-dimension PID controller.

        Args:
            kp (float or list): Proportional gain(s) per axis (or scalar).
            ki (float or list): Integral gain(s) per axis (or scalar).
            kd (float or list): Derivative gain(s) per axis (or scalar).
            target (tuple or array): Target position of any dimension.
        """
        self.setpoint = np.array(target, dtype=np.float64)
        dim = self.setpoint.shape[0]

        self.kp = np.array(kp if hasattr(kp, '__len__') else [kp] * dim, dtype=np.float64)
        self.ki = np.array(ki if hasattr(ki, '__len__') else [ki] * dim, dtype=np.float64)
        self.kd = np.array(kd if hasattr(kd, '__len__') else [kd] * dim, dtype=np.float64)

        self._prev_error = np.zeros(dim)
        self._integral = np.zeros(dim)
        
    def set_target(self, target = None):
        """
        Set a new target for the PID controller.

        Args:
            target (tuple or array, optional): New target position.
        """
        if target is not None:
            self.setpoint = np.array(target, dtype=np.float64)
            dim = self.setpoint.shape[0]

    def reset(self, target=None):
        """
        Reset the internal state of the PID controller.

        Args:
            target (optional): New target to reset to.
        """
        if target is not None:
            self.setpoint = np.array(target, dtype=np.float64)
            dim = self.setpoint.shape[0]
            self._prev_error = np.zeros(dim)
            self._integral = np.zeros(dim)
        else:
            self._prev_error.fill(0)
            self._integral.fill(0)

    def get_error(self):
        """
        Returns:
            float: Magnitude of the last error vector.
        """
        return np.linalg.norm(self._prev_error)

    def update(self, current_pos, dt):
        """
        Compute the PID control signal.

        Args:
            current_pos (array-like): Current position (any dimension).
            dt (float): Time delta in seconds.

        Returns:
            np.ndarray: Control output vector.
        """
        current_pos = np.array(current_pos, dtype=np.float64)
        error = self.setpoint - current_pos

        self._integral += error * dt
        derivative = (error - self._prev_error) / dt if dt > 0 else np.zeros_like(error)

        output = (
            self.kp * error +
            self.ki * self._integral +
            self.kd * derivative
        )

        self._prev_error = error
        return output