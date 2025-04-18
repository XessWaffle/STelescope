import numpy as np

from uart_handler import OrientationDataLogPacket

class VirtualTelescope:
    def __init__(self):
        # Initialize euler angles (yaw, pitch, roll) in degrees
        self.euler_angles = np.array([0.0, 0.0, 0.0])  # [yaw, pitch, roll]
        # Initialize angular velocities (degrees per second)
        self.angular_velocity = np.array([0.0, 0.0, 0.0])  # [yaw_rate, pitch_rate, roll_rate]


class Orientation:

    @staticmethod
    def estimate_euler_angles(log_packet: OrientationDataLogPacket):
        """
        Estimate euler angles (yaw, pitch, roll) from the orientation log packet.

        Args:
            log_packet (dict): A dictionary containing orientation data. Expected keys:
                - 'yaw': Yaw angle in degrees
                - 'pitch': Pitch angle in degrees
                - 'roll': Roll angle in degrees

        Returns:
            np.ndarray: Estimated euler angles as a numpy array [yaw, pitch, roll].
        """
        pass