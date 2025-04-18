from uart_handler import OrientationDataLogPacket, UARTTxManager, UARTCommand, UARTCommands

import numpy as np
from enum import Enum

class Axes(Enum):
    YAW = 0
    ROLL = 1
    PITCH = 2
    AXES = 3

class Microstep(Enum):
    FULL = 0
    HALF = 1
    QUARTER = 2
    EIGHTH = 3
    SIXTEENTH = 4
    INVALID = 5

class StepperOperatingMode(Enum):
    TRACKING = 0
    ALIGNING = 1
    HOMING = 2
    SLEEP = 3

class PhysicalTelescope:

    def __init__(self, tx_mgr: UARTTxManager):
        self.orientation = np.array([0.0, 0.0, 0.0])
        self.microstep_mode = Microstep.FULL
        self.homed = False
        self.tx_mgr = tx_mgr

    def _home_pitch(self):
        pass

    def _home_yaw(self):
        pass

    def _home_roll(self):
        pass

    def orientation_update(self, log_packet: OrientationDataLogPacket):
        if(not self.homed):
            self.orientation = PhysicalOrientation.estimate_orientation(log_packet)
        else:
            self.orientation = PhysicalOrientation.absolute_orientation(log_packet)

        self.microstep_mode = Microstep(log_packet["microstep_mode"])
    
    def home(self):





class PhysicalOrientation:
    
    @staticmethod
    def estimate_orientation(log_packet: OrientationDataLogPacket):
        """
            Use magnetometer, gyro and accel data to estimate the orientation of the telescope
            Note:
                - Roll orientation will always be zero here as no valid way of homing the roll axis exists
        """
        
        pitch = np.rad2deg(np.atan2(log_packet["out_y_a_raw"], log_packet["out_x_a_raw"]))
        roll = 0.0
        yaw = np.rad2deg(np.atan2(log_packet["out_y_raw_onboard"], log_packet["out_x_raw_onboard"]))

        return np.array([yaw, roll, pitch])


    @staticmethod
    def absolute_orientation(log_packet: OrientationDataLogPacket):
        """
            Use step based positions to track finer resolution orientation
        """
        pass
