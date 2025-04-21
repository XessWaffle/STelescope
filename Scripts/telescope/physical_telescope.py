from uart_handler import OrientationDataLogPacket, UARTTxManager, UARTCommand, UARTCommands

import numpy as np
from enum import Enum
import threading

from queue import Queue
import random

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
    SLEWING = 1
    HOMING = 2
    SLEEP = 3

class Direction(Enum):
    PLUS = 0
    MINUS = 1
    STOP = 2


""" 
Pitch goes from -90 -> 0
Roll goes from -90 -> 90
Yaw goes full 360
"""
class DegMinSec:

    def __init__(self, degree, minute, second):
        self.degree = degree
        self.minute = minute
        self.second = second

        self.yaw_steps = DegMinSec.dms_to_steps(Axes.YAW, degree, minute, second)
        self.roll_steps = DegMinSec.dms_to_steps(Axes.ROLL, degree, minute, second)
        self.pitch_steps = DegMinSec.dms_to_steps(Axes.PITCH, degree, minute, second)

    def __str__(self):
        return "[" + str(self.degree) + "° " + str(self.minute) + "\' " + str(self.second) + "\" : " \
            + str(self.get_steps(Axes.YAW)) + ", " + str(self.get_steps(Axes.ROLL)) +  ",  " + str(self.get_steps(Axes.PITCH)) + "]"

    def get_steps(self, axis: Axes, microstep: Microstep = Microstep.SIXTEENTH):

        steps = 0

        match axis:
            case Axes.YAW:
                steps = self.yaw_steps
            case Axes.ROLL:
                steps = self.roll_steps
            case Axes.PITCH:
                steps = self.pitch_steps
            case _:
                raise ValueError("Invalid axis")

        return int(steps / (1 << (4 - microstep.value)))

    @staticmethod
    def dms_to_steps(axis: Axes, degree, arcminutes, arcseconds):
        """
            Returns the number of steps, at SIXTEENTH microstep resolution needed to
            traverse to the angle from 0
        """
        total_arcseconds = degree * 3600 + arcminutes * 60 + arcseconds
        steps_div_arcseconds_in_360_degrees = 0

        if axis == Axes.ROLL or axis == Axes.PITCH:
            steps_div_arcseconds_in_360_degrees = 3.52839515519
        elif axis == Axes.YAW:
            steps_div_arcseconds_in_360_degrees = 4
        
        return int(total_arcseconds * steps_div_arcseconds_in_360_degrees)
    
    @staticmethod
    def from_random(min = -180, max = 180):
        degree = random.randint(min, max)
        minute = random.randint(0, 60)
        second = random.randint(0, 60)
        return DegMinSec(degree, minute, second)


class MicrostepModeManager:
    def __init__(self, tx_mgr: UARTTxManager):
        self.microstep_mode = None
        self.microstep_vote = Microstep.FULL
        self._delivered_vote = None
        self.tx_mgr = tx_mgr

        self._voters = {}
        self._winning_voter = None

        self._set_microstep()

    def _set_microstep(self):
        if self.microstep_vote == Microstep.INVALID or self._delivered_vote == self.microstep_vote:
            return    

        print(f"Switched from {self.microstep_mode} -> {self.microstep_vote} for {self._winning_voter}")

        self._delivered_vote = self.microstep_vote
        self.tx_mgr.write_command(UARTCommand(UARTCommands.CMD_MICST_MODE, self.microstep_vote.value))
    
    def add_voter(self, voter_id: Axes):
        if voter_id not in self._voters.keys():
            print(f"Reg voter {voter_id}")
            self._voters[voter_id] = Microstep.INVALID

    def sub_voter(self, voter_id: Axes):
        if voter_id in self._voters.keys():
            print(f"Dereg voter {voter_id}")
            self._voters.pop(voter_id, None)

    def vote(self, voter: Axes, microstep_vote: Microstep):
        if (microstep_vote != Microstep.INVALID) and (voter in self._voters.keys()) and self._voters[voter] != microstep_vote:
            self._voters[voter] = microstep_vote
            print(f"Client {voter} changed vote to {microstep_vote}")
       
    def perform_vote(self, largest = False):
       
        self.microstep_vote = Microstep.INVALID

        for voter in self._voters.keys():
            vote = self._voters[voter]
            if vote != Microstep.INVALID:        
                if self.microstep_vote == Microstep.INVALID or \
                    (not largest and vote.value >= self.microstep_vote.value) or \
                        (largest and vote.value <= self.microstep_vote.value):
                    
                    self.microstep_vote = vote
                    self._winning_voter = voter



        self._set_microstep()


    def microstep_update(self, log_packet: OrientationDataLogPacket):
        self.microstep_mode = Microstep(log_packet["microstep_mode"])

    def get_microstep(self) -> Microstep:
        return self._delivered_vote

class PhysicalTelescope:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(PhysicalTelescope, cls).__new__(cls)
        return cls._instance

    def __init__(self, tx_mgr: UARTTxManager):
        if hasattr(self, "_initialized") and self._initialized:
            return
        self._initialized = True

        self.orientation = np.array([0.0, 0.0, 0.0])
        self.step_orientation = np.array([0.0, 0.0, 0.0])
        self.rates = np.array([0, 0, 0])
        self.desired_positions = np.array([0, 0, 0])

        self.microstep_mgr = MicrostepModeManager(tx_mgr)
        
        self.pitch_homed = False
        self.yaw_homed = False
        self.pitch_calibrated = False
        self.yaw_calibrated = False
        
        self.slew_complete = np.array([False, False, False])
        self.tx_mgr = tx_mgr

        self._max_rate = 60000

        self._pkp = 1
        self._pki = 0
        self._pkd = 0

        self._prev_p_err = 0.0
        self._p_p_err = 0.0
        self._i_p_err = 0.0
        self._d_p_err = 0.0

        self._p_y_arr = []
        self._last_idx = 0
        self._p_y_err = 0.0
        self._i_y_err = 0.0
        self._d_y_err = 0.0

        self.task_queue = Queue()
        self.task_queue_lock = threading.Lock()
        self.running = True

        self._processing = False
        self._tracking = False

        handler_thread = threading.Thread(target=self._phy_telescope_handler)
        handler_thread.start()

    def _phy_telescope_handler(self):
        while self.running:
            if self.task_queue.qsize() > 0:
                self._processing = True

                self.task_queue_lock.acquire()
                task = self.task_queue.get()
                self.task_queue_lock.release()
        
                task()
            else:
                self._processing = False

    def _set_axis_desired_pos(self, axis: Axes, position, force = False):
        if self.desired_positions[axis.value] == position and not force:
            return

        self.desired_positions[axis.value] = position

        cmd_byte = UARTCommands.CMD_INVALID

        match axis:
            case Axes.PITCH:
                cmd_byte = UARTCommands.CMD_SET_P_POS
            case Axes.ROLL:
                cmd_byte = UARTCommands.CMD_SET_R_POS
            case Axes.YAW:
                cmd_byte = UARTCommands.CMD_SET_Y_POS

        self.tx_mgr.write_command(UARTCommand(cmd_byte, UARTCommand.to_int24(position)))

    def _set_axis_rate(self, axis: Axes, rate, force = False):
        if not (rate <= 100000 and rate >= -100000):
            return
        
        if self.rates[axis.value] == rate and not force:
            return

        self.rates[axis.value] = rate

        cmd_byte = UARTCommands.CMD_INVALID

        match axis:
            case Axes.PITCH:
                cmd_byte = UARTCommands.CMD_SET_P_RATE
            case Axes.ROLL:
                cmd_byte = UARTCommands.CMD_SET_R_RATE
            case Axes.YAW:
                cmd_byte = UARTCommands.CMD_SET_Y_RATE

        self.tx_mgr.write_command(UARTCommand(cmd_byte, UARTCommand.to_int24(rate)))
                
    
    def _set_state(self, state: StepperOperatingMode):
        self.tx_mgr.write_command(UARTCommand(UARTCommands.CMD_SET_STATE, state.value))

    def _reset_desired_pos(self, axis: Axes):
        self.tx_mgr.write_command(UARTCommand(UARTCommands.CMD_RESET_DES_POS, axis.value))

    def _reset_pos(self, axis: Axes):
        self.tx_mgr.write_command(UARTCommand(UARTCommands.CMD_RESET_POS, axis.value))

    def _home_yaw(self, angle = 0):

        """
            For yaw, positive is up, negative is down
        """
        sampled_error = (self.orientation[Axes.YAW.value] - angle)/ 180
        if len(self._p_y_arr) < 10:
            self._p_y_arr.append(sampled_error)
        else:
            self._p_y_arr[self._last_idx] = sampled_error
            self._last_idx = self._last_idx + 1
            self._last_idx = self._last_idx % 10

        self._prev_y_err = self._p_y_err
        self._p_y_err = np.average(self._p_y_arr)
       
        # Determine if new log packet is received
        if self._prev_y_err == self._p_y_err:
            return (Microstep.INVALID, 0)

        error = self._p_y_err
        rate_sign = 1 if error < 0 else -1

        rate = 0
        microstep_vote = Microstep.FULL

        if abs(error) < 0.01:
            self._pkp = 15
            self._pki = 0.005   
            self._pkd = 0

            self._i_y_err += self._p_y_err
            self._d_y_err = (self._p_y_err - self._prev_y_err)

            error_sum = self._pkp * self._p_y_err + self._pki * self._i_y_err + self._pkd * self._d_y_err

            rate = abs(error_sum) * self._max_rate

            if rate < 100:
                rate = 0
                self.yaw_homed = True
                self.yaw_calibrated = True
            else:
                microstep_vote = Microstep.EIGHTH
                rate = int(rate_sign * rate)

        elif abs(error) < 0.05:
            self._pki = 0
            microstep_vote = Microstep.QUARTER
            rate = int(rate_sign * self._max_rate)

        elif abs(error) < 0.1:
            self._pki = 0
            microstep_vote = Microstep.HALF
            rate = int(rate_sign * self._max_rate)

        elif abs(error) < 0.15:
            self._pki = 0
            microstep_vote = Microstep.FULL
            rate = int(rate_sign * self._max_rate / 2)
        else:
            microstep_vote = Microstep.FULL
            rate = rate_sign * self._max_rate
        
        return (microstep_vote, rate)

    """
    def _calculate_iron_offsets(self):

        self._start_collecting = True

        if len(self._x_data) != 0 and len(self._y_data) != 0:
            hard_x_cal = (np.max(self._x_data) + np.min(self._x_data)) / 2
            hard_y_cal = (np.max(self._y_data) + np.min(self._y_data)) / 2
            print(f"Hard Iron Calibration X Offset: {int(hard_x_cal)}, Y Offset: {int(hard_y_cal)}")

        return False
    """

    def _home_pitch(self, angle = 0):
        """
            For pitch, positive is up, negative is down
        """
        self._prev_p_err = self._p_p_err
        self._p_p_err = (self.orientation[Axes.PITCH.value] - angle) / 90

        # Determine if new log packet is received
        if self._prev_p_err == self._p_p_err:
            return (Microstep.INVALID, 0)

        error = self._p_p_err
        rate_sign = 1 if error < 0 else -1

        rate = 0
        microstep_vote = Microstep.FULL

        if abs(error) < 0.01:
            self._pkp = 225
            self._pki = 0.5   
            self._pkd = 0

            self._i_p_err += self._p_p_err
            self._d_p_err = (self._p_p_err - self._prev_p_err)

            error_sum = self._pkp * self._p_p_err + self._pki * self._i_p_err + self._pkd * self._d_p_err

            microstep_vote = Microstep.EIGHTH
            rate = abs(error_sum) * self._max_rate

            if rate < 100:
                rate = 0
                self.pitch_homed = True
                self.pitch_calibrated = True
            else:
                rate = int(rate_sign * rate)

        elif abs(error) < 0.05:
            self._pki = 0
            microstep_vote = Microstep.HALF
            rate = int(rate_sign * self._max_rate / 2)
        elif abs(error) < 0.1:
            microstep_vote = Microstep.HALF
            rate = rate_sign * self._max_rate
        else:
            microstep_vote = Microstep.FULL
            rate = rate_sign * self._max_rate

        return (microstep_vote, rate)

    def _home_roll(self):
        pass

    def _track(self, axis: Axes, position: DegMinSec):
        pdps = int(position.get_steps(axis) - self.step_orientation[axis.value]) 
        rate_sign = -1 if pdps < 0 else 1

        rate = min(abs(pdps * 60), self._max_rate)
        rate = rate * rate_sign
        microstep_vote = Microstep.SIXTEENTH

        print(f"PDPS: {pdps}, Rate: {rate}")

        return (microstep_vote, rate)

    def _slew(self, axis:Axes , position: DegMinSec):

        pd = int(position.get_steps(axis) - self.step_orientation[axis.value]) 
        rate_sign = -1 if pd < 0 else 1

        rate = rate_sign * self._max_rate
        microstep_vote = Microstep.FULL

        if abs(pd >> Microstep.SIXTEENTH.value) > 10000:
            microstep_vote = Microstep.FULL
        elif abs(pd >> Microstep.EIGHTH.value) > 10000:
            microstep_vote = Microstep.HALF
        elif abs(pd >> Microstep.QUARTER.value) > 10000:
            microstep_vote = Microstep.QUARTER
        elif abs(pd >> Microstep.HALF.value) > 10000:
            microstep_vote = Microstep.EIGHTH
        else:
            microstep_vote = Microstep.SIXTEENTH

        if pd == 0:
            self.slew_complete[axis.value] = True
            microstep_vote = Microstep.SIXTEENTH
            rate = 0

        return (microstep_vote, rate)

    def get_orientation(self, axis: Axes):
        return self.orientation[axis.value]

    def orientation_update(self, log_packet: OrientationDataLogPacket):
       
        self.orientation = PhysicalOrientation.estimate_orientation(log_packet)

        #print(PhysicalOrientation.absolute_orientation(log_packet))

        self.step_orientation[Axes.YAW.value] = log_packet["yaw_position"]
        self.step_orientation[Axes.ROLL.value] = log_packet["roll_position"]
        self.step_orientation[Axes.PITCH.value] = log_packet["pitch_position"]

        self.microstep_mgr.microstep_update(log_packet)

    def home(self):
        def process_home_axes():
            print("Homing Start")

            self._set_state(StepperOperatingMode.HOMING)
            self._reset_desired_pos(Axes.PITCH)
            self._reset_desired_pos(Axes.YAW)
            self._reset_desired_pos(Axes.ROLL)

            self.pitch_homed = False
            self.yaw_homed = False
            pitch_closed = False
            yaw_closed = False

            # Two voters for yaw and pitch
            self.microstep_mgr.add_voter(Axes.YAW)
            self.microstep_mgr.add_voter(Axes.PITCH)

            while self.running:

                pitch_microstep_vote = Microstep.INVALID
                pitch_rate = None
                yaw_microstep_vote = Microstep.INVALID
                yaw_rate = None

                if not self.pitch_homed:
                    pitch_microstep_vote, pitch_rate = self._home_pitch()
                    self.microstep_mgr.vote(Axes.PITCH, pitch_microstep_vote)
                elif not pitch_closed:
                    self.microstep_mgr.sub_voter(Axes.PITCH)
                    self._set_axis_rate(Axes.PITCH, 0, True)
                    pitch_closed = True

                if not self.yaw_homed:
                    yaw_microstep_vote, yaw_rate = self._home_yaw()
                    self.microstep_mgr.vote(Axes.YAW, yaw_microstep_vote)
                elif not yaw_closed:
                    self.microstep_mgr.sub_voter(Axes.YAW)
                    self._set_axis_rate(Axes.YAW, 0, True)
                    yaw_closed = True

                self.microstep_mgr.perform_vote()
                microstep_mode = self.microstep_mgr.get_microstep()

                if pitch_microstep_vote != Microstep.INVALID:
                    true_pitch_rate = (pitch_rate >> pitch_microstep_vote.value) << microstep_mode.value
                    self._set_axis_rate(Axes.PITCH, true_pitch_rate)

                if yaw_microstep_vote != Microstep.INVALID:
                    true_yaw_rate = (yaw_rate >> yaw_microstep_vote.value) << microstep_mode.value
                    self._set_axis_rate(Axes.YAW, true_yaw_rate)

                if self.yaw_homed and self.pitch_homed:
                    break
            
            self._reset_pos(Axes.PITCH)
            self._reset_pos(Axes.YAW)
            self._reset_pos(Axes.ROLL)

            print("Homing Complete")
        
        if not self._tracking:
            self.task_queue_lock.acquire()  
            self.task_queue.put(process_home_axes)
            self.task_queue_lock.release()

    def calibrate(self):
        def process_calibrate_axes():

            print("Calibration Start")

            self._set_state(StepperOperatingMode.HOMING)

            self.pitch_calibrated = False
            self.yaw_calibrated = False
            pitch_closed = False
            yaw_closed = False

            # Two voters for yaw and pitch
            self.microstep_mgr.add_voter(Axes.YAW)
            self.microstep_mgr.add_voter(Axes.PITCH)

            while self.running:

                pitch_microstep_vote = Microstep.INVALID
                pitch_rate = None
                yaw_microstep_vote = Microstep.INVALID
                yaw_rate = None

                if not self.pitch_calibrated:
                    pitch_microstep_vote, pitch_rate = self._home_pitch(-45)
                    self.microstep_mgr.vote(Axes.PITCH, pitch_microstep_vote)
                elif not pitch_closed:
                    self.microstep_mgr.sub_voter(Axes.PITCH)
                    self._set_axis_rate(Axes.PITCH, 0, True)
                    pitch_closed = True

                if not self.yaw_calibrated:
                    yaw_microstep_vote, yaw_rate = self._home_yaw(90)
                    self.microstep_mgr.vote(Axes.YAW, yaw_microstep_vote)
                elif not yaw_closed:
                    self.microstep_mgr.sub_voter(Axes.YAW)
                    self._set_axis_rate(Axes.YAW, 0, True)
                    yaw_closed = True

                self.microstep_mgr.perform_vote()
                microstep_mode = self.microstep_mgr.get_microstep()

                if pitch_microstep_vote != Microstep.INVALID:
                    true_pitch_rate = (pitch_rate >> pitch_microstep_vote.value) << microstep_mode.value
                    self._set_axis_rate(Axes.PITCH, true_pitch_rate)

                if yaw_microstep_vote != Microstep.INVALID:
                    true_yaw_rate = (yaw_rate >> yaw_microstep_vote.value) << microstep_mode.value
                    self._set_axis_rate(Axes.YAW, true_yaw_rate)

                if self.pitch_calibrated and self.yaw_calibrated:
                    break

            print("Calibration Complete")
            
        if not self._tracking:
            self.task_queue_lock.acquire()  
            self.task_queue.put(process_calibrate_axes)
            self.task_queue_lock.release()


    def goto(self, yaw: DegMinSec, roll: DegMinSec, pitch: DegMinSec):
        
        def process_goto_axes():
            self._set_state(StepperOperatingMode.SLEWING)
            print(f"Slew Start (YRP): ({str(yaw)}, {str(roll)}, {str(pitch)})")

            self.slew_complete = np.array([False, False, False])
            
            process_roll = True
            process_pitch = True
            process_yaw = True

            pitch_closed = False
            yaw_closed = False
            roll_closed = False

            pitch_waiting = False
            yaw_waiting = False
            roll_waiting = False

            if pitch.degree < -60 or pitch.degree > 0:
                process_pitch = False

            if roll.degree < -60 or roll.degree > 60:
                process_roll = False

            if process_roll:
                self.microstep_mgr.add_voter(Axes.ROLL)
                self._set_axis_desired_pos(Axes.ROLL, roll.get_steps(Axes.ROLL))
            else:
                self.slew_complete[Axes.ROLL.value] = True

            if process_pitch:
                self.microstep_mgr.add_voter(Axes.PITCH)
                self._set_axis_desired_pos(Axes.PITCH, pitch.get_steps(Axes.PITCH))
            else:
                self.slew_complete[Axes.PITCH.value] = True

            if process_yaw:
                self.microstep_mgr.add_voter(Axes.YAW)
                self._set_axis_desired_pos(Axes.YAW, yaw.get_steps(Axes.YAW))
            else:
                self.slew_complete[Axes.YAW.value] = True

            while self.running:
                pitch_microstep_vote = Microstep.INVALID
                pitch_rate = None
                yaw_microstep_vote = Microstep.INVALID
                yaw_rate = None
                roll_microstep_vote = Microstep.INVALID
                roll_rate = None

                if process_pitch and (not self.slew_complete[Axes.PITCH.value]):
                    pitch_microstep_vote, pitch_rate = self._slew(Axes.PITCH, pitch)
                    self.microstep_mgr.vote(Axes.PITCH, pitch_microstep_vote)
                elif not pitch_closed:
                    self.microstep_mgr.sub_voter(Axes.PITCH)
                    self._set_axis_rate(Axes.PITCH, 0, True)
                    pitch_closed = True

                if process_yaw and (not self.slew_complete[Axes.YAW.value]):
                    yaw_microstep_vote, yaw_rate = self._slew(Axes.YAW, yaw)
                    self.microstep_mgr.vote(Axes.YAW, yaw_microstep_vote)
                elif not yaw_closed:
                    self.microstep_mgr.sub_voter(Axes.YAW)    
                    self._set_axis_rate(Axes.YAW, 0, True)
                    yaw_closed = True

                if process_roll and (not self.slew_complete[Axes.ROLL.value]):
                    roll_microstep_vote, roll_rate = self._slew(Axes.ROLL, roll)
                    self.microstep_mgr.vote(Axes.ROLL, roll_microstep_vote)
                elif not roll_closed:
                    self.microstep_mgr.sub_voter(Axes.ROLL)    
                    self._set_axis_rate(Axes.ROLL, 0, True)
                    roll_closed = True

                self.microstep_mgr.perform_vote(True)
                microstep_mode = self.microstep_mgr.get_microstep()

                move_pitch = pitch_microstep_vote.value <= microstep_mode.value
                move_yaw = yaw_microstep_vote.value <= microstep_mode.value
                move_roll = roll_microstep_vote.value <= microstep_mode.value
                
                if pitch_microstep_vote != Microstep.INVALID:
                    if move_pitch:
                        true_pitch_rate = (pitch_rate >> pitch_microstep_vote.value) << microstep_mode.value
                        self._set_axis_rate(Axes.PITCH, true_pitch_rate)
                        pitch_waiting = False
                    elif process_pitch and not pitch_closed and not pitch_waiting:
                        self._set_axis_rate(Axes.PITCH, 0)
                        pitch_waiting = True

                if yaw_microstep_vote != Microstep.INVALID:
                    if move_yaw:
                        true_yaw_rate = (yaw_rate >> yaw_microstep_vote.value) << microstep_mode.value
                        self._set_axis_rate(Axes.YAW, true_yaw_rate)
                        yaw_waiting = False
                    elif process_yaw and not yaw_closed and not yaw_waiting:
                        self._set_axis_rate(Axes.YAW, 0)
                        yaw_waiting = True

                if roll_microstep_vote != Microstep.INVALID:
                    if move_roll:
                        true_roll_rate = (roll_rate >> roll_microstep_vote.value) << microstep_mode.value
                        self._set_axis_rate(Axes.ROLL, true_roll_rate)
                        roll_waiting = False
                    elif process_roll and not roll_closed and not roll_waiting:
                        self._set_axis_rate(Axes.ROLL, 0)
                        roll_waiting = True


                if self.slew_complete[Axes.ROLL.value] and self.slew_complete[Axes.YAW.value] and self.slew_complete[Axes.PITCH.value]:
                    break
            
            self._reset_desired_pos(Axes.PITCH)
            self._reset_desired_pos(Axes.YAW)
            self._reset_desired_pos(Axes.ROLL)
        
            print("Slew Complete")

        if not self._tracking:
            self.task_queue_lock.acquire()
            self.task_queue.put(process_goto_axes)
            self.task_queue_lock.release()

    def start_tracking(self):
        self._tracking = True
        self._set_state(StepperOperatingMode.TRACKING)
    
    def track(self, yaw : DegMinSec, roll : DegMinSec, pitch : DegMinSec):
        """
            This function gets called once per second, and values provided here are for 1 second into the future.
        """

        def process_track_axes():
            print(f"Tracking (YRP): ({str(yaw)}, {str(roll)}, {str(pitch)})")
            
            process_roll = True
            process_pitch = True
            process_yaw = True

            if pitch.degree < -60 or pitch.degree > 0:
                process_pitch = False

            if roll.degree < -60 or roll.degree > 60:
                process_roll = False

            if process_roll:
                self.microstep_mgr.add_voter(Axes.ROLL)
                self._set_axis_desired_pos(Axes.ROLL, roll.get_steps(Axes.ROLL))

            if process_pitch:
                self.microstep_mgr.add_voter(Axes.PITCH)
                self._set_axis_desired_pos(Axes.PITCH, pitch.get_steps(Axes.PITCH))

            if process_yaw:
                self.microstep_mgr.add_voter(Axes.YAW)
                self._set_axis_desired_pos(Axes.YAW, yaw.get_steps(Axes.YAW))

            pitch_microstep_vote = Microstep.INVALID
            pitch_rate = None
            yaw_microstep_vote = Microstep.INVALID
            yaw_rate = None
            roll_microstep_vote = Microstep.INVALID
            roll_rate = None

            if process_pitch:
                pitch_microstep_vote, pitch_rate = self._track(Axes.PITCH, pitch)
                self.microstep_mgr.vote(Axes.PITCH, pitch_microstep_vote)

            if process_yaw:
                yaw_microstep_vote, yaw_rate = self._track(Axes.YAW, yaw)
                self.microstep_mgr.vote(Axes.YAW, yaw_microstep_vote)

            if process_roll:
                roll_microstep_vote, roll_rate = self._track(Axes.ROLL, roll)
                self.microstep_mgr.vote(Axes.ROLL, roll_microstep_vote)

            self.microstep_mgr.perform_vote()
            microstep_mode = self.microstep_mgr.get_microstep()
            
            if process_pitch:
                true_pitch_rate = (pitch_rate >> pitch_microstep_vote.value) << microstep_mode.value
                self._set_axis_rate(Axes.PITCH, true_pitch_rate)

            if process_yaw:
                true_yaw_rate = (yaw_rate >> yaw_microstep_vote.value) << microstep_mode.value
                self._set_axis_rate(Axes.YAW, true_yaw_rate)

            if process_roll:
                true_roll_rate = (roll_rate >> roll_microstep_vote.value) << microstep_mode.value
                self._set_axis_rate(Axes.ROLL, true_roll_rate)


        self.task_queue_lock.acquire()
        self.task_queue.put(process_track_axes)
        self.task_queue_lock.release()


    def stop_tracking(self):
        self._tracking = False
        self._set_state(StepperOperatingMode.SLEWING)

    def is_processing(self):
        return self._processing

    def ack(self):
        self.tx_mgr.write_command(UARTCommand(UARTCommands.CMD_ACK, UARTCommand.to_int24(0)))

    def stop(self):
        self.tx_mgr.write_command(UARTCommand(UARTCommands.CMD_STOP, UARTCommand.to_int24(0)))
    
    def enable(self):
        self._set_state(StepperOperatingMode.SLEWING)
    
    def disconnect(self):
        self.running = False


class PhysicalOrientation:

    @staticmethod
    def estimate_orientation(log_packet: OrientationDataLogPacket):
        """
            Use magnetometer, gyro and accel data to estimate the orientation of the telescope
            Note:
                - Roll orientation will always be zero here as no valid way of homing the roll axis exists
        """
        
        _hard_x_offset = 182
        _hard_y_offset = 212

        pitch = np.rad2deg(np.atan2(log_packet["out_y_a_raw"], log_packet["out_x_a_raw"]))
        roll = 0.0
        yaw = np.rad2deg(np.atan2(log_packet["out_y_raw_onboard"] - _hard_y_offset, log_packet["out_x_raw_onboard"] - _hard_x_offset))

        return np.array([yaw, roll, pitch])


    @staticmethod
    def absolute_orientation(log_packet: OrientationDataLogPacket):
        """
            Use step based positions to track finer resolution orientation
        """
        microstep_mode = log_packet["microstep_mode"]

        yaw = log_packet["yaw_position"]
        roll = log_packet["roll_position"]
        pitch = log_packet["pitch_position"]

        pitch_steps = 1143200
        yaw_steps = 200 * 60 * 27 * 16
        roll_steps = 1143200

        yaw_angle = yaw / yaw_steps * 360
        roll_angle = roll / roll_steps * 90
        pitch_angle = pitch / pitch_steps * 90

        return np.array([yaw_angle, roll_angle, pitch_angle])