import control
from enum import Enum, auto
from tracking import TrackStatus, SingleObjectTracker
import numpy as np


class LauncherMode(Enum): 
    SEARCHING = auto() #idle, no valid tracks to point to
    SLEWING_TO_LEAD = auto()
    LEAD_TRACKING = auto()

class LauncherCommand:
    def __init__(self, pan_deg: float, tilt_deg: float, fire: bool, launcher_mode: LauncherMode):
        self.pan_deg = pan_deg
        self.tilt_deg = tilt_deg
        self.fire = fire
        self.mode = launcher_mode.name


SERVO_NAMES = ("pan", "tilt")
N_SERVOS = len(SERVO_NAMES)
SERVO_INDEX = {
    name: i for i, name in enumerate(SERVO_NAMES)
}


class ActivePlan:
    def __init__(self, track_id: int, intercept_position: np.ndarray, intercept_time: float,
        raw_servo_angles: np.ndarray, cmd_servo_angles: np.ndarray, estimate_time: float,
        created_time: float, expected_ready_time: float, is_feasible: bool
    ):
        self.track_id = track_id

        self.intercept_position = np.asarray(intercept_position, dtype=float).copy()
        self.intercept_time = intercept_time

        self.raw_servo_angles = np.asarray(raw_servo_angles, dtype=float).copy()
        self.cmd_servo_angles = np.asarray(cmd_servo_angles, dtype=float).copy()

        self.estimate_time = estimate_time # will use the track state_time
        self.created_time = created_time                # is this necessary?
        self.expected_ready_time = expected_ready_time  # is this necessary?

        self.feasible = is_feasible


class Launcher:
    def __init__(self):
        self.mode = LauncherMode.SEARCHING
        self.active_plan = None
        # 
        # ...
        # #self. 

    def update(self, tracker: SingleObjectTracker):
       
        if tracker.track_status == TrackStatus.DEAD:


        if self.mode == LauncherMode.SEARCHING:
            if 
