import control
from enum import Enum, auto
from tracking import Track, TrackStatus, SingleObjectTracker
import numpy as np
import time


class LauncherMode(Enum): 
    SEARCHING = auto() #idle, no valid tracks to point to
    SLEWING_TO_LEAD = auto()
    FOLLOWING_LEAD = auto()

class LauncherCmd:
    def __init__(self, pan_deg: float, tilt_deg: float, fire: bool, launcher_mode: LauncherMode):
        self.pan_deg = pan_deg
        self.tilt_deg = tilt_deg
        self.fire = fire
        self.mode = launcher_mode.name


SERVO_NAMES = ("pan", "tilt")
NUM_SERVOS = len(SERVO_NAMES)
SERVO_IDX = {
    name: i for i, name in enumerate(SERVO_NAMES)
}

DEFAULT_SERVO_ANGLES = np.zeros(NUM_SERVOS, dtype=float)
DEFAULT_SERVO_ANGLES[SERVO_IDX["pan"]] = 90.0
DEFAULT_SERVO_ANGLES[SERVO_IDX["tilt"]] = 45.0

MIN_SERVO_ANGLES = np.zeros(NUM_SERVOS, dtype=float)
MIN_SERVO_ANGLES[SERVO_IDX["pan"]] = 0.0
MIN_SERVO_ANGLES[SERVO_IDX["tilt"]] = 0.0

MAX_SERVO_ANGLES = np.zeros(NUM_SERVOS, dtype=float)
MIN_SERVO_ANGLES[SERVO_IDX["pan"]] = 180.0
MIN_SERVO_ANGLES[SERVO_IDX["tilt"]] = 85.0



class ActivePlan:
    def __init__(
        self, 
        track_id: int,
        raw_servo_angles: np.ndarray,
        estimate_time: float,
        created_time: float, # not that important
        expected_ready_time: float, 
        # is_feasible: bool,
        intercept_position: np.ndarray | None = None, 
        intercept_time: float | None = None,
        fire_time: float | None = None,
    ):
        self.track_id = track_id

        self.intercept_position = None if intercept_position is None else np.asarray(intercept_position, dtype=float).copy()
        self.intercept_time = None if intercept_time is None else intercept_time

        # Desired planner targets
        self.raw_servo_angles = np.asarray(raw_servo_angles, dtype=float).copy()
        
        self.estimate_time = estimate_time # will use the track state_time
        self.created_time = created_time                # is this necessary?
        self.expected_ready_time = expected_ready_time  

        # self.feasible = is_feasible           # is this necessary?

        self.fire_time = fire_time


MAX_SEQ = 2**32 - 1 # max uint32 number

class Launcher:
    def __init__(
        self,
        cmd_frequency = 30
    ):
        self.mode = LauncherMode.SEARCHING
        
        now = time.perf_counter() # not a self variable
        self.active_plan = self._make_search_plan(now)

        self.last_cmd_servo_angles = None # Latest actual cmd sent after filtering / rate limiting
        self.last_cmd_time = None 

        self.cmd_frequency = cmd_frequency
        self.cmd_period = 1.0 / self.cmd_frequency

        self.next_seq = 0   # will loop back around based on MAX_SEQ

    
    def _tracker_is_usable(self, tracker: SingleObjectTracker):
        if tracker is None:
            raise ValueError("None type Tracker passed to Launcher")
        
        if tracker.track_status == TrackStatus.CONFIRMED and tracker.track == None:
            raise ValueError("Tracker with CONFIRMED track_status but None type Track passed to Launcher")

        if tracker.track_status in {TrackStatus.TENTATIVE, TrackStatus.DEAD}:
            return False
        
        return True


    def update(self, tracker: SingleObjectTracker):
        now = time.perf_counter()
        
        ########## 0. UNIVERSAL TRACKER VALIDITY GUARD #############

        if not self._tracker_is_usable(tracker):
            self._set_searching(now)
            self._send_cmd_if_due(now)
            return
        
        ########## 1. TRACK ID SWITCH GUARD ########################

        if (self.mode != LauncherMode.SEARCHING and self.active_plan.track_id != tracker.track_id):
            self.active_plan = self._make_search_plan(now)
            self.mode = LauncherMode.SEARCHING
            # Don't return. Let SEARCHING immediately try to acquire the new track
        
        ########## 2. MODE LOGIC ##################################

        if self.mode == LauncherMode.SEARCHING:
            
            valid_plan_computed, plan = self._make_best_valid_first_intercept_plan(tracker.track, now)
            
            if valid_plan_computed:
                self.active_plan = plan
                self.mode = LauncherMode.SLEWING_TO_LEAD
            else:
                self.active_plan = self._make_search_plan(now)
                self.mode = LauncherMode.SEARCHING
        
        if self.mode == LauncherMode.SLEWING_TO_LEAD:
            
            if not self._active_plan_still_valid(tracker.track, now):
                valid_plan_computed, plan = self._make_best_valid_first_intercept_plan(tracker.track, now)

                if valid_plan_computed:
                    self.active_plan = plan
                else:
                    self.active_plan = self._make_search_plan(now)
                    self.mode = LauncherMode.SEARCHING
            
            # if (and not elif) incase the new best first intercept plan somehow chooses a point that the launcher is already pointed toward
            if (self.mode == LauncherMode.SLEWING_TO_LEAD and self._close_to_fire_time(now)):
                self.mode = LauncherMode.FOLLOWING_LEAD

        # elif (and not if) because the we dont want to get into receding horizon stuff until first launch
        elif self.mode == LauncherMode.FOLLOWING_LEAD:
            valid_plan_computed, plan = self._make_best_follow_receding_horizon_plan(tracker.track, now)

            if valid_plan_computed:
                self.active_plan = plan
            else:
                valid_plan_computed, plan = self._make_best_valid_first_intercept_plan(tracker.track, now)
                if valid_plan_computed: # tbh this case probably wont ever happen
                    self.active_plan = plan
                    self.mode = LauncherMode.SLEWING_TO_LEAD
                else:
                    self.active_plan = self._make_search_plan(now)
                    self.mode = LauncherMode.SEARCHING

    
        ########## 3. CMD OUTPUT ##############################
        
        self._send_cmd_if_due(now)
    
    


    def _send_cmd_if_due(self, now):
        
        cmd_due = True if (self.last_cmd_time is None or (now - self.last_cmd_time) >= self.cmd_period) else False
        if not cmd_due:
            return

        # First cmd: no previous actuator cmd exists
        if self.last_cmd_servo_angles is None:
            q_cmd = np.asarray(self.active_plan.raw_servo_angles, dtype=float).copy()
            q_cmd = np.clip(q_cmd, MIN_SERVO_ANGLES, MAX_SERVO_ANGLES) #clip for sanity, should already be fine though
        else:
            dt_cmd = now - self.last_cmd_time
            q_cmd = self._cmd_filter(dt_cmd) 
            # could maybe just have filter implemented here if one time use...
        
        self._send_cmd(q_cmd, now) # could maybe just have this implemented here if one time use...

    
    def _cmd_filter(self, dt_cmd):
      
        # uses self.active_plan.raw_servo_angles
        # uses self.last_cmd_servo_angles
        # uses MIN_SERVO_ANGLES, MAX_SERVO_ANGLES, MAX_SERVO_SPEEDS 
        # uses SERVO_DEADBAND and CMD_SMOOTHING_TAU
        # uses dt_cmd 
        #
        #
        raise NotImplementedError

    
    def _send_cmd(self, q_cmd: np.ndarray, now):
        #...uses self.seq, will increment it after and loop back to 0 upon reaching MAZ_SEQ
        # uses self.mode.name
        # uses self.active_plan.track_id
        # uses q_cmd
        # maybe need to have some class params related to the ethernet udp connetion??
        # 
        #
        #
        #
        #
        #
        self.last_cmd_servo_angles = q_cmd.copy()
        self.last_cmd_time = now
        raise NotImplementedError
    
    
    
    def _make_search_plan(self, now) -> ActivePlan:
        #...
        #-1 for the track id
        #
        raise NotImplementedError


    def _close_to_fire_time(self, now, fire_time=None) -> bool:
        
        # either specify a fire time or use the current active_plan's fire_time
        if fire_time == None: 
            fire_time = self.active_plan.fire_time 
        if fire_time == None:
            raise ValueError("Cannot compare now to None type fire_time")
        
        #...
        # we can fire a bit early or a bit later, by some small threshold(s)
        #
        #
        raise NotImplementedError

    def _make_best_valid_first_intercept_plan(self, track: Track, now) -> tuple[bool, ActivePlan]:
        #
        #
        # return valid_plan_computed (boolean), plan (None if not feasible)
        # can make a first plan that has fire time that is within _close_to_fire_time of now
        #
        raise NotImplementedError
    
    def _make_best_follow_receding_horizon_plan(self, track: Track, now) -> tuple[bool, ActivePlan]:
        #
        #
        #
        raise NotImplementedError
    
    def _active_plan_still_valid(self, track: Track, now) -> bool:
        #...
        # still valid if the anticipated intercept location and time is within the some threshold of the plan
        # if now is still _close_to_fire_time, still valid as long as the intercept point adds up
        # of course now can be any amount of time earlier than the fire_time
        raise NotImplementedError  
