import cv2
import models
import numpy as np
from enum import Enum, auto
from models import Detection, Measurement
import config

class TrackStatus(Enum):
    TENTATIVE = auto()
    CONFIRMED = auto()
    DEAD = auto()

X, Y, Z, DZ, DX, DY, DZ = 0,1,2,3,4,5

class Track:
    
    def __init__ (self, initial_measurement: Measurement, min_hits: int, track_id=0):
    
        self.id = track_id # not useful until MOT

        # x, y, z, dx, dy, dz
        self.state = np.array([
            initial_measurement.x, initial_measurement.y, initial_measurement.z,
            0.0, 0.0, 0.0
        ], dtype=float)
    
        # Need to tune starting covariance...
        self.covariance = np.diag([
            config.INIT_TRACK_SIGMA_X ** 2,
            config.INIT_TRACK_SIGMA_Y ** 2,
            config.INIT_TRACK_SIGMA_Z ** 2,
            config.INIT_TRACK_SIGMA_DX ** 2,
            config.INIT_TRACK_SIGMA_DY ** 2,
            config.INIT_TRACK_SIGMA_DZ ** 2
        ], dtype=float)

        self.hit_streak = 1       # consecutive hits   
        self.missed_streak = 0    # consecutive misses
        self.confirmed = False if min_hits > 1 else True

        self.last_measurement = initial_measurement


    def mark_hit(self, measurement, min_hits: int):
        self.hit_streak += 1
        self.missed_streak = 0
        self.last_measurement = measurement

        if self.hit_streak >= min_hits:
            self.confirmed = True


    def mark_missed(self):
        self.missed_streak += 1
        self.hit_streak = 0


    def is_dead(self, max_missed: int) -> bool:
        return self.missed_streak > max_missed
    
    @property
    def x(self):
        return self.state[X]

    @property
    def y(self):
        return self.state[Y]

    @property
    def z(self):
        return self.state[Z]
    
    @property
    def dx(self):
        return self.state[DX]

    @property
    def dy(self):
        return self.state[DY]

    @property
    def dz(self):
        return self.state[DZ]   




class SingleObjectTracker:
    def __init__(
        self,
        min_hits: int = 3,
        max_missed_on_confirmed: int = 15,
        max_missed_on_tentative: int = 1,
        process_noise: float = 0.01,
        measurement_noise: float = 0.05,
        gate_threshold: float = 0.25,
    ):
        self.track = None
        self.min_hits = min_hits
        self.max_missed_on_confirmed = max_missed_on_confirmed
        self.max_missed_on_tentative = max_missed_on_tentative
        
        # Kalman tuning parameters
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        self.gate_threshold = gate_threshold

    
    def gating_distance(self, measurement: Measurement):
        if self.track is None:
            raise ValueError("Cannot compute gating distance without an active track.")
        
        # Euclidian distance 
        measurement_position = np.array([measurement.x, measurement.y, measurement.z], dtype=float)  
        diff = measurement_position - self.track.state[:3]
        return float(diff @ diff)

    def predict(self, dt):
        """
        Kalman prediction step.

        Theory:
            x_k_pred = F_k @ x_k_prev
            P_k_pred = F_k @ P_k_prev @ F_k.T + Q_k

        In this tracker:
            state = [x, y, z, dx, dy, dz]
        """

        if self.track is None:
            return
        
        x_k_prev = self.track.state

        P_k_prev = self.track.covariance

        F_k = np.eye(6, dtype=float) # State transition matrix
        F_k[X, DX] = dt
        F_k[Y, DY] = dt
        F_k[Z, DZ] = dt

        Q_k = np.eye(6, dtype=float) * self.process_noise
        x_k_pred = F_k @ x_k_prev
        P_k_pred = F_k @ P_k_prev @ F_k.T + Q_k

        self.track.state = x_k_pred
        self.track.covariance = P_k_pred
        return
       
    def update_with_measurement(self, measurement: Measurement):
        
       #... self.track.mark_detected <--DO NOT DO THIS HERE

        return


    def update(self, object_detected, measurement: Measurement, dt):
        
        # None track logic:
        #     any measurement should create a track.
        #     a missing measurement should do nothing.
        if self.track is None:
            if not object_detected:
                return TrackStatus.DEAD  
            else:
                self.track = Track(measurement)

        # TENTATIVE track logic:
        #     both a far & missing measurement should count as a miss, then check if dead.
        # CONFIRMED track logic:
        #     both a far & missing measurement should count as a miss, then check if dead.
        #     defferent max_missed
        #     they do not become TENTATIVE tracks just because of a miss
        else:
            self.predict(dt) # Kalman Prediction update
            
            if (not object_detected) or (self.gating_distance(measurement) > self.gate_threshold):
                # miss 
                self.track.mark_missed()
                max_missed = self.max_missed_on_confirmed if self.track.confirmed else self.max_missed_on_tentative
                if self.track.is_dead(max_missed):
                    if not object_detected:
                        self.track = None 
                        return TrackStatus.DEAD
                    else:
                        self.track = Track(measurement)
            else:
                # hit
                self.track.mark_hit()
                self.update_with_measurement(measurement) # Kalman Measurement update
        
        return TrackStatus.CONFIRMED if self.track.confirmed else TrackStatus.TENTATIVE
    

        




# for future
# class MultiObjectTracker:
#     def __init__(self):
#         self.tracks = []
#         #etc...
#   def update(self, detections, dt):
        # 1. Predict every existing track forward
        # 2. Match detections to tracks
        # 3. Update matched tracks
        # 4. Mark unmatched tracks as missed
        # 5. Create new tracks for unmatched detections
        # 6. Delete dead tracks
        # 7. Return confirmed active tracks