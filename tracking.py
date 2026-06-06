import cv2
import models
import numpy as np
from enum import Enum, auto
from models import Detection, Measurement


class TrackStatus(Enum):
    TENTATIVE = auto()
    CONFIRMED = auto()
    DEAD = auto()


class Track:
    
    def __init__ (self, initial_measurement: Measurement, min_hits: int, track_id=0):
    
        self.id = track_id # not useful until MOT

        # x, y, z, dx, dy, dz
        self.state = np.array([
            initial_measurement.x, initial_measurement.y, initial_measurement.z,
            0.0, 0.0, 0.0
        ], dtype=float)
    
        self.covariance = np.eye(6, dtype=float)

        self.hit_streak = 1       # consecutive hits   
        self.missed_streak = 0    # consecutive misses
        self.confirmed = False if min_hits > 1 else True

        self.last_measurement = initial_measurement


    def mark_detected(self, measurement, min_hits: int):
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
        return self.state[0]

    @property
    def y(self):
        return self.state[1]

    @property
    def z(self):
        return self.state[2]
    
    @property
    def dx(self):
        return self.state[3]

    @property
    def dy(self):
        return self.state[4]

    @property
    def dz(self):
        return self.state[5]   




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

    
    def predict(dt):
        
        #...
        
        return
    
    def gating_distance(measurement: Measurement):
        #...

        return
    
    def update_with_measurement(measurement: Measurement):
        
       #... self.track.mark_detected <--DO NOT DO THIS HERE

        return


    def update(self, object_detected, measurement: Measurement, dt):
        
        # None track logic:
        #     any measurement should create a track.
        #     a missing measurement should do nothing.
        if self.track is None:
            if object_detected:
                self.track = Track(measurement)
            else:
                return TrackStatus.DEAD        

        # TENTATIVE track logic:
        #     a far measurement should replace it with new Track.
        #     a missing measurement should set the track to None.
        elif not self.track.confirmed:
            self.predict(dt) # Kalman Prediction update

            # if not object_detected:
            #     self.track = None
            #     return TrackStatus.DEAD
            
            # elif self.gating_distance(measurement) > self.gate_threshold:
            #     self.track = Track(measurement)
        
            # else: 
            #     self.track.mark_detected()
            #     self.update_with_measurement(measurement) # Kalman Measurement update
                
            if (not object_detected) or (self.gating_distance(measurement) > self.gate_threshold):
                # DEAD tracks are dropped
                self.track.mark_missed()
                if self.track.is_dead(self.max_missed_on_tentative):
                    if not object_detected:
                        self.track = None 
                        return TrackStatus.DEAD
                    else:
                        self.track = Track(measurement)
            
            else:
                self.track.mark_detected()
                self.update_with_measurement(measurement) # Kalman Measurement update
                return TrackStatus.CONFIRMED
                
        # CONFIRMED track logic:
        #     both a far & missing measurement should count as a miss, then check if dead.
        else:
            self.predict(dt) # Kalman Prediction update

            if (not object_detected) or (self.gating_distance(measurement) > self.gate_threshold):
                # DEAD tracks are dropped
                self.track.mark_missed()
                if self.track.is_dead(self.max_missed_on_confirmed):
                    if not object_detected:
                        self.track = None 
                        return TrackStatus.DEAD
                    else:
                        self.track = Track(measurement)
            
            else:
                self.track.mark_detected()
                self.update_with_measurement(measurement) # Kalman Measurement update
                return TrackStatus.CONFIRMED
        
        return #TENTATIVE or CONFIRMED based on ...
    

        




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