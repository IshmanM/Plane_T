import cv2
import numpy as np


class Detection:
    def __init__(self, u: float | None, v: float | None, px_w: float | None, px_h: float | None):
        self.u = u 
        self.v = v
        self.px_w = px_w 
        self.px_h = px_h 

class Measurement:
    def __init__(self, x: float | None, y: float | None, z: float | None = None, pitch: float | None = None, roll: float | None = None, yaw: float | None = None):
        self.x = x 
        self.y = y
        self.y = z
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw 


def detector(frame):
    
    #implementation TBD#################3

    u, v, px_w, px_h = None, None, None, None

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_hsv = np.array([0, 30, 40])
    upper_hsv = np.array([25, 200, 255])
    mask = cv2.inRange(frame, lower_hsv, upper_hsv)
    contours, heirarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0: 
        largest_contour = max(contours, key = cv2.contourArea)
        u, v, px_w, px_h = cv2.boundingRect(largest_contour)
        u = u + px_w/2
        v = v + px_h/2

    print(u, v, px_w, px_h)

    detection = Detection(u, v, px_w, px_h)
    measurement = Measurement(0,0,0,0,0,0) 

    ################

    return detection, measurement



# hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

# mask = clean_mask(mask)

# contours = cv2.findContours(mask, ...)

# largest_contour = max(contours, key=cv2.contourArea)

# x, y, w, h = cv2.boundingRect(largest_contour)

# center_u = x + w / 2
# center_v = y + h / 2




