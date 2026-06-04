import cv2
import numpy as np
import config


class Detection:
    def __init__(self, u: float | None, v: float | None, px_w: float | None, px_h: float | None):
        self.u = u 
        self.v = v
        self.px_w = px_w 
        self.px_h = px_h 

class Measurement:
    def __init__(self, x: float | None, y: float | None, z: float | None = None, pitch: float | None = None, roll: float | None = None, yaw: float | None = None):
        self.x = x # x points right
        self.y = y # y points down
        self.z = z # z points away from the camera
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw 



# LOWER_HSV = np.array([14, 50, 100])
# UPPER_HSV = np.array([25, 130, 200])
LOWER_HSV = np.array([13, 35, 70])
UPPER_HSV = np.array([28, 190, 255])
MIN_CONTOUR_AREA = 100

# approximate the focal length
px_focal_length = 500  # = average of a few (reference_pixel_width * reference_distance / reference_width)
w =  0.065 # real marker width 

def detector(frame):
    
    #implementation TBD#################
    # need to account for case where nothing detected...
    # need to throw in some extra blur/filtering/rejection here, before even fedd to kalman later...

    # Classical CV Detector using thresholding
    u, v, px_w, px_h = None, None, None, None
    x, y, z, pitch, roll, yaw = None, None, None, None, None, None

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(frame, LOWER_HSV, UPPER_HSV) # Color threshold
    mask = cv2.medianBlur(mask, 5) # apply blur
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) # remove small random white specks
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) # fil small black holes/gaps 

    contours, heirarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0: 
        largest_contour = max(contours, key = cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        if area > MIN_CONTOUR_AREA:
            u, v, px_w, px_h = cv2.boundingRect(largest_contour)
            u = u + px_w/2
            v = v + px_h/2

            # Depth estimator based on emperical calibration & Pose Converter
            z = px_focal_length*w/px_w
            x = (u - config.FRAME_W/2)*z/px_focal_length 
            y = (v - config.FRAME_H/2)*z/px_focal_length 

    detection = Detection(u, v, px_w, px_h)
    measurement = Measurement(x,y,z,pitch,roll,yaw) 
    return detection, measurement




