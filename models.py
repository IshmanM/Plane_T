
class Detection:
    def __init__(self, u: float | None, v: float | None, px_w: float | None, px_h: float | None):
        self.u = u 
        self.v = v
        self.px_w = px_w 
        self.px_h = px_h 

class Measurement:
    def __init__(self, x: float | None, y: float | None, z: float | None = None, 
                 pitch: float | None = None, roll: float | None = None, yaw: float | None = None):
                
        self.x = x # x points right
        self.y = y # y points down
        self.z = z # z points away from the camera
        
        #probably not used:
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw 


