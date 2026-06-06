import config

# image frame to world frame relative to camera lens

def estimateWorldDepth(px_w, px_h):
    # Depth estimator based on emperical calibration & Pose Converter
    return config.PX_FOCAL_LENGTH*config.W/max(px_w, px_h) # z

def estimateWorldPosition(u, v, px_w, px_h):
    z = estimateWorldDepth(px_w, px_h)
    x = (u - config.FRAME_W/2)*z/config.PX_FOCAL_LENGTH 
    y = (v - config.FRAME_H/2)*z/config.PX_FOCAL_LENGTH 
    return x, y, z


# world frame to image frame relative to camera lens

def estimateImagePosition(x, y, z):
    if x is None or y is None or z is None:
        return None, None
    if z == 0:
        return 0, 0 
    u = (x * config.PX_FOCAL_LENGTH / z) + config.FRAME_W / 2
    v = (y * config.PX_FOCAL_LENGTH / z) + config.FRAME_H / 2
    return u, v