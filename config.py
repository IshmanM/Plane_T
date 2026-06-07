
FRAME_W = 640
FRAME_H = 480
FPS = 30
PX_FOCAL_LENGTH = 500  # depends on frame width/height and = to the average of a few (reference_pixel_width * reference_distance / reference_width).
W =  0.0635 # real marker width (assuming W is appx H)


# Note smaller initial sigma/covariance indicates higher initial position certainty 
INIT_TRACK_SIGMA_X = 0.1
INIT_TRACK_SIGMA_Y = 0.1
INIT_TRACK_SIGMA_Z = 0.2
INIT_TRACK_SIGMA_DX = 1.0
INIT_TRACK_SIGMA_DY = 1.0
INIT_TRACK_SIGMA_DZ = 1.0
