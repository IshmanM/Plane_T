import cv2
import tracking as tra
import detection as det
import visualization as vis
import config
from util import estimateImagePosition



cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_H)
cap.set(cv2.CAP_PROP_FPS, config.FPS)

if __name__ == "__main__": 

    tracker = tra.SingleObjectTracker("""insert params""")
    dt = 1/config.FPS

    while cap.isOpened():
        ret, frame = cap.read()

        object_detected, detection, measurement = det.detectSingleObject(frame)
        detection_label = "No detections"

        # Draw the detection
        if object_detected:
            cv2.rectangle(frame, 
                        (int(detection.u - detection.px_w/2), int(detection.v - detection.px_h/2)),
                        (int(detection.u + detection.px_w/2), int(detection.v + detection.px_h/2)),
                        color=(0,255,0), thickness=2)
            
            detection_label = "Measurement: (x: " + f"{measurement.x:.4f}" + ", y: " + f"{measurement.y:.4f}"  + ", z: " + f"{measurement.z:.4f}" + ")"

        frame = cv2.flip(frame, 1) # optional flip as a last step before labelling, for viewing only 
        cv2.putText(frame, detection_label,
                    (10,20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color=(0,255,0), thickness=2)

        



        # Track the object state...

        tracker.update(object_detected, measurement, dt)
        tracku, v = 
        if tracker.track is not None:
        u, v = estimateImagePosition(tracker.track.x, tracker.track.y, tracker.track.z)
        
        #etc...


        # Lock on to object... (will need rpi version of code)








        cv2.imshow("Webcam Feed", frame)

        # Press ESC to close
        if cv2.waitKey(40) == 27:
            break


    cap.release()
    cv2.destroyAllWindows()
    
