import cv2
import numpy as np
from tracking import TrackStatus, SingleObjectTracker
from detection import detectSingleObject
import config
from camera_geometry import estimateImagePosition
from datetime import datetime
import time



cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_H)
cap.set(cv2.CAP_PROP_FPS, config.FPS)

if __name__ == "__main__": 

    tracker = SingleObjectTracker()
    # dt = 1/config.FPS # replacing this with the frame_time idea

    last_detection_px_w = 0
    last_detection_px_h = 0

    paused = False
    last_frame = None
    while cap.isOpened():
        

        if not paused:
            
            frame_time = time.perf_counter()
            ret, frame = cap.read() #doesnt oalways give latest frame but that's a future optimization.
            if not ret:
                print("Possible camera failure")
                break

            # Detect the object and produce a measurement
            object_detected, detection, measurement = detectSingleObject(frame)
            detection_label = "No detections"

            if object_detected:
                last_detection_px_w = detection.px_w
                last_detection_px_h = detection.px_h
                cv2.rectangle(frame, 
                            (int(detection.u -detection.px_w/2), int(detection.v - detection.px_h/2)),
                            (int(detection.u + detection.px_w/2), int(detection.v + detection.px_h/2)),
                            color=(0,255,0), thickness=2)
                
                cv2.circle(frame, (int(detection.u), int(detection.v)), radius=5, color=(0,255,0), thickness=-1)

                detection_label = "Measurement: (x: " + f"{measurement.x:.4f}" + ", y: " + f"{measurement.y:.4f}"  + ", z: " + f"{measurement.z:.4f}" + ")"
            


            # Track the object state
            track_status = tracker.update(object_detected, measurement, frame_time)
            
            track_label = "Dead track"
            if track_status == TrackStatus.CONFIRMED or track_status == TrackStatus.TENTATIVE:
                track_u, track_v = estimateImagePosition(tracker.track.x, tracker.track.y, tracker.track.z)

                # rectangle is drawn based on last detected px_w, px_h. might change this...
                cv2.rectangle(frame,
                            (int(track_u - last_detection_px_w/2), int(track_v - last_detection_px_h/2)),
                            (int(track_u + last_detection_px_w/2), int(track_v + last_detection_px_h/2)),
                            color=(0,0,255), thickness=2)

                cv2.circle(frame, (int(track_u), int(track_v)), radius=5, color=(0,0,255), thickness=-1)

                velocity_2d = np.array([tracker.track.dx, tracker.track.dy], dtype=float)
                velocity_norm = np.linalg.norm(velocity_2d)
                if velocity_norm > 1e-6:
                    arrow_length_px = 40
                    direction = velocity_2d / velocity_norm
                    arrow_end = (
                        int(round(int(track_u) + arrow_length_px * direction[0])),
                        int(round(int(track_v) + arrow_length_px * direction[1])),
                    )
                    cv2.arrowedLine(frame, (int(track_u), int(track_v)), arrow_end, (0, 0, 255), thickness=2, tipLength=0.25)

                track_label = ("Confirmed" if track_status == TrackStatus.CONFIRMED else "Tentative") 
                track_label = track_label + " track: (x: " + f"{tracker.track.x:.4f}" + ", y: " + f"{tracker.track.y:.4f}"  + ", z: " + f"{tracker.track.z:.4f}" 
                track_label = track_label + ", dx: " + f"{tracker.track.dx:.4f}" + ", dy: " + f"{tracker.track.dy:.4f}"  + ", dz: " + f"{tracker.track.dz:.4f}" + ")"



            
            # flip (optional) as a last step before then labelling, for viewing only 
            frame = cv2.flip(frame, 1) 
            cv2.putText(frame, detection_label, (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color=(0,255,0), thickness=1)
            cv2.putText(frame, track_label, (10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color=(0,0,255), thickness=1)


            # Predict the future position of the object
            #
            #
            #


            # Lock on to object... (will need rpi version of code)
            #
            #
            #



            last_frame = frame.copy()
        else:
            frame = last_frame.copy()



        cv2.imshow("Webcam Feed", frame)

        key = cv2.waitKey(1) & 0xFF
        # q = quit
        if key == ord("q"):
            break
        # p = pause/play toggle
        elif key == ord("p"):
            paused = not paused
        # s = screenshot
        elif key == ord("s"):
            filename = datetime.now().strftime("screenshot_%Y%m%d_%H%M%S.png")
            cv2.imwrite("screenshots/" + filename, frame)
            print(f"Saved {filename}")


    cap.release()
    cv2.destroyAllWindows()
    
