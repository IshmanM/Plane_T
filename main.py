import cv2
import tracking as tra
import detection as det
import visualization as vis
import config


cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_H)
cap.set(cv2.CAP_PROP_FPS, config.FPS)

if __name__ == "__main__": 

    while cap.isOpened():
        ret, frame = cap.read()

        detection, measurement = det.detector(frame)
        
        if detection.u != None:
            cv2.rectangle(frame, 
                        (int(detection.u - detection.px_w/2), int(detection.v - detection.px_h/2)),
                        (int(detection.u + detection.px_w/2), int(detection.v + detection.px_h/2)),
                        color=(0,255,0), thickness=2)
            
            frame = cv2.flip(frame, 1) # optional flip as a last step before labelling, for viewing only 
            label = "Measurement: (x: " + f"{measurement.x:.4f}" + ", y: " + f"{measurement.y:.4f}"  + ", z: " + f"{measurement.z:.4f}" + ")"
            cv2.putText(frame, label,
                        (10,10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color=(0,255,0), thickness=2)

        
        cv2.imshow("Webcam Feed", frame)

    
        # Press ESC to close
        if cv2.waitKey(40) == 27:
            break


    cap.release()
    cv2.destroyAllWindows()
    
