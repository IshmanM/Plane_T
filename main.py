import cv2
import tracking as tra
import detection as det
import visualization as vis
import config


cap = cv2.VideoCapture(0)

if __name__ == "__main__": 

    while cap.isOpened():
        ret, frame = cap.read()

        detection, measurement = det.detector(frame)
        
        if detection.u != None:
            cv2.rectangle(frame, 
                        (int(detection.u - detection.px_w/2), int(detection.v - detection.px_h/2)),
                        (int(detection.u + detection.px_w/2), int(detection.v + detection.px_h/2)),
                        (0,255,0), 2)

        cv2.imshow("Webcam Feed", frame)
    
        # Press ESC to close
        if cv2.waitKey(40) == 27:
            break


    cap.release()
    cv2.destroyAllWindows()
    
