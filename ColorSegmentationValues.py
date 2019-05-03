import numpy as np
import cv2

cap=cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

while (True):
    ret,frame = cap.read()
    
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    
    # Range for upper range for red
    lower_red = np.array([0,0,0])
    upper_red = np.array([255,255,255])
    mask = cv2.inRange(hsv,lower_red,upper_red)
    
    result = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow('Res',result)
    if cv2.waitKey(10) & 0xFF == ord('b'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
