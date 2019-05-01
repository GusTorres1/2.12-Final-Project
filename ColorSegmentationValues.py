import numpy as np
import cv2

cap=cv2.VideoCapture(0)

while True:
    ret,frame = cap.read()
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    
    # Range for upper range
    lower_red = np.array([170,120,70])
    upper_red = np.array([180,255,255])
    mask = cv2.inRange(hsv,lower_red,upper_red)
    
    result = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow(result)

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()