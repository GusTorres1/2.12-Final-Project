import numpy as np
import cv2

cap=cv2.VideoCapture(0)

ret,frame = cap.read()
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

mask = cv2.inRange()

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()