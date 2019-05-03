import cv2
import numpy as np

cap = cv2.VideoCapture(0)
ret,frame = cap.read()
frame = frame[0:440, 100:560]
cv2.imwrite("picture.png",frame)
cap.release()
