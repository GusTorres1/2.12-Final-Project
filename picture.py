import cv2
import numpy as np


frame_scale = 1.5
ycl = 27
ych = 662
xcl = 68
xch = 700

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # turn the autofocus off
while True:
    ret,frame = cap.read()
    frame = cv2.resize(frame, None, fx = frame_scale, fy = frame_scale )
    frame = frame[ ycl:ych, xcl:xch ]
    cv2.imshow('cool',frame)
    #frame = frame[0:440, 100:560]
    cv2.imwrite("test.png",frame)
    if cv2.waitKey(10) & 0xFF == ord('b'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
