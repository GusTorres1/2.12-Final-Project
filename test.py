import cv2
import numpy as np

def nothing(x):
    pass

cap = cv2.VideoCapture(0)

cv2.namedWindow('f')
cv2.createTrackbar('Hl', 'f', 0, 255, nothing)
cv2.createTrackbar('Sl', 'f', 0, 255, nothing)
cv2.createTrackbar('Vl', 'f', 0, 255, nothing)
cv2.createTrackbar('Hh', 'f', 0, 255, nothing)
cv2.createTrackbar('Sh', 'f', 0, 255, nothing)
cv2.createTrackbar('Vh', 'f', 0, 255, nothing)

while(True):
    rl = cv2.getTrackbarPos('Hl', 'f')
    gl = cv2.getTrackbarPos('Sl', 'f')
    bl = cv2.getTrackbarPos('Vl', 'f')
    rh = cv2.getTrackbarPos('Hh', 'f')
    gh = cv2.getTrackbarPos('Sh', 'f')
    bh = cv2.getTrackbarPos('Vh', 'f')

    # Capture frame-by-frame
    ret, frame = cap.read()
    frame = frame[0:440,100:560]
    #print(frame.shape)

    # Our operations on the frame come here
    # gray = cv2.cvtColor(frame,cv2.COLOR_RGB2RGBA)
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    lower_red=np.array([rl,gl,bl])
    upper_red = np.array([rh,gh,bh])
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_red, upper_red)
    # Bitwise-AND mask and original image
    res1 = cv2.bitwise_and(frame,frame, mask= mask)


    #cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res1',res1)

    if cv2.waitKey(10) & 0xFF == ord('b'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
