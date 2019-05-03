import cv2
import numpy as np
from matplotlib import pyplot as plt

def nothing(x):
    pass

cap = cv2.VideoCapture(0)

cv2.namedWindow('f')
cv2.createTrackbar('Rl', 'f', 0, 255, nothing)
cv2.createTrackbar('Gl', 'f', 0, 255, nothing)
cv2.createTrackbar('Bl', 'f', 0, 255, nothing)
cv2.createTrackbar('Rh', 'f', 0, 255, nothing)
cv2.createTrackbar('Gh', 'f', 0, 255, nothing)
cv2.createTrackbar('Bh', 'f', 0, 255, nothing)

while(True):
    rl = cv2.getTrackbarPos('Rl', 'f')
    gl = cv2.getTrackbarPos('Gl', 'f')
    bl = cv2.getTrackbarPos('Bl', 'f')
    rh = cv2.getTrackbarPos('Rh', 'f')
    gh = cv2.getTrackbarPos('Gh', 'f')
    bh = cv2.getTrackbarPos('Bh', 'f')

    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    # gray = cv2.cvtColor(frame,cv2.COLOR_RGB2RGBA)
    hsv=cv2.cvtColor(frame,cv2.COLOR_RGB2HSV)
    # hsv=cv2.cvtColor(frame,cv2.COLOR_RGB2HSV)
    # hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    # Display the resulting frame
    # cv2.imshow('frame',hs1)
    # cv2.imshow('frame',hs2)

    # define range of blue color in HSV
    # lower_red = np.array([[[-5,0,0]]])+hsv

    lower_red=np.array([rl,gl,bl])
    upper_red = np.array([rh,gh,bh])
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_red, upper_red)
    # Bitwise-AND mask and original image
    res1 = cv2.bitwise_and(frame,frame, mask= mask)

    # cv2.calcHist([frame],[0],None,[256],[0,256])
    # plt.hist(frame.ravel(), 256, [0, 256])
    # plt.show()

    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res1',res1)

    if cv2.waitKey(10) & 0xFF == ord('b'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()