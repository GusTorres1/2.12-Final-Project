import cv2
import numpy as np

def nothing(x):
    pass

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # turn the autofocus off

cv2.namedWindow('f')
cv2.createTrackbar('hl', 'f', 0, 255, nothing)
cv2.createTrackbar('sl', 'f', 0, 255, nothing)
cv2.createTrackbar('vl', 'f', 0, 255, nothing)
cv2.createTrackbar('hh', 'f', 179, 179, nothing)
cv2.createTrackbar('sh', 'f', 255, 255, nothing)
cv2.createTrackbar('vh', 'f', 255, 255, nothing)

#ret,frame = cap.read()
#frame = cv2.resize( frame, None, fx = 1.5, fy = 1.5 )
#xmax = frame.shape[1]-1
#ymax = frame.shape[0]-1
#cv2.createTrackbar('Xl', 'f', 0, 255, nothing)
#cv2.createTrackbar('Yl', 'f', 0, 255, nothing)
#cv2.createTrackbar('Xh', 'f', xmax, xmax, nothing)
#cv2.createTrackbar('Yh', 'f', ymax, ymax, nothing)

(ycl,ych,xcl,xch) = (27, 662, 68, 700)
#red = np.uint8([[[0,0,255 ]]])
#print(cv2.cvtColor(red,cv2.COLOR_BGR2HSV) )

while(True):
    hl = cv2.getTrackbarPos('hl', 'f')
    sl = cv2.getTrackbarPos('sl', 'f')
    vl = cv2.getTrackbarPos('vl', 'f')
    hh = cv2.getTrackbarPos('hh', 'f')
    sh = cv2.getTrackbarPos('sh', 'f')
    vh = cv2.getTrackbarPos('vh', 'f')

    ##xcl = cv2.getTrackbarPos('Xl', 'f')
    #ycl = cv2.getTrackbarPos('Yl', 'f')
    #xch = cv2.getTrackbarPos('Xh', 'f')
    #ych = cv2.getTrackbarPos('Yh', 'f')

    # Capture frame-by-frame
    ret, frame = cap.read()
    frame = cv2.resize( frame, None, fx = 1.5, fy = 1.5 )
    #frame = frame[0:440,100:560]
    frame = frame[ ycl:ych, xcl:xch ]
    blur = cv2.GaussianBlur( frame, (5,5), 0 )
    #print(frame.shape)

    # Our operations on the frame come here
    hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
    #print( (hl,hh,sl,sh,vl,vh) )
    lower =np.array([hl,sl,vl])
    upper = np.array([hh,sh,vh])
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower, upper)
    # Bitwise-AND mask and original image
    res1 = cv2.bitwise_and(frame,frame, mask= mask)

    #cv2.imshow('frame',frame)
    cv2.imshow('hsv',hsv)
    cv2.imshow('mask',mask)
    cv2.imshow('res1',res1)

    if cv2.waitKey(10) & 0xFF == ord('b'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
