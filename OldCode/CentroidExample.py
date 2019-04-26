# CENTROID DETECTION EXAMPLE

import cv2
import numpy as np

pict = 'circle.png'
img = cv2.resize(cv2.imread(pict,1),(500,500))
image = cv2.resize(cv2.imread(pict,0),(500,500))
# cv2.imshow('a',image)
# cv2.waitKey(0)
ret,thresh = cv2.threshold(image,100,255,0)
cv2.imshow('b',thresh)
cv2.waitKey(0)

contours,hierarchy = cv2.findContours(thresh, 1, 2)

M = [cv2.moments(contours[i]) for i in range(0, len(contours))]
cx = [ ( int( m['m10']/m['m00'] ) ) for m in M]
cy = [ ( int( m['m01']/m['m00'] ) ) for m in M]

cnt = contours[0]
M = cv2.moments(cnt)

# print(M)
#
# cx = int(M['m10']/M['m00'])
# cy = int(M['m01']/M['m00'])

print(cx)
print(cy)

cv2.drawContours(img, contours, -1, (255,0,0),2)
lineThickness = 6
for i in range(0,len(cx)):
    cv2.line(img, (cx[i],cy[i]), (cx[i]+1,cy[i]+1), (120,120,0), lineThickness)

cv2.imshow("Cool",img)

cv2.waitKey(0)
cv2.destroyAllWindows()