import cv2
import numpy as np

frame_scale = 1.5
ycl = 27
ych = 662
xcl = 68
xch = 700

# get xy from centroids
def getControl():
    cen = centroid_from_Picture()
    return xy_from_centroid(cen)


offset = 94.5


# captures picture and processes centroids
def centroid_from_Picture():
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    cap.release()
    #frame = cv2.imread('picture.png')
    frame = cv2.resize(frame, None, fx = frame_scale, fy = frame_scale )
    frame = frame[ ycl:ych, xcl:xch ]
    blur = cv2.GaussianBlur( frame, (5,5), 0 )

    img = frame
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    shapes = getShapes(frame, hsv)

    # centroids = getCentroids(shapes, gray)
    centroids = getCentroids2(shapes,frame)
    return centroids


# should be tuples of (
# color (String),
# lower bound (array), upper bound (array) )
#colors = [('red', np.array([177, 76, 92]), np.array([255, 255, 255])),
 #         ('blue', np.array([52, 58, 77]), np.array([130, 255, 255])),
  #        ('yellow', np.array([19, 57, 108]), np.array([87, 255, 255])),
   #       ('yellow', np.array([19, 57, 108]), np.array([87, 255, 255])),
    #      ('brown', np.array([150, 47, 43]), np.array([180, 143, 88])),
     #     ('black', np.array([52, 0, 0]), np.array([148, 74, 74]))]
colors = [('red', np.array([0, 187, 46]), np.array([179, 255, 132]) ),
	('blue', np.array([65, 69, 77]), np.array([142, 255, 255]) ),
	('yellow', np.array([0, 64, 146]), np.array([65, 255, 255]) ),
	('pink', np.array([159, 77, 113]), np.array([179, 212, 255]) ),
	('brown', np.array([16, 102, 42]), np.array([171, 255, 93]) ),
	('black', np.array([0, 0, 0]), np.array([179, 71, 67]) )]

# creates color segmentation of the workspace.
def getShapes(image, h):
    masks = []
    for (c, l, u) in colors:
        mask = cv2.inRange(h, l, u)
        mod = morphologicalTrans(mask)
	cv2.imshow('m',mask)
	cv2.waitKey(1000)
	cv2.imshow('mod',mod)
	cv2.waitKey(1000)
        masks.append(mod)

    # # gets first shape from image
    # shapes = cv2.bitwise_and(image, image, mask = masks[0])
    #
    # # gets resulting shapes and or with current shape.
    # for i in range(1, len(masks)):
    #     sh = cv2.bitwise_and(image, image, mask = masks[i])
    #     # shapes = cv2.bitwise_or(shapes, shapes, mask = sh)
    #     shapes = cv2.add(shapes, sh)
    # cv2.imshow('cool',shapes)
    # cv2.waitKey(1000)
    return masks


lower_thresh = 30
def getCentroids2(shapes,frame):
    i = 5
    frame = frame[ ycl:ych, xcl:xch ]
    blur = cv2.GaussianBlur( frame, (5,5), 0 )
    c = spotCentroid( shapes[i] )
    print(c)

def spotCentroid( mask ):
    ret, thresh = cv2.threshold( mask, lower_thresh, 240, 0 )
    contours, hierarchy = cv2.findContours( thresh, 1, 2 )

    M = [cv2.moments(contours[i]) for i in range(0, len(contours))]
    cx = [(int(m['m10'] / m['m00'])) for m in M]
    cy = [(int(m['m01'] / m['m00'])) for m in M]
    cen = list(zip(cx, cy))

    cv2.drawContours(mask, contours, -1, (255, 0, 0), 2)
    lineThickness = 6
    for i in range(0, len(cx)):
        cv2.line(mask, (cx[i], cy[i]), (cx[i] + 1, cy[i] + 1), (120, 120, 0), lineThickness)

    cv2.imshow("Cool", mask)
    cv2.waitKey(2000)
    return cen

# gets the centroid from segmentation
def getCentroids(shapes, g):
    ret, thresh = cv2.threshold(g, lower_thresh, 240, 0)
    contours, hierarchy = cv2.findContours(thresh, 1, 2)

    M = [cv2.moments(contours[i]) for i in range(0, len(contours))]
    cx = [(int(m['m10'] / m['m00'])) for m in M]
    cy = [(int(m['m01'] / m['m00'])) for m in M]
    cen = list(zip(cx, cy))
    return cen


# processes all centroids for publishing
def xy_from_centroid(centroid_points):
    result = list(map(camera_transfer, centroid_points))
    return result


# converts a centroid point to a camera function
def camera_transfer(centroid_point):
    return centroid_point

def morphologicalTrans(mask):
    # kernel = np.ones((5, 5), np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    # dilation = cv2.dilate(mask, kernel, iterations=2)
    # erosion = cv2.erode(dilation, kernel, iterations=5)

    # opening = cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, kernel)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel )
    # closing = cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, kernel)
    return opening

if __name__ == '__main__':
    centroid_from_Picture()
    # try:
    #     Centroid()
    # except rospy.ROSInterruptException:
    #     pass
