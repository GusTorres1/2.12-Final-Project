#!/usr/bin/env python
#import rospy
#from std_msgs.msg import String
import cv2
import numpy as np

#topic for publishing
input_topic = '/pos'
output_topic = '/pos'
# publishes centroid and verifies when to take a new picture
def Centroid():
    rospy.init_node('Shapes', anonymous=True)
    pub = rospy.Publisher(input_topic, String, queue_size=10)
    #rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #current ROS upper level logic
        control_inputs = getControl()
        for ci in control_inputs:
            pub.publish(ci)
            tup = (pizza_location[0],pizza_location[1])
            pub.publish(tup)
            rospy.sleep(0.3)
        #pub.publish(control_inputs)
        #rate.sleep()

frame_scale = 1.5
ycl = 27
ych = 662
xcl = 68
xch = 700
pizza_location = [0,0,0]

# get xy from centroids
def getControl():
    # pizza_location = [0,0,0]
    ( frame, toppi ) = centroids_from_Picture()
    #cen = centroid_from_Picture()
    print('bool')
    print(toppi)
    #toppi = chooseToppings(cen)
    lineThickness = 6
    for i in range(0, len(toppi)):
        cv2.line(frame, toppi[i], (toppi[i][0] + 1, toppi[i][1]+1), (255, 255, 255), lineThickness)

    cv2.imshow("Cool", frame)
    cv2.waitKey(2500)
    return xy_from_centroid(toppi)

offset = 94.5

def chooseToppings( topp ):
    #print(topp)
    pizza_tops = topp['red']
    tops = []
    anch = topp['blue']
    ham = topp['pink']
    pep = topp['brown']
    olive = topp['black']
    pine = topp['yellow']
    stuff = [anch, ham, pep, olive, pine]
    le = [len(anch), len(ham), len(pep), len(olive), len(pine)]
    ml = max(le)
    for i in range(0,ml):
        for s in stuff:
            if i < len(s):
                tops.append(s[i])
    return tops

# def getCurr( list_top ):
#     for t in list_top:
        
    
# def withinPizza( topp ):
#     pass

def useCam( booo ):
    if booo:
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # turn the autofocus off
        ret, frame = cap.read()
        cap.release()
        frame = cv2.resize(frame, None, fx = frame_scale, fy = frame_scale )
        frame = frame[ ycl:ych, xcl:xch ]
        return frame
    else:
        return cv2.imread('test.png')

# captures picture and processes centroids
def centroids_from_Picture():
    pizza_location = [0, 0, 0]
    #cap = cv2.VideoCapture(0)
    #cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # turn the autofocus off
    #ret, frame = cap.read()
    #cap.release()
    frame = useCam( False );
    #frame = cv2.imread('test.png')
    #frame = cv2.resize(frame, None, fx = frame_scale, fy = frame_scale )
    #frame = frame[ ycl:ych, xcl:xch ]
    blur = cv2.GaussianBlur( frame, (5,5), 0 )

    img = frame
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    shapes = getShapes(frame, hsv)

    toppings_centroids = getCentroids(shapes, frame)
    # centroids = getCentroids2(shapesW,frame)
    # getCentroids2( shapes, frame )
    # return centroids
    #print(toppings_centroids)
    return (frame, chooseToppings( toppings_centroids ))
    

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
        #cv2.imshow('m',mask)
        #cv2.waitKey(1000)
        #cv2.imshow('mod',mod)
        #cv2.waitKey(1000)
        masks.append( mod )
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
# def getCentroids2(shapes,frame):
#     ab = 1
#     for i in range(ab, ab+1):
#     	c = spotCentroid( shapes[i] )
#     	print(c)
area_thresh = 75
lower_thresh = 40
def spotCentroid( mask, col ):
    thresh = mask
    #ret, thresh = cv2.threshold( mask, lower_thresh, 240, 0 )
    _, contours, _ = cv2.findContours( thresh, 1, 2 )

    M = [cv2.moments(contours[i]) for i in range(0, len(contours))]
    cx = []
    cy = []
    for m in M:
        if m['m00'] > area_thresh:
            cx.append( int(m['m10'] / m['m00']) )
            cy.append( int(m['m01'] / m['m00']) )
            # tries to find
            if col == 'red':
                if m['m00'] > pizza_location[2]:
                    pizza_location[0] = cx[-1]
                    pizza_location[1] = cy[-1]
                    pizza_location[2] = m['m00']
        #print(m['m00'])
    #cx = [(int(m['m10'] / m['m00'])) for m in M]
    #cy = [(int(m['m01'] / m['m00'])) for m in M]
    cen = list(zip(cx, cy))
    #cv2.drawContours(thresh, contours, -1, (255, 0, 0), 2)
    #lineThickness = 6
    #for i in range(0, len(cx)):
     #   cv2.line(mask, (cx[i], cy[i]), (cx[i] + 1, cy[i] + 1), (120, 120, 0), lineThickness)
    #cv2.imshow("Cool", thresh)
    #cv2.waitKey(2000)
    return cen

# gets the centroid from segmentation
def getCentroids(shapes, g):
    colors = ['red','blue','yellow','pink','brown','black']
    #colors = ['pizza', 'anch', '']
    #i = 0
    #print(shapes)
    topping_locations = {}
    for i in range(0,len(shapes)):
        #tl = spotCentroid( mask, colors[i] )
    	#thresh = mask
    	#ret, thresh = cv2.threshold(g, lower_thresh, 240, 0)
    	#_, contours, _ = cv2.findContours(thresh, 1, 2)
    	#M = [cv2.moments(contours[i]) for i in range(0, len(contours))]
    	#cx = [(int(m['m10'] / m['m00'])) for m in M]
    	#cy = [(int(m['m01'] / m['m00'])) for m in M]
    	#cen = list(zip(cx, cy))
        # topping_locations.append((colors[i],tl))
        #cv2.imshow('f',shapes[i])
        #cv2.waitKey(2000)
        tl = spotCentroid( shapes[i], colors[i] )
        #print(colors[i])
        #print(tl)
        topping_locations[ colors[i] ] = tl
        #i += 1
    return topping_locations

# processes all centroids for publishing
def xy_from_centroid(centroid_points):
    result = list(map(camera_transfer, centroid_points))
    return result

# in cm !
posx = 28.5*10
posy = 24.5*10
negx = -31*10
negy = -29*10
mx = (posx-negx)/(xch-xcl)
my = (posy-negy)/(ych-ycl)
# converts a centroid point to a camera function
def camera_transfer(centroid_point):
    (cx,cy) = centroid_point
    del_rox = negx + mx*(cy-10)
    del_roy = negy + my*(cx-10)
    return (del_rox,del_roy)

def morphologicalTrans(mask):
    # kernel = np.ones((5, 5), np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
    # dilation = cv2.dilate(mask, kernel, iterations=2)
    # erosion = cv2.erode(dilation, kernel, iterations=5)
    # opening = cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, kernel)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel )
    # closing = cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, kernel)
    return opening

# if __name__ == '__main__':
#     #centroids_from_Picture()
#     getControl()
#     # try:
#     #     Centroid()
#     # except rospy.ROSInterruptException:
#     #     pass

if __name__ == '__main__':
    #centroids_from_Picture()
    #getControl()
    try:
        Centroid()
    except rospy.ROSInterruptException:
        pass
