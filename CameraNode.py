#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import cv2
import numpy as np

#topic for publishing
input_topic = 'Centroid_Data'
output_topic = 'Motor_Control'
# publishes centroid and verifies when to take a new picture
def Centroid():
    rospy.init_node('Shapes', anonymous=True)
    pub = rospy.Publisher(input_topic, String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #current ROS upper level logic
        control_inputs = getControl()
        pub.publish(control_inputs)
        rate.sleep()

# get xy from centroids
def getControl():
    cen = centroid_from_Picture()
    return xy_from_centroid( cen )
    
offset = 94.5
# captures picture and processes centroids
def centroid_from_Picture():
    cap=cv2.VideoCapture(0)
    ret, frame = cap.read()
    img = cv2.resize(frame,(1000,1000))
    
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    cap.release()
    shapes = getShapes( img,hsv )
    centroids = getCentroids( shapes, gray )
    return centroids

# should be tuples of (
# color (String),
# lower bound (array), upper bound (array) )
colors = [( 'red', np.array([0,100,100]), np.array([5,255,255]) ), 
    ('blue', np.array([110,50,50]), np.array([130,255,255]) ),
    ('yellow', np.array([20,70,70]), np.array([25,255,255]) ),
    ('pink', np.array([160,70,80]), np.array([175,220,255]) ),
    ('brown', np.array([160,100,10]), upper_red = np.array([180,200,60]) ),
    ('black', np.array([120,0,0]), np.array([170,70,60]) ) ]
    
# creates color segmentation of the workspace.
def getShapes( image, h ):
    masks = []
    for (c, l ,u) in colors:
        mask = cv2.inRange( h, np.array(l), np.array(u) )
        masks.append(mask)
        
    # gets first shape from image    
    shapes = cv2.bitwise_and( image, image, masks[0] )
    # gets resulting shapes and or with current shape.
    for i in range(1 , len(masks) ):
        sh = cv2.bitwise_and( image, image, masks[i] )
        shapes = cv2.bitwise_or( shapes, shapes, sh )
    return shapes


lower_thresh = 100
# gets the centroid from segmentation
def getCentroids( shapes, g ):
    ret, thresh = cv2.threshold( g, lower_thresh, 220, 0)
    contours,hierarchy = cv2.findContours(thresh, 1, 2)

    M = [cv2.moments(contours[i]) for i in range(0, len(contours))]
    cx = [ ( int( m['m10']/m['m00'] ) ) for m in M]
    cy = [ ( int( m['m01']/m['m00'] ) ) for m in M]
    cen = list( zip(cx,cy) )
    return cen

# processes all centroids for publishing
def xy_from_centroid( centroid_points ):
    result = list( map( camera_transfer, centroid_points ) )
    return result

# converts a centroid point to a camera function
def camera_transfer( centroid_point ):
    pass

if __name__ == '__main__':
    try:
        Centroid()
    except rospy.ROSInterruptException:
        pass