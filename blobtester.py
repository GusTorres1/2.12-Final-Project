import numpy as np
import cv2

cap=cv2.VideoCapture(0)

while True:
    ret,frame=cap.read()

    hsv=frame

    # Set up the detector with default parameters.
    detector = cv2.SimpleBlobDetector_create()

    # Detect blobs.
    keypoints = detector.detect(hsv)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 0, 255),
                                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Show keypoints
    cv2.imshow("Keypoints", im_with_keypoints)

    if cv2.waitKey(10) & 0xFF == ord('b'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()