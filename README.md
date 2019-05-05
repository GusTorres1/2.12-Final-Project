# 2.12 Updates

#5/5/2019
1. We finished the ODrive.
2. We need the limit switches for zero-setpoint and do inverse kinematics.
3. Make the ROS Node for the ODrive.
4. Integration with the manipulator.
5. Finish communication with the Mobile Robot.
6. Finish the Computer Vision stack.
7. Integrate the dough.

# 5/1/2019 : Gustavo
The segmentation HSV values have been found empirically.
The Camera has been implemented on the frame.
Only the camera coordinates need to be implemented.

# 4/26/2019 : Gustavo
CameraNode.py is the ROS Node handling centroid detection.
The transfer from camera coordinates to xy coordinates need to be implemented and found.
The colors for segmentation need to be found.
Once these values are found, the system should be able to run.

# Old
Will include Lab 5 code and adapt it to a better control scheme.