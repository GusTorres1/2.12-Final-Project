execute_process(COMMAND "/home/robot/Final/2.12-Final-Project/build/odrive_ros/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/robot/Final/2.12-Final-Project/build/odrive_ros/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
