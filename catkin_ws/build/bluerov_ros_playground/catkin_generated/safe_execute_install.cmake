execute_process(COMMAND "/home/tihan/catkin_ws/build/bluerov_ros_playground/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/tihan/catkin_ws/build/bluerov_ros_playground/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
