execute_process(COMMAND "/home/zkhappyfol/catkin_ws/build/my_first_robot/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/zkhappyfol/catkin_ws/build/my_first_robot/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
