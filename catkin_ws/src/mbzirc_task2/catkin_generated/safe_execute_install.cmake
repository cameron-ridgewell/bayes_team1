execute_process(COMMAND "/home/cam/Documents/catkin_ws/src/mbzirc_task2/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/cam/Documents/catkin_ws/src/mbzirc_task2/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
