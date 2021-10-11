execute_process(COMMAND "/home/robot5/tyler_scratch/build/shifty_common/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/robot5/tyler_scratch/build/shifty_common/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
