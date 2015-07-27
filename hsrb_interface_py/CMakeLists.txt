cmake_minimum_required(VERSION 2.8.3)
project(hsrb_interface_py)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS)

install(PROGRAMS scripts/hsrb_operator.py DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(test)
endif()

