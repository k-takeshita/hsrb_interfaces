cmake_minimum_required(VERSION 2.8.3)
project(hsrb_interface_py)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS)

if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(test)
endif()
