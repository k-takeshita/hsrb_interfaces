cmake_minimum_required(VERSION 2.8.3)
project(hsrb_interface_py)

find_package(
    catkin REQUIRED
    COMPONENTS
        roscpp
        actionlib
        tf
        tf2
        tf_conversions
        eigen_conversions
        move_base_msgs
        sensor_msgs
        geometry_msgs
        trajectory_msgs
        tmc_manipulation_msgs
)
find_package(cmake_modules REQUIRED)

find_package(Boost REQUIRED COMPONENTS python)

find_package(Eigen REQUIRED)

find_package(PythonLibs REQUIRED)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hsrb_python
    CATKIN_DEPENDS
        roscpp
        move_base_msgs
        actionlib
        sensor_msgs
        geometry_msgs
        trajectory_msgs
        tmc_manipulation_msgs
#  DEPENDS system_lib
)



add_definitions(${EIGEN_DEFINITIONS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${Boost_INCLUDE_DIR})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

#add_library(_${PROJECT_NAME} SHARED src/${PROJECT_NAME}/extentions/module.cpp src/${PROJECT_NAME}/extensions/behaviors.cpp)
#target_link_libraries(_${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
#set_target_properties(_${PROJECT_NAME} PROPERTIES
#    OUTPUT_NAME _${PROJECT_NAME}
#    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}
#    PREFIX "")

#install(TARGETS _${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
