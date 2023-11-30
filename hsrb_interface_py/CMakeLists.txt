cmake_minimum_required(VERSION 3.5)
project(hsrb_interface_py)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(hsr_kinematics REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(pybind11 REQUIRED)


find_package(trajectory_msgs REQUIRED)
find_package(tmc_vision_msgs REQUIRED)
find_package(tmc_planning_msgs REQUIRED)
find_package(tmc_manipulation_msgs REQUIRED)
find_package(tmc_control_msgs REQUIRED)
find_package(tmc_task_msgs REQUIRED)
find_package(tmc_manipulation_types REQUIRED)
find_package(tmc_robot_kinematics_model REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
)

set(_package_namespace hsrb_interface)

add_library(${PROJECT_NAME}_extension SHARED ${_package_namespace}/ext/kinematics.cpp)

target_include_directories(${PROJECT_NAME}_extension PUBLIC include)
target_link_libraries(${PROJECT_NAME}_extension ${rclcpp_LIBRARIES} ${Boost_LIBRARIES})

set_target_properties(
    ${PROJECT_NAME}_extension PROPERTIES
    OUTPUT_NAME _extension
    LIBRARY_OUTPUT_DIRECTORY lib/${_package_namespace}
    PREFIX ""
)

ament_target_dependencies(${PROJECT_NAME}_extension
  hsr_kinematics
  cv_bridge
  Boost
  sensor_msgs
  control_msgs
  geometry_msgs
  trajectory_msgs
  tmc_vision_msgs
  tmc_planning_msgs
  tmc_manipulation_msgs
  tmc_control_msgs
  tmc_task_msgs
  tmc_manipulation_types
  tmc_robot_kinematics_model
  rclcpp
  rclpy
  Eigen3
)

ament_python_install_package(hsrb_interface)


# Install python scripts


install(PROGRAMS
  scripts/hsrb_operator.py
  scripts/ihsrb.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_extension 
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)


    ament_add_pytest_test(robot_auto_init_test test/robot-auto-init-test.py)
    ament_add_pytest_test(robot_duplicate_init test/robot-duplicate-init-test.py)
    ament_add_pytest_test(simtest_gripper test/simtest_gripper.py)
    ament_add_pytest_test(simtest_omni_base test/omni_base-test.py)
    ament_add_pytest_test(simtest_whole_body test/simtest_whole_body.py)
    #   以下は未テスト(物体認識のパッケージが必要)
    #    add_rostest(test/simtest_collision.test)
    #    add_rostest(test/simtest_marker.test)
    #    add_rostest(test/simtest_linemod.test)
    #    add_rostest(test/simtest_fetch.test)

endif()


ament_export_include_directories(
  include 
)
ament_export_libraries(${PROJECT_NAME}_extension)
ament_export_dependencies(tmc_robot_kinematics_model pybind11)
ament_package()


