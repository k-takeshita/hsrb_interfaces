cmake_minimum_required(VERSION 2.8.3)
project(hsrb_interface_py)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS)

install(PROGRAMS scripts/hsrb_operator.py DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(PROGRAMS scripts/ihsrb DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    catkin_add_nosetests(test)
    add_rostest(test/simtest_gripper.test)
    add_rostest(test/simtest_omni_base.test)
    add_rostest(test/simtest_whole_body.test)
    add_rostest(test/simtest_collision.test)
    add_rostest(test/simtest_marker.test)
    add_rostest(test/simtest_linemod.test)
    add_rostest(test/simtest_fetch.test)
endif()

