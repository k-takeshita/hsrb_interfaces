#!/usr/bin/env python
# -*- coding: utf-8 -*-

from hsrb_interface import Robot

'''
Usage: $ ipython -i hsrb_operatory.py
'''

if __name__ == '__main__':
    robot = Robot()
    whole_body = robot.get('whole_body', robot.Items.JOINT_GROUP)
    omni_base = robot.get('omni_base', robot.Items.MOBILE_BASE)
    head_r_stereo_camera = robot.get('head_r_stereo_camera',
                                     robot.Items.CAMERA)
    head_l_stereo_camera = robot.get('head_l_stereo_camera',
                                     robot.Items.CAMERA)
    head_rgbd_sensor_depth = robot.get('head_rgbd_sensor_depth',
                                       robot.Items.CAMERA)
    head_rgbd_sensor_rgb = robot.get('head_rgbd_sensor_rgb',
                                     robot.Items.CAMERA)
    collision_world = robot.get('default', robot.Items.COLLISION_WORLD)
    suction = robot.get('suction', robot.Items.END_EFFECTOR)
    gripper = robot.get('gripper', robot.Items.END_EFFECTOR)
    wrist_wrench = robot.get('wrist_wrench', robot.Items.FORCE_TORQUE)
    base_imu = robot.get('base_imu', robot.Items.IMU)
    base_scan = robot.get('base_scan', robot.Items.LIDAR)
    marker = robot.get('marker', robot.Items.OBJECT_DETECTION)
#   battery = robot.get('battery', robot.Items.BATTERY)
    tts = robot.get('default', robot.Items.TEXT_TO_SPEECH)
