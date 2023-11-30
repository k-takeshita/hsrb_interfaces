#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (C) 2016 Toyota Motor Corporation

import rclpy
from rclpy.node import Node
from hsrb_interface import Robot

# メイン
def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    whole_body = robot.get('whole_body', robot,robot.Items.JOINT_GROUP)
    omni_base = robot.get('omni_base', robot,robot.Items.MOBILE_BASE)
    head_r_stereo_camera = robot.get('head_r_stereo_camera',
                                     robot,
                                     robot.Items.CAMERA)
    head_l_stereo_camera = robot.get('head_l_stereo_camera',
                                     robot,
                                     robot.Items.CAMERA)
    head_rgbd_sensor_depth = robot.get('head_rgbd_sensor_depth',
                                       robot,
                                       robot.Items.CAMERA)
    head_rgbd_sensor_rgb = robot.get('head_rgbd_sensor_rgb',
                                       robot,
                                     robot.Items.CAMERA)
    base_imu = robot.get('base_imu', robot,  robot.Items.IMU)
    base_scan = robot.get('base_scan', robot, robot.Items.LIDAR)
    gripper = robot.get('gripper', robot, robot.Items.END_EFFECTOR)

    """
    collision_world = robot.get('default', robot, robot.Items.COLLISION_WORLD)
    suction = robot.get('suction', robot, robot.Items.END_EFFECTOR)
    wrist_wrench = robot.get('wrist_wrench', robot, robot.Items.FORCE_TORQUE)
    marker = robot.get('marker', robot, robot.Items.OBJECT_DETECTION)
    """
#   battery = robot.get('battery', robot.Items.BATTERY)
    # debug-test voice tts = robot.get('default', robot, robot.Items.TEXT_TO_SPEECH)
if __name__ == '__main__':
    main()

