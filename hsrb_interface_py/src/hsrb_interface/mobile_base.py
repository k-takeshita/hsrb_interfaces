#!/usr/bin/env python
# vim: fileencoding=utf-8

import rospy
import tf
import actionlib

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


from . import utils
from . import robot
from . import settings
from . import exceptions
from . import geometry

# Action接続タイムアウト[sec]
_ACTION_TIMEOUT = 30.0

# tf受信待ちタイムアウト[sec]
_TF_TIMEOUT = 1.0

class MobileBase(robot.Resource):
    u"""移動台車を制御するクラス

    Example:

        ::
            with hsrb_interface.Robot() as robot:
                omni_base = robot.get('omni_base')
                omni_base.go(1, 2, math.pi / 2.0)
                print(omni_base.pose)

    """

    def __init__(self, name):
        super(MobileBase, self).__init__()
        self._setting = settings.get_entry('mobile_base', name)

        self._tf2_buffer = robot._get_tf2_buffer()

        action_name =self._setting['move_base_action']
        self._action_client = actionlib.SimpleActionClient(action_name, MoveBaseAction)


    def move(self, pose, timeout=0.0, ref_frame_id=None):
        u"""
        Args:
            pose (Tuple[Vector3[m], Quaternion]): ``ref_frame_id`` から見た目標姿勢
            timeout (float): 移動のタイムアウト[sec]（省略時は0となり、無期限に待つ）
            ref_frame_id (str): ゴールの基準座標系名(省略時は ``map`` 座標系）

        Returns:
            None

        Example:
            Usage::

                with Robot() as robot:
                    base = robot.get('omni_base', robot.Items.MOBILE_BASE)
                    pose = (Vector3(0.1, 0.2, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)
                    base.move(pose, 10.0, 'base_footprint')
        """
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('map')

        target_pose = PoseStamped()
        target_pose.header.frame_id = ref_frame_id
        target_pose.header.stamp = rospy.Time(0)
        target_pose.pose = geometry.tuples_to_pose(pose)
        goal = MoveBaseGoal()
        goal.target_pose = target_pose
        self._action_client.send_goal(goal)

        self._action_client.wait_for_result(rospy.Duration(timeout))
        if self._action_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            raise exceptions.MobileBaseError('Failed to reach goal')

    @property
    def pose(self):
        u"""``map`` 座標系での自己位置推定値

        Returns:
            List[float]: (x[m], y[m], yaw[rad])
        """
        pose = self._pose_sub.data
        x = pose.pose.position.x
        y = pose.pose.position.y
        q = [ pose.pose.orientation.x,
              pose.pose.orientation.y,
              pose.pose.orientation.z,
              pose.pose.orientation.w ]
        euler_angles = tf.transformations.euler_from_quaternion(q)
        yaw = euler_angles[2]
        return [x, y, yaw]

    def go(self, x, y, yaw, timeout=0.0, relative=False):
        u"""指定した座標まで移動する

        Args:
            x   (float): X軸座標[m]
            y   (float): Y軸座標[m]
            yaw (float): ヨー軸座標[rad]
            timeout (float): 移動のタイムアウト[sec]（省略時は0となり、無期限に待つ）
            relative (bool): ``True`` ならロボット基準座標系で移動する。``False`` なら ``map`` 座標系。

        Returns:
            None

        Example:
            Usage::

                with Robot() as robot:
                    base = robot.get('omni_base', robot.Items.MOBILE_BASE)
                    base.go(0.1, 0.2, 0.0, 10.0)
        """
        position = geometry.Vector3(x, y, 0)
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        orientation = geometry.Quaternion(*q)
        pose = (position, orientation)

        if relative:
            ref_frame_id = settings.get_frame('base')
        else:
            ref_frame_id = settings.get_frame('map')
        self.move(pose, timeout, ref_frame_id)


    @property
    def pose(self):
        u"""地図座標系での自己位置推定値

        Returns:
            List[float]: (x[m], y[m], yaw[rad])
        """
	pos, ori = self.get_pose()
        q = [ ori.x, ori.y, ori.z, ori.w ]
        euler_angles = tf.transformations.euler_from_quaternion(q)
        yaw = euler_angles[2]
        return [pos.x, pos.y, yaw]

    def get_pose(self, ref_frame_id=None):
        u"""自己位置推定値を取得する

        Args:
             ref_frame_id (str): 返り値となる姿勢の基準座標系（省略時は地図座標系）
        Returns:
             (Vector3, Quaternion): ``ref_frame_id``から見た``base_footprint``座標系の姿勢
        """
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('map')

        transform = self._tf2_buffer.lookup_transform(ref_frame_id,
                                                      settings.get_frame('base'),
                                                      rospy.Time(0),
                                                      rospy.Duration(_TF_TIMEOUT))
        return geometry.transform_to_tuples(transform.transform)
