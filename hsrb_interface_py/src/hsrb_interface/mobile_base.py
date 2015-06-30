#!/usr/bin/env python
# vim: fileencoding=utf-8

import rospy

import tf
import actionlib

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


from .utils import CachingSubscriber
from .robot import Resource
from .settings import get_setting, get_frame

_ACTION_TIMEOUT = 30.0

class MobileBase(Resource):
    def __init__(self, name):
        super(MobileBase, self).__init__()
        self._setting = get_setting('mobile_base', name)

        self._pose_sub = CachingSubscriber(self._settings['pose_topic'], PoseStamped)

        self._action_client = actionlib.SimpleActionClient(self._settings['move_base_action'], MoveBaseAction)

    def go(self, x, y, yaw, timeout=0.0, ref_frame_id=None):
        u"""指定した姿勢まで移動する

        Args:
            x   (float): X軸座標[m]
            y   (float): Y軸座標[m]
            yaw (float): ヨー軸座標[rad]
            timeout (float): 移動のタイムアウト[sec]（省略時は0となり、無期限に待つ）
            ref_frame_id (str): ゴールの基準座標系名(省略時はマップ座標系）

        Returns:
            None
        """
        if ref_frame_id is None:
            ref_frame_id = get_frame('map')

        target_pose = PoseStamped()
        target_pose.header.frame_id = ref_frame_id
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]
        goal = MoveBaseGoal()
        goal.target_pose = target_pose
        self._action_client.send_goal(goal)

        self._action_client.wait_for_result(rospy.Duration(timeout))
        if self._action_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            raise MoveBaseError('Failed to reach goal')

    @property
    def pose(self):
        u"""地図座標系での自己位置推定値

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
        yaw = tf.transformations.euler_from_quaternion(q)[2]
        return [x, y, yaw]

