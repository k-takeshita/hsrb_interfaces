# + "/hsrb/omni_base_controllere!/usr/bin/env python
# vim: fileencoding=utf-8

import copy
from itertools import repeat

import numpy

import tf
import tf2_ros
import actionlib
import rospy

from geometry_msgs.msg import (
    Pose,
    Transform,
)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult,
)

from tmc_manipulation_msgs.msg import (
    BaseMovementType,
    ArmManipulationErrorCodes,
)
from tmc_manipulation_msgs.srv import (
    FilterJointTrajectoryWithConstraints,
    FilterJointTrajectoryWithConstraintsRequest,
)

from tmc_planning_msgs.msg import (
    JointPosition,
)
from tmc_planning_msgs.srv import (
    PlanWithJointGoals,
    PlanWithJointGoalsRequest,
    PlanWithHandGoals,
    PlanWithHandGoalsRequest,
)

from .utils import (
    CachingSubscriber,
    iterate,
    shortest_angular_distance,
)

from .settings import settings


class TrajectoryLengthError(Exception):
    pass


class TrajectoryFilterError(Exception):
    pass


class FollowTrajectoryError(Exception):
    pass


class PlannerError(Exception):
    pass

# subscriberのバッファ数
_DEFAULT_SUB_BUFFER = 1

# publisherのバッファ数
_DEFAULT_PUB_BUFFER = 10

# 軌道再生時等にafter_publish_callbackを呼ぶ周期[hz]
_CALLBACK_RATE = 50.0

#  wrenchの平均値を取る際のwrench取得周期[hz]
_GET_WRENCH_RATE = 10.0

#  trajectory_filterのタイムアウト[sec]
_TRAJECTORY_FILTER_TIMEOUT = 30

# アームプランニングのタイムアウト[sec]
_PLANNING_ARM_TIMEOUT = 10.0

# プランニングの最大繰り返し回数
_PLANNING_MAX_ITERATION = 10000

# planning中のgoalの発生確率
_PLANNING_GOAL_GENERATION = 0.3

# planningのゴールを決める際の分散
_PLANNING_GOAL_DEVIATION = 0.3

# 台車移動のタイムアウト[sec]
_MOVE_BASE_TIMEOUT = 60.0

# トピック受信待ちタイムアウト[sec]
_WAIT_TOPIC_TIMEOUT = 20.0

# tf受信待ちタイムアウト[sec]
_TF_TIMEOUT = 1.0

# Action接続待ちタイムアウト[sec]
_ACTION_WAIT_TIMEOUT = 30.0

# baseのmanipulation利用時の基準frame
_BASE_TRAJECTORY_ORIGIN = "odom"

# actionの接続タイムアウト[sec]
_TIMEOUT_ACTION = 600.0

# trajectoryタイムアウト[sec]
_TRAJECTORY_TIMEOUT = 60.0

_TRAJECTORY_FILTER_TIMEOUT = 30.0

# handのOpenにかかる時間[sec]
_TIME_OPEN_HAND = 10.0

def tuples_to_pose(tuples):
    trans, rot = tuples
    pose = Pose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = rot[0]
    pose.orientation.y = rot[1]
    pose.orientation.z = rot[2]
    pose.orientation.w = rot[3]
    return pose

def pose_to_tuples(pose):
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    qx = pose.orientation.x
    qy = pose.orientation.y
    qz = pose.orientation.z
    qw = pose.orientation.w
    return ((x, y, z), (qx, qy, qz, qw))

def tuples_to_transform(tuples):
    trans, rot = tuples
    transform = Transform()
    transform.translation.x = trans[0]
    transform.translation.y = trans[1]
    transform.translation.z = trans[2]
    transform.rotation.x = rot[0]
    transform.rotation.y = rot[1]
    transform.rotation.z = rot[2]
    transform.rotation.w = rot[3]
    return transform

def transform_to_tuples(transform):
    x = transform.translation.x
    y = transform.translation.y
    z = transform.translation.z
    qx = transform.rotation.x
    qy = transform.rotation.y
    qz = transform.rotation.z
    qw = transform.rotation.w
    return ((x, y, z), (qx, qy, qz, qw))

def multiply_transforms(t1, t2):
    trans1, rot1 = t1
    trans1_mat = tf.transformations.translation_matrix(trans1)
    rot1_mat = tf.transformations.quaternion_matrix(rot1)
    mat1 = numpy.dot(trans1_mat, rot1_mat)

    trans2, rot2 = t2
    trans2_mat = tf.transformations.translation_matrix(trans2)
    rot2_mat = tf.transformations.quaternion_matrix(rot2)
    mat2 = numpy.dot(trans2_mat, rot2_mat)

    mat3 = numpy.dot(mat1, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)

    return (trans3, rot3)



def extract_trajectory(joint_trajectory, joint_names, joint_state):
    """関節軌道から指定した関節の軌道のみを抜き出し、残りを現在値で埋める

    Args:
        joint_trajectory (trajectory_msgs.msg.JointTrajector): 処理対象のJointTrajectory
        joint_names (List[str]):
        joint_state (sensor_msgs.msg.JointState):

    Returns:
        trajectory_msgs.msg.JointTrajectory: 部分的な関節軌道
    """
    num_points = len(joint_trajectory.points)
    num_joints = len(joint_names)
    index_map = list(repeat(0, num_joints))
    for joint_index in range(num_joints):
        index_map[joint_index] = -1
        for input_joint_index in range(len(joint_trajectory.joint_names)):
            if joint_names[joint_index] == joint_trajectory.joint_names[input_joint_index]:
                index_map[joint_index] = input_joint_index
    trajectory_out = JointTrajectory()
    trajectory_out.joint_names = joint_names
    trajectory_out.points = list(iterate(JointTrajectoryPoint, num_points))
    for point_index in range(num_points):
        target = trajectory_out.points[point_index]
        source = joint_trajectory.points[point_index]
        target.positions = list(repeat(0, num_joints))
        target.velocities = list(repeat(0, num_joints))
        target.accelerations= list(repeat(0, num_joints))
        target.time_from_start = source.time_from_start
        has_velocities = len(source.velocities)
        has_accelerations = len(source.accelerations)
        for joint_index in range(num_joints):
            if index_map[joint_index] != -1:
                target.positions[joint_index] = source.positions[index_map[joint_index]]
                if has_velocities:
                    target.velocities[joint_index] = source.velocities[index_map[joint_index]]
                if has_accelerations:
                    target.accelerations[joint_index] = source.accelerations[index_map[joint_index]]
            else:
                i = joint_state.name.index(joint_names[joint_index])
                angle = joint_state.position[i]
                target.positions[joint_index] = angle
                target.velocities[joint_index] = angle
                target.accelerations[joint_index] = angle
    return trajectory_out


def merge_trajectory(original_trajectory, additional_trajectory):
    """互いにpoints数が同じな別の関節のsrc_trajectory1とsrc_trajectory2をマージして一つのtrajectoryにする。

    pointsのサイズが異なると失敗となる。
    time_from_startはoriginalに合わせられる。

    Args:
        original_trajectory (trajectory_msgs.msg.JointTrajectory): original入力軌道
        additional_trajectory (trajectory_msgs.msg.JointTrajectory): additional入力軌道
    Returns:
        trajectory_msgs.msg.JointTrajectory: 出力軌道
    Raises:
        ValueError:
    """
    if len(original_trajectory.points) != len(additional_trajectory.points):
        raise ValueError
    merged = copy.deepcopy(original_trajectory)
    merged.joint_names = list(merged.joint_names)
    merged.joint_names.extend(additional_trajectory.joint_names)

    num_points = len(merged.points)
    for point_index in range(num_points):
        merged.points[point_index].positions = list(merged.points[point_index].positions)
        merged.points[point_index].positions.extend(additional_trajectory.points[point_index].positions)
        merged.points[point_index].velocities = list(merged.points[point_index].velocities)
        merged.points[point_index].velocities.extend(additional_trajectory.points[point_index].velocities)
        merged.points[point_index].accelerations = list(merged.points[point_index].accelerations)
        merged.points[point_index].accelerations.extend(additional_trajectory.points[point_index].accelerations)
    return merged


class FollowTrajectoryActionClient(object):
    u"""関節軌道追従アクションのクライアント処理ラッパー

    Args:
        controller_name (str):    接続するROSコントローラの名前
        joint_names_suffix (str): コントローラ名前空間内の、制御対象関節名パラメータの名前

    """
    def __init__(self, controller_name, joint_names_suffix="/joints"):
        self._controller_name = controller_name
        self._client = actionlib.SimpleActionClient(controller_name + "/follow_joint_trajectory", FollowJointTrajectoryAction)
        self._client.wait_for_server(rospy.Duration(_ACTION_WAIT_TIMEOUT))
        self._joint_names = rospy.get_param("{0}/{1}".format(self._controller_name, joint_names_suffix), None)

    def send_goal(self, trajectory):
        u"""ゴールをコントローラに送る"""
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        self._client.send_goal(goal)

    def cancel_goal(self):
        u"""現在のゴールをキャンセル"""
        self._client.cancel_goal()

    def get_state(self):
        return self._client.get_state()

    def get_results(self, timeout=None):
        """
        Returns:
            FollowJointTrajectoryResult
        """
        if self._client.get_result() is None:
            return None

        state = self._client.get_state()
        result = self._client.get_result()

        if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            raise FollowTrajectoryError("{0}".format(result.error_code))
        if state != actionlib.GoalStatus.SUCCEEDED:
            raise FollowTrajectoryError("{0}".format(state))
        return result

    @property
    def joint_names(self):
        return self._joint_names

    @property
    def controller_name(self):
        return self._controller_name


class JointGroup(object):
    """制御用
    """
    def __init__(self):
        self._arm_client = FollowTrajectoryActionClient(settings['arm_controller_prefix'])
        self._head_client = FollowTrajectoryActionClient(settings['head_controller_prefix'])
        self._hand_client = FollowTrajectoryActionClient(settings["hand_controller_prefix"])
        self._base_client = FollowTrajectoryActionClient(settings["omni_base_controller_prefix"], "/base_coordinates")
        self._joint_state_sub = CachingSubscriber(settings["joint_states_topic"], JointState)
        self._tf2_buffer = tf2_ros.Buffer()
        self._tf2_listener = tf2_ros.TransformListener(self._tf2_buffer)

    def _get_joint_state(self):
        u"""
        Returns:
            sensor_msgs.JointState: Current joint state
        """
        return self._joint_state_sub.data

    @property
    def joint_names(self):
        self.get_joint_state().name

    @property
    def joint_positions(self):
        joint_state = self._get_joint_state()
        return dict(zip(joint_state.name, joint_state.position))

    @property
    def joint_velocities(self):
        joint_state = self._get_joint_state()
        return dict(zip(joint_state.name, joint_state.position))

    @property
    def joint_state(self):
        return self._behavior._get_joint_state()

    @property
    def collision_environment(self):
        return self._collision_env

    @collision_environment.setter
    def collision_environment(self, value):
        self._collision_env = value

    def clear_collision_environment(self):
        self._collision_env = None

    def change_joint_state(self, goal_state):
        u"""外部干渉を考慮しない指定関節角度までの遷移

        Args:
            goal_state (sensor_msgs.msg.JointState): 目標関節状態

        Returns:
            None
        """
        origin_to_basejoint = Pose()
        origin_to_basejoint.orientation.w = 1.0
        basejoint_to_base = Pose()
        basejoint_to_base.orientation.w = 1.0

        req = PlanWithJointGoalsRequest()
        goal_position = JointPosition()
        goal_position.position = goal_state.position

        req.origin_to_basejoint = origin_to_basejoint
        req.initial_joint_state = self._get_joint_state()
        req.use_joints = goal_state.name
        req.goal_joint_states.append(goal_position)
        req.goal_basejoint_to_bases.append(basejoint_to_base)
        req.timeout = rospy.Duration(_PLANNING_ARM_TIMEOUT)
        req.max_iteration = _PLANNING_MAX_ITERATION
        req.base_movement_type.val = BaseMovementType.NONE

        plan_service = rospy.ServiceProxy(settings['plan_with_joint_goals_service'], PlanWithJointGoals)
        res = plan_service.call(req)
        if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
            raise PlannerError("Fail to plan change_joint_state: {0}".format(res.error_code.val))
        self.play_trajectory(res.solution)

    def move_to_joint_positions(self, goals):
        u"""外部干渉を考慮せず指定の姿勢に遷移する

        Args:
            positions (Dict[str, float]): 関節名と目標位置の組による辞書

        Returns:
            None
        """
        goal_state = JointState()
        for k, v in goals.items():
            goal_state.name.append(k)
            goal_state.position.append(v)
        self.change_joint_state(goal_state)

    def move_to_neutral(self):
        u"""外部干渉を考慮せず基準姿勢に遷移する

        Returns:
            None
        """
        goals = {
            'arm_lift_joint': 0.0,
            'arm_flex_joint': 0.0,
            'arm_roll_joint': 0.0,
            'wrist_flex_joint': 0.0,
            'wrist_roll_joint': 0.0,
            'head_pan_joint': 0.0,
            'head_tilt_joint': 0.0,
        }
        self.move_to_joint_positions(goals)


    def get_endeffector_pose(self, ref_frame_id=None):
        u"""現在のオドメトリ基準のエンドエフェクタの姿勢を返す

        Returns:
            タプル((x, y, z), (qx, qy, qz, qw))
        """
        # 基準座標省略時はロボット座標系
        if ref_frame_id is None:
            ref_frame_id = settings['base_frame_id']
        transform = self._tf2_buffer.lookup_transform(ref_frame_id,
                                                      settings['hand_frame_id'],
                                                      rospy.Time(0),
                                                      rospy.Duration(_TF_TIMEOUT))
        return transform_to_tuples(transform.transform)

    def move_endeffector(self, hand_pose, ref_frame_id=None):
        u"""外部干渉を考慮せずグリッパを指定姿勢まで動かす

        Args
            hand_pose (Tuple[float,float,float,float,float,float,float]):
            ref_frame_id (): 手先の基準座標

        Returns:
            None
        """

        use_joints = (
            'wrist_flex_joint',
            'wrist_roll_joint',
            'arm_roll_joint',
            'arm_flex_joint',
            'arm_lift_joint'
        )

        # 基準座標省略時はロボット座標系
        if ref_frame_id is None:
            ref_frame_id = settings['base_frame_id']

        odom_to_robot_transform = self._tf2_buffer.lookup_transform(settings['odom_frame_id'],
                                                                    settings['base_frame_id'],
                                                                    rospy.Time(0),
                                                                    rospy.Duration(_TF_TIMEOUT))
        odom_to_robot_pose = tuples_to_pose(transform_to_tuples(odom_to_robot_transform.transform))

        odom_to_ref_transform = self._tf2_buffer.lookup_transform(settings['odom_frame_id'],
                                                                  ref_frame_id,
                                                                  rospy.Time(0),
                                                                  rospy.Duration(_TF_TIMEOUT))
        odom_to_ref = transform_to_tuples(odom_to_ref_transform.transform)
        odom_to_hand = multiply_transforms(odom_to_ref, hand_pose)
        odom_to_hand_pose = tuples_to_pose(odom_to_hand)

        req = PlanWithHandGoalsRequest()
        req.base_movement_type.val = BaseMovementType.PLANAR
        req.origin_to_basejoint = odom_to_robot_pose
        req.initial_joint_state = self._get_joint_state()
        req.use_joints = use_joints
        req.origin_to_hand_goals.append(odom_to_hand_pose)
        req.ref_frame_id = settings['hand_frame_id']
        #req.environment_before_planning = collision_env
        req.probability_goal_generate = _PLANNING_GOAL_GENERATION
        req.timeout = rospy.Duration(_PLANNING_ARM_TIMEOUT)
        req.max_iteration = _PLANNING_MAX_ITERATION
        req.uniform_bound_sampling = False
        req.deviation_for_bound_sampling = _PLANNING_GOAL_DEVIATION

        plan_service = rospy.ServiceProxy(settings['plan_with_hand_goals_service'], PlanWithHandGoals)
        res = plan_service.call(req)
        if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
            raise PlannerError("Fail to plan move_endpoint: {0}".format(res.error_code.val))
        res.base_solution.header.frame_id = settings['odom_frame_id']
        self.play_trajectory(res.solution, res.base_solution)


    def play_trajectory(self, joint_trajectory, base_trajectory=None):
        u"""指定の軌道を再生する

        Args:
            joint_trajectory (trajectory_msgs.msg.JointTrajectory): 関節軌道
            base_trajectory (tmc_manipulation_msgs.msg.MultiDOFJointTrajectory): 台車の軌道
        Returns:
            None
        """
        if base_trajectory is None:
            clients = [self._head_client, self._arm_client]
            return self._execute_trajectory(clients, joint_trajectory)
        else:
            clients = [self._head_client, self._arm_client, self._base_client]
            if len(joint_trajectory.points) != len(base_trajectory.points):
                raise TrajectoryLengthError("Uneven joint_trajectory size and base_trajectory size.")

            odom_to_frame_transform = self._tf2_buffer.lookup_transform(_BASE_TRAJECTORY_ORIGIN,
                                                                        base_trajectory.header.frame_id,
                                                                        rospy.Time(0),
                                                                        rospy.Duration(_TF_TIMEOUT))
            odom_to_frame = transform_to_tuples(odom_to_frame_transform.transform)

            num_points = len(joint_trajectory.points)
            odom_base_trajectory = JointTrajectory()
            odom_base_trajectory.points = list(iterate(JointTrajectoryPoint, num_points))
            odom_base_trajectory.header = base_trajectory.header
            odom_base_trajectory.joint_names = self._base_client.joint_names

            # 各ポイントをodom座標系に変換する
            previous_theta = 0.0
            for i in range(num_points):
                t = base_trajectory.points[i].transforms[0]
                frame_to_base = transform_to_tuples(t)

                # odom_to_base = odom_to_frame * frame_to_base
                (odom_to_base_trans, odom_to_base_rot) = multiply_transforms(odom_to_frame, frame_to_base)

                odom_base_trajectory.points[i].positions = [0, 0, 0]
                odom_base_trajectory.points[i].positions[0] = odom_to_base_trans[0]
                odom_base_trajectory.points[i].positions[1] = odom_to_base_trans[1]

                roll, pitch, yaw = tf.transformations.euler_from_quaternion(odom_to_base_rot)
                theta = previous_theta * shortest_angular_distance(previous_theta, yaw)

                odom_base_trajectory.points[i].positions[2] = theta
                previous_theta = theta

            # 台車と腕の軌道をマージ
            merged_trajectory = merge_trajectory(joint_trajectory, odom_base_trajectory)

            # 軌道を再生
            return self._execute_trajectory(clients, merged_trajectory)


    def _execute_trajectory(self, clients, joint_trajectory):
        u"""軌道再生を実行する
        Parameters:
            clients (List[FollowTrajectoryActionClient]):
            joint_trajectory (trajectory_msgs.msg.JointTrajectory): 再生する関節軌道
            interrupt (callable):
        Returns:
            None
        Raises:
            TrajectoryFilterError: 関節軌道フィルタが失敗した
        """
        filter_service = rospy.ServiceProxy(settings["trajectory_filter_service"],
                                            FilterJointTrajectoryWithConstraints)
        req = FilterJointTrajectoryWithConstraintsRequest()
        req.trajectory = joint_trajectory
        req.allowed_time = rospy.Duration(_TRAJECTORY_FILTER_TIMEOUT)
        try:
            res = filter_service.call(req)
            if res.error_code.val != res.error_code.SUCCESS:
                raise TrajectoryFilterError("Failed to filter trajectory: {0}".fomrat(res.error_code.val))
        except rospy.ServiceException:
            import traceback
            traceback.print_exc()
            raise
        filtered_traj = res.trajectory

        joint_states = self._get_joint_state()

        for client in clients:
            traj = extract_trajectory(filtered_traj, client.joint_names, joint_states)
            client.send_goal(traj)

        rate = rospy.Rate(0.1)
        while True:
            ok_set = (
                actionlib.GoalStatus.PENDING,
                actionlib.GoalStatus.ACTIVE,
                actionlib.GoalStatus.SUCCEEDED,
            )
            states = [c.get_state() for c in clients]
            if any(map(lambda s: s not in ok_set, states)):
                log = []
                for c in clients:
                    log.append("{0}={1}".format(c.name, c.get_state()))
                    c.cancel_goal()
                raise FollowTrajectoryError("Playing trajecotry failed: {0}".format(', '.join(log)))
            if all(map(lambda s: s == actionlib.GoalStatus.SUCCEEDED, states)):
                break
            rate.sleep()
        return (client.get_results() for client in clients)
