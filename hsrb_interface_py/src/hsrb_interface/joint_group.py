#!/usr/bin/env python
# vim: fileencoding=utf-8

import copy
from itertools import repeat

import tf
import actionlib
import rospy
import urdf_parser_py

from geometry_msgs.msg import Pose
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
    SelectConfig,
    SelectConfigRequest,
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
    PlanWithHandLine,
    PlanWithHandLineRequest,
)

from urdf_parser_py.urdf import (
    xmlr,
)

# urdf_parser_pyのモンキーパッチ
# https://github.com/k-okada/urdfdom/commit/9c9e28e9f1de29228def48a4d49315b2f4fbf2d2
# を含むコードがリリースされたら消してOK
xmlr.reflect(urdf_parser_py.urdf.JointLimit, params = [
    xmlr.Attribute('effort', float),
    xmlr.Attribute('lower', float, False, 0),
    xmlr.Attribute('upper', float, False, 0),
    xmlr.Attribute('velocity', float)
    ])

# urdf_parser_pyのモンキーパッチ2
# http://wiki.ros.org/urdf/XML/link
# を見るとvisualとcollisionは複数可能になっているのでその対応
xmlr.reflect(urdf_parser_py.urdf.Link, params = [
    xmlr.Attribute('name', str),
    xmlr.Element('origin', urdf_parser_py.urdf.Pose, False),
    xmlr.Element('inertial', urdf_parser_py.urdf.Inertial, False),
    xmlr.AggregateElement('visual', urdf_parser_py.urdf.Visual, 'visual'),
    xmlr.AggregateElement('collision', urdf_parser_py.urdf.Collision, 'collision')
])

def _get_aggregate_list(self, xml_var):
    var = self.XML_REFL.paramMap[xml_var].var
    if not getattr(self, var):
        self.aggregate_init()
        setattr(self, var, [])
    return getattr(self, var)

urdf_parser_py.urdf.Link.get_aggregate_list = _get_aggregate_list
urdf_parser_py.urdf.Collision.get_aggregate_list = _get_aggregate_list
urdf_parser_py.urdf.Material.check_valid = lambda self: None

from urdf_parser_py.urdf import Robot as RobotUrdf

from . import exceptions
from . import settings
from . import robot
from . import utils
from . import geometry
from . import collision_world


# subscriberのバッファ数
_DEFAULT_SUB_BUFFER = 1

# publisherのバッファ数
_DEFAULT_PUB_BUFFER = 10

# 軌道再生時等にafter_publish_callbackを呼ぶ周期[hz]
_CALLBACK_RATE = 50.0

#  wrenchの平均値を取る際のwrench取得周期[hz]
_GET_WRENCH_RATE = 10.0

# trajectoryの結果を取得する周期[hz]
_TRAJECTORY_RATE = 30.0

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

def _refer_planning_error(error_code):
    """プラニングのエラーコードを名前に変換する

    Args:
        error_code (ArmManipulationErrorCodes): エラーコード
    Returns:
        エラーの名前
    """
    error_codes = ArmManipulationErrorCodes.__dict__.items()
    error_names = [k for k, v in error_codes
                   if v == error_code and k.isupper()]
    if len(error_names) != 0:
        return error_names[0]
    else:
        return str(error_code)

def _extract_trajectory(joint_trajectory, joint_names, joint_state):
    """関節軌道から指定した関節の軌道のみを抜き出し、残りを現在値で埋める

    Args:
        joint_trajectory (trajectory_msgs.msg.JointTrajector):
            処理対象のJointTrajectory
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
    trajectory_out.points = list(utils.iterate(JointTrajectoryPoint, num_points))
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
                target.velocities[joint_index] = 0.0
                target.accelerations[joint_index] = 0.0
    return trajectory_out


def _merge_trajectory(original_trajectory, additional_trajectory):
    """互いにpoints数が同じな別の関節のsrc_trajectory1とsrc_trajectory2をマージして一つのtrajectoryにする。

    pointsのサイズが異なると失敗となる。
    ``time_from_start`` は ``original`` に合わせられる。

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
            raise exceptions.FollowTrajectoryError("{0}".format(result.error_code))
        if state != actionlib.GoalStatus.SUCCEEDED:
            raise exceptions.FollowTrajectoryError("{0}".format(state))
        return result

    @property
    def joint_names(self):
        return self._joint_names

    @property
    def controller_name(self):
        return self._controller_name

class ImpedanceControlActionClient(FollowTrajectoryActionClient):
    u"""インピーダンス制御のクライアント処理ラッパー

    Args:
        controller_name (str):    接続するROSコントローラの名前
        joint_names_suffix (str): コントローラ名前空間内の、制御対象関節名パラメータの名前

    """
    def __init__(self, controller_name, joint_names_suffix="/joint_names"):
        super(ImpedanceControlActionClient, self).__init__(
            self, controller_name, joint_names_suffix)
        self._config = None
        self._config_names = rospy.get_param(controller_name + "/config_names")

    @property
    def config(self):
        return self._config

    @config.setter
    def config(self, value):
        if value is not None:
            setter_service = rospy.ServiceProxy(
                self._controller_name + "/select_config", SelectConfig)
            req = SelectConfigRequest()
            req.name = value
            try:
                res = setter_service.call(req)
                if not res.is_success:
                    raise exceptions.FollowTrajectoryError(
                        "Failed to set impedance config")
            except rospy.ServiceException:
                import traceback
                traceback.print_exc()
                raise
        self._config = value

    @property
    def config_names(self):
        return self._config_names

class JointGroup(robot.Item):
    u"""関節グループの制御を行うクラス

    """
    def __init__(self, name):
        super(JointGroup, self).__init__()
        self._setting = settings.get_entry('joint_group', name)
        self._arm_client = FollowTrajectoryActionClient(
            self._setting['arm_controller_prefix'])
        self._head_client = FollowTrajectoryActionClient(
            self._setting['head_controller_prefix'])
        self._hand_client = FollowTrajectoryActionClient(
            self._setting["hand_controller_prefix"])
        self._base_client = FollowTrajectoryActionClient(
            self._setting["omni_base_controller_prefix"], "/base_coordinates")
        self._impedance_client = ImpedanceControlActionClient(
            self._setting["impedance_control"], "/joint_names")
        joint_state_topic = self._setting["joint_states_topic"]
        self._joint_state_sub = utils.CachingSubscriber(joint_state_topic,
                                                        JointState,
                                                        default=JointState())
        timeout = self._setting.get('timeout', None)
        self._joint_state_sub.wait_for_message(timeout)
        self._tf2_buffer = robot._get_tf2_buffer()

        self._robot_urdf = RobotUrdf.from_parameter_server()

        self._collision_world = None
        self._linear_weight = 3.0
        self._angular_weight = 1.0
        self._planning_timeout = _PLANNING_ARM_TIMEOUT

    def _get_joint_state(self):
        u"""
        Returns:
            sensor_msgs.JointState: Current joint state
        """
        return self._joint_state_sub.data

    @property
    def joint_names(self):
        u"""List[str]: 各関節の名称リスト"""
        return self._get_joint_state().name

    @property
    def joint_positions(self):
        joint_state = self._get_joint_state()
        return dict(zip(joint_state.name, joint_state.position))

    @property
    def joint_velocities(self):
        joint_state = self._get_joint_state()
        return dict(zip(joint_state.name, joint_state.velocity))

    @property
    def joint_state(self):
        return self._get_joint_state()

    @property
    def joint_limits(self):
        joint_map = self._robot_urdf.joint_map
        return {joint_name: (joint_map[joint_name].limit.lower, joint_map[joint_name].limit.upper)
                for joint_name in self.joint_names}

    @property
    def collision_world(self):
        u"""CollisionWorld: 現在設定されている動作計画の干渉検知用環境。
        """
        return self._collision_world

    @collision_world.setter
    def collision_world(self, value):
        u"""CollisionWorld: 動作計画の干渉検知用環境を設定する。

        Noneが設定された場合は干渉検知が無効化。
        """
        if value is None:
            self._collision_world = None
        elif isinstance(value, collision_world.CollisionWorld):
            self._collision_world = value
        else:
            raise TypeError("value should be CollisionWorld instance")

    @property
    def linear_weight(self):
        return self._linear_weight

    @linear_weight.setter
    def linear_weight(self, value):
        self._linear_weight = value

    @property
    def angular_weight(self):
        return self._angular_weight

    @angular_weight.setter
    def angular_weight(self, value):
        self._angular_weight = value

    @property
    def planning_timeout(self):
        return self._planning_timeout

    @planning_timeout.setter
    def planning_timeout(self, value):
        self._planning_timeout = value

    @property
    def impedance_config(self):
        return self._impedance_client.config

    @impedance_config.setter
    def impedance_config(self, value):
        self._impedance_client.config = value

    @property
    def impedance_config_names(self):
        return self._impedance_client.config_names

    def _change_joint_state(self, goal_state):
        u"""干渉を考慮した指定関節角度までの遷移

        Args:
            goal_state (sensor_msgs.msg.JointState): 目標関節状態
        Returns:
            None
        Raises:
            ValueError: 引数内にに指定されたジョイント名が見つからない
        """
        # Validate joint names
        initial_joint_state = self._get_joint_state()
        active_joint_set = set(initial_joint_state.name)
        target_joint_set = set(goal_state.name)
        if not target_joint_set.issubset(active_joint_set):
            unknown_set = target_joint_set.difference(active_joint_set)
            raise ValueError("No such joint(s): [{0}]".format(', '.join(unknown_set)))

        odom_to_robot_transform = self._tf2_buffer.lookup_transform(
                                    settings.get_frame('odom'),
                                    settings.get_frame('base'),
                                    rospy.Time(0),
                                    rospy.Duration(_TF_TIMEOUT))
        odom_to_robot_tuples = geometry.transform_to_tuples(odom_to_robot_transform.transform)
        odom_to_robot_pose = geometry.tuples_to_pose(odom_to_robot_tuples)

        req = PlanWithJointGoalsRequest()
        goal_position = JointPosition()
        goal_position.position = goal_state.position

        req.origin_to_basejoint = odom_to_robot_pose
        req.initial_joint_state = initial_joint_state
        req.use_joints = goal_state.name
        req.goal_joint_states.append(goal_position)
        req.timeout = rospy.Duration(self._planning_timeout)
        req.max_iteration = _PLANNING_MAX_ITERATION
        req.base_movement_type.val = BaseMovementType.ROTATION_Z
        if self._collision_world is not None:
            req.environment_before_planning = self._collision_world.snapshot('odom')

        plan_service = rospy.ServiceProxy(self._setting['plan_with_joint_goals_service'], PlanWithJointGoals)
        res = plan_service.call(req)
        if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
            error = _refer_planning_error(res.error_code.val)
            raise exceptions.PlannerError("Fail to plan change_joint_state: {0}".format(error))
        res.base_solution.header.frame_id = settings.get_frame('odom')
        self._play_trajectory(res.solution, res.base_solution)

    def move_to_joint_positions(self, goals={}, **kwargs):
        u"""指定の姿勢に遷移する。

        Args:
            positions (Dict[str, float]): 関節名と目標位置[m or rad]の組による辞書
            **kwargs: 関節名をキーワード、その目標値を引数にして指定できる
        Returns:
            None

        See Also:
            JointGroup.joint_names

        Examples:

            .. sourcecode:: python

               import math
               import hsrb_interface

               with hsrb_interface.Robot() as robot:
                   whole_body = robot.get('whole_body')
                   goals = {
                       'arm_lift_joint': 0.5,
                       'arm_flex_joint': math.radians(-90)
                   }
                   whole_body.move_to_joint_positions(goals)

                   # The method also accept keyword arguments
                   whole_body.move_to_joint_positions(head_tilt_joint=math.radians(30))

        """
        if goals is None:
            goals = {}
        goals.update(kwargs)
        if not goals:
            return
        goal_state = JointState()
        for k, v in goals.items():
            goal_state.name.append(k)
            goal_state.position.append(v)
        self._change_joint_state(goal_state)

    def move_to_neutral(self):
        u"""干渉を考慮して基準姿勢に遷移する
        Returns:
            None
        """
        goals = {
            'arm_lift_joint': 0.0,
            'arm_flex_joint': 0.0,
            'arm_roll_joint': 0.0,
            'wrist_flex_joint': -1.57,
            'wrist_roll_joint': 0.0,
            'head_pan_joint': 0.0,
            'head_tilt_joint': 0.0,
        }
        self.move_to_joint_positions(goals)

    def move_to_go(self):
        u"""干渉を考慮して移動向け基準姿勢に遷移する
        Returns:
            None
        """
        goals = {
            'arm_flex_joint': 0.0,
            'arm_lift_joint': 0.0,
            'arm_roll_joint': -1.57,
            'wrist_flex_joint': -1.57,
            'wrist_roll_joint': 0.0,
            'head_pan_joint': 0.0,
            'head_tilt_joint': 0.0
        }
        self.move_to_joint_positions(goals)


    def get_end_effector_pose(self, ref_frame_id=None):
        u"""現在のオドメトリ基準のエンドエフェクタの姿勢を返す

        Returns:
            Tuple[Vector3, Quaternion]
        """
        # 基準座標省略時はロボット座標系
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('base')
        transform = self._tf2_buffer.lookup_transform(ref_frame_id,
                                                      settings.get_frame('hand'),
                                                      rospy.Time(0),
                                                      rospy.Duration(_TF_TIMEOUT))
        result = geometry.transform_to_tuples(transform.transform)
        return result

    def move_end_effector_pose(self, pose, ref_frame_id=None):
        u"""指定姿勢まで動かす

        Args
            pose (Tuple[Vector3, Quaternion]):
            ref_frame_id (str): 手先の基準座標(デフォルトはロボット座標系)
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
            ref_frame_id = settings.get_frame('base')

        odom_to_robot_transform = self._tf2_buffer.lookup_transform(settings.get_frame('odom'),
                                                                    settings.get_frame('base'),
                                                                    rospy.Time(0),
                                                                    rospy.Duration(_TF_TIMEOUT))
        odom_to_robot_pose = geometry.tuples_to_pose(geometry.transform_to_tuples(odom_to_robot_transform.transform))

        odom_to_ref_transform = self._tf2_buffer.lookup_transform(settings.get_frame('odom'),
                                                                  ref_frame_id,
                                                                  rospy.Time(0),
                                                                  rospy.Duration(_TF_TIMEOUT))
        odom_to_ref = geometry.transform_to_tuples(odom_to_ref_transform.transform)
        odom_to_hand = geometry.multiply_tuples(odom_to_ref, pose)
        odom_to_hand_pose = geometry.tuples_to_pose(odom_to_hand)

        req = PlanWithHandGoalsRequest()
        req.base_movement_type.val = BaseMovementType.PLANAR
        req.origin_to_basejoint = odom_to_robot_pose
        req.initial_joint_state = self._get_joint_state()
        req.use_joints = use_joints
        req.origin_to_hand_goals.append(odom_to_hand_pose)
        req.ref_frame_id = settings.get_frame('hand')
        req.probability_goal_generate = _PLANNING_GOAL_GENERATION
        req.timeout = rospy.Duration(self._planning_timeout)
        req.max_iteration = _PLANNING_MAX_ITERATION
        req.uniform_bound_sampling = False
        req.deviation_for_bound_sampling = _PLANNING_GOAL_DEVIATION
        req.weighted_joints = ['_linear_base', '_rotational_base']
        req.weight = [self._linear_weight, self._angular_weight]
        if self._collision_world is not None:
            req.environment_before_planning = self._collision_world.snapshot('odom')

        plan_service = rospy.ServiceProxy(self._setting['plan_with_hand_goals_service'], PlanWithHandGoals)
        res = plan_service.call(req)
        if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
            error = _refer_planning_error(res.error_code.val)
            raise exceptions.PlannerError("Fail to plan move_endpoint: {0}".format(error))
        res.base_solution.header.frame_id = settings.get_frame('odom')
        self._play_trajectory(res.solution, res.base_solution)

    def move_end_effector_by_line(self, axis, distance, ref_frame_id=None):
        u"""3次元空間上の直線に沿って手先を動かす

        Args:
            axis (Vector3): 動かす軸方向
            distance (float): 移動量[m]
            ref_frame_id (str): 基準となる座標系
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
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('hand')
        odom_to_robot_transform = self._tf2_buffer.lookup_transform(
            settings.get_frame('odom'),
            settings.get_frame('base'),
            rospy.Time(0),
            rospy.Duration(_TF_TIMEOUT))
        odom_to_robot_pose = geometry.tuples_to_pose(
            geometry.transform_to_tuples(odom_to_robot_transform.transform))

        req = PlanWithHandLineRequest()
        req.base_movement_type.val = BaseMovementType.PLANAR
        req.origin_to_basejoint = odom_to_robot_pose
        req.initial_joint_state = self._get_joint_state()
        req.use_joints = use_joints
        req.axis.x = axis[0]
        req.axis.y = axis[1]
        req.axis.z = axis[2]
        req.local_origin_of_axis = True
        req.ref_frame_id = ref_frame_id
        req.goal_value = distance
        req.probability_goal_generate = _PLANNING_GOAL_GENERATION
        req.attached_objects = []
        req.timeout = rospy.Duration(self._planning_timeout)
        req.max_iteration = _PLANNING_MAX_ITERATION
        req.uniform_bound_sampling = False
        req.deviation_for_bound_sampling = _PLANNING_GOAL_DEVIATION
        req.extra_goal_constraints = []
        if self._collision_world is not None:
            req.environment_before_planning = self._collision_world.snapshot('odom')

        plan_service = rospy.ServiceProxy(self._setting['plan_with_hand_line_service'], PlanWithHandLine)
        res = plan_service.call(req)
        if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
            error = _refer_planning_error(res.error_code.val)
            raise exceptions.PlannerError("Fail to plan move_hand_line: {0}".format(error))
        res.base_solution.header.frame_id = settings.get_frame('odom')
        self._play_trajectory(res.solution, res.base_solution)

    def _play_trajectory(self, joint_trajectory, base_trajectory):
        u"""指定の軌道を再生する

        Args:
            joint_trajectory (trajectory_msgs.msg.JointTrajectory): 関節軌道
            base_trajectory (tmc_manipulation_msgs.msg.MultiDOFJointTrajectory): 台車の軌道
        Returns:
            None
        """
        whole_body_trajectory = self._build_whole_body_trajectory(
            joint_trajectory, base_trajectory)
        constrained_trajectory = self._constrain_trajectory(
            whole_body_trajectory)

        clients = []
        if self._impedance_client.config is not None:
            clients.append(self._impedance_client)
        else:
            clients.extend([self._arm_client, self._base_client])
        for joint in constrained_trajectory.joint_names:
            if joint in self._head_client.joint_names:
                clients.append(self._head_client)
                break
        self._execute_trajectory(clients, constrained_trajectory)


    def _build_whole_body_trajectory(self, joint_trajectory, base_trajectory):
        u"""関節軌道と台車軌道を組み合わせて全身軌道を生成する

        Args:
            joint_trajectory (trajectory_msgs.msg.JointTrajectory): 関節軌道
            base_trajectory (tmc_manipulation_msgs.msg.MultiDOFJointTrajectory): 台車の軌道
        Returns:
            trajectory_msgs.msg.JointTrajectory: 全身軌道
        """
        if len(joint_trajectory.points) != len(base_trajectory.points):
            raise exceptions.TrajectoryLengthError(
                "Uneven joint_trajectory size and base_trajectory size.")

        odom_to_frame_transform = self._tf2_buffer.lookup_transform(
            _BASE_TRAJECTORY_ORIGIN,
            base_trajectory.header.frame_id,
            rospy.Time(0),
            rospy.Duration(_TF_TIMEOUT))
        odom_to_frame = geometry.transform_to_tuples(
            odom_to_frame_transform.transform)

        num_points = len(joint_trajectory.points)
        odom_base_trajectory = JointTrajectory()
        odom_base_trajectory.points = list(
            utils.iterate(JointTrajectoryPoint, num_points))
        odom_base_trajectory.header = base_trajectory.header
        odom_base_trajectory.joint_names = self._base_client.joint_names

        # 各ポイントをodom座標系に変換する
        previous_theta = 0.0
        for i in range(num_points):
            t = base_trajectory.points[i].transforms[0]
            frame_to_base = geometry.transform_to_tuples(t)

            # odom_to_base = odom_to_frame * frame_to_base
            (odom_to_base_trans, odom_to_base_rot) =\
                geometry.multiply_tuples(odom_to_frame, frame_to_base)

            odom_base_trajectory.points[i].positions = [odom_to_base_trans[0],
                                                        odom_to_base_trans[1],
                                                        0]
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(
                odom_to_base_rot)
            theta = previous_theta +\
                    geometry.shortest_angular_distance(previous_theta, yaw)

            odom_base_trajectory.points[i].positions[2] = theta
            previous_theta = theta
        # 台車と腕の軌道をマージ
        return _merge_trajectory(joint_trajectory, odom_base_trajectory)
    
    def _constrain_trajectory(self, trajectory):
        u"""軌道に制約を課す

        Parameters:
            trajectory (trajectory_msgs.msg.JointTrajectory): 関節軌道
        Returns:
            trajectory_msgs.msg.JointTrajectory: 制約された関節軌道
        Raises:
            TrajectoryFilterError: 関節軌道フィルタが失敗した
        """
        filter_service = rospy.ServiceProxy(
            self._setting["trajectory_filter_service"],
            FilterJointTrajectoryWithConstraints)
        req = FilterJointTrajectoryWithConstraintsRequest()
        req.trajectory = trajectory
        req.allowed_time = rospy.Duration(_TRAJECTORY_FILTER_TIMEOUT)
        try:
            res = filter_service.call(req)
            if res.error_code.val != res.error_code.SUCCESS:
                raise exceptions.TrajectoryFilterError(
                    "Failed to filter trajectory: {0}".fomrat(
                        res.error_code.val))
        except rospy.ServiceException:
            import traceback
            traceback.print_exc()
            raise
        return res.trajectory

    def _execute_trajectory(self, clients, joint_trajectory):
        u"""軌道再生を実行する

        Parameters:
            clients (List[FollowTrajectoryActionClient]):
            joint_trajectory (trajectory_msgs.msg.JointTrajectory):
                再生する関節軌道
        Returns:
            None
        """
        joint_states = self._get_joint_state()

        for client in clients:
            traj = _extract_trajectory(
                joint_trajectory, client.joint_names, joint_states)
            client.send_goal(traj)

        rate = rospy.Rate(_TRAJECTORY_RATE)
        try:
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
                        log.append("{0}={1}".format(c.controller_name,
                                                    c.get_state()))
                        c.cancel_goal()
                    text = "Playing trajecotry failed: {0}".format(', '.join(log))
                    raise exceptions.FollowTrajectoryError(text)
                if all(map(lambda s: s == actionlib.GoalStatus.SUCCEEDED, states)):
                    break
                rate.sleep()
        except KeyboardInterrupt:
            for client in clients:
                client.cancel_goal()
