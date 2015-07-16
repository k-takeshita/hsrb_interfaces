# + "/hsrb/omni_base_controllere!/usr/bin/env python
# vim: fileencoding=utf-8

import copy
from itertools import repeat

import tf
import tf2_ros
import actionlib
import rospy

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

from . import exceptions
from . import settings
from . import robot
from . import utils
from . import geometry


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



def _extract_trajectory(joint_trajectory, joint_names, joint_state):
    """関節軌道から指定した関節の軌道のみを抜き出し、残りの関節目標値を現在値で埋める

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


class JointGroup(robot.Resource):
    u"""関節グループの制御を行うクラス


    Attributes:
        joint_names (List[str]):

    """
    def __init__(self, name):
        super(JointGroup, self).__init__()
        self._setting = settings.get_entry('joint_group', name)
        self._arm_client = FollowTrajectoryActionClient(self._setting['arm_controller_prefix'])
        self._head_client = FollowTrajectoryActionClient(self._setting['head_controller_prefix'])
        self._hand_client = FollowTrajectoryActionClient(self._setting["hand_controller_prefix"])
        self._base_client = FollowTrajectoryActionClient(self._setting["omni_base_controller_prefix"], "/base_coordinates")
        self._joint_state_sub = utils.CachingSubscriber(self._setting["joint_states_topic"], JointState, default=JointState())
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
        return self._get_joint_state()

    @property
    def collision_world(self):
        return self._collision_world

    @collision_world.setter
    def collision_world(self, value):
        self._collision_world = value


    def _change_joint_state(self, goal_state):
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

        plan_service = rospy.ServiceProxy(self._setting['plan_with_joint_goals_service'], PlanWithJointGoals)
        res = plan_service.call(req)
        if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
            raise exceptions.PlannerError("Fail to plan change_joint_state: {0}".format(res.error_code.val))
        self._play_trajectory(res.solution)

    def move_to_joint_positions(self, goals):
        u"""指定の姿勢に遷移する

        Args:
            positions (Dict[str, float]): 関節名と目標位置の組による辞書

        Returns:
            None
        """
        goal_state = JointState()
        for k, v in goals.items():
            goal_state.name.append(k)
            goal_state.position.append(v)
        self._change_joint_state(goal_state)

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
        req.timeout = rospy.Duration(_PLANNING_ARM_TIMEOUT)
        req.max_iteration = _PLANNING_MAX_ITERATION
        req.uniform_bound_sampling = False
        req.deviation_for_bound_sampling = _PLANNING_GOAL_DEVIATION

        plan_service = rospy.ServiceProxy(self._setting['plan_with_hand_goals_service'], PlanWithHandGoals)
        res = plan_service.call(req)
        if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
            raise exceptions.PlannerError("Fail to plan move_endpoint: {0}".format(res.error_code.val))
        res.base_solution.header.frame_id = settings.get_frame('odom')
        self._play_trajectory(res.solution, res.base_solution)

    def move_end_effetor_by_line(self, axis, distance, ref_frame_id=None):
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
        odom_to_robot_transform = self._tf2_buffer.lookup_transform(settings.get_frame('odom'),
                                                                    settings.get_frame('base'),
                                                                    rospy.Time(0),
                                                                    rospy.Duration(_TF_TIMEOUT))
        odom_to_robot_pose = geometry.tuples_to_pose(geometry.transform_to_tuples(odom_to_robot_transform.transform))

        req = PlanWithHandLineRequest()
        req.base_movement_type.val = BaseMovementType.PLANAR
        req.origin_to_basejoint = odom_to_robot_pose
        req.initial_joint_state = self._get_joint_state()
        req.use_joints = use_joints
        req.axis.x = axis.x
        req.axis.y = axis.y
        req.axis.z = axis.z
        req.local_origin_of_axis = True
        req.ref_frame_id = ref_frame_id
        req.goal_value = distance
        req.probability_goal_generate = _PLANNING_GOAL_GENERATION
        req.attached_objects = []
        req.timeout = rospy.Duration(_PLANNING_ARM_TIMEOUT)
        req.max_iteration = _PLANNING_MAX_ITERATION
        req.uniform_bound_sampling = False
        req.deviation_for_bound_sampling = _PLANNING_GOAL_DEVIATION
        req.extra_goal_constraints = []

        plan_service = rospy.ServiceProxy(self._setting['plan_with_hand_line_service'], PlanWithHandLine)
        res = plan_service.call(req)
        if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
            raise exceptions.PlannerError("Fail to plan move_hand_line: {0}".format(res.error_code.val))
        res.base_solution.header.frame_id = settings.get_frame('odom')
        self._play_trajectory(res.solution, res.base_solution)

    def _play_trajectory(self, joint_trajectory, base_trajectory=None):
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
                raise exceptions.TrajectoryLengthError("Uneven joint_trajectory size and base_trajectory size.")

            odom_to_frame_transform = self._tf2_buffer.lookup_transform(_BASE_TRAJECTORY_ORIGIN,
                                                                        base_trajectory.header.frame_id,
                                                                        rospy.Time(0),
                                                                        rospy.Duration(_TF_TIMEOUT))
            odom_to_frame = geometry.transform_to_tuples(odom_to_frame_transform.transform)

            num_points = len(joint_trajectory.points)
            odom_base_trajectory = JointTrajectory()
            odom_base_trajectory.points = list(utils.iterate(JointTrajectoryPoint, num_points))
            odom_base_trajectory.header = base_trajectory.header
            odom_base_trajectory.joint_names = self._base_client.joint_names

            # 各ポイントをodom座標系に変換する
            previous_theta = 0.0
            for i in range(num_points):
                t = base_trajectory.points[i].transforms[0]
                frame_to_base = geometry.transform_to_tuples(t)

                # odom_to_base = odom_to_frame * frame_to_base
                (odom_to_base_trans, odom_to_base_rot) = geometry.multiply_tuples(odom_to_frame, frame_to_base)

                odom_base_trajectory.points[i].positions = [0, 0, 0]
                odom_base_trajectory.points[i].positions[0] = odom_to_base_trans[0]
                odom_base_trajectory.points[i].positions[1] = odom_to_base_trans[1]

                roll, pitch, yaw = tf.transformations.euler_from_quaternion(odom_to_base_rot)
                theta = previous_theta * geometry.shortest_angular_distance(previous_theta, yaw)

                odom_base_trajectory.points[i].positions[2] = theta
                previous_theta = theta

            # 台車と腕の軌道をマージ
            merged_trajectory = _merge_trajectory(joint_trajectory, odom_base_trajectory)

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
        filter_service = rospy.ServiceProxy(self._setting["trajectory_filter_service"],
                                            FilterJointTrajectoryWithConstraints)
        req = FilterJointTrajectoryWithConstraintsRequest()
        req.trajectory = joint_trajectory
        req.allowed_time = rospy.Duration(_TRAJECTORY_FILTER_TIMEOUT)
        try:
            res = filter_service.call(req)
            if res.error_code.val != res.error_code.SUCCESS:
                raise exceptions.TrajectoryFilterError("Failed to filter trajectory: {0}".fomrat(res.error_code.val))
        except rospy.ServiceException:
            import traceback
            traceback.print_exc()
            raise
        filtered_traj = res.trajectory

        joint_states = self._get_joint_state()

        for client in clients:
            traj = _extract_trajectory(filtered_traj, client.joint_names, joint_states)
            client.send_goal(traj)

        rate = rospy.Rate(_TRAJECTORY_RATE)
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
                    log.append("{0}={1}".format(c.controller_name, c.get_state()))
                    c.cancel_goal()
                text = "Playing trajecotry failed: {0}".format(', '.join(log))
                raise exceptions.FollowTrajectoryError(text)
            if all(map(lambda s: s == actionlib.GoalStatus.SUCCEEDED, states)):
                break
            rate.sleep()
        return (client.get_results() for client in clients)
