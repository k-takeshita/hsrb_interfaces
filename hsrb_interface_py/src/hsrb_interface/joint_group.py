# vim: fileencoding=utf-8
"""This module contains classes and functions to move joints."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import copy
from itertools import repeat
import traceback

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import FollowJointTrajectoryResult
import rospy
import tf

from sensor_msgs.msg import JointState

from tmc_manipulation_msgs.msg import ArmManipulationErrorCodes
from tmc_manipulation_msgs.msg import BaseMovementType

from tmc_manipulation_msgs.srv import FilterJointTrajectory
from tmc_manipulation_msgs.srv import FilterJointTrajectoryRequest
from tmc_manipulation_msgs.srv import FilterJointTrajectoryWithConstraints
from tmc_manipulation_msgs.srv import FilterJointTrajectoryWithConstraintsRequest
from tmc_manipulation_msgs.srv import SelectConfig
from tmc_manipulation_msgs.srv import SelectConfigRequest

from tmc_planning_msgs.msg import JointPosition

from tmc_planning_msgs.srv import PlanWithHandGoals
from tmc_planning_msgs.srv import PlanWithHandGoalsRequest
from tmc_planning_msgs.srv import PlanWithHandLine
from tmc_planning_msgs.srv import PlanWithHandLineRequest
from tmc_planning_msgs.srv import PlanWithJointGoals
from tmc_planning_msgs.srv import PlanWithJointGoalsRequest
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from urdf_parser_py import urdf
from urdf_parser_py.urdf import xmlr


# Monkeypatching urdf_parser_py (1)
# https://github.com/k-okada/urdfdom/commit/9c9e28e9f1de29228def48a4d49315b2f4fbf2d2
# If urdf_parser_py is released with merging the commit above,
# we can safely remove following code.
xmlr.reflect(urdf.JointLimit, params=[
    xmlr.Attribute('effort', float),
    xmlr.Attribute('lower', float, False, 0),
    xmlr.Attribute('upper', float, False, 0),
    xmlr.Attribute('velocity', float)
])

# Monkeypatching urdf_parser_py (2)
# URDF specification allow multiple <visual> and <collision> elements.
# http://wiki.ros.org/urdf/XML/link
xmlr.reflect(urdf.Link, params=[
    xmlr.Attribute('name', str),
    xmlr.Element('origin', urdf.Pose, False),
    xmlr.Element('inertial', urdf.Inertial, False),
    xmlr.AggregateElement('visual', urdf.Visual, 'visual'),
    xmlr.AggregateElement('collision', urdf.Collision, 'collision')
])

def _get_aggregate_list(self, xml_var):
    """Get """
    var = self.XML_REFL.paramMap[xml_var].var
    if not getattr(self, var):
        self.aggregate_init()
        setattr(self, var, [])
    return getattr(self, var)

urdf.Link.get_aggregate_list = _get_aggregate_list
urdf.Collision.get_aggregate_list = _get_aggregate_list
urdf.Material.check_valid = lambda self: None

from urdf_parser_py.urdf import Robot as RobotUrdf

from . import exceptions
from . import settings
from . import robot
from . import utils
from . import geometry
from . import collision_world


# Rate to check trajectory action hz]
_TRAJECTORY_RATE = 30.0

# Timeout to call trajectory_filter [sec]
_TRAJECTORY_FILTER_TIMEOUT = 30

# Timeout for motion planning [sec]
_PLANNING_ARM_TIMEOUT = 10.0

# Max number of iteration of moition planning
_PLANNING_MAX_ITERATION = 10000

# Goal generation probability in moition planning
_PLANNING_GOAL_GENERATION = 0.3

# Goal deviation in motion planning
_PLANNING_GOAL_DEVIATION = 0.3

# Timeout to receive a tf message [sec]
_TF_TIMEOUT = 1.0

# Timeout to connect ot an action [sec]
_ACTION_WAIT_TIMEOUT = 30.0

# Base frame of a mobile base in moition planning
_BASE_TRAJECTORY_ORIGIN = "odom"


# Duration to open hand [sec]
_TIME_OPEN_HAND = 10.0

def _refer_planning_error(error_code):
    """Translate a motion planning error code to a human readable text.

    Args:
        error_code (ArmManipulationErrorCodes): An error code
    Returns:
        A human readable error description
    """
    error_codes = ArmManipulationErrorCodes.__dict__.items()
    error_names = [k for k, v in error_codes
                   if v == error_code and k.isupper()]
    if len(error_names) != 0:
        return error_names[0]
    else:
        return str(error_code)

def _extract_trajectory(trajectory, joint_names, joint_state):
    """Extract trajectories of specified joints from a given trajectory.

    If a given trajectory doesn't have a trajectory for a specified joint name,
    the trajctory of the joint is filled with currenta joint state.

    Args:
        trajectory (trajectory_msgs.msg.JointTrajector):
            A JointTrajectory to work on
        joint_names (List[str]):
            Target joint names
        joint_state (sensor_msgs.msg.JointState):
            A initial joint state to fill unspecified joint trajectory
    Returns:
        trajectory_msgs.msg.JointTrajectory: An extracted trajectory
    """
    num_points = len(trajectory.points)
    num_joints = len(joint_names)
    index_map = list(repeat(0, num_joints))
    for joint_index in range(num_joints):
        index_map[joint_index] = -1
        for input_joint_index in range(len(trajectory.joint_names)):
            if joint_names[joint_index] == trajectory.joint_names[input_joint_index]:
                index_map[joint_index] = input_joint_index
    trajectory_out = JointTrajectory()
    trajectory_out.joint_names = joint_names
    trajectory_out.points = list(utils.iterate(JointTrajectoryPoint, num_points))
    for point_index in range(num_points):
        target = trajectory_out.points[point_index]
        source = trajectory.points[point_index]
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


def _merge_trajectory(target, source):
    """Merge two trajectories into single trajectory.

    Those trajectories should have exactly same number of trajectory points.
    Result trajectory's ``time_from_start`` is set as same as `target` .

    Args:
        target(trajectory_msgs.msg.JointTrajectory):
            An original trajectory
        source(trajectory_msgs.msg.JointTrajectory):
            An additional trajectory
    Returns:
        trajectory_msgs.msg.JointTrajectory: A result trajectory
    Raises:
        ValueError: Two trajectories has different points size.
    """
    if len(target.points) != len(source.points):
        raise ValueError
    merged = copy.deepcopy(target)
    merged.joint_names = list(merged.joint_names)
    merged.joint_names.extend(source.joint_names)

    num_points = len(merged.points)
    for i in range(num_points):
        merged.points[i].positions = list(merged.points[i].positions)
        merged.points[i].positions.extend(source.points[i].positions)
        merged.points[i].velocities = list(merged.points[i].velocities)
        merged.points[i].velocities.extend(source.points[i].velocities)
        merged.points[i].accelerations = list(merged.points[i].accelerations)
        merged.points[i].accelerations.extend(source.points[i].accelerations)
    return merged


def _adjust_trajectory_time(trajectory1, trajectory2):
    """Adjust time_from_start of trajectory points in trajectory2 to another.

    Velocities and accelerations are re-computed from differences.

    Two given trajectories must have exactly same timestamp.
    """
    if len(trajectory1.points) != len(trajectory2.points):
        raise ValueError
    num_points = len(trajectory1.points)
    # Adjust ``time_from_start`` of trajectory points in trajectory2 to
    # trajectory1
    for (point1, point2) in zip(trajectory1.points, trajectory2.points):
        point2.time_from_start = point1.time_from_start
    # Re-compute velocities in trajecotry2 from difference of positions
    for index in range(num_points-1):
        dt = trajectory2.points[index+1].time_from_start.to_sec() - trajectory2.points[index].time_from_start.to_sec()
        trajectory2.points[index].velocities = [(x1-x0)/dt for (x0, x1) in zip(trajectory2.points[index].positions,
                                                                           trajectory2.points[index+1].positions)]
    zero_vector2 = [0]*len(trajectory2.joint_names)
    trajectory2.points[-1].velocities = zero_vector2
    # Re-compute accelerations in trajecotry2 from difference of velocties
    trajectory2.points[0].accelerations = zero_vector2
    for index in range(1, num_points-1):
        dt = trajectory2.points[index+1].time_from_start.to_sec() - trajectory2.points[index-1].time_from_start.to_sec()
        trajectory2.points[index].accelerations = [(x1-x0)/dt for (x0, x1) in zip(trajectory2.points[index-1].velocities,
                                                                              trajectory2.points[index+1].velocities)]
    trajectory2.points[-1].accelerations = zero_vector2


class FollowTrajectoryActionClient(object):
    """Wrapper class for FollowJointTrajectoryAction

    Args:
        controller_name (str):
            A name of a ros-controls controller
        joint_names_suffix (str):
            A name of a parameter to specify target joint names
    """
    def __init__(self, controller_name, joint_names_suffix="/joints"):
        self._controller_name = controller_name
        self._client = actionlib.SimpleActionClient(controller_name + "/follow_joint_trajectory", FollowJointTrajectoryAction)
        self._client.wait_for_server(rospy.Duration(_ACTION_WAIT_TIMEOUT))
        self._joint_names = rospy.get_param("{0}/{1}".format(self._controller_name, joint_names_suffix), None)

    def send_goal(self, trajectory):
        """Send a goal to a connecting controller."""
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        self._client.send_goal(goal)

    def cancel_goal(self):
        """Cancel a current goal."""
        self._client.cancel_goal()

    def get_state(self):
        return self._client.get_state()

    def get_results(self, timeout=None):
        """Get a result of a current goal.

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
    """Wrapper class to invoke impedance control action

    Args:
        controller_name (str):
            A name of ros-controls controller
        joint_names_suffix (str):
            A name of a parameter to specify target joint names

    """
    def __init__(self, controller_name, joint_names_suffix="/joint_names"):
        super(ImpedanceControlActionClient, self).__init__(
            controller_name, joint_names_suffix)
        self._config = None
        self._config_names = rospy.get_param(
            controller_name + "/config_names", [])

    def send_goal(self, trajectory):
        """Send a goal to a connecting controller"""
        if self._config is not None:
            setter_service = rospy.ServiceProxy(
                self._controller_name + "/select_config", SelectConfig)
            req = SelectConfigRequest()
            req.name = self._config
            try:
                res = setter_service.call(req)
                if not res.is_success:
                    raise exceptions.FollowTrajectoryError(
                        "Failed to set impedance config")
            except rospy.ServiceException:
                import traceback
                traceback.print_exc()
                raise
        else:
            raise exceptions.FollowTrajectoryError(
                "Impedance config is None. But impedance control is called.")
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        self._client.send_goal(goal)

    @property
    def config(self):
        return self._config

    @config.setter
    def config(self, value):
        if value in self._config_names:
            self._config = value
        elif value is None:
            self._config = None
        else:
            raise exceptions.FollowTrajectoryError(
                "Failed to set impedance config")

    @property
    def config_names(self):
        self._config_names = rospy.get_param(
            controller_name + "/config_names", [])
        return self._config_names

class JointGroup(robot.Item):
    """Abstract interface to control a group of joints."""

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
        self._use_base_timeopt = True

    def _get_joint_state(self):
        """Get a current joint state.

        Returns:
            sensor_msgs.JointState: Current joint state
        """
        return self._joint_state_sub.data

    @property
    def joint_names(self):
        """List[str]: List of joint names"""
        return self._get_joint_state().name

    @property
    def joint_positions(self):
        """Dict[str, float]: """
        joint_state = self._get_joint_state()
        return dict(zip(joint_state.name, joint_state.position))

    @property
    def joint_velocities(self):
        """"""
        joint_state = self._get_joint_state()
        return dict(zip(joint_state.name, joint_state.velocity))

    @property
    def joint_state(self):
        """sensor_msgs.JointState: A latest joint state"""
        return self._get_joint_state()

    @property
    def joint_limits(self):
        joint_map = self._robot_urdf.joint_map
        return {joint_name: (joint_map[joint_name].limit.lower, joint_map[joint_name].limit.upper)
                for joint_name in self.joint_names}

    @property
    def collision_world(self):
        """CollisionWorld: A present collision world to check collision.

        """
        return self._collision_world

    @collision_world.setter
    def collision_world(self, value):
        """CollisionWorld: Set a collision world for motion planning.

        If None, collision checking is disabled.
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

    @property
    def use_base_timeopt(self):
        """If True, Time-optimal mobile base control is adopted."""
        return self._use_base_timeopt

    @use_base_timeopt.setter
    def use_base_timeopt(self, value):
        self._use_base_timeopt = value

    def _change_joint_state(self, goal_state):
        """Move joints to specified joint state while checking self collision.

        Args:
            goal_state (sensor_msgs.msg.JointState): Target joint state
        Returns:
            None
        Raises:
            ValueError: Some specified joints are not found in an acutual robot.
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
        """Move joints to a specified goal positions.

        Args:
            goals (Dict[str, float]):
                A dict of pair of joint name and target position [m or rad].
            **kwargs:
                Use keyword arguments to specify joint_name/posiion pairs.
                The keyword arguments overwrite `goals` argument.


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
        """Move joints to neutral(initial) pose of a robot.

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
        """Move joints to a suitable pose for moving a mobile base.

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
        """Get a pose of end effector based on robot frame.

        Returns:
            Tuple[Vector3, Quaternion]
        """
        # Default reference frame is a robot frame
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('base')
        transform = self._tf2_buffer.lookup_transform(
            ref_frame_id,
            settings.get_frame('hand'),
            rospy.Time(0),
            rospy.Duration(_TF_TIMEOUT)
        )
        result = geometry.transform_to_tuples(transform.transform)
        return result

    def move_end_effector_pose(self, pose, ref_frame_id=None):
        """Move an end effector to a given pose.

        Args
            pose (Tuple[Vector3, Quaternion]):
            ref_frame_id (str): A base frame of an end effector.
                The default is the robot frame(```base_footprint``).
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

        # Default is the robot frame (the base frame)
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('base')

        odom_to_robot_transform = self._tf2_buffer.lookup_transform(
            settings.get_frame('odom'),
            settings.get_frame('base'),
            rospy.Time(0),
            rospy.Duration(_TF_TIMEOUT)
        )
        odom_to_robot_pose = geometry.tuples_to_pose(geometry.transform_to_tuples(odom_to_robot_transform.transform))

        odom_to_ref_transform = self._tf2_buffer.lookup_transform(
            settings.get_frame('odom'),
            ref_frame_id,
            rospy.Time(0),
            rospy.Duration(_TF_TIMEOUT)
        )
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

        plan_service = rospy.ServiceProxy(self._setting['plan_with_hand_goals_service'],
                                          PlanWithHandGoals)
        res = plan_service.call(req)
        if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
            error = _refer_planning_error(res.error_code.val)
            msg = "Fail to plan move_endpoint: {0}".format(error)
            raise exceptions.PlannerError(msg)
        res.base_solution.header.frame_id = settings.get_frame('odom')
        self._play_trajectory(res.solution, res.base_solution)

    def move_end_effector_by_line(self, axis, distance, ref_frame_id=None):
        """Move an end effector along with a line in a 3D space.

        Args:
            axis (Vector3): A axis to move along with
            distance (float): Distance to move [m]
            ref_frame_id (str): a base frame
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

        plan_service = rospy.ServiceProxy(self._setting['plan_with_hand_line_service'],
                                          PlanWithHandLine)
        res = plan_service.call(req)
        if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
            error = _refer_planning_error(res.error_code.val)
            msg = "Fail to plan move_hand_line: {0}".format(error)
            raise exceptions.PlannerError(msg)
        res.base_solution.header.frame_id = settings.get_frame('odom')
        self._play_trajectory(res.solution, res.base_solution)

    def _play_trajectory(self, joint_trajectory, base_trajectory):
        """Play given trajectories.

        Args:
            joint_trajectory (trajectory_msgs.msg.JointTrajectory):
                An upper body trajectory
            base_trajectory (tmc_manipulation_msgs.msg.MultiDOFJointTrajectory):
                A base trajectory
        Returns:
            None
        """
        constrained_trajectory = self._constrain_trajectory(joint_trajectory, base_trajectory)

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

    def _build_whole_body_trajectory(self, joint_trajectory, odom_base_trajectory):
        """Combine an upper body trajectory and a base one into whole body one.

        Args:
            joint_trajectory (trajectory_msgs.msg.JointTrajectory):
                An upper body trajectory
            odom_base_trajectory (trajectory_msgs.msg.JointTrajectory):
                A base trajectory (Based on odom frame)
        Returns:
            trajectory_msgs.msg.JointTrajectory: A whole body trajectory.
        """
        if len(joint_trajectory.points) != len(odom_base_trajectory.points):
            raise exceptions.TrajectoryLengthError(
                "Uneven joint_trajectory size and base_trajectory size.")
        # Merge arm and base trajectory
        return _merge_trajectory(joint_trajectory, odom_base_trajectory)

    def _transform_base_trajectory(self, base_trajectory):
        """"Transform a base trajectory to an ``odom`` frame based trajectory.

        Args:
            base_trajectory (tmc_manipulation_msgs.msg.MultiDOFJointTrajectory):
                A base trajectory
        Returns:
            trajectory_msgs.msg.JointTrajectory:
                A base trajectory based on ``odom`` frame
        """
        odom_to_frame_transform = self._tf2_buffer.lookup_transform(
            _BASE_TRAJECTORY_ORIGIN,
            base_trajectory.header.frame_id,
            rospy.Time(0),
            rospy.Duration(_TF_TIMEOUT))
        odom_to_frame = geometry.transform_to_tuples(
            odom_to_frame_transform.transform)

        num_points = len(base_trajectory.points)
        odom_base_trajectory = JointTrajectory()
        odom_base_trajectory.points = list(
            utils.iterate(JointTrajectoryPoint, num_points))
        odom_base_trajectory.header = base_trajectory.header
        odom_base_trajectory.joint_names = self._base_client.joint_names

        # Transform each point into odom frame
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
        return odom_base_trajectory

    def _constrain_trajectory(self, joint_trajectory, base_trajectory):
        """Apply constraints to given trajectories.

        Parameters:
            trajectory (trajectory_msgs.msg.JointTrajectory):
                A trajectory
        Returns:
            trajectory_msgs.msg.JointTrajectory:
                A constrained trajectory
        Raises:
            TrajectoryFilterError:
                Failed to execute trajectory-filtering
        """

        odom_base_trajectory = self._transform_base_trajectory(base_trajectory)
        trajectory = self._build_whole_body_trajectory(
            joint_trajectory, odom_base_trajectory)

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
            traceback.print_exc()
            raise
        filtered_merged_trajectory = res.trajectory
        all_joint_time = filtered_merged_trajectory.points[-1].time_from_start.to_sec()
        if not self._use_base_timeopt:
            return filtered_merged_trajectory
        # Transform arm and base trajectories to time-series trajectories
        filtered_joint_trajectory = self._filter_arm_trajectory(joint_trajectory)
        filtered_base_trajectory = self._filter_base_trajectory(odom_base_trajectory)
        arm_joint_time = filtered_joint_trajectory.points[-1].time_from_start.to_sec()
        base_timeopt_time = filtered_base_trajectory.points[-1].time_from_start.to_sec()

        # Play a trajectory
        # If end of a base trajectory is later than arm one,
        # an arm trajectory is made slower.
        # (1) arm_joint_time < base_timeopt_time: use timeopt trajectory
        if arm_joint_time < base_timeopt_time:
            # Adjusting arm trajectory points to base ones as much as possible
            _adjust_trajectory_time(filtered_base_trajectory, filtered_joint_trajectory)
            return _merge_trajectory(filtered_joint_trajectory, filtered_base_trajectory)
        else:
            return filtered_merged_trajectory

    def _execute_trajectory(self, clients, joint_trajectory):
        """Execute a trajectory playback with given action clients.

        Parameters:
            clients (List[FollowTrajectoryActionClient]):
                Action clients actually execute trajectories
            joint_trajectory (trajectory_msgs.msg.JointTrajectory):
                A trajectory to be executed
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

    def _filter_arm_trajectory(self, joint_trajectory):
        """Apply joint constraint fileter to an upper body trajectory."""
        service = self._setting["trajectory_filter_service"]
        filter_service = rospy.ServiceProxy(service,
                                            FilterJointTrajectoryWithConstraints)
        req = FilterJointTrajectoryWithConstraintsRequest()
        req.trajectory = joint_trajectory

        req.allowed_time = rospy.Duration(_TRAJECTORY_FILTER_TIMEOUT)
        try:
            res = filter_service.call(req)
            if res.error_code.val != res.error_code.SUCCESS:
                raise exceptions.TrajectoryFilterError("Failed to filter trajectory: {0}".fomrat(res.error_code.val))
        except rospy.ServiceException:
            traceback.print_exc()
            raise
        filtered_traj = res.trajectory
        return filtered_traj

    def _filter_base_trajectory(self, base_trajectory):
        """Apply timeopt filter to a omni-base trajectory."""
        filter_service = rospy.ServiceProxy(self._setting["omni_base_timeopt_service"], FilterJointTrajectory)
        req = FilterJointTrajectoryRequest()
        req.trajectory = base_trajectory
        try:
            res = filter_service.call(req)
            if res.error_code.val != res.error_code.SUCCESS:
                raise exceptions.TrajectoryFilterError("Failed to filter trajectory: {0}".fomrat(res.error_code.val))
        except rospy.ServiceException:
            traceback.print_exc()
            raise
        filtered_traj = res.trajectory
        return filtered_traj
