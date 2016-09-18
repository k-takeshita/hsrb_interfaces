# vim: fileencoding=utf-8
"""This module contains classes and functions to move joints."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import warnings

import actionlib
import rospy
import tf

from sensor_msgs.msg import JointState

from tmc_manipulation_msgs.msg import ArmManipulationErrorCodes
from tmc_manipulation_msgs.msg import BaseMovementType
from tmc_planning_msgs.msg import JointPosition
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from tmc_planning_msgs.srv import PlanWithHandGoals
from tmc_planning_msgs.srv import PlanWithHandGoalsRequest
from tmc_planning_msgs.srv import PlanWithHandLine
from tmc_planning_msgs.srv import PlanWithHandLineRequest
from tmc_planning_msgs.srv import PlanWithJointGoals
from tmc_planning_msgs.srv import PlanWithJointGoalsRequest

from . import collision_world
from . import exceptions
from . import trajectory
from . import geometry
from . import robot
from . import settings
from . import utils
from . import robot_model


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

# Base frame of a mobile base in moition planning
_BASE_TRAJECTORY_ORIGIN = "odom"


class JointGroup(robot.Item):
    """Abstract interface to control a group of joints.

    Attributes:
        joint_names (List[str]):
            A list of available joints.
        joint_positions (Dict[str, float]):
            Current joint positions.
        joint_states (sensor_msgs.msg.JointState):
            A latest joint states.
        joint_limits (Dict[str, float]):
            Joint limits of a robot.
        collision_world (hsrb_interface.collsion_world.CollisionWorld):
            A present collision world to check collision.
            If None, collision checking is disabled.
        linear_weight (float):
            How much laying weight on linear movement of a mobile base.
            This attirbute affect a output trajectory of motion planning.
        angular_weight (float):
            How much laying weight on angular movement of a mobile base.
            This attirbute affect a output trajectory of motion planning.
        planning_timeout (float):
            Timeout for motion planning [sec].
        impedance_config (str):
            A name of impedance control preset config.
            If None, impeance control is disabled.
            Default is None.
        use_base_timeopt (bool):
            If true, time-optimal filter is applied to a base trajectory.
    """

    def __init__(self, name):
        """See class docstring."""
        super(JointGroup, self).__init__()
        self._setting = settings.get_entry('joint_group', name)
        arm_config = self._setting['arm_controller_prefix']
        self._arm_client = trajectory.TrajectoryController(arm_config)
        head_config = self._setting['head_controller_prefix']
        self._head_client = trajectory.TrajectoryController(head_config)
        hand_config = self._setting["hand_controller_prefix"]
        self._hand_client = trajectory.TrajectoryController(hand_config)
        base_config = self._setting["omni_base_controller_prefix"]
        self._base_client = trajectory.TrajectoryController(base_config,
                                                            "/base_coordinates")
        imp_config = settings.get_entry("trajectory", "impedance_control")
        self._impedance_client = trajectory.ImpedanceController(imp_config)
        joint_state_topic = self._setting["joint_states_topic"]
        self._joint_state_sub = utils.CachingSubscriber(joint_state_topic,
                                                        JointState,
                                                        default=JointState())
        timeout = self._setting.get('timeout', None)
        self._joint_state_sub.wait_for_message(timeout)
        self._tf2_buffer = robot._get_tf2_buffer()
        self._end_effector_frames = self._setting['end_effector_frames']
        self._end_effector_frame = self._end_effector_frames[0]
        self._robot_urdf = robot_model.RobotModel.from_parameter_server()

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
        return {joint_name: (joint_map[joint_name].limit.lower,
                             joint_map[joint_name].limit.upper)
                for joint_name in self.joint_names}

    @property
    def collision_world(self):
        return self._collision_world

    @collision_world.setter
    def collision_world(self, value):
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
        return self._use_base_timeopt

    @use_base_timeopt.setter
    def use_base_timeopt(self, value):
        self._use_base_timeopt = value

    @property
    def end_effector_frame(self):
        """Get or set the target end effector frame of motion planning.

        This attribute affects behaviors of following methods:
        * get_end_effector_pose
        * move_end_effector_pose
        * move_end_effector_by_line
        """
        return self._end_effector_frame

    @end_effector_frame.setter
    def end_effector_frame(self, value):
        if value in set(self._end_effector_frames):
            self._end_effector_frame = value
        else:
            msg = "`ref_frame_id` must be one of end-effector frames({0})"
            raise ValueError(msg.format(self._end_effector_frames))

    @property
    def end_effector_frames(self):
        return tuple(self._end_effector_frames)

    def _change_joint_state(self, goal_state):
        """Move joints to specified joint state while checking self collision.

        Args:
            goal_state (sensor_msgs.msg.JointState): Target joint state
        Returns:
            None
        Raises:
            ValueError: Some specified joints are not found.
        """
        # Validate joint names
        initial_joint_state = self._get_joint_state()
        active_joint_set = set(initial_joint_state.name)
        target_joint_set = set(goal_state.name)
        if not target_joint_set.issubset(active_joint_set):
            unknown_set = target_joint_set.difference(active_joint_set)
            msg = "No such joint(s): [{0}]".format(', '.join(unknown_set))
            raise ValueError(msg)

        odom_to_robot_transform = self._tf2_buffer.lookup_transform(
            settings.get_frame('odom'),
            settings.get_frame('base'),
            rospy.Time(0),
            rospy.Duration(_TF_TIMEOUT)
        )
        odom_to_robot_tuples = geometry.transform_to_tuples(
            odom_to_robot_transform.transform
        )
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
            snapshot = self._collision_world.snapshot('odom')
            req.environment_before_planning = snapshot

        service_name = self._setting['plan_with_joint_goals_service']
        plan_service = rospy.ServiceProxy(service_name, PlanWithJointGoals)
        res = plan_service.call(req)
        if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
            msg = "Fail to plan change_joint_state"
            raise exceptions.MotionPlanningError(msg, res.error_code)
        res.base_solution.header.frame_id = settings.get_frame('odom')
        constrained_traj = self._constrain_trajectories(res.solution,
                                                        res.base_solution)
        self._execute_trajectory(constrained_traj)

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
            :py:attr:`.joint_names`

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
                   whole_body.move_to_joint_positions(
                       head_tilt_joint=math.radians(30)
                   )

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
        """Move joints to neutral(initial) pose of a robot."""
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
        """Move joints to a suitable pose for moving a mobile base."""
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
            self._end_effector_frame,
            rospy.Time(0),
            rospy.Duration(_TF_TIMEOUT)
        )
        result = geometry.transform_to_tuples(transform.transform)
        return result

    def move_end_effector_pose(self, pose, ref_frame_id=None):
        """Move an end effector to a given pose.

        Args
            pose (Tuple[Vector3, Quaternion]):
                A target pose of the end effector frame.
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
        ).transform
        odom_to_robot_tuples = geometry.transform_to_tuples(
            odom_to_robot_transform
        )
        odom_to_robot_pose = geometry.tuples_to_pose(odom_to_robot_tuples)

        odom_to_ref_transform = self._tf2_buffer.lookup_transform(
            settings.get_frame('odom'),
            ref_frame_id,
            rospy.Time(0),
            rospy.Duration(_TF_TIMEOUT)
        ).transform
        odom_to_ref = geometry.transform_to_tuples(odom_to_ref_transform)
        odom_to_hand = geometry.multiply_tuples(odom_to_ref, pose)
        odom_to_hand_pose = geometry.tuples_to_pose(odom_to_hand)

        req = PlanWithHandGoalsRequest()
        req.base_movement_type.val = BaseMovementType.PLANAR
        req.origin_to_basejoint = odom_to_robot_pose
        req.initial_joint_state = self._get_joint_state()
        req.use_joints = use_joints
        req.origin_to_hand_goals.append(odom_to_hand_pose)
        req.ref_frame_id = self._end_effector_frame
        req.probability_goal_generate = _PLANNING_GOAL_GENERATION
        req.timeout = rospy.Duration(self._planning_timeout)
        req.max_iteration = _PLANNING_MAX_ITERATION
        req.uniform_bound_sampling = False
        req.deviation_for_bound_sampling = _PLANNING_GOAL_DEVIATION
        req.weighted_joints = ['_linear_base', '_rotational_base']
        req.weight = [self._linear_weight, self._angular_weight]
        if self._collision_world is not None:
            snapshot = self._collision_world.snapshot('odom')
            req.environment_before_planning = snapshot

        service_name = self._setting['plan_with_hand_goals_service']
        plan_service = rospy.ServiceProxy(service_name,
                                          PlanWithHandGoals)
        res = plan_service.call(req)
        if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
            msg = "Fail to plan move_endpoint"
            raise exceptions.MotionPlanningError(msg, res.error_code)
        res.base_solution.header.frame_id = settings.get_frame('odom')
        constrained_traj = self._constrain_trajectories(res.solution,
                                                        res.base_solution)
        self._execute_trajectory(constrained_traj)

    def move_end_effector_by_line(self, axis, distance, ref_frame_id=None):
        """Move an end effector along with a line in a 3D space.

        Args:
            axis (Vector3): A axis to move along with
            distance (float): Distance to move [m]
            ref_frame_id (str):
                [DEPRECATED] The frame name of the target end effector.
                ``axis`` is defined on this frame.
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
            end_effector_frame = self._end_effector_frame
        else:
            msg = ' '.join(["`ref_frame_id` argument is deprecated."
                            "Use `end_effector_frame` attribute instead."])
            warnings.warn(msg, exceptions.DeprecationWarning)
            if ref_frame_id not in self._end_effector_frames:
                msg = "ref_frame_id must be one of end-effector frames({0})"
                raise ValueError(msg.format(self._end_effector_frames))
            else:
                end_effector_frame = ref_frame_id

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
        req.ref_frame_id = end_effector_frame
        req.goal_value = distance
        req.probability_goal_generate = _PLANNING_GOAL_GENERATION
        req.attached_objects = []
        req.timeout = rospy.Duration(self._planning_timeout)
        req.max_iteration = _PLANNING_MAX_ITERATION
        req.uniform_bound_sampling = False
        req.deviation_for_bound_sampling = _PLANNING_GOAL_DEVIATION
        req.extra_goal_constraints = []
        if self._collision_world is not None:
            snapshot = self._collision_world.snapshot('odom')
            req.environment_before_planning = snapshot

        service_name = self._setting['plan_with_hand_line_service']
        plan_service = rospy.ServiceProxy(service_name,
                                          PlanWithHandLine)
        res = plan_service.call(req)
        if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
            msg = "Fail to plan move_hand_line"
            raise exceptions.MotionPlanningError(msg, res.error_code)
        res.base_solution.header.frame_id = settings.get_frame('odom')
        constrained_traj = self._constrain_trajectories(res.solution,
                                                        res.base_solution)
        self._execute_trajectory(constrained_traj)

    def _transform_base_trajectory(self, base_traj):
        """Transform a base trajectory to an ``odom`` frame based trajectory.

        Args:
            base_traj (tmc_manipulation_msgs.msg.MultiDOFJointTrajectory):
                A base trajectory
        Returns:
            trajectory_msgs.msg.JointTrajectory:
                A base trajectory based on ``odom`` frame.
        """
        odom_to_frame_transform = self._tf2_buffer.lookup_transform(
            _BASE_TRAJECTORY_ORIGIN,
            base_traj.header.frame_id,
            rospy.Time(0),
            rospy.Duration(_TF_TIMEOUT))
        odom_to_frame = geometry.transform_to_tuples(
            odom_to_frame_transform.transform)

        num_points = len(base_traj.points)
        odom_base_traj = JointTrajectory()
        odom_base_traj.points = list(utils.iterate(JointTrajectoryPoint,
                                                   num_points))
        odom_base_traj.header = base_traj.header
        odom_base_traj.joint_names = self._base_client.joint_names

        # Transform each point into odom frame
        previous_theta = 0.0
        for i in range(num_points):
            t = base_traj.points[i].transforms[0]
            frame_to_base = geometry.transform_to_tuples(t)

            # odom_to_base = odom_to_frame * frame_to_base
            (odom_to_base_trans, odom_to_base_rot) = geometry.multiply_tuples(
                odom_to_frame,
                frame_to_base
            )

            odom_base_traj.points[i].positions = [odom_to_base_trans[0],
                                                  odom_to_base_trans[1],
                                                  0]
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(
                odom_to_base_rot)
            dtheta = geometry.shortest_angular_distance(previous_theta, yaw)
            theta = previous_theta + dtheta

            odom_base_traj.points[i].positions[2] = theta
            previous_theta = theta
        return odom_base_traj

    def _constrain_trajectories(self, joint_trajectory, base_trajectory):
        """Apply constraints to given trajectories.

        Parameters:
            joint_trajectory (trajectory_msgs.msg.JointTrajectory):
                A upper body trajectory
            base_trajectory (trajectory_msgs.msg.JointTrajectory):
                A base trajectory
        Returns:
            trajectory_msgs.msg.JointTrajectory:
                A constrained trajectory
        Raises:
            TrajectoryFilterError:
                Failed to execute trajectory-filtering
        """
        odom_base_trajectory = self._transform_base_trajectory(base_trajectory)
        merged_traj = trajectory.merge(joint_trajectory, odom_base_trajectory)

        filtered_merged_traj = trajectory.constraint_filter(merged_traj)
        if not self._use_base_timeopt:
            return filtered_merged_traj

        # Transform arm and base trajectories to time-series trajectories
        filtered_joint_traj = trajectory.constraint_filter(joint_trajectory)
        filtered_base_traj = trajectory.timeopt_filter(odom_base_trajectory)
        last_joint_point = filtered_joint_traj.points[-1]
        last_base_point = filtered_base_traj.points[-1]
        arm_joint_time = last_joint_point.time_from_start.to_sec()
        base_timeopt_time = last_base_point.time_from_start.to_sec()

        # If end of a base trajectory is later than arm one,
        # an arm trajectory is made slower.
        # (1) arm_joint_time < base_timeopt_time: use timeopt trajectory
        if arm_joint_time < base_timeopt_time:
            # Adjusting arm trajectory points to base ones as much as possible
            trajectory.adjust_time(filtered_base_traj, filtered_joint_traj)
            return trajectory.merge(filtered_joint_traj, filtered_base_traj)
        else:
            return filtered_merged_traj

    def _execute_trajectory(self, joint_traj):
        """Execute a trajectory with given action clients.

        Action clients that actually execute trajectories are selected
        automatically.

        Parameters:
            joint_traj (trajectory_msgs.msg.JointTrajectory):
                A trajectory to be executed
        Returns:
            None
        """
        clients = []
        if self._impedance_client.config is not None:
            clients.append(self._impedance_client)
        else:
            clients.extend([self._arm_client, self._base_client])
        for joint in joint_traj.joint_names:
            if joint in self._head_client.joint_names:
                clients.append(self._head_client)
                break

        joint_states = self._get_joint_state()

        for client in clients:
            traj = trajectory.extract(joint_traj, client.joint_names,
                                      joint_states)
            client.submit(traj)

        trajectory.wait_controllers(clients)
