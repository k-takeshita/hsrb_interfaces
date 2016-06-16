#!/usr/bin/env python
# vim: fileencoding=utf-8 :
"""Test collision_world object .

"""

import math
import rospy
from tmc_manipulation_msgs.msg import CollisionObject
from hsrb_interface import geometry
import testing

COLLISION_ENVIRONMENT_TIMEOUT = 30.0

class CollisionWorldTest(testing.HsrbInterfaceTest):
    def setUp(self):
        # XXX: Waiting collision_environment start subscription.
        #      We need more reliable method to wait until the node start up.
        known_object_pub = rospy.Publisher('known_object', CollisionObject,
                                           queue_size=1)
        start = rospy.Time.now()
        while known_object_pub.get_num_connections() == 0:
            now = rospy.Time.now()
            if (now - start).to_sec() < COLLISION_ENVIRONMENT_TIMEOUT:
                rospy.sleep(0.1)
            else:
                self.fail("Faild to connect to collision environemnt.")

    def test_create_collision_objects(self):
        """It should be able to create obejects in collision world."""
        box_id = self.collision_world.add_box(x=0.3, y=0.3, z=0.3,
                                              pose=geometry.pose(x=1.0, z=0.15),
                                              timeout=3.0)
        self.assertIsNotNone(box_id)

        sphere_id = self.collision_world.add_sphere(radius=0.3,
                                                    pose=geometry.pose(x=1.0, y=1.0, z=0.5),
                                                    timeout=3.0)
        self.assertIsNotNone(sphere_id)

        cylinder_id = self.collision_world.add_cylinder(radius=0.1, length=1.0,
                                                        pose=geometry.pose(x=1.0, y=-1.0, z=0.5),
                                                        timeout=3.0)
        self.assertIsNotNone(cylinder_id)

        mesh_id = self.collision_world.add_mesh(filename='pacakge:///hsrb_interface_py/test/chair.stl',
                                                pose=geometry.pose(x=2.0, ei=math.radians(90)),
                                                frame_id='map', name='chair')
        self.assertIsNotNone(mesh_id)

        snapshot = self.collision_world.snapshot()
        objects = [o.id.object_id for o in snapshot.known_objects]
        self.assertIn(box_id[0], objects)
        self.assertIn(sphere_id[0], objects)
        self.assertIn(cylinder_id[0], objects)
        self.assertIn(mesh_id[0], objects)

        self.collision_world.remove_all()

        snapshot = self.collision_world.snapshot()
        objects = [o.id.object_id for o in snapshot.known_objects]
        self.assertNotIn(box_id[0], objects)
        self.assertNotIn(sphere_id[0], objects)
        self.assertNotIn(cylinder_id[0], objects)
        self.assertNotIn(mesh_id[0], objects)

    def test_remove_object(self):
        """It should remove only specified obeject from collision world."""
        box_id = self.collision_world.add_box(x=0.3, y=0.3, z=0.3,
                                              pose=geometry.pose(x=1.0, z=0.15),
                                              timeout=3.0)
        self.assertIsNotNone(box_id)

        sphere_id = self.collision_world.add_sphere(radius=0.3,
                                                    pose=geometry.pose(x=1.0, y=1.0, z=0.5),
                                                    timeout=3.0)
        self.assertIsNotNone(sphere_id)

        self.collision_world.remove(sphere_id)

        snapshot = self.collision_world.snapshot()
        objects = [o.id.object_id for o in snapshot.known_objects]
        self.assertIn(box_id[0], objects)
        self.assertNotIn(sphere_id[0], objects)

        self.collision_world.remove_all()

    def test_collision_avoidance(self):
        """Testing collision check works correctly."""
        box_id = self.collision_world.add_box(x=0.3, y=0.3, z=0.3,
                                              pose=geometry.pose(x=0.6, y=0.0, z=0.65),
                                              timeout=3.0)
        self.assertIsNotNone(box_id)

        self.whole_body.move_to_neutral()
        self.whole_body.collision_world = self.collision_world

        hand_pose = self.whole_body.get_end_effector_pose('map')
        goal = geometry.pose(0.7, 0.0, 0.25, ej=math.pi/2.0)
        self.whole_body.move_end_effector_pose(goal, 'map')
        self.expect_hand_reach_goal(goal, frame='map', pos_delta=0.02,
                                    ori_delta=math.radians(2.0))
        self.whole_body.collision_world = None

if __name__ == '__main__':
    import rostest
    rostest.rosrun('hsrb_interface_py', 'simtest_collision', CollisionWorldTest)

