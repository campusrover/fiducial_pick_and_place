#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
from interbotix_xs_modules.arm import InterbotixManipulatorXS
import math

sys.path.insert(0, '/home/brl/interbotix_ws/')

class ArmController:

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.xs_api = InterbotixManipulatorXS(
                "px100", 
                "arm", 
                "gripper", 
                init_node=False)
        self.INIT_DIST = 0.22
        self.INIT_HEIGHT = 0.1
        self.PLACE_HEIGHT_OFFSET = 0.045
        self.MOVING_TIME = 1
        self.DEBUG = True

    def run(self):
        arm_command_sent = False
        rate = rospy.Rate(10.0)

        while not (rospy.is_shutdown() or arm_command_sent):
            try:
                self._init_arm_pose()

                (height, dist, yaw) = self._get_pick_args()
                self._pick(height, dist, yaw)

                self._init_arm_pose()

                (height, dist, yaw) = self._get_place_args()
                self._place(height, dist, yaw)

                self._retire_robot()

                arm_command_sent = True

            except (tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException):
                continue

            rate.sleep()

    def _init_arm_pose(self):
        self.xs_api.arm.go_to_sleep_pose()
        self.xs_api.arm.set_ee_pose_components(
                x=self.INIT_DIST,
                z=self.INIT_HEIGHT,
                moving_time=self.MOVING_TIME)

    def _get_pick_args(self):
        tf_base_to_pickup = self.tf_buffer.lookup_transform(
                'px100/base_link',
                'pickup',
                rospy.Time()).transform
        dist = self._get_dist_base_orig_to_dest_orig(tf_base_to_pickup)
        yaw = self._get_yaw(tf_base_to_pickup.translation.y, dist)
        return tf_base_to_pickup.translation.z, dist, yaw

    def _pick(self, height, dist, yaw):
        self._open_gripper()
        self._move_robot_waist(yaw)
        self._move_robot_limbs(dist, height)
        self._close_gripper()

    def _get_place_args(self):
        tf_base_to_place = self.tf_buffer.lookup_transform(
                'px100/base_link',
                'place',
                rospy.Time()).transform
        dist = self._get_dist_base_orig_to_dest_orig(tf_base_to_place)
        yaw = self._get_yaw(tf_base_to_place.translation.y, dist)
        return tf_base_to_place.translation.z, dist, yaw

    def _place(self, height, dist, yaw):
        self._move_robot_waist(yaw)
        self._move_robot_limbs(dist, height + self.PLACE_HEIGHT_OFFSET)
        self._open_gripper()
    
    def _get_dist_base_orig_to_dest_orig(self, tf_base_to_dest):
        tr_base_to_dest = [tf_base_to_dest.translation.x,
                tf_base_to_dest.translation.y,
                tf_base_to_dest.translation.z]
        result = np.linalg.norm(np.array(tr_base_to_dest))
        if self.DEBUG:
            rospy.loginfo(f'Distance from base-frame origin to destination-frame origin: {result}')
        return result

    def _get_yaw(self, opposite, hypotenuse):
        if self.DEBUG:
            rospy.loginfo(f'Opposite length: {opposite}')
        yaw = math.asin(abs(opposite) / hypotenuse)
        if opposite < 0:
            yaw = -1 * yaw
        if self.DEBUG:
            rospy.loginfo(f'Yaw: {yaw}')
        return yaw

    def _move_robot_waist(self, yaw):
        self.xs_api.arm.set_single_joint_position('waist', yaw)

    def _move_robot_limbs(self, x_from_base, z_from_base):
        self.xs_api.arm.set_ee_cartesian_trajectory(
                x=x_from_base - self.INIT_DIST,
                z=z_from_base - self.INIT_HEIGHT)

    def _open_gripper(self):
        self.xs_api.gripper.open()

    def _close_gripper(self):
        self.xs_api.gripper.close()

    def _retire_robot(self):
        self.xs_api.arm.go_to_sleep_pose()

if __name__ == '__main__':
    rospy.init_node('arm_controller')
    res = 'n'
    while res != 'y' and res != 'yes':
        res = input('Are all fiducials being detected? (y/n) ').lower()
    ArmController().run()

