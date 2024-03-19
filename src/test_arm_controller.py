#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion
import sys
from interbotix_xs_modules.arm import InterbotixManipulatorXS

sys.path.insert(0, '/home/brl/interbotix_ws/')

class ArmController:

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.arm_api = InterbotixManipulatorXS("px100", "arm", "gripper", init_node=False) 

        arm_command_sent = False

        rate = rospy.Rate(10.0)

        while not (rospy.is_shutdown() or arm_command_sent):
            try:
                self.arm_api.gripper.open()

                self.arm_api.arm.go_to_sleep_pose()

                tf_base_to_test= self.tf_buffer.lookup_transform('px100/base_link', 'test_frame', rospy.Time()).transform
                quat_tf_base_to_test= [tf_base_to_test.rotation.x,
                        tf_base_to_test.rotation.y,
                        tf_base_to_test.rotation.z,
                        tf_base_to_test.rotation.w]
                euler_tf_base_to_test = euler_from_quaternion(quat_tf_base_to_test)
                
                self.arm_api.arm.set_single_joint_position('waist', euler_tf_base_to_test[2])
                
                self.arm_api.arm.set_ee_cartesian_trajectory(x=0.15, z=0.1)
                
                tf_gripper_to_test= self.tf_buffer.lookup_transform('px100/ee_gripper_link', 'test_frame', rospy.Time()).transform
                print(tf_gripper_to_test.translation)
                self.arm_api.arm.set_ee_cartesian_trajectory(x=tf_base_to_test.translation.x - 0.15, z=tf_gripper_to_test.translation.z - 0.1)

                self.arm_api.arm.go_to_sleep_pose()
                arm_command_sent = True

            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                continue

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('arm_controller')
    res = 'n'
    while res != 'y' and res != 'yes':
        res = input('Are all fiducials being detected? (y/n) ').lower() 
    ArmController()
