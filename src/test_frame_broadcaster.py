#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from math import pi

class TestFrameBroadcaster:

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        new_tfs = TransformStamped()
        new_tfs.header.frame_id = 'px100/base_link'
        new_tfs.child_frame_id = 'test_frame'

        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            try:
                tf_base_link = self.tf_buffer.lookup_transform('px100/base_link', 'px100/base_link', rospy.Time()).transform
                new_tfs.header.stamp = rospy.Time.now()
                new_tfs.transform.translation = tf_base_link.translation
                new_tfs.transform.translation.x = tf_base_link.translation.x + 0.18749999998
                new_tfs.transform.translation.y = tf_base_link.translation.y + 0.18749999998 

# 0.12499999999

                base_quat = [tf_base_link.rotation.x,
                        tf_base_link.rotation.y,
                        tf_base_link.rotation.z,
                        tf_base_link.rotation.w]
                base_euler = euler_from_quaternion(base_quat)
                
                test_frame_quat = quaternion_from_euler(base_euler[0],
                        base_euler[1],
                        base_euler[2] + (pi / 4))

                new_tfs.transform.rotation.x = test_frame_quat[0]
                new_tfs.transform.rotation.y = test_frame_quat[1]
                new_tfs.transform.rotation.z = test_frame_quat[2]
                new_tfs.transform.rotation.w = test_frame_quat[3]

                self.tf_broadcaster.sendTransform(new_tfs)
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                continue
            
            rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node('test_frame_broadcaster')
    TestFrameBroadcaster()

