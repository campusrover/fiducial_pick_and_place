#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

class PickupFrameBroadcaster:

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        new_tfs = TransformStamped()
        new_tfs.header.frame_id = 'cargo'
        new_tfs.child_frame_id = 'pickup'

        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            try:
                tf_cargo = self.tf_buffer.lookup_transform('cargo', 'cargo', rospy.Time()).transform
                new_tfs.header.stamp = rospy.Time.now()
                new_tfs.transform.translation = tf_cargo.translation
                new_tfs.transform.translation.x = tf_cargo.translation.x
                new_tfs.transform.translation.y = tf_cargo.translation.y

                tf_cargo_to_base_link = self.tf_buffer.lookup_transform('cargo', 'px100/base_link', rospy.Time()).transform 
                new_tfs.transform.rotation = tf_cargo_to_base_link.rotation

                self.tf_broadcaster.sendTransform(new_tfs)
            except (tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException, 
                    tf2_ros.ConnectivityException):
                continue
            
            rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node('pickup_frame_broadcaster')
    PickupFrameBroadcaster()

