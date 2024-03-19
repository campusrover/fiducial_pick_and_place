#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

class PlaceFrameBroadcaster:

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        new_tfs = TransformStamped()
        new_tfs.header.frame_id = 'fiducial_2'
        new_tfs.child_frame_id = 'place'

        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            try:
                tf_fid = self.tf_buffer.lookup_transform('fiducial_2', 'fiducial_2', rospy.Time()).transform
                new_tfs.header.stamp = rospy.Time.now()
                new_tfs.transform.translation = tf_fid.translation
                new_tfs.transform.translation.x = tf_fid.translation.x
                new_tfs.transform.translation.y = tf_fid.translation.y

                tf_fid_to_base_link = self.tf_buffer.lookup_transform('fiducial_2', 'px100/base_link', rospy.Time()).transform 
                new_tfs.transform.rotation = tf_fid_to_base_link.rotation

                self.tf_broadcaster.sendTransform(new_tfs)
            except (tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException, 
                    tf2_ros.ConnectivityException):
                continue
            
            rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node('place_frame_broadcaster')
    PlaceFrameBroadcaster()

