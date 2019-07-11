#!/usr/bin/env python

import sys
import rospy

from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped

class Transform2Pose:
    def __init__(self, pub, parent_frame_id, child_frame_id):
        self.pub = pub
        self.parent_frame_id = parent_frame_id
        self.child_frame_id = child_frame_id

    def callback(self, data):
        if (data.header.frame_id == self.parent_frame_id and
                data.child_frame_id == self.child_frame_id):
            out = PoseWithCovarianceStamped()
            out.header = data.header
            out.pose.pose.position = data.transform.translation
            out.pose.pose.orientation = data.transform.rotation
            self.pub.publish(out)

if __name__=="__main__":
    rospy.init_node('transform2pose', anonymous=True)

    pub = rospy.Publisher("output_topic", PoseWithCovarianceStamped, queue_size=1)
    parent_frame_id = rospy.get_param('~parent_frame_id')
    child_frame_id = rospy.get_param('~child_frame_id')
    converter = Transform2Pose(pub, parent_frame_id, child_frame_id)
    rospy.Subscriber("input_topic", TransformStamped, converter.callback)

    rospy.spin()
