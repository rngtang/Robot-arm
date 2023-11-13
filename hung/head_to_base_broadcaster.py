#!/usr/bin/env python2
import rospy
import tf
from tf.transformations import quaternion_from_euler
from mycobot_communication.msg import MycobotState


class HeadToBaseBroadcaster:
    def __init__(self):
        rospy.init_node('head_to_base_broadcaster')
        self.broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber("/mycobot/state", MycobotState, self._on_receive_state)

    def _on_receive_state(self, data):
        coords = (data.coords.x, data.coords.y, data.coords.z)
        q = quaternion_from_euler(data.coords.rx, data.coords.ry, data.coords.rz)
        self.broadcaster.sendTransform(coords, q, rospy.Time.now(), 'head', 'base')


if __name__ == '__main__':
    broadcaster = HeadToBaseBroadcaster()
    while not rospy.is_shutdown():
        rospy.spin()
