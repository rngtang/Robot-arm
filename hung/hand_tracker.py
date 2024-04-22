#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseStamped
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from mycobot_communication.msg import MycobotState, MycobotSetCoords

class HandTracker:
    def __init__(self):
        rospy.init_node("hand_tracker")

        self.listener = tf.TransformListener()
        self.listener.waitForTransform('head', 'base', rospy.Time(), rospy.Duration(10))

        self.coords_publisher = rospy.Publisher("/mycobot/coords_goal", MycobotSetCoords, queue_size=5)

        self.state = MycobotState()
        rospy.Subscriber("/mycobot/state", MycobotState, self._on_receive_state)

    def _on_receive_state(self, data):
        """
        Upon receiving data from /mycobot/state, update the instance variables with the new state.
        This allows us to keep track of the most current state of the robot.
        """
        self.state = data

    def transform_and_publish_target(self, target_local):
        target_local_pose = Pose()

        target_local_pose.position.x = target_local[0]
        target_local_pose.position.y = target_local[1]
        target_local_pose.position.z = target_local[2]

        q = quaternion_from_euler(target_local[3], target_local[4], target_local[5])
        target_local_pose.orientation.x = q[0]
        target_local_pose.orientation.y = q[1]
        target_local_pose.orientation.z = q[2]
        target_local_pose.orientation.w = q[3]

        pose_stamped = PoseStamped()
        pose_stamped.pose = target_local_pose
        pose_stamped.header.frame_id = "base"

        target_transformed_pose = self.listener.transformPose("head", pose_stamped).pose

        quaternion_list = [target_transformed_pose.orientation.x, target_transformed_pose.orientation.y,
                           target_transformed_pose.orientation.z, target_transformed_pose.orientation.w]
        target_coords = [target_transformed_pose.position.x,
                         target_transformed_pose.position.y,
                         target_transformed_pose.position.z] + list(euler_from_quaternion(quaternion_list))

        coords_msg = MycobotSetCoords()

        coords_msg.x = target_coords[0]
        coords_msg.y = target_coords[1]
        coords_msg.z = target_coords[2]
        coords_msg.rx = target_coords[3]
        coords_msg.ry = target_coords[4]
        coords_msg.rz = target_coords[5]

        coords_msg.speed = 70
        coords_msg.model = 2

        # Publish the message to /mycobot/coords_goal
        self.coords_publisher.publish(coords_msg)


def main():
    hand_tracker = HandTracker()

    while not rospy.is_shutdown():
        hand_tracker.transform_and_publish_target([10, 0, 0, 0, 0, 0])
        rospy.sleep(2)


if __name__ == '__main__':
    main()
