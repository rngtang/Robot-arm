#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from controls import Controls
from tf import TransformListener
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from mycobot_communication.msg import MycobotState


class HandTracker:
    def __init__(self) -> None:
        rospy.init_node("hand_tracker")

        self.listener = TransformListener()
        self.listener.waitForTransform('odom', 'base_link', rospy.Time(), rospy.Duration(10))

        self.controls = Controls()

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
        pose_stamped.header.frame_id = "base_link"

        target_transformed_pose = self.listener.transformPose("odom", pose_stamped).pose

        quaternion_list = [target_transformed_pose.orientation.x, target_transformed_pose.orientation.y,
                           target_transformed_pose.orientation.z, target_transformed_pose.orientation.w]
        target_coords = [target_transformed_pose.position.x,
                         target_transformed_pose.position.y,
                         target_transformed_pose.position.z] + euler_from_quaternion(quaternion_list)

        self.controls.send_coords(target_coords)


def main():
    hand_tracker = HandTracker()


if __name__ == '__main__':
    main()
