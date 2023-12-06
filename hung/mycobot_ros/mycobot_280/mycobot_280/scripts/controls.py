#!/usr/bin/env python3
# coding:utf-8
import rospy
import rosgraph
from rospy import ServiceException
import serial
from mycobot_communication.msg import MycobotSetAngles, MycobotSetCoords, MycobotState, MycobotAngles, MycobotCoords
from mycobot_communication.srv import ServoStatus

import os
import threading


class Controls:
    def __init__(self):
        print("FROM CONTROLS: ZHICHEN AND FRIENDS")
        # threading.Thread(rospy.init_node("controls", disable_signals=True, log_level=rospy.DEBUG)).start()

        if rosgraph.is_master_online():  # Checks the master uri and results boolean (True or False)
            print('ROS MASTER is Online')
        else:
            print('ROS MASTER is Offline')

        try:
            rospy.init_node("controls", disable_signals=True, log_level=rospy.DEBUG)
        # except Exception as error:
        #     print("An exception occurred:", type(error).__name__)
        except Exception:
            print("FROM CONTROLS: was error but couldn't print exception")

        print("Node Created")

        # Initialize publishers
        self.angles_publisher = rospy.Publisher("mycobot/angles_goal", MycobotSetAngles, queue_size=5)
        self.coords_publisher = rospy.Publisher("mycobot/coords_goal", MycobotSetCoords, queue_size=5)

        print("publisher done")

        # We use ROS Service to release all servos on the robot
        rospy.wait_for_service("release_servos")
        self.release_servos = rospy.ServiceProxy("release_servos", ServoStatus)

        print("servos released")

        # Subscriber to get the robot's state, which includes coords and angles
        rospy.Subscriber("mycobot/state", MycobotState, self._on_receive_state)

        print("Sets subscriber")

        # Instance variables used to keep track of the robot's state, coords, and angles
        self.state = MycobotState()
        self.angles = MycobotAngles()
        self.coords = MycobotCoords()

        print("Instance variables created")

    def _on_receive_state(self, data):
        """
        Upon receiving data from /mycobot/state, update the instance variables with the new state.
        This allows us to keep track of the most current state of the robot.
        """
        self.state = data
        self.angles = data.angles
        self.coords = data.coords

    def send_coords(self, coords, speed=70, model=2):
        """
        Send desired coordinates to the robot. These coordinates are published to the topic /mycobot/coords_goal.
        Upon receiving data from /mycobot/coords_goal, ROS sends these desired coordinates to the robot (through
        the normal MyCobot interface).

        :param: coords: List of desired coordinates. This should be in the form [x, y, z, rx, ry, rz].
        :param: speed: Desired speed for the robot's movements. Default is 70.
        :param: model: Controls the movement path of the robot arm head. Default is 2.
        """
        # Initialize new MycobotSetCoords message
        coords_msg = MycobotSetCoords()

        coords_msg.x = coords[0]
        coords_msg.y = coords[1]
        coords_msg.z = coords[2]
        coords_msg.rx = coords[3]
        coords_msg.ry = coords[4]
        coords_msg.rz = coords[5]

        coords_msg.speed = speed
        coords_msg.model = model

        # Publish the message to /mycobot/coords_goal
        self.coords_publisher.publish(coords_msg)
        print("Coords published \n")

    def send_angles(self, angles, speed=70):
        """
        Send desired joint angles to the robot. These angles are published to the topic /mycobot/angles_goal.
        Upon receiving data from /mycobot/angles_goal, ROS sends these desired joint angles to the robot (through
        the normal MyCobot interface).

        :param: angles: List of desired angles. This should be in the form
        [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6].
        :param: speed: Desired speed for the robot's movements. Default is 70.
        """
        # Initialize new MycobotSetAngles message
        angles_msg = MycobotSetAngles()

        angles_msg.joint_1 = float(angles[0])
        angles_msg.joint_2 = float(angles[1])
        angles_msg.joint_3 = float(angles[2])
        angles_msg.joint_4 = float(angles[3])
        angles_msg.joint_5 = float(angles[4])
        angles_msg.joint_6 = float(angles[5])

        angles_msg.speed = speed

        # Publish the message to /mycobot/angles_goal
        self.angles_publisher.publish(angles_msg)
        print("Angles published \n")

    def get_coords(self):
        """
        Get the current coordinates of the robot.

        :return: List of the robot's coordinates, in the form [x, y, z, rx, ry, rz].
        """
        return [
            self.coords.x, self.coords.y, self.coords.z,
            self.coords.rx, self.coords.ry, self.coords.rz
        ]

    def get_angles(self):
        """
        Get the current joint angles of the robot.

        :return: List of the robot's joint angles, in the form [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6].
        """
        return [
            self.angles.joint_1, self.angles.joint_2, self.angles.joint_3,
            self.angles.joint_4, self.angles.joint_5, self.angles.joint_6,
        ]

    def toggle_servo_released(self, status):
        """
        Release or lock all servos of the robot arm.
        - For releasing all servos, we send a ROS Service call to the service release_servos.
        Upon receiving the service call, ROS calls the release_all_servos function from the MyCobot interface.
        - For locking all servos, we set the current coordinates of the robot as the desired coordinates.
        This locks all servos without moving the arm.

        :param: status: True for releasing all servos. False for locking all servos.
        """
        # If True, calls the ROS Service
        if status:
            self.release_servos(True)

        # If False, send current coordinates as the desired coordinates.
        else:
            coords = [
                self.coords.x, self.coords.y, self.coords.z,
                self.coords.rx, self.coords.ry, self.coords.rz
            ]
            self.send_coords(coords)

    def reset_position(self):
        """
        Utility function to move the robot back to its upright position by setting all 0s as the desired joint angles.
        """
        self.send_angles([0, 0, 0, 0, 0, 0])

    def test_controls(self):
        """
        Test the controls system of the robot through a series of send_coords and send_angles commands.
        """
        for _ in range(10):
            # top left
            self.send_coords([249, 32, -12, 180, 1, -48], 70, 2)
            rospy.sleep(1)

            self.send_angles([0, 0, 0, 0, 0, 0], 70)
            rospy.sleep(3)

            # center
            self.send_coords([239, 4, -12, -177, 1, -54], 70, 2)
            rospy.sleep(1)

            self.send_angles([0, 0, 0, 0, 0, 0], 70)
            rospy.sleep(3)

            # bottom right
            self.send_coords([221, -23, -12, -180, 0, -59], 70, 2)
            rospy.sleep(1)

            self.send_angles([0, 0, 0, 0, 0, 0], 70)
            rospy.sleep(3)

        self.toggle_servo_released(True)


def main():
    # controls = Controls()
    # rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        # controls.test_controls()
        rospy.spin()


if __name__ == "__main__":
    main()
