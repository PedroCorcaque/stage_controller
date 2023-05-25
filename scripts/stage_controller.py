#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

MIN_DISTANCE = 0.1
SECURE_DISTANCE = 0.3
TARGET = [3.0, 3.0]

class StageController():
    """ The StageController class is used to control the robot in the Stage Simulator """

    def __init__(self, target_position) -> None:
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.Subscriber("/base_pose_ground_truth",
                         Odometry, self.odometry_callback)
        rospy.Subscriber("/base_scan", LaserScan, self.laser_scan_callback)

        self.scan = None

        self.robot_position = np.zeros(3)
        self.target_position = np.array(target_position)

        self.linear_velocity = 0.1
        self.angular_velocity = 0.5
        self.kp_linear = 0.2
        self.kp_angular = 1.0
        self.min_distance = 0.5

    def laser_scan_callback(self, msg) -> None:
        """ Callback function for laser scan """
        self.scan = np.array(msg.ranges)
        self.move_to_target()

    def odometry_callback(self, msg) -> None:
        """ Callback function for odometry """
        self.robot_position[0] = msg.pose.pose.position.x
        self.robot_position[1] = msg.pose.pose.position.y
        self.robot_position[2] = 2 * np.arcsin(msg.pose.pose.orientation.z)

    def move_forward(self, direction, distance) -> None:
        """ Move the robot to forward """
        direction /= distance

        target_angle = np.arctan2(direction[1], direction[0])
        current_angle = self.robot_position[2]
        angle_diff = target_angle - current_angle
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
        linear_vel = self.kp_linear * distance
        angular_vel = self.kp_angular * angle_diff
        self.publish_cmd(linear_vel, angular_vel)
        rospy.loginfo("Navegando...")

    def rotate(self) -> None:
        """ Rotate to avoid the obstacle """
        linear_vel = 0.0
        angular_vel = 5.0
        self.publish_cmd(linear_vel, angular_vel)
        rospy.loginfo("Desviando de um obstáculo...")

    def move_to_target(self) -> None:
        """ Move to target position """
        direction = self.target_position - self.robot_position[:2]
        distance = np.linalg.norm(direction)

        if distance <= MIN_DISTANCE:
            rospy.loginfo("Target alcançado!!")
            self.stop_robot()
        else:
            self.move_forward(direction, distance)

            if len(self.scan) > 0:
                if min(self.scan[45*4:225*4]) < SECURE_DISTANCE:
                    self.rotate()
                else:
                    self.move_forward(direction, distance)

    def publish_cmd(self, linear_vel, angular_vel):
        """ Publish Twist command """
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.publisher.publish(twist)

    def stop_robot(self):
        """ Stop the robot """
        self.publish_cmd(0.0, 0.0)

if __name__ == "__main__":
    rospy.init_node("stage_controller")
    _ = StageController(TARGET)
    rospy.spin()
