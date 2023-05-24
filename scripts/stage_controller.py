#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class StageController():
	
	def __init__(self) -> None:

		self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

		rospy.Subscriber("/base_pose_ground_truth", Odometry, self.odometry_callback)
		rospy.Subscriber("/base_scan", LaserScan, self.laser_scan_callback)

		self.scan = None

		self.robot_position = np.zeros(3)  						
		self.target_position = np.array([3.0, 3.0])  			

		self.linear_velocity = 0.1  							
		self.angular_velocity = 0.5  							
		self.kp_linear = 0.2  									
		self.kp_angular = 1.0  									
		self.min_distance = 0.5  								

	def laser_scan_callback(self, msg):
		""" Callback function for laser scan """
		self.scan = np.array(msg.ranges)
		self.move_to_target()

	def odometry_callback(self, msg):
		""" Callback function for odometry """
		self.robot_position[0] = msg.pose.pose.position.x
		self.robot_position[1] = msg.pose.pose.position.y
		self.robot_position[2] = 2 * np.arcsin(msg.pose.pose.orientation.z)

	def move_to_target(self):
		""" Move to target position """
		direction = self.target_position - self.robot_position[:2]
		distance = np.linalg.norm(direction)

		if distance <= 0.1:
			rospy.loginfo("The robot has reached the target!")
			self.stop_robot()
		else:
			direction /= distance

			target_angle = np.arctan2(direction[1], direction[0])
			current_angle = self.robot_position[2]

			angle_diff = target_angle - current_angle
			angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))

			linear_vel = self.kp_linear * distance
			angular_vel = self.kp_angular * angle_diff
			self.publish_cmd(linear_vel, angular_vel)
	
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
    _ = StageController()
    rospy.spin()
