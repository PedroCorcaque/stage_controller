#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import numpy as np

class MonteCarloLocalization():
	
	def __init__(self) -> None:

		self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

		rospy.Subscriber("/base_pose_ground_truth", Odometry, self.odometry_callback)
		rospy.Subscriber("/base_scan", LaserScan, self.laser_scan_callback)

		self.num_particles = 100
		self.motion_noise = 0.1
		self.scan_noise = 0.05

		self.particles = np.random.rand(self.num_particles, 3)  # [x, y, theta]
		self.weights = np.ones(self.num_particles) / self.num_particles

		self.scan = None
		self.estimated_pose = None

		self.robot_position = np.zeros(3)  						# [x, y, theta]
		self.target_position = np.array([0.0, 3.0])  			# [x, y]

		self.linear_velocity = 0.1  							# Constant linear velocity
		self.angular_velocity = 0.5  							# Constant angular velocity
		self.kp_linear = 0.2  									# Proportional gain for linear velocity control
		self.kp_angular = 1.0  									# Proportional gain for angular velocity control
		self.min_distance = 0.5  								# Minimum distance threshold for obstacle avoidance

	def laser_scan_callback(self, msg):
		""" Callback function for laser scan """
		self.scan = np.array(msg.ranges)
		self.measurement_update()
		self.resampling()
		self.estimated_pose = np.average(self.particles[:, :2], axis=0, weights=self.weights)
		self.move_to_target()

	def odometry_callback(self, msg):
		""" Callback function for odometry """
		self.robot_position[0] = msg.pose.pose.position.x
		self.robot_position[1] = msg.pose.pose.position.y
		self.robot_position[2] = 2 * np.arcsin(msg.pose.pose.orientation.z)
		self.motion_update()

	def motion_update(self):
		""" Motion update """
		delta_x = self.robot_position[0]
		delta_y = self.robot_position[1]
		delta_theta = self.robot_position[2]

		self.particles[:, 0] += delta_x + np.random.randn(self.num_particles) * self.motion_noise
		self.particles[:, 1] += delta_y + np.random.randn(self.num_particles) * self.motion_noise
		self.particles[:, 2] += delta_theta + np.random.randn(self.num_particles) * self.motion_noise

	def measurement_update(self):
		""" Measurement update """
		for i in range(self.num_particles):
			particle_scan = self.scan  # Use actual laser scan data directly

			diff = self.scan - particle_scan
			self.weights[i] = np.exp(-np.sum(diff**2) / (2 * self.scan_noise**2))

		self.weights /= np.sum(self.weights)  # Normalize weights

	def resampling(self):
		""" Resampling """
		indices = np.random.choice(range(self.num_particles), size=self.num_particles, p=self.weights)
		self.particles[:] = self.particles[indices]
		self.weights.fill(1.0 / self.num_particles)

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
    robot = MonteCarloLocalization()
    rospy.spin()
