#!/usr/bin/env python

import rospy
from math import tan, cos, sin
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from i2cpwm_board.msg import Servo, ServoArray

class SteeringAngle():
	def __init__(self):
		self.sub_servos = rospy.Subscriber("/servos_absolute", ServoArray, self.servo_callback)
		self.pub_steer = rospy.Publisher("/steering_angle", Float64, queue_size=10)
		self.steering_angle = 0
		self.steering_angle_rad = 0
		self.left_wheel_angle = 0
		self.right_wheel_angle = 0

	def servo_callback(self, msg):
		# y = mx +c 
		# linear regression based on emperical wheel angle, pwm measurements
		self.left_wheel_angle = (msg.servos[1].value * 0.28184) - 85.65100
		self.right_wheel_angle = (msg.servos[1].value * 0.22425) - 70.66250
		# Steering angle as average of left and right wheel regression and
		# overall steering map, resulting in a crudely filtered value. Needs to be refined
		self.steering_angle = (((msg.servos[1].value * 0.28515) -87.34828) + (self.left_wheel_angle + self.right_wheel_angle)/2)/2
		# Convert to radians, per ros convention
		self.steering_angle_rad = (self.steering_angle * 3.1415926) / 180
		self.pub_steer.publish(self.steering_angle_rad)


class OdometryPopulate():

	def __init__(self):
		self.pub_odometry = rospy.Publisher("/odom_dead_reckoning", Odometry, queue_size=10)
		self.steer = SteeringAngle()
		self.wheel_base = 0.175
		self.odom = Odometry()
		self.last_state = rospy.Time(0)
		self.yaw = 0
		self.x = 0
		self.y = 0
		self.hall_sub = rospy.Subscriber("/hall_state", Vector3, self.publish_odometry)

	def publish_odometry(self, msg):
		state_ = rospy.Time.now()
		angular_velocity_ = (msg.y * tan(self.steer.steering_angle_rad)) / self.wheel_base
		dt = state_ - self.last_state
		self.last_state = state_

		self.yaw += angular_velocity_ * dt.to_sec()

		x_dot_ = msg.y * cos(self.yaw)
		y_dot_ = msg.y * sin(self.yaw)
		self.x += x_dot_ * dt.to_sec()
		self.y += y_dot_ * dt.to_sec()

		# Pose
		self.odom.header.stamp =  state_
		self.odom.header.frame_id =  "base_link"
		self.odom.child_frame_id = "odom"
		self.odom.pose.pose.position.x = self.x
		self.odom.pose.pose.position.y = self.y
		# self.odom.pose.pose.orientation.x = 0.0
		# self.odom.pose.pose.orientation.y = 0.0
		self.odom.pose.pose.orientation.z = sin(self.yaw/2.0)
		self.odom.pose.pose.orientation.w = cos(self.yaw/2.0)

		# Covariance should be cacluated empirically
		self.odom.pose.covariance[0]  = 0.2 # x
		self.odom.pose.covariance[7]  = 0.2 # y
		self.odom.pose.covariance[35] = 0.4 # yaw

		# Velocity
		self.odom.twist.twist.linear.x = msg.y
		# self.odom.twist.twist.linear.y = 0.0
		self.odom.twist.twist.angular.z = angular_velocity_
		self.pub_odometry.publish(self.odom)

if __name__ == "__main__":
	rospy.init_node('dead_reckoning')
	OdometryPopulate()
	rospy.spin()