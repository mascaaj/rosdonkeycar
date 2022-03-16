#!/usr/bin/python

import rospy
import pigpio
import hall_rpm as hr
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from i2cpwm_board.msg import Servo, ServoArray

class HallSensorOdometry:

	def __init__(self, gpio_number, pulses_per_rev):
		self.pi = pigpio.pi()
		self.hall_sensor = hr.HallRPM(self.pi, gpio_number, pulses_per_rev)
		self.hall_state = rospy.Publisher("/hall_state", Vector3, queue_size=10)
		self.state_msg = Vector3()

	def publish_data(self):
		self.hall_sensor.RPM()
		# print(self.hall_sensor.rpm,self.hall_sensor.mps,self.hall_sensor.distance_travelled)
		self.state_msg.x = self.hall_sensor.rpm
		self.state_msg.y = self.hall_sensor.mps
		self.state_msg.z = self.hall_sensor.distance_travelled
		self.hall_state.publish(self.state_msg)

	def destroy_instances(self):
		print("Shutting down")
		self.hall_sensor.cancel()
		self.pi.stop()

if __name__ == "__main__":
	rospy.init_node("hall_sensor")
	rate = rospy.Rate(10)
	hs = HallSensorOdometry(12,2)
	while not rospy.is_shutdown():
		hs.publish_data()
		rate.sleep()
	hs.destroy_instances()
