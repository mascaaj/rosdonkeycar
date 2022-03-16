#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from tfmini_pigpio_scanner import *
import math
import time

class TfminiLaserscanPublisher():

    def __init__(self, gpio=18, 
                servo_angle_min=math.radians(-85), 
                servo_angle_max=math.radians(85), 
                duty_cycle_min=500,
                duty_cycle_max=2500, 
                laser_samples=50, 
                scan_time_minmax=0.7):

        self.scan_pub = rospy.Publisher('tfmini_laser', LaserScan, queue_size=1)
        self.tfminiscanner = TfminiServoScanner(gpio, servo_angle_min, servo_angle_max, 
                                    duty_cycle_min, duty_cycle_max, laser_samples, 
                                    scan_time_minmax)
        self.scan_time = 0.74*2
        self.scan = LaserScan()
        frame_id = rospy.get_param('~frame_id', '/tfmini')
        self.scan.header.frame_id   = frame_id
        self.scan.range_min         = self.tfminiscanner.laser.distance_min*0.01
        self.scan.range_max         = self.tfminiscanner.laser.distance_max*0.01
        self.tfminiscanner.reset_servo()


    def publish_laserscan(self):
        self.scan.header.stamp =  rospy.Time.now()
        self.scan.scan_time = self.scan_time
        ini_angle, end_angle, time_increment, angle_increment, ranges = self.tfminiscanner.scan(scale_factor=0.01, reset=True)
        self.scan.angle_min = ini_angle
        self.scan.angle_max = end_angle
        self.scan.angle_increment = angle_increment
        self.scan.time_increment = time_increment
        self.scan.ranges = ranges
        self.scan_pub.publish(self.scan)
        self.scan_time = (rospy.Time.now() - self.scan.header.stamp).to_sec()


if __name__ == "__main__":
    rospy.init_node("tfmini_laserscan")
    tfm = TfminiLaserscanPublisher()
    while not rospy.is_shutdown():
        tfm.publish_laserscan()