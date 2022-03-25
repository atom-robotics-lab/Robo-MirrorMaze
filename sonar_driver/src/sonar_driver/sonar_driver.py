#! /usr/bin/env python

import rospy
import math
from gpiozero import AngularServo, DistanceSensor
from sensor_msgs.msg import LaserScan
from time import sleep


class SonarDriver:
    
    def __init__(self):
        rospy.init_node("sonar_driver")

        self.sonar_pub = rospy.Publisher("/sonar/scan", LaserScan, queue_size=10)
        self.scan_servo = AngularServo(25, min_angle=0, max_angle=180, min_pulse_width=0.0004, max_pulse_width=0.0025)
        
        # ultrasonic dist sensors (echo pin, trig pin)
        self.front_dist_sensor = DistanceSensor(26, 19)
        self.back_dist_sensor = DistanceSensor(13, 6)

        self.front_ranges = []
        self.back_ranges = []

        # sonar scanner params
        self.angle_min = math.radians(0)
        self.angle_max = math.radians(360)

        # angular increment is set to 1 degree
        self.angular_increment = math.radians(1)
        
        self.time_increment = 0.0
        self.scan_time = 0.0
        self.range_min = 0.02
        self.range_max = 4

    def publish_scan_data(self):
        scan_msg = LaserScan()

        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angular_increment = self.angular_increment

        scan_msg.time_increment = self.time_increment
        scan_msg.scan_time = self.scan_time

        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max

        scan_msg.ranges = self.front_ranges + self.back_ranges
        scan_msg.intensities = []

        self.sonar_pub.publish(scan_msg)


    def scan360(self):
        try:
            rospy.loginfo(".... STARTING SCAN ....")

            angle = 0

            while True:
                rospy.sleep(0.1)
                self.scan_servo.angle = angle 

                self.front_ranges.append(self.front_dist_sensor.distance)
                self.back_ranges.append(self.back_dist_sensor.distance)

                rospy.loginfo("Aft angle: " + str(angle))

                angle = angle + 1

                if angle > 180:
                    self.publish_scan_data()
                    angle = 0

        except KeyboardInterrupt:
            rospy.logerr("STOPPED SCAN")


if __name__ == "__main__":
    sonar_driver = SonarDriver()

    rospy.loginfo("Starting Scan")
    sonar_driver.scan360()

    rospy.spin()


