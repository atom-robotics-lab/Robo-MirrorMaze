#! /usr/bin/env python

import rospy
import math
from gpiozero import AngularServo
from sensor_msgs.msg import LaserScan
from time import sleep


class SonarDriver:
    
    def __init__(self):
        rospy.init_node("sonar_driver")

        self.sonar_pub = rospy.Publisher("/sonar/scan", LaserScan, queue_size=10)
        self.scan_servo = AngularServo(25, min_angle=0, max_angle=180, min_pulse_width=0.0004, max_pulse_width=0.0025)

        self.angle_min = math.radians(0)
        self.angle_max = math.radians(360)

        # angular increment is set to 1 degree
        self.angular_increment = math.radians(1)
        
        self.time_increment = 0.0
        self.scan_time = 0.0
        self.range_min = 0.02
        self.angle_max = 4

    def scan360(self):
        try:
            rospy.loginfo(".... STARTING SCAN ....")

            angle = 0

            while True:
                rospy.sleep(0.1)
                self.scan_servo.angle = angle 

                # do stuff
                rospy.loginfo("Aft angle: " + str(angle))

                angle = angle + 1
                if angle > 180:
                    angle = 0

        except KeyboardInterrupt:
            rospy.logerr("STOPPED SCAN")


if __name__ == "__main__":
    sonar_driver = SonarDriver()

    rospy.loginfo("Starting Scan")
    sonar_driver.scan360()

    rospy.spin()


