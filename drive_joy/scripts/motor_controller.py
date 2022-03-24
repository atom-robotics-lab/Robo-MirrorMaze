#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Joy

class Robot_Controller:
    #initialised values
    def __init__(self):

        rospy.init_node('controller')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/joy', Joy, self.Joy_callback)
        
        self.ya_axis = 0
        self.yb_axis = 0
        self.velocity_msg = Twist()
    
    def Joy_callback(self, data):
        axes = data.axes
        self.ya_axis = axes[1]
        self.yb_axis = axes[4]
        print("a = " + str(self.ya_axis) + "  " + "b = " + str(self.yb_axis))



if __name__ == "__main__":
    Robot = Robot_Controller()
    rospy.spin()