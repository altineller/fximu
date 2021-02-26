#!/usr/bin/env python

import roslib
import rospy
import math
from std_msgs.msg import Int16MultiArray

class MeasureGravity():

    def __init__(self):
        rospy.init_node('measure_gravity', anonymous=False)
        rospy.wait_for_message('/imu/array', Int16MultiArray)
        rospy.Subscriber('/imu/array', Int16MultiArray, self.sub_imu)
        rospy.loginfo("listening on /imu/array")

    def sub_imu(self, msg):
	x = msg.data[0]
	y = msg.data[1]
	z = msg.data[2]
	acc_magnitude = math.sqrt(x*x + y*y + z*z)
	print(acc_magnitude)
if __name__ == '__main__':
    try:
        MeasureGravity()
        rospy.spin()
    except:
        pass
