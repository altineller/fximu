#!/usr/bin/env python

import roslib
import rospy
import math
from sensor_msgs.msg import Imu

class MeasureGravity():

    def __init__(self):
        rospy.init_node('measure_gravity', anonymous=False)
        rospy.wait_for_message('/imu/data', Imu)
        rospy.Subscriber('/imu/data', Imu, self.sub_imu)
        rospy.loginfo("listening on /imu/data")

    	global counter
    	counter = 0

	global data
	data = [0.0] * 128

    def sub_imu(self, msg):
	x = msg.linear_acceleration.x
	y = msg.linear_acceleration.y
	z = msg.linear_acceleration.z
	acc_magnitude = math.sqrt(x*x + y*y + z*z)
	global counter
	counter += 1
	global data
	data[counter%128] = acc_magnitude
	gravity = sum(data)/128
	print(gravity)
if __name__ == '__main__':
    try:
        MeasureGravity()
        rospy.spin()
    except:
        pass
