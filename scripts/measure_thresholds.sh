#!/usr/bin/env python

import roslib
import rospy
import math
from sensor_msgs.msg import Imu

class MeasureThresholds():

    def __init__(self):
        rospy.init_node('measure_thresholds', anonymous=False)
        rospy.wait_for_message('/imu/data', Imu)
        rospy.Subscriber('/imu/data', Imu, self.sub_imu)
        rospy.loginfo("listening on /imu/data")

	global kAccelerationThreshold
        kAccelerationThreshold = 0.19
	global kAngularVelocityThreshold
	kAngularVelocityThreshold = 0.055
	global kDeltaAngularVelocityThreshold
	kDeltaAngularVelocityThreshold = 0.033

	global wx
	global wy
	global wz
	global wx_prev
	global wy_prev
	global wz_prev

	wz = 0
	wy = 0
	wz = 0
	wx_prev = 0
	wy_prev = 0
	wz_prev = 0

    def sub_imu(self, msg):

	global wx
	global wy
	global wz
	global wx_prev
	global wy_prev
	global wz_prev

	ax = msg.linear_acceleration.x
	ay = msg.linear_acceleration.y
	az = msg.linear_acceleration.z
	wx = msg.angular_velocity.x
	wy = msg.angular_velocity.y
	wz = msg.angular_velocity.z

	acc_magnitude = math.sqrt(ax*ax + ay*ay + az*az)
	delta = math.fabs(acc_magnitude - 9.80665)

  	if(delta > kAccelerationThreshold): print('acc_delta:',delta)

	delta_x = wx - math.fabs(wx_prev)
	delta_y = wy - math.fabs(wy_prev)
	delta_z = wz - math.fabs(wz_prev)

  	if(delta_x > kDeltaAngularVelocityThreshold or delta_y > kDeltaAngularVelocityThreshold or delta_z > kDeltaAngularVelocityThreshold): print('angular_delta', delta_x, delta_y, delta_z)
        if(math.fabs(wx) > kAngularVelocityThreshold or math.fabs(wy) > kAngularVelocityThreshold or math.fabs(wz) > kAngularVelocityThreshold): print('angular:', wx, wy, wz)

	wx_prev = wx
	wy_prev = wy
	wz_prev = wz

if __name__ == '__main__':
    try:
        MeasureThresholds()
        rospy.spin()
    except:
        pass
