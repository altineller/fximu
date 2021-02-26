#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Imu

class MeasureThresholds():

    def __init__(self):
        rospy.init_node('measure_thresholds', anonymous=False)
        rospy.wait_for_message('/imu/data', Imu)
        rospy.Subscriber('/imu/data', Imu, self.sub_imu)
        rospy.loginfo('listening on /imu/data')

        self.kAccelerationThreshold = 0.19
        self.kAngularVelocityThreshold = 0.055
        self.kDeltaAngularVelocityThreshold = 0.033

        self.wx_prev = 0
        self.wy_prev = 0
        self.wz_prev = 0

    def sub_imu(self, msg):

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        acc_magnitude = math.sqrt(ax*ax + ay*ay + az*az)
        acc_minus_g = math.fabs(acc_magnitude - 9.80665)

        if acc_minus_g > self.kAccelerationThreshold:
            rospy.loginfo('kAccelerationThreshold: %s', acc_minus_g)

        delta_x = wx - math.fabs(self.wx_prev)
        delta_y = wy - math.fabs(self.wy_prev)
        delta_z = wz - math.fabs(self.wz_prev)

        if delta_x > self.kDeltaAngularVelocityThreshold or delta_y > self.kDeltaAngularVelocityThreshold or delta_z > self.kDeltaAngularVelocityThreshold:
            rospy.loginfo('kDeltaAngularVelocityThreshold: %s, %s, %s', delta_x, delta_y, delta_z)

        if math.fabs(wx) > self.kAngularVelocityThreshold or math.fabs(wy) > self.kAngularVelocityThreshold or math.fabs(wz) > self.kAngularVelocityThreshold:
            rospy.loginfo('kAngularVelocityThreshold: %s, %s, %s', wx, wy, wz)

        self.wx_prev = wx
        self.wy_prev = wy
        self.wz_prev = wz


if __name__ == '__main__':
    try:
        MeasureThresholds()
        rospy.spin()
    except:
        pass
