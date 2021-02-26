#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Imu


class MeasureGravity():

    def __init__(self):
        rospy.init_node('measure_gravity', anonymous=False)
        rospy.wait_for_message('/imu/data', Imu)
        rospy.Subscriber('/imu/data', Imu, self.sub_imu)
        rospy.loginfo('listening on /imu/data')

        self.gravity = None

    def sub_imu(self, msg):
        x = msg.linear_acceleration.x
        y = msg.linear_acceleration.y
        z = msg.linear_acceleration.z
        acc_magnitude = math.sqrt(x*x + y*y + z*z)
        if self.gravity is not None:
            self.gravity = (acc_magnitude * 0.001) + (self.gravity * 0.999)
        else:
            self.gravity = acc_magnitude

        rospy.loginfo(self.gravity)


if __name__ == '__main__':
    try:
        MeasureGravity()
        rospy.spin()
    except:
        pass
