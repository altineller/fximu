#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import MagneticField


class MeasureMagnetic():

    def __init__(self):
        rospy.init_node('measure_magnetic', anonymous=False)
        rospy.wait_for_message('/imu/mag', MagneticField)
        rospy.Subscriber('/imu/mag', MagneticField, self.sub_mag)
        rospy.loginfo('listening on /imu/mag')

        self.magnetic = None

    def sub_mag(self, msg):
        x = msg.magnetic_field.x
        y = msg.magnetic_field.y
        z = msg.magnetic_field.z
        mag_magnitude = math.sqrt(x*x + y*y + z*z)
        if self.magnetic is not None:
            self.magnetic = (mag_magnitude * 0.01) + (self.magnetic * 0.99)
        else:
            self.magnetic = mag_magnitude

        rospy.loginfo(self.magnetic)


if __name__ == '__main__':
    try:
        MeasureMagnetic()
        rospy.spin()
    except:
        pass
