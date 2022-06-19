#!/usr/bin/env python3

import rospy
import math
import numpy as np
from sensor_msgs.msg import Imu

AVG_ARRAY_SIZE = 256

class MeasureThresholds:

    def __init__(self):

        rospy.init_node('measure_thresholds', anonymous=False)
        rospy.wait_for_message('/imu/data', Imu)
        rospy.Subscriber('/imu/data', Imu, self.sub_imu)
        rospy.loginfo('listening on /imu/data')

        # init arrays with 0
        self.accel_vector = np.zeros(AVG_ARRAY_SIZE)
        self.angular_vector = np.zeros(AVG_ARRAY_SIZE * 3)
        self.delta_vector = np.zeros(AVG_ARRAY_SIZE + 3)

        # fill accel_vector with g
        self.accel_vector.fill(9.80665)

        self.wx_prev = 0
        self.wy_prev = 0
        self.wz_prev = 0

        self.vector_index = 0


    def sub_imu(self, msg):

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        # g vector
        acc_magnitude = math.sqrt(ax * ax + ay * ay + az * az)

        self.accel_vector[self.vector_index] = acc_magnitude

        self.angular_vector[self.vector_index + 0] = wx
        self.angular_vector[self.vector_index + 1] = wy
        self.angular_vector[self.vector_index + 2] = wz

        delta_x = wx - math.fabs(self.wx_prev)
        delta_y = wy - math.fabs(self.wy_prev)
        delta_z = wz - math.fabs(self.wz_prev)

        self.delta_vector[self.vector_index + 0] = delta_x
        self.delta_vector[self.vector_index + 1] = delta_y
        self.delta_vector[self.vector_index + 2] = delta_z

        # average acceleration vector
        steady_gravity = np.average(self.accel_vector)

        # min / max acceleration
        min_accel = np.min(self.accel_vector)
        max_accel = np.max(self.accel_vector)

        # high / low limit on acceleration
        high_limit = max_accel - steady_gravity
        low_limit = steady_gravity - min_accel

        # select the higher of the values
        k_acceleration_threshold = max(high_limit, low_limit)

        # calculate angular vector average, find min and max
        avg_angular = np.average(self.angular_vector)
        min_angular = math.fabs(np.min(self.angular_vector))
        max_angular = math.fabs(np.max(self.angular_vector))

        # get the larger of thresholds
        k_angular_velocity_threshold = max(max_angular, min_angular)

        # calculate angular delta average, find min and max
        avg_delta = np.average(self.delta_vector)
        min_delta = math.fabs(np.min(self.delta_vector))
        max_delta = math.fabs(np.max(self.delta_vector))

        # get the larger of the thresholds
        k_delta_angular_threshold = max(max_delta, min_delta)

        rospy.loginfo('angularVelocityThreshold: %f, accelerationThreshold: %f, steadyGravity: %f, deltaAngularThreshold: %f', k_angular_velocity_threshold, k_acceleration_threshold, steady_gravity, k_delta_angular_threshold)

        self.wx_prev = wx
        self.wy_prev = wy
        self.wz_prev = wz

        self.vector_index += 1
        self.vector_index = self.vector_index % AVG_ARRAY_SIZE

if __name__ == '__main__':
    try:
        MeasureThresholds()
        rospy.spin()
    except:
        pass
