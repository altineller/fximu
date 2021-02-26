#!/usr/bin/env python3

import roslib
import rospy
import os, pty, serial
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Int16MultiArray

# TODO: move to fximu package

ser = serial.Serial('/dev/ttyCAL0', 115200, rtscts=True, dsrdtr=True)

def callback(msg):
    ax = msg.data[0]
    ay = msg.data[1]
    az = msg.data[2]
    gx = msg.data[3]
    gy = msg.data[4]
    gz = msg.data[5]
    mx = msg.data[6]
    my = msg.data[7]
    mz = msg.data[8]
    raw_str = 'Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n' % (ax, ay, az, gx, gy, gz, mx, my, mz)
    ser.write(str.encode(raw_str))

def cal_bridge():
    rospy.init_node('cal_bridge', anonymous=True)
    rospy.Subscriber("/imu/array", Int16MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    cal_bridge()

