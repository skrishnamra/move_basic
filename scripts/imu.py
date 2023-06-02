#!/usr/bin/env python3
import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import Imu    
import numpy as np

def main():
    rospy.Subscriber("/mobile_imu_control/imu", Imu, imu_cb)
    while not rospy.is_shutdown():
        rospy.spin()

def imu_cb(msg):
    _,_,yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    rospy.loginfo(np.rad2deg(yaw))


if __name__ == "__main__":
    rospy.init_node("imu")
    main()