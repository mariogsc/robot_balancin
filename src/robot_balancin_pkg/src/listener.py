#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

def imu_callback(msg):
    (roll, pitch, yaw)= euler_from_quaternion([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
    rospy.loginfo("roll= %f, pitch= %f, yaw=%f \n",roll,pitch,yaw)


if __name__ == '__main__':
    rospy.init_node("imu_subscriber")

    sub = rospy.Subscriber("imu/imu_sensor",Imu,callback=imu_callback)

    rospy.loginfo("Imu listener node has been created")

    rospy.spin()