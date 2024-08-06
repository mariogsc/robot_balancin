#!/usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from simple_pid import PID

# Constantes PID
Kp = 0.15
Ki = 0.005
Kd = 0.0

cmd_vel_topic = "cmd_vel"
imu_topic = "imu/imu_sensor"

class SelfBalance:
    def __init__(self):
        self.pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.subscriber = rospy.Subscriber(imu_topic, Imu, self.callback)
        self.yPrev = 0

        self.pid_controller = PID(Kp, Ki, Kd, setpoint=0)
        self.pid_controller.output_limits = (-26, 26)  # Limitar la salida del PID

    def callback(self, data):
        orientation_q = data.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, pitch, _ = euler_from_quaternion(orientation_list)

        error=0-pitch
        correction = self.pid_controller(error)

        vel = Twist()
        vel.linear.x = correction
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0

        self.pub.publish(vel)
        rospy.loginfo("pitch: %f, accion de control: %f",pitch,correction)
        self.yPrev = pitch

def main(args):
    rospy.init_node('SelfBalance', anonymous=True)
    rospy.loginfo("Imu listener node has been created")
    self_balance = SelfBalance()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS")

if __name__ == '__main__':
    main(sys.argv)
