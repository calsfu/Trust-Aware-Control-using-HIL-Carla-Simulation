#!/usr/bin/env python3
#PUT ON LIMO
import rospy
# from geometry_msgs.msg import PoseStamped
# from nav_msgs.msg import Odometry
# from std_msgs.msg import String
from pylimo import limo
# from ackermann_msgs.msg import AckermannDrive
import time
# import numpy as np
import pickle
from cav_project.msg import limo_info, limo_info_array, ControlInfo, QP_solution


class LimoInfoPublisher:
    def __init__(self):

        rospy.init_node('limo_info_publisher_')
        self.info_pub_cav1 = rospy.Publisher('/limo_info_', limo_info, queue_size=10)
        self.control_info_sub_cav1 = rospy.Subscriber("control_info_", ControlInfo, self.control_info_callback)

        self.limo_data_cav1 = limo_info()
        self.rate = rospy.Rate(10)

    def control_info_callback(self, data):
        self.control_info_cav1 = data

    def publish_info(self):
        self.info_pub_cav1.publish(self.limo_data_cav1)

    #def odom_callback_cav1(self, msg):
    #    self.limo_data_cav1.vel.data = msg.twist.twist.linear.x
    #    self.publish_info()

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
    publisher = LimoInfoPublisher()
    limo= limo.LIMO()
    limo.EnableCommand()
    limo.SetMotionCommand(linear_vel=0, steering_angle=0)
    time.sleep(1)

    iter = 0
    while True:
        if publisher.control_info_cav1.desired_velocity is not None:
            #listener_ins.data.speed
            #listener to message and set velocity and steering
            limo.SetMotionCommand(linear_vel=publisher.control_info_cav1.desired_velocity, steering_angle=publisher.control_info_cav1.steering_angle)
            print("Limo velocity command:", publisher.control_info_cav1.desired_velocity, "Limo steering:", publisher.control_info_cav1.steering_angle)
            actual_velocity = limo.GetLinearVelocity()
            print("actual_velocity", actual_velocity)
            publisher.limo_data_cav1.vel.data = actual_velocity
            publisher.publish_info()
            iter += 1
            time.sleep(0.1)
        else:
            print("[WARNING] Skipping ros messages...")
            if iter>10:
                break
    publisher.publish_info()
