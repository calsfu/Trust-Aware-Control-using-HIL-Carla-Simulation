#!/usr/bin/env python3
#PUT ON CARLA MACHINE
import rospy
# from pylimo import limo
import time
import numpy as np
from cav_project.msg import ControlInfo


class CarlaLimo:
	def __init__(self):
		rospy.init_node("carla_publisher")
		self.control_info_topic = rospy.Publisher("control_info_", ControlInfo, queue_size=10)
		self.rate = rospy.Rate(10)
		print("CARLA publisher created.")


def msg_format(): #add input from Carla
	msgCI = ControlInfo()
	#GET CARLA VALUES HERE
	msgCI.steering_angle = 1 #eventually from Carla
	msgCI.desired_velocity = 1 #eventually from Carla
	msgCI.control_input = 1 #eventually from Carla
	return msgCI

if __name__ == '__main__':
	carla_limo = CarlaLimo()
	while not rospy.is_shutdown():
		msgCI = msg_format()
		rospy.loginfo(msgCI)
		carla_limo.control_info_topic.publish(msgCI)
		print("published")
		time.sleep(0.1)
	print("publishing ended")
