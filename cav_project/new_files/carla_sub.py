#shebang
#PUT ON CARLA MACHINE
import rospy
import time
import math
from cav_project.msg import ControlInfo #message file

class CarlaSub:

    def __init__(self):
        rospy.Subscriber("m_to_c", ControlInfo, self.callback)

    def callback(self, data):
        self.data = data
        self.local_x = data.desired_velocity
        self.local_y = data.control_input
        self.local_angle = data.steering_angle
        self.carla_teleport(self.local_x, self.local_y, self.local_angle)

    def carla_teleport(self, local_x, local_y, local_angle):
        #CODE TO USE VALUES TO TELEPORT CARLA