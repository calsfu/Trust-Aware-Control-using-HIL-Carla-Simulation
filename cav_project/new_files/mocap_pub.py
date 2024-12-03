#shebang
#PUT ON MOCAP MACHINE

import rospy
import time
import math
from cav_project.msg import ControlInfo #message file


class MocapProxy:

    def __init__(self):
        rospy.init_node('mocap_to_carla')
        self.mocap_info = rospy.Publisher("m_to_c", ControlInfo, queue_size=10)
        self.rate=rospy.Rate(10)

def msg_format():
    msgCarla = ControlInfo()
    #QUERY MoCap
    #MATH FOR TRANSFORMATION FROM GLOBAL LIMO STATES TO RELATIVE CARLA POSITION AND ORIENTATION
    msgCarla.desired_velocity = ##X
    msgCarla.control_input = ##Y
    msgCarla.steering_angle = ##ORIENTATION

    return msgCarla


if __name__ == '__main__':
    mocap_to_carla = MocapProxy()
    while not rospy.is_shutdown():
        msg_carla = msg_format()
        mocap_to_carla.mocap_info.publish(msg_carla)
        time.sleep(0.1)
    print('publishing ended')