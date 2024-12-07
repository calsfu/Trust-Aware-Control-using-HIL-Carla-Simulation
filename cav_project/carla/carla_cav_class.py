#!/usr/bin/env python3
import rospy 
import time
import numpy as np
import threading
import math
from std_msgs.msg import String
# from cav_project.dev import cav_control_id_info, cav_vrpn_client_node, cav_limo_info, cav_qp_solution
from cav_project.msg import limo_info, QP_solution, ControlInfo
from geometry_msg.msg import PoseStamped

#Message Formats 
#ControlInfo
    # float64 steering_angle
    # float64 desired_velocity
    # float64 control_input

#vrpn_client_node
    #float64 position
    #float64 orientation

#limo_info
    #float64 velocity
    
#qp_solution
    #float64 state SUBJECT TO CHANGE, CURRENTLY UNDEFINED

class carla_cav:
    
    def __init__(self, cav_id):
        self.cav_id = cav_id 
    
        # Initialize ROS node w/ cav id
        # I think it's not supposed to be a node? Change if needed
        # rospy.init_node('carla_cav_node_'+self.cav_id, anonymous=True)
        
        #Subscribe to controller info ROS topic w/ cav_control_id_info message
        self.cav_control_data_sub = rospy.Subscriber("control_info_"+self.cav_id, ControlInfo, self.cav_control_callback)
        
        #sub data
        self.cav_data = ControlInfo()                   #subscriber data
        
        #pub data
        self.limo_data             = limo_info()                    #cav limo data
        self.vrpn_client_node_data = PoseStamped() #vrpn client data
        self.qp_solution_data      = QP_solution() 
        
        #set rate
        self.rate = rospy.Rate(10)            #sleep @ 10 HZ        
        
        #Publish to cav state ROS topics w/ cav info
        self.limo_info_pub        = rospy.Publisher('/limo_info_' + self.cav_id, limo_info, queue_size=10)       
        self.vrpn_client_node_pub = rospy.Publisher('/vrpn_client_node' + self.cav_id, PoseStamped, queue_size=10) 
        self.qp_solution_pub      = rospy.Publisher('/qp_solution_' + self.cav_id, QP_solution, queue_size=10) 
        
    #sub callback function 
    def cav_control_callback(self,data):
        self.cav_data = data

    def get_control_data(self):
        return self.cav_data
    
    #limo data publish callback function
    def cav_limo_info_publish(self): 
        self.limo_info_pub.publish(self.cav_limo_data)
    
    #vrpn client node data publish callback function
    def cav_vrpn_client_node_publish(self): 
        self.vrpn_client_node_pub.publish(self.cav_vrpn_client_node_data)
    
    #vrpn client node data publish callback function
    def cav_qp_solution_publish(self): 
        self.qp_solution_pub.publish(self.cav_qp_solution_data)
    
    def update_and_publish(self, velocity, state, transform):
        # POSE
        # might have to update to match mocap
        self.vrpn_client_node_data.pose.position.x = transform.location.x
        self.vrpn_client_node_data.pose.position.y = -transform.location.y
        self.vrpn_client_node_data.pose.position.z = transform.location.z

        # quaternion!
        pitch = transform.rotation.pitch
        yaw = -transform.rotation.yaw  
        roll = transform.rotation.roll

        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        self.vrpn_client_node_data.pose.orientation.x = qx
        self.vrpn_client_node_data.pose.orientation.y = qy
        self.vrpn_client_node_data.pose.orientation.z = qz
        self.vrpn_client_node_data.pose.orientation.w = qw

        # VELOCITY (assuming they mean speed cause it's only 1 float)
        self.limo_data.velocity = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

        # STATE
        self.qp_solution_data = state

        self.cav_control_callback()
        self.cav_vrpn_client_node_publish()
        self.cav_qp_solution_publish()


# testing
# if __name__ == '__main__':
    
#     cav_instances = []
    
#     #Create # Cav vehicles
#     for i in range(12):
#         cav_id = f"cav{i}" 
#         cav_inst = carla_cav_class(cav_id) #creates instance
#         cav_instances.append(cav_inst)
    
#     #create threads torun each cav instance independenty
#     cav_threads = []
#     for inst in cav_instances:
#         thread = threading.Thread(target=inst.run)
#         thread.start()
#         cav_threads.append(thread)
    
#     for cav_thread in cav_threads:
#         cav_thread.join()

