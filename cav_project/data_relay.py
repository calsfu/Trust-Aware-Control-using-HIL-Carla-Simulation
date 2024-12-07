#!/usr/bin/env python3
import rospy 
import time
import numpy as np
import threading
import math
from std_msgs.msg import String
from cav_project.msg import limo_info, QP_solution, ControlInfo
from geometry_msg.msg import PoseStamped

class data_relay_class:
    
    def __init__(self, cav_id, attack_sel):
        self.cav_id = cav_id 
        self.attack_sel = attack_sel

        #Subscribe to CAV Agent Data Topics 
        self.cav_limo_data_sub = rospy.Subscriber('/limo_info_' + self.cav_id, limo_info, self.cav_limo_callback )       
        self.cav_client_data_sub = rospy.Subscriber('/vrpn_client_node_' + self.cav_id, PoseStamped, self.cav_client_callback) 
        self.cav_qp_data_sub = rospy.Subscriber('/qp_solution_' + self.cav_id, QP_solution, self.cav_qp_callback) 
        
        #Subscriber data
        self.cav_limo_data             = limo_info()        
        self.cav_vrpn_client_node_data = PoseStamped() 
        self.cav_qp_solution_data      = QP_solution()
        
        #Set Attack Data          
        self.attack_relay_data     = max(range(Float64))
        
        #set rate
        self.rate = rospy.Rate(10)            #sleep @ 10 HZ        
        
        #Publish data_relay info to data_relay topics
        self.cav_limo_info_pub        = rospy.Publisher('/limo_info_relay_' + self.cav_id, limo_info, queue_size=10)       
        self.cav_vrpn_client_node_pub = rospy.Publisher('/vrpn_client_node_relay_' + self.cav_id, PoseStamped, queue_size=10) 
        self.cav_qp_solution_pub      = rospy.Publisher('/qp_solution_relay_' + self.cav_id, QP_solution, queue_size=10)       
    
    #sub callback function 
    def cav_limo_callback(self,data):
        self.limo_data = data
    
    def cav_client_callback(self,data):
        self.vrpn_client_node_data = data
        
    def cav_qp_callback(self,data):
        self.qp_solution_data = data    
    
    #Set true data or max value based on malicous data attack        
    #limo data publish callback function
    def cav_limo_info_publish(self):
        if self.attack_sel == 1:
            self.cav_limo_info_pub.publish(self.attack_relay_data)
        else:
            self.cav_limo_info_pub.publish(self.cav_limo_data)
    
    #vrpn client node data publish callback function
    def cav_vrpn_client_node_publish(self): 
        if self.attack_sel == 1:
            self.cav_vrpn_client_node_pub.publish(self.attack_relay_data)
        else:
            self.cav_vrpn_client_node_pub.publish(self.cav_vrpn_client_node_data)
    
    #vrpn client node data publish callback function
    def cav_qp_solution_publish(self): 
        if self.attack_sel == 1:
            self.cav_qp_solution_pub.publish(self.attack_relay_data)
        else:
            self.cav_qp_solution_pub.publish(self.cav_qp_solution_data)
           


