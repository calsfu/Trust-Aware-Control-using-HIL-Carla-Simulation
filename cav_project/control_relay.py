#!/usr/bin/env python3
import rospy 
import time
import numpy as np
import threading
import math
from std_msgs.msg import String
from cav_project.msg import ControlInfo
from geometry_msgs.msg import PoseStamped



class control_relay_class:
    
    def __init__(self, cav_id):
        self.cav_id = cav_id 
            
        #Subscribe to controller info ROS topic w/ cav_control_id_info message
        self.cav_control_data_sub = rospy.Subscriber("control_info_"+self.cav_id, ControlInfo, self.cav_control_callback)
        
        #sub data
        self.control_relay_data    = ControlInfo() # Subscriber Data
        
        #pub data
                            #cav limo data
        self.attack_relay_data     = ControlInfo() 
        
        #set rate
        self.rate = rospy.Rate(10)            #sleep @ 10 HZ        
        
        #Publish control_relay info to Control Relay
        self.control_relay_info_pub        = rospy.Publisher('/control_info_relay_' + self.cav_id, ControlInfo, queue_size=10)       
        
    #sub callback function 
    def cav_control_callback(self,data):
        self.control_relay_data = data
    
    #Publish Data if Attack Boolean is True
    def control_relay_info_publish(self): 
        if self.attack_sel == 1:
            #Want to control each control variabe independently
            self.control_relay_info_pub.publish(self.attack_relay_data)
            
        else:
            self.control_relay_info_pub.publish(self.control_relay_data)
            


