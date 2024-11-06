  #!/usr/bin/env python3
import math
import numpy as np
import time
from cvxopt.solvers import qp
import rospy
from std_msgs.msg import Float64, Bool, Float64MultiArray, String
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
from scipy.integrate import odeint
from cvxopt import matrix, solvers
from cav_project.msg import limo_info, QP_solution, ControlInfo
from set_map import set_map
import random

class CAV:
    def __init__(self, ID, enter = 0):
        self.ID = ID
        self.zone = []

        #rospy.init_node('CAV' + self.ID, anonymous=True)
        self.control_info_pub = rospy.Publisher('/control_info_' + self.ID, ControlInfo, queue_size=1)
        self.mocap_sub = rospy.Subscriber('/vrpn_client_node/' + self.ID + '/pose', PoseStamped, self.mocap_callback)
        self.qp_solution_sub = rospy.Subscriber('/qp_solution_' + self.ID, QP_solution, self.qp_solution_callback)
        self.cav_info_sub = rospy.Subscriber('/limo_info_' + self.ID, limo_info, self.cav_info_callback)
        #self.rate = rospy.Rate(25)
        self.generate_map(enter)

        self.qp_solution = QP_solution()
        self.e_prev_lateral = 0
        self.e_int_lateral = 0
        self.e_prev_longitudinal = 0
        self.e_int_longitudinal = 0
        self.delta_t = 0.1
        self.position_yaw = 0
        self.velocity = 0
        self.acceleration = 0
        self.Receivedata = 0
        self.v_min = 0.15
        self.v_max = 1
        self.u_min = -10
        self.u_max = 2
        self.Delta_T = 0.1

        self.phiRearEnd = 1.8
        self.phiLateral = 1.8
        self.deltaSafetyDistance = 0.3
        self.max_steering_angle = 7000

        self.position_x = 0
        self.position_y = 0
        self.position_z = 0
        self.current_position = (self.position_x, self.position_z)

        #for run()
        self.lateral_error = 0
        self.desired_velocity = 0.15
        self.within_critical_range = False
        self.line_changed = True
        self.within_collision_range = False
        self.exit_collision_range = True
        self.current = 0
        self.next = 1
        self.current_collision = 0
        self.next_collision = 1

        # Ensure self.lines and other lists are initialized before accessing
        if not self.lines or not self.turning_pts or not self.collision_pts:
            rospy.logerr(f"Initialization error in CAV {self.ID}: Lines or points are not properly initialized.")
            return

        self.current_line = self.lines[self.current]
        self.current_end_pt = self.turning_pts[self.next]
        if len(self.collision_pts) == 1:
            self.current_collision_pt1 = self.collision_pts[self.current_collision]
        elif self.current_collision <= len(self.collision_pts)-2:
            self.current_collision_pt1 = self.collision_pts[self.current_collision]
        else:
            self.current_collision_pt1 = (-1, -1)
        if len(self.collision_pts) == 2:
            self.current_collision_pt2 = self.collision_pts[self.next_collision]
        elif self.next_collision <= len(self.collision_pts)-1:
            self.current_collision_pt2 = self.collision_pts[self.next_collision]
        else:
            self.current_collision_pt2 = (-1, -1)


    def update_initial_conditions(self):
        self.current_line = self.lines[self.current]
        self.current_end_pt = self.turning_pts[self.next]
        if len(self.collision_pts) >= 1:
            self.current_collision_pt1 = self.collision_pts[self.current_collision]
        if len(self.collision_pts) >= 2:
            self.current_collision_pt2 = self.collision_pts[self.next_collision]



    def mocap_callback(self, msg):
        self.position_z = msg.pose.position.z * 1000
        self.position_x = msg.pose.position.x * 1000
        self.position_y = msg.pose.position.y * 1000
        self.position_yaw = 0
        self.Receivedata = 1
        self.current_position = (self.position_x, self.position_z)

    def qp_solution_callback(self, msg):
        self.qp_solution = msg

    def cav_info_callback(self, msg):
        self.cav_info = msg
        self.velocity = self.cav_info.vel.data
    
    def generate_map(self, starting_pt):
        set_map(self, self.ID)

        #equations for each line, in the A B C form, each variable is a tuple (A, B, C)
        self.path_A = self.generate_line(self.pt_a, self.pt_r)
        self.path_B = self.generate_line(self.pt_a, self.pt_b)
        self.path_C = self.generate_line(self.pt_b, self.pt_u)
        self.path_D = self.generate_line(self.pt_c, self.pt_e)
        self.path_E = self.generate_line(self.pt_d, self.pt_f)
        self.path_F = self.generate_line(self.pt_i, self.pt_h)
        self.path_G = self.generate_line(self.pt_j, self.pt_m)
        self.path_H = self.generate_line(self.pt_n, self.pt_q)
        self.path_I = self.generate_line(self.pt_g, self.pt_s)
        self.path_J = self.generate_line(self.pt_h, self.pt_t)
        self.path_K = self.generate_line(self.pt_r, self.pt_u)

        #all possible paths to exit link, assuming passing point a:
        self.c_to_e = {
            "turning_pts": [self.pt_c, self.pt_a, self.pt_b, self.pt_e],
            "collision_pts": [self.pt_c],
            "all_pts": [self.pt_c, self.pt_a, self.pt_b, self.pt_e],
            "lines": [self.path_A, self.path_B, self.path_C],
            "ranges": [self.act_range_c, self.act_range_a, self.act_range_b, self.act_range_e],
            "circles": [self.circle_c, self.circle_a, self.circle_b, self.circle_e],
            "PIDs": [self.path_A_PID, self.path_B_PID, self.path_C_PID],
            "curve_PIDs": [self.circle_c_PID, self.circle_a_PID, self.circle_b_PID, self.circle_e_PID]
        }

        self.c_to_f = {
            "turning_pts": [self.pt_c, self.pt_a, self.pt_b, self.pt_f],
            "collision_pts": [self.pt_c],
            "all_pts": [self.pt_c, self.pt_a, self.pt_b, self.pt_f],
            "lines": [self.path_A, self.path_B, self.path_C],
            "ranges": [self.act_range_c, self.act_range_a, self.act_range_b, self.act_range_f],
            "circles": [self.circle_c, self.circle_a, self.circle_b, self.circle_f],
            "PIDs": [self.path_A_PID, self.path_B_PID, self.path_C_PID],
            "curve_PIDs": [self.circle_c_PID, self.circle_a_PID, self.circle_b_PID, self.circle_f_PID]
        }

        self.c_to_m = {
            "turning_pts": [self.pt_c, self.pt_a, self.pt_b, self.pt_m],
            "collision_pts": [self.pt_c],
            "all_pts": [self.pt_c, self.pt_a, self.pt_b, self.pt_m],
            "lines": [self.path_A, self.path_B, self.path_C],
            "ranges": [self.act_range_c, self.act_range_a, self.act_range_b, self.act_range_m],
            "circles": [self.circle_c, self.circle_a, self.circle_b, self.circle_m],
            "PIDs": [self.path_A_PID, self.path_B_PID, self.path_C_PID],
            "curve_PIDs": [self.circle_c_PID, self.circle_a_PID, self.circle_b_PID, self.circle_m_PID]
        }

        self.c_to_t = {
            "turning_pts": [self.pt_c, self.pt_a, self.pt_b, self.pt_u, self.pt_t],
            "collision_pts": [self.pt_c, self.pt_q],
            "all_pts": [self.pt_c, self.pt_a, self.pt_b, self.pt_q, self.pt_u, self.pt_t],
            "lines": [self.path_A, self.path_B, self.path_C, self.path_K],
            "ranges": [self.act_range_c, self.act_range_a, self.act_range_b, self.act_range_u, self.act_range_t],
            "circles": [self.circle_c, self.circle_a, self.circle_b, self.circle_u, self.circle_t],
            "PIDs": [self.path_A_PID, self.path_B_PID, self.path_C_PID, self.path_K_PID],
            "curve_PIDs": [self.circle_c_PID, self.circle_a_PID, self.circle_b_PID, self.circle_u_PID, self.circle_t_PID]
        }

        self.c_to_n = {
            "turning_pts": [self.pt_c, self.pt_a, self.pt_b, self.pt_u, self.pt_r, self.pt_n],
            "collision_pts": [self.pt_c, self.pt_q, self.pt_s],
            "all_pts": [self.pt_c, self.pt_a, self.pt_b, self.pt_q, self.pt_u, self.pt_s, self.pt_r, self.pt_n],
            "lines": [self.path_A, self.path_B, self.path_C, self.path_K, self.path_A],
            "ranges": [self.act_range_c, self.act_range_a, self.act_range_b, self.act_range_u, self.act_range_r, self.act_range_n],
            "circles": [self.circle_c, self.circle_a, self.circle_b, self.circle_u, self.circle_r, self.circle_n],
            "PIDs": [self.path_A_PID, self.path_B_PID, self.path_C_PID, self.path_K_PID, self.path_A_PID],
            "curve_PIDs": [self.circle_c_PID, self.circle_a_PID, self.circle_b_PID, self.circle_u_PID, self.circle_r_PID, self.circle_n_PID]
        }

        self.j_to_e = {
            "turning_pts": [self.pt_j, self.pt_a, self.pt_b, self.pt_e],
            "collision_pts": [self.pt_j, self.pt_c],
            "all_pts": [self.pt_j, self.pt_c, self.pt_a, self.pt_b, self.pt_e],
            "lines": [self.path_A, self.path_B, self.path_C],
            "ranges": [self.act_range_j, self.act_range_a, self.act_range_b, self.act_range_e],
            "circles": [self.circle_j, self.circle_a, self.circle_b, self.circle_e],
            "PIDs": [self.path_A_PID, self.path_B_PID, self.path_C_PID],
            "curve_PIDs": [self.circle_j_PID, self.circle_a_PID, self.circle_b_PID, self.circle_e_PID]
        }

        self.j_to_f = {
            "turning_pts": [self.pt_j, self.pt_a, self.pt_b, self.pt_f],
            "collision_pts": [self.pt_j, self.pt_c],
            "all_pts": [self.pt_j, self.pt_c, self.pt_a, self.pt_b, self.pt_f],
            "lines": [self.path_A, self.path_B, self.path_C],
            "ranges": [self.act_range_j, self.act_range_a, self.act_range_b, self.act_range_f],
            "circles": [self.circle_j, self.circle_a, self.circle_b, self.circle_f],
            "PIDs": [self.path_A_PID, self.path_B_PID, self.path_C_PID],
            "curve_PIDs": [self.circle_j_PID, self.circle_a_PID, self.circle_b_PID, self.circle_f_PID]
        }

        self.j_to_m = {
            "turning_pts": [self.pt_j, self.pt_a, self.pt_b, self.pt_m],
            "collision_pts": [self.pt_j, self.pt_c],
            "all_pts": [self.pt_j, self.pt_c, self.pt_a, self.pt_b, self.pt_m],
            "lines": [self.path_A, self.path_B, self.path_C],
            "ranges": [self.act_range_j, self.act_range_a, self.act_range_b, self.act_range_m],
            "circles": [self.circle_j, self.circle_a, self.circle_b, self.circle_m],
            "PIDs": [self.path_A_PID, self.path_B_PID, self.path_C_PID],
            "curve_PIDs": [self.circle_j_PID, self.circle_a_PID, self.circle_b_PID, self.circle_m_PID]
        }

        self.j_to_t = {
            "turning_pts": [self.pt_j, self.pt_a, self.pt_b, self.pt_u, self.pt_t],
            "collision_pts": [self.pt_j, self.pt_c, self.pt_q],
            "all_pts": [self.pt_j, self.pt_c, self.pt_a, self.pt_b, self.pt_q, self.pt_u, self.pt_t],
            "lines": [self.path_A, self.path_B, self.path_C, self.path_K],
            "ranges": [self.act_range_j, self.act_range_a, self.act_range_b, self.act_range_u, self.act_range_t],
            "circles": [self.circle_j, self.circle_a, self.circle_b, self.circle_u, self.circle_t],
            "PIDs": [self.path_A_PID, self.path_B_PID, self.path_C_PID, self.path_K_PID],
            "curve_PIDs": [self.circle_j_PID, self.circle_a_PID, self.circle_b_PID, self.circle_u_PID, self.circle_t_PID]
        }

        self.j_to_n = {
            "turning_pts": [self.pt_j, self.pt_a, self.pt_b, self.pt_u, self.pt_r, self.pt_n],
            "collision_pts": [self.pt_j, self.pt_c, self.pt_q, self.pt_s],
            "all_pts": [self.pt_j, self.pt_c, self.pt_a, self.pt_b, self.pt_q, self.pt_u, self.pt_s, self.pt_r, self.pt_n],
            "lines": [self.path_A, self.path_B, self.path_C, self.path_K, self.path_A],
            "ranges": [self.act_range_j, self.act_range_a, self.act_range_b, self.act_range_u, self.act_range_r, self.act_range_n],
            "circles": [self.circle_j, self.circle_a, self.circle_b, self.circle_u, self.circle_r, self.circle_n],
            "PIDs": [self.path_A_PID, self.path_B_PID, self.path_C_PID, self.path_K_PID, self.path_A_PID],
            "curve_PIDs": [self.circle_j_PID, self.circle_a_PID, self.circle_b_PID, self.circle_u_PID, self.circle_r_PID, self.circle_n_PID]
        }

        self.s_to_e = {
            "turning_pts": [self.pt_s, self.pt_r, self.pt_a, self.pt_b, self.pt_e],
            "collision_pts": [self.pt_s, self.pt_j, self.pt_c],
            "all_pts": [self.pt_s, self.pt_r, self.pt_j, self.pt_c, self.pt_a, self.pt_b, self.pt_e],
            "lines": [self.path_K, self.path_A, self.path_B, self.path_C],
            "ranges": [self.act_range_s, self.act_range_r, self.act_range_a, self.act_range_b, self.act_range_e],
            "circles": [self.circle_s, self.circle_r, self.circle_a, self.circle_b, self.circle_e],
            "PIDs": [self.path_K_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID],
            "curve_PIDs": [self.circle_s_PID, self.circle_r_PID, self.circle_a_PID, self.circle_b_PID, self.circle_e_PID]
        }

        self.s_to_f = {
            "turning_pts": [self.pt_s, self.pt_r, self.pt_a, self.pt_b, self.pt_f],
            "collision_pts": [self.pt_s, self.pt_j, self.pt_c],
            "all_pts": [self.pt_s, self.pt_r, self.pt_j, self.pt_c, self.pt_a, self.pt_b, self.pt_f],
            "lines": [self.path_K, self.path_A, self.path_B, self.path_C],
            "ranges": [self.act_range_s, self.act_range_r, self.act_range_a, self.act_range_b, self.act_range_f],
            "circles": [self.circle_s, self.circle_r, self.circle_a, self.circle_b, self.circle_f],
            "PIDs": [self.path_K_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID],
            "curve_PIDs": [self.circle_s_PID, self.circle_r_PID, self.circle_a_PID, self.circle_b_PID, self.circle_f_PID]
        }
        
        self.s_to_m = {
            "turning_pts": [self.pt_s, self.pt_r, self.pt_a, self.pt_b, self.pt_m],
            "collision_pts": [self.pt_s, self.pt_j, self.pt_c],
            "all_pts": [self.pt_s, self.pt_r, self.pt_j, self.pt_c, self.pt_a, self.pt_b, self.pt_m],
            "lines": [self.path_K, self.path_A, self.path_B, self.path_C],
            "ranges": [self.act_range_s, self.act_range_r, self.act_range_a, self.act_range_b, self.act_range_m],
            "circles": [self.circle_s, self.circle_r, self.circle_a, self.circle_b, self.circle_m],
            "PIDs": [self.path_K_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID],
            "curve_PIDs": [self.circle_s_PID, self.circle_r_PID, self.circle_a_PID, self.circle_b_PID, self.circle_m_PID]
        }

        self.s_to_t = {
            "turning_pts": [self.pt_s, self.pt_r, self.pt_a, self.pt_b, self.pt_u, self.pt_t],
            "collision_pts": [self.pt_s, self.pt_j, self.pt_c, self.pt_q, self.pt_t],
            "all_pts": [self.pt_s, self.pt_r, self.pt_j, self.pt_c, self.pt_a, self.pt_b, self.pt_q, self.pt_u, self.pt_t],
            "lines": [self.path_K, self.path_A, self.path_B, self.path_C, self.path_K],
            "ranges": [self.act_range_s, self.act_range_r, self.act_range_a, self.act_range_b, self.act_range_u, self.act_range_t],
            "circles": [self.circle_s, self.circle_r, self.circle_a, self.circle_b, self.circle_u, self.circle_t],
            "PIDs": [self.path_K_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID, self.path_K_PID],
            "curve_PIDs": [self.circle_s_PID, self.circle_r_PID, self.circle_a_PID, self.circle_b_PID, self.circle_u_PID, self.circle_t_PID]
        }

        self.s_to_n = {
            "turning_pts": [self.pt_s, self.pt_r, self.pt_a, self.pt_b, self.pt_u, self.pt_r, self.pt_n],
            "collision_pts": [self.pt_s, self.pt_j, self.pt_c, self.pt_q, self.pt_s, self.pt_n],
            "all_pts": [self.pt_s, self.pt_r, self.pt_j, self.pt_c, self.pt_a, self.pt_b, self.pt_q, self.pt_u, self.pt_s, self.pt_r, self.pt_n],
            "lines": [self.path_K, self.path_A, self.path_B, self.path_C, self.path_K, self.path_A],
            "ranges": [self.act_range_s, self.act_range_r, self.act_range_a, self.act_range_b, self.act_range_u, self.act_range_r, self.act_range_n],
            "circles": [self.circle_s, self.circle_r, self.circle_a, self.circle_b, self.circle_u, self.circle_r, self.circle_n],
            "PIDs": [self.path_K_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID, self.path_K_PID, self.path_A_PID],
            "curve_PIDs": [self.circle_s_PID, self.circle_r_PID, self.circle_a_PID, self.circle_b_PID, self.circle_u_PID, self.circle_r_PID, self.circle_n_PID]
        }

        self.q_to_e = {
            "turning_pts" : [self.pt_q, self.pt_u, self.pt_r, self.pt_a, self.pt_b, self.pt_e],
            "collision_pts": [self.pt_q, self.pt_s, self.pt_j, self.pt_c],
            "all_pts": [self.pt_q, self.pt_u, self.pt_s, self.pt_r, self.pt_j, self.pt_c, self.pt_a, self.pt_b, self.pt_e],
            "lines" : [self.path_C, self.path_K, self.path_A, self.path_B, self.path_C],
            "ranges" : [self.act_range_q, self.act_range_u, self.act_range_r, self.act_range_a, self.act_range_b, self.act_range_e],
            "circles" : [self.circle_q, self.circle_u, self.circle_r, self.circle_a, self.circle_b, self.circle_e],
            "PIDs" : [self.path_C_PID, self.path_K_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID],
            "curve_PIDs" : [self.circle_q_PID, self.circle_u_PID, self.circle_r_PID, self.circle_a_PID, self.circle_b_PID, self.circle_e_PID]
        }

        self.q_to_f = {
            "turning_pts" : [self.pt_q, self.pt_u, self.pt_r, self.pt_a, self.pt_b, self.pt_f],
            "collision_pts": [self.pt_q, self.pt_s, self.pt_j, self.pt_c],
            "all_pts": [self.pt_q, self.pt_u, self.pt_s, self.pt_r, self.pt_j, self.pt_c, self.pt_a, self.pt_b, self.pt_f],
            "lines" : [self.path_C, self.path_K, self.path_A, self.path_B, self.path_C],
            "ranges" : [self.act_range_q, self.act_range_u, self.act_range_r, self.act_range_a, self.act_range_b, self.act_range_f],
            "circles" : [self.circle_q, self.circle_u, self.circle_r, self.circle_a, self.circle_b, self.circle_f],
            "PIDs" : [self.path_C_PID, self.path_K_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID],
            "curve_PIDs" : [self.circle_q_PID, self.circle_u_PID, self.circle_r_PID, self.circle_a_PID, self.circle_b_PID, self.circle_f_PID]
        }

        self.q_to_m = {
            "turning_pts" : [self.pt_q, self.pt_u, self.pt_r, self.pt_a, self.pt_b, self.pt_m],
            "collision_pts": [self.pt_q, self.pt_s, self.pt_j, self.pt_c],
            "all_pts": [self.pt_q, self.pt_u, self.pt_s, self.pt_r, self.pt_j, self.pt_c, self.pt_a, self.pt_b, self.pt_m],
            "lines" : [self.path_C, self.path_K, self.path_A, self.path_B, self.path_C],
            "ranges" : [self.act_range_q, self.act_range_u, self.act_range_r, self.act_range_a, self.act_range_b, self.act_range_m],
            "circles" : [self.circle_q, self.circle_u, self.circle_r, self.circle_a, self.circle_b, self.circle_m],
            "PIDs" : [self.path_C_PID, self.path_K_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID],
            "curve_PIDs" : [self.circle_q_PID, self.circle_u_PID, self.circle_r_PID, self.circle_a_PID, self.circle_b_PID, self.circle_m_PID]
        }

        self.q_to_t = {
            "turning_pts" : [self.pt_q, self.pt_u, self.pt_r, self.pt_a, self.pt_b, self.pt_u, self.pt_t],
            "collision_pts": [self.pt_q, self.pt_s, self.pt_j, self.pt_c, self.pt_q],
            "all_pts": [self.pt_q, self.pt_u, self.pt_s, self.pt_r, self.pt_j, self.pt_c, self.pt_a, self.pt_b, self.pt_q, self.pt_u, self.pt_t],
            "lines" : [self.path_C, self.path_K, self.path_A, self.path_B, self.path_C, self.path_K],
            "ranges" : [self.act_range_q, self.act_range_u, self.act_range_r, self.act_range_a, self.act_range_b, self.act_range_u, self.act_range_t],
            "circles" : [self.circle_q, self.circle_u, self.circle_r, self.circle_a, self.circle_b, self.circle_u, self.circle_t],
            "PIDs" : [self.path_C_PID, self.path_K_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID, self.path_K_PID],
            "curve_PIDs" : [self.circle_q_PID, self.circle_u_PID, self.circle_r_PID, self.circle_a_PID, self.circle_b_PID, self.circle_u_PID, self.circle_t_PID]
        }

        self.q_to_n = {
            "turning_pts" : [self.pt_q, self.pt_u, self.pt_r, self.pt_a, self.pt_b, self.pt_u, self.pt_r, self.pt_n],
            "collision_pts": [self.pt_q, self.pt_s, self.pt_j, self.pt_c, self.pt_q, self.pt_s],
            "all_pts": [self.pt_q, self.pt_u, self.pt_s, self.pt_r, self.pt_j, self.pt_c, self.pt_a, self.pt_b, self.pt_q, self.pt_u, self.pt_s, self.pt_r, self.pt_n],
            "lines" : [self.path_C, self.path_K, self.path_A, self.path_B, self.path_C, self.path_K, self.path_A],
            "ranges" : [self.act_range_q, self.act_range_u, self.act_range_r, self.act_range_a, self.act_range_b, self.act_range_u, self.act_range_r, self.act_range_n],
            "circles" : [self.circle_q, self.circle_u, self.circle_r, self.circle_a, self.circle_b, self.circle_u, self.circle_r, self.circle_n],
            "PIDs" : [self.path_C_PID, self.path_K_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID, self.path_K_PID, self.path_A_PID],
            "curve_PIDs" : [self.circle_q_PID, self.circle_u_PID, self.circle_r_PID, self.circle_a_PID, self.circle_b_PID, self.circle_u_PID, self.circle_r_PID, self.circle_n_PID]
        }

        #all possible paths to enter link:
        self.f_to_c = {
            "turning_pts": [self.pt_f, self.pt_d, self.pt_c],
            "collision_pts": [self.pt_d, self.pt_c],
            "all_pts": [self.pt_f, self.pt_d, self.pt_c],
            "lines": [self.path_E, self.path_D],
            "ranges": [self.act_range_f, self.act_range_d, self.act_range_c],
            "circles": [self.circle_f, self.circle_d, self.circle_c],
            "PIDs": [self.path_E_PID, self.path_D2_PID],
            "curve_PIDs": [self.circle_f_PID, self.circle_d_PID, self.circle_c_PID]
        }

        self.e_to_c = {
            "turning_pts": [self.pt_e, self.pt_c],
            "collision_pts": [self.pt_d, self.pt_c],
            "all_pts": [self.pt_e, self.pt_d, self.pt_c],
            "lines": [self.path_D],
            "ranges": [self.act_range_e, self.act_range_c],
            "circles": [self.circle_e, self.circle_c],
            "PIDs": [self.path_D_PID],
            "curve_PIDs": [self.circle_e_PID, self.circle_c_PID]
        }

        self.m_to_j = {
            "turning_pts": [self.pt_m, self.pt_j],
            "collision_pts": [self.pt_l, self.pt_k],
            "all_pts": [self.pt_m, self.pt_l, self.pt_k, self.pt_j],
            "lines": [self.path_G],
            "ranges": [self.act_range_m, self.act_range_j],
            "circles": [self.circle_m_PID, self.circle_j_PID],
            "PIDs": [self.path_G_PID],
            "curve_PIDs": [self.circle_m_PID, self.circle_j_PID]
        }

        self.n_to_q = {
            "turning_pts": [self.pt_n, self.pt_q],
            "collision_pts": [self.pt_o, self.pt_p],
            "all_pts": [self.pt_n, self.pt_o, self.pt_p, self.pt_q],
            "lines": [self.path_H],
            "ranges": [self.act_range_n, self.act_range_q],
            "circles": [self.circle_n_PID, self.circle_q_PID],
            "PIDs": [self.path_H_PID],
            "curve_PIDs": [self.circle_n_PID, self.circle_q_PID]
        }

        self.g_to_s = {
            "turning_pts": [self.pt_g, self.pt_s],
            "collision_pts": [self.pt_k, self.pt_o],
            "all_pts": [self.pt_g, self.pt_k, self.pt_o, self.pt_s],
            "lines": [self.path_I],
            "ranges": [self.act_range_g, self.act_range_s],
            "circles": [self.circle_g_PID, self.circle_s_PID],
            "PIDs": [self.path_I_PID],
            "curve_PIDs": [self.circle_g_PID, self.circle_s_PID]
        }

        self.t_to_h = {
            "turning_pts": [self.pt_t, self.pt_h],
            "collision_pts": [self.pt_p, self.pt_l],
            "all_pts": [self.pt_t, self.pt_p, self.pt_l, self.pt_h],
            "lines": [self.path_J],
            "ranges": [self.act_range_t, self.act_range_h],
            "circles": [self.circle_t_PID, self.circle_h_PID],
            "PIDs": [self.path_J_PID],
            "curve_PIDs": [self.circle_t_PID, self.circle_h_PID]
        }

        self.m_to_c = {
            "turning_pts": [self.pt_m, self.pt_l, self.pt_h, self.pt_c],
            "collision_pts": [self.pt_l, self.pt_c],
            "all_pts": [self.pt_m, self.pt_l, self.pt_h, self.pt_c],
            "lines": [self.path_G, self.path_J, self.path_F],
            "ranges": [self.act_range_m, self.act_range_l, self.act_range_h, self.act_range_i], # change c to i
            "circles": [self.circle_m, self.circle_l, self.circle_h, self.circle_i], # change circle_c to circle_i
            "PIDs": [self.path_G_PID, self.path_J_PID, self.path_F_PID],
            "curve_PIDs": [self.circle_m_PID, self.circle_l_PID, self.circle_h_PID, self.circle_c_PID] # change circle_c to circle_i
        }

        self.t_to_q = {
            "turning_pts": [self.pt_t, self.pt_p, self.pt_q],
            "collision_pts": [self.pt_p, self.pt_q],
            "all_pts": [self.pt_t, self.pt_p, self.pt_q],
            "lines": [self.path_J, self.path_H],
            "ranges": [self.act_range_t, self.act_range_p, self.act_range_q],
            "circles": [self.circle_t, self.circle_p, self.circle_q],
            "PIDs": [self.path_J_PID, self.path_H_PID],
            "curve_PIDs": [self.circle_t_PID, self.circle_p_PID, self.circle_q_PID]
        }

        self.n_to_s = {
            "turning_pts": [self.pt_n, self.pt_o, self.pt_s],
            "collision_pts": [self.pt_o, self.pt_s],
            "all_pts": [self.pt_n, self.pt_o, self.pt_s],
            "lines": [self.path_H, self.path_I],
            "ranges": [self.act_range_n, self.act_range_o, self.act_range_s],
            "circles": [self.circle_n, self.circle_o, self.circle_s],
            "PIDs": [self.path_H_PID, self.path_J_PID],
            "curve_PIDs": [self.circle_n_PID, self.circle_o_PID, self.circle_s_PID]
        }

        self.g_to_j = {
            "turning_pts": [self.pt_g, self.pt_k, self.pt_j],
            "collision_pts": [self.pt_k, self.pt_j],
            "all_pts": [self.pt_g, self.pt_k, self.pt_j],
            "lines": [self.path_I, self.path_G],
            "ranges": [self.act_range_g, self.act_range_k, self.act_range_j],
            "circles": [self.circle_g, self.circle_k, self.circle_j],
            "PIDs": [self.path_I_PID, self.path_G_PID],
            "curve_PIDs": [self.circle_g_PID, self.circle_k_PID, self.circle_j_PID]
        }   


        self.c_paths = [self.c_to_e, self.c_to_f, self.c_to_m, self.c_to_t, self.c_to_n]
        self.j_paths = [self.j_to_e, self.j_to_f, self.j_to_m, self.j_to_t, self.j_to_n]
        self.s_paths = [self.s_to_e, self.s_to_f, self.s_to_m, self.s_to_t, self.s_to_n]
        self.q_paths = [self.q_to_e, self.q_to_f, self.q_to_m, self.q_to_t, self.q_to_n]
        self.f_paths = [self.f_to_c]
        self.e_paths = [self.e_to_c]
        self.m_paths = [self.m_to_j, self.m_to_c]
        self.n_paths = [self.n_to_q, self.n_to_s]
        self.g_paths = [self.g_to_s, self.g_to_j]
        self.t_paths = [self.t_to_h, self.t_to_q]

        self.regenerate_map(starting_pt, True)


    def regenerate_map(self, starting_pt=0, first_time=False):
        if first_time == True:
            self.turning_pts = []
            self.lines = []
            self.ranges = []
            self.circles = []
            self.current_circles = []
            self.PIDs = []
            self.curve_PIDs = []

            #pick from one of the entering link zone paths that also start with the starting point
            if starting_pt == 'f':
                index = random.randint(0, len(self.f_paths) -1)
                self.enter_path = self.f_paths[index]
            elif starting_pt == 'e':
                index = random.randint(0, len(self.e_paths) -1)
                self.enter_path = self.e_paths[index]
            elif starting_pt == 'm':
                index = random.randint(0, len(self.m_paths) -1)
                self.enter_path = self.m_paths[index]
            elif starting_pt == 't':
                index = random.randint(0, len(self.t_paths) -1)
                self.enter_path = self.t_paths[index]
            elif starting_pt == 'n':
                index = random.randint(0, len(self.n_paths) -1)
                self.enter_path = self.n_paths[index]
            elif starting_pt == 'g':
                index = random.randint(0, len(self.g_paths) -1)
                self.enter_path = self.g_paths[index]

            #store the turning points, lines, ranges, circles, etc of that path to self.turning points, lines, ranges
            print(index)
            self.turning_pts = self.enter_path["turning_pts"]
            self.lines = self.enter_path["lines"]
            self.ranges = self.enter_path["ranges"]
            self.circles = self.enter_path["circles"]
            self.PIDs = self.enter_path["PIDs"]
            self.curve_PIDs = self.enter_path["curve_PIDs"]

            #then, pick from one of the paths exit link paths that also start with the last turning point
            if self.turning_pts[-1] == self.pt_c:
                index = random.randint(0, len(self.c_paths) -1)
                self.exit_path = self.c_paths[index]
            elif self.turning_pts[-1] == self.pt_j:
                index = random.randint(0, len(self.j_paths) -1)
                self.exit_path = self.j_paths[index]
            elif self.turning_pts[-1] == self.pt_s:
                index = random.randint(0, len(self.s_paths) -1)
                self.exit_path = self.s_paths[index]
            elif self.turning_pts[-1] == self.pt_q:
                index = random.randint(0, len(self.q_paths) -1)
                self.exit_path = self.q_paths[index]

            #append the turning points, lines, ranges, circles, etc of that path to self.turning points, lines, ranges
            #remove the last entry of turning points, ranges, circles, curve_pids because those are repeated
            print(index)
            self.turning_pts += self.exit_path["turning_pts"][1:]
            self.lines += self.exit_path["lines"]
            self.ranges += self.exit_path["ranges"][1:]
            self.circles += self.exit_path["circles"][1:]
            self.PIDs += self.exit_path["PIDs"]
            self.curve_PIDs += self.exit_path["curve_PIDs"][1:]


        #clear current path, except for relevant info
        if (first_time == False):
            #keep the last two points, ranges, circles, curve PIDs
            #keep the last line and PIDs
            self.turning_pts = self.turning_pts[-2:]
            self.lines = self.lines[-1:]
            self.ranges = self.ranges[-2:]
            self.circles = self.circles[-2:]
            self.PIDs = self.PIDs[-1:]
            self.curve_PIDs = self.curve_PIDs[-2:]

            #pick from one of the enter link paths that also start with the last turning point
            if self.turning_pts[-1] == self.pt_f:
                index = random.randint(0, len(self.f_paths) -1)
                self.enter_path = self.f_paths[index]
            elif self.turning_pts[-1] == self.pt_e:
                index = random.randint(0, len(self.e_paths) -1)
                self.enter_path = self.e_paths[index]
            elif self.turning_pts[-1] == self.pt_m:
                index = random.randint(0, len(self.m_paths) -1)
                self.enter_path = self.m_paths[index]
            elif self.turning_pts[-1] == self.pt_t:
                index = random.randint(0, len(self.t_paths) -1)
                self.enter_path = self.t_paths[index]
            elif self.turning_pts[-1] == self.pt_n:
                index = random.randint(0, len(self.n_paths) -1)
                self.enter_path = self.n_paths[index]


            #append the turning points, lines, ranges, circles, etc of that path to self.turning points, lines, ranges
            #remove the last entry of turning points, ranges, circles, curve_pids because those are repeated
            self.turning_pts += self.enter_path["turning_pts"][1:]
            self.lines += self.enter_path["lines"]
            self.ranges += self.enter_path["ranges"][1:]
            self.circles += self.enter_path["circles"][1:]
            self.PIDs += self.enter_path["PIDs"]
            self.curve_PIDs += self.enter_path["curve_PIDs"][1:]


            #pick from one of the exit link paths that also start with the last turning point
            if self.turning_pts[-1] == self.pt_c:
                index = random.randint(0, len(self.c_paths) -1)
                self.exit_path = self.c_paths[index]
            elif self.turning_pts[-1] == self.pt_j:
                index = random.randint(0, len(self.j_paths) -1)
                self.exit_path = self.j_paths[index]
            elif self.turning_pts[-1] == self.pt_s:
                index = random.randint(0, len(self.s_paths) -1)
                self.exit_path = self.s_paths[index]
            elif self.turning_pts[-1] == self.pt_q:
                index = random.randint(0, len(self.q_paths) -1)
                self.exit_path = self.q_paths[index]

            #append the turning points, lines, ranges, circles, etc of that path to self.turning points, lines, ranges
            #remove the last entry of turning points, ranges, circles, curve_pids because those are repeated
            self.turning_pts += self.exit_path["turning_pts"][1:]
            self.lines += self.exit_path["lines"]
            self.ranges += self.exit_path["ranges"][1:]
            self.circles += self.exit_path["circles"][1:]
            self.PIDs += self.exit_path["PIDs"]
            self.curve_PIDs += self.exit_path["curve_PIDs"][1:]

        return
    #helper functions for generate_map()
    def generate_line(self, pt_1, pt_2):
        A = -(pt_2[1] - pt_1[1])
        B = -(pt_1[0] - pt_2[0])
        C = -(pt_1[1] * (pt_2[0] - pt_1[0]) - (pt_2[1] - pt_1[1]) * pt_1[0])
        return A, B, C

    def calc_distance(self, pt_1, pt_2):
        distance = ((pt_1[0]- pt_2[0]) ** 2 + (pt_1[1] - pt_2[1]) ** 2) ** 0.5
        return distance

    def calc_dist_array(self, points):
        dist = []
        for i in range(len(points)-1):
            dist.append(self.calc_distance(points[i], points[i+1]))
        return dist

    def pid_lateral_controller(self, lateral_error, e_prev, e_int):
        e_int += lateral_error * self.delta_t
        e_der = (lateral_error - e_prev) / self.delta_t
        steering_angle = self.kp * lateral_error + self.ki * e_int + self.kd * e_der
        steering_angle = max(min(steering_angle, 7000), -7000)
        return steering_angle, lateral_error, e_int

    def run(self):
        self.kp, self.ki, self.kd = self.PIDs[self.current]
        self.current_line = self.lines[self.current]
        self.current_end_pt = self.turning_pts[self.next]

        # Check for zone transition and update collision points

        print(self.current_end_pt)
        print(self.current_line)
        print(self.current_collision_pt1)
        print(self.current_collision_pt2)



        #if the cav is near a critical point (which are turning corners), set path to a circle, change starting point and PID values to fit
        if abs(self.position_x  - self.current_end_pt[0])  < self.ranges[self.next][0] and \
            abs(self.position_z - self.current_end_pt[1]) < self.ranges[self.next][1] and \
            self.current_end_pt != self.pt_d:
            #delete if statement for infinite loop
            if (self.next == len(self.turning_pts) -1): 
                self.regenerate_map()
                self.current = 0
                self.next = 1
                self.kp, self.ki, self.kd = self.PIDs[self.current]
                self.current_line = self.lines[self.current]
                self.current_end_pt = self.turning_pts[self.next]                
            self.within_critical_range = True
            self.line_changed = False
            self.kp, self.ki, self.kd = self.curve_PIDs[self.next]
            lateral_error = (((self.position_x - self.circles[self.next][0])**2 + (self.position_z - self.circles[self.next][1])**2)**0.5 - self.circles[self.next][2])
            #print(self.ID, "in corner", lateral_error)

        #if a merging cav is near the merging point, switch to main path
        elif abs(self.position_x  - self.current_end_pt[0])  < self.ranges[self.next][0] and \
            abs(self.position_z - self.current_end_pt[1]) < self.ranges[self.next][1] and\
            self.current_end_pt == self.pt_d:
            if self.next == len(self.turning_pts)-1:
                control_info = ControlInfo()
                control_info.steering_angle = 0
                control_info.desired_velocity = 0
                control_info.control_input = 0
                self.control_info_pub.publish(control_info)
                print(self.ID, "finished running")
                return
            self.within_critical_range = True
            self.line_changed = False
            self.current_line = self.lines[self.next]
            self.kp, self.ki, self.kd = self.PIDs[self.next]
            lateral_error = (self.current_line[0]*self.position_x + self.current_line[1]*self.position_z + self.current_line[2])/((self.current_line[0]**2 + self.current_line[1]**2)**0.5)
            #print(self.ID, "merging", lateral_error)

        #when the cav is on a straight path
        else:
            self.within_critical_range = False
            self.current_line = self.lines[self.current]
            #print(self.ID, self.current_line)
            lateral_error = (self.current_line[0]*self.position_x + self.current_line[1]*self.position_z + self.current_line[2])/((self.current_line[0]**2 + self.current_line[1]**2)**0.5)
            #print(self.ID, "out of corner", lateral_error)

        #once out of the turning point, follow the next line
        if not self.line_changed and not self.within_critical_range:
            #self.current = (self.current+1) % len(self.turning_pts)
            #self.next = (self.next+1) % len(self.turning_pts)
            self.current = self.current+1
            self.next = self.next+1
            self.line_changed = True
            self.within_critical_range = False
            self.current_line = self.lines[self.current]
            self.current_end_pt = self.turning_pts[self.next]
            self.kp, self.ki, self.kd = self.PIDs[self.current]
            self.e_prev_lateral= 0
            self.e_int_lateral = 0
            lateral_error = (self.current_line[0]*self.position_x + self.current_line[1]*self.position_z + self.current_line[2])/((self.current_line[0]**2 + self.current_line[1]**2)**0.5)



        #increament collision points as they are traversed
        if abs(self.position_x  - self.current_collision_pt1[0])  < self.lane_width/2 and \
            abs(self.position_z - self.current_collision_pt1[1]) < self.lane_width/2:
            self.within_collision_range = True
            self.exit_collision_range = False
        else:
            self.exit_collision_range = True

        # Update collision points after exiting the collision range
        if self.within_collision_range and self.exit_collision_range:
            # Update current and next collision points
            self.current_collision = min(self.current_collision + 1, len(self.collision_pts) - 1)
            self.next_collision = min(self.next_collision + 1, len(self.collision_pts) - 1)

            # Assign new collision points
            if self.current_collision < len(self.collision_pts):
                self.current_collision_pt1 = self.collision_pts[self.current_collision]
            else:
                self.current_collision_pt1 = (-1, -1)

            if self.next_collision < len(self.collision_pts):
                self.current_collision_pt2 = self.collision_pts[self.next_collision]
            else:
                self.current_collision_pt2 = (-1, -1)

            self.within_collision_range = False


        #calculate steering and publisher to the listener node on the limo
        actual_velocity = self.velocity
        #if self.ID == "limo770":
        desired_velocity = actual_velocity + self.qp_solution.u* 0.1 # Use control input from QP solution
            #print("act vel", actual_velocity, "desired_velocity", desired_velocity,"qp_solutn", self.qp_solution.u)
        #else:
        #    desired_velocity = 0.4

        steering_angle, self.e_prev_lateral, self.e_int_lateral = self.pid_lateral_controller(lateral_error, self.e_prev_lateral, self.e_int_lateral)
        control_input = -1
        desired_velocity = min(0.75, max(0, desired_velocity))
        #print("lateral_error of", self.ID, lateral_error)
        #rospy.loginfo(f"CAV{self.ID} Control Info - Steering Angle: {steering_angle}72Desired Velocity: {desired_velocity}, Control Input: {control_input}")

        # Publish control info
        control_info = ControlInfo()
        control_info.steering_angle = steering_angle
        control_info.desired_velocity = desired_velocity
        control_info.control_input = control_input
        self.control_info_pub.publish(control_info)


    def update_zone(self, coordinator):
        zones = []

        # Check for Path J
        if self.current_line == self.path_J:
            if self.current_collision_pt1 in [self.pt_p, self.pt_l] or self.current_collision_pt2 in [self.pt_p, self.pt_l]:
                zones.append("Intersection Zone")
            else:
                zones.append("Intersection Zone")
                zones.append("Link Zone")

        # Check for Path H
        elif self.current_line == self.path_H:
            if self.current_collision_pt1 in [self.pt_o, self.pt_p] or self.current_collision_pt2 in [self.pt_o, self.pt_p]:
                zones.append("Intersection Zone")
            else:
                zones.append("Intersection Zone")
                zones.append("Link Zone")

        # Check for Path G
        elif self.current_line == self.path_G:
            if self.current_collision_pt1 in [self.pt_l, self.pt_k] or self.current_collision_pt2 in [self.pt_l, self.pt_k]:
                zones.append("Intersection Zone")
            else:
                zones.append("Intersection Zone")
                zones.append("Link Zone")

        # Check for Path I
        elif self.current_line == self.path_I:
            if self.current_collision_pt1 in [self.pt_k, self.pt_o] or self.current_collision_pt2 in [self.pt_k, self.pt_o]:
                zones.append("Intersection Zone")
            else:
                zones.append("Intersection Zone")
                zones.append("Link Zone")

            # Link Zone only
        elif self.current_line in [self.path_A, self.path_B, self.path_C, self.path_K, self.path_F]:
                zones.append("Link Zone")

        # Check for Merging Path Zone
        elif self.current_line == self.path_D or self.current_line == self.path_E:
            if self.current_collision_pt1 == self.pt_d or self.current_collision_pt2 == self.pt_d:
                zones.append("Merging Path Zone")
            else:
                zones.append("Merging Path Zone")
                zones.append("Link Zone")

        # Update the order lists in the coordinator
        #for zone in zones:
         #   if self not in coordinator.order_list[zone]:
          #      coordinator.order_list[zone].append(self)
           #     print(f"Added CAV {self.ID} to {zone}")

        # Track the zones the CAV belongs to
        self.zone = zones
