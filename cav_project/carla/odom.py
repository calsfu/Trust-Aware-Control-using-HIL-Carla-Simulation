import rospy
from nav_msgs.msg import Odometry

class Odom(): ##
    def __init__(self):
        subscriber = rospy.Subscriber('/limo/odom', Odometry, self.odom_callback) ##
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_yaw = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        
    def odom_callback(self, data): ##
        self.odom_x = data.pose.pose.position.x - self.last_x
        self.odom_y = data.pose.pose.position.y - self.last_y
        self.odom_yaw = data.pose.pose.orientation.z - self.last_yaw

        self.last_x = data.pose.pose.position.x
        self.last_y = data.pose.pose.position.y
        self.last_yaw = data.pose.pose.orientation.z


    def get_odom(self):
        return self.odom_x, self.odom_y, self.odom_yaw
	
